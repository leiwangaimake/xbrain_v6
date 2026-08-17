#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: run_stack_dev.sh
# Brief: DEV-ONLY direct launcher for the runnable software stack (integration test)
#
# Description:
# What this solves. The production launcher scripts/start_all.sh drives the
# 10 S3.3 systemd sequence, which today CANNOT run on a dev box: Stage 0c
# (config-freeze) refuses to start on the still-uncalibrated null safety params
# (10 S5.4.4 assertion A, by design), GATE-6 needs /etc/xbrain/hw_profile, and
# zenohd-gen wants real LAN2/wifi IPs. So there is no way to bring the stack up
# via systemctl until deployment-time config + hardware exist.
#
# This script is the DEV path: it starts the RUNNABLE software subset directly
# (no systemd, no freeze, no GATE) so the voice / RTK / HMI chain can be
# integration-tested now. It is NOT the deployment path -- start_all.sh is, and
# stays systemd-pure. The C++ robot side (quadruped / perception / chassis_relay
# / behavior_proxy / teleop_input / nav2 / zenoh-bridge) is not built yet and is
# simply not started here.
#
# Startup order mirrors the SPIRIT of 10 S3.3 for the pieces that run:
#   routers (RT then GEN) -> rtk_driver -> p1_motion (cross-plane bridge)
#   -> p2..p5 (general plane) -> AI services (asr / llm / payload).
# There is no BIT release gate here (P2 dev mode); this is a bring-up harness,
# not a safety-gated boot.
#
# Two known dev gotchas baked in so callers do not re-hit them:
#   * --resolved-root MUST be ABSOLUTE. A relative path fails the snapshot
#     root-confinement check (10 S5.4.1 / CFG-ROOT-5): MANIFEST records absolute
#     paths and the loader refuses to follow one that is 'outside' a relative
#     root string.
#   * the two routers listen on LOOPBACK only (127.0.0.1:7449 / :7447) with
#     multicast scouting off. gossip stays on (zenohd default) so a peer client
#     publishing through the router is actually forwarded (the F4 lesson).
#
# Usage:
#   scripts/dev/run_stack_dev.sh          start the stack (background)
#   scripts/dev/run_stack_dev.sh --stop   stop everything this script started
#   scripts/dev/run_stack_dev.sh --status show what is alive
# Env overrides: XBRAIN_ROBOT_ID (default dev) / RESOLVED_DIR / LOG_DIR.

set -uo pipefail   # NO -e: a dev process that fails must not abort the rest;
                   # we start everything and report status at the end.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"

# Absolute resolved-root (the gotcha above). Overridable but defaults to the
# dev materialised snapshot.
RESOLVED_DIR="${RESOLVED_DIR:-$REPO_ROOT/data/run/resolved}"
LOG_DIR="${LOG_DIR:-$REPO_ROOT/data/run/dev-logs}"
PIDFILE="$LOG_DIR/dev-stack.pids"
ROBOT_ID="${XBRAIN_ROBOT_ID:-dev}"
ZENOHD="${ZENOHD:-/usr/local/bin/zenohd}"
RTK_BIN="$REPO_ROOT/ros2_ws/sensor/build/rtk_driver"

# libzenohc lives in /usr/local/lib, which is not on the default ldconfig path.
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
# NEITHER XBRAIN_ROBOT_ID NOR XBRAIN_RESOLVED_DIR is exported here -- both break
# the materialise step (which runs first), for DIFFERENT reasons:
#   * XBRAIN_RESOLVED_DIR is not in the L5 whitelist (11 S1.1.9.7), so freeze
#     aborts on it (E_CONFIG_INVALID) and wipes the snapshot. It is a dev-only
#     rtk_driver override, passed INLINE to that one process.
#   * XBRAIN_ROBOT_ID is whitelisted, BUT if it is set while materialise runs the
#     freeze resolves per-proc refs against that rid and aborts on p2_core
#     (per_proc_ref_unresolved). So the rid is exported ONLY AFTER materialise
#     (below), once the snapshot is built -- the running processes still get it.

mkdir -p "$LOG_DIR"

# --- stop / status --------------------------------------------------------
_stop_all() {
    if [[ ! -f "$PIDFILE" ]]; then
        echo "no pidfile ($PIDFILE); nothing this script tracked is running"
        return 0
    fi
    # Kill in REVERSE start order (dependents first). Each line is 'name pid'.
    tac "$PIDFILE" | while read -r name pid; do
        if kill -0 "$pid" 2>/dev/null; then
            printf '  stop %-14s pid %s\n' "$name" "$pid"
            kill "$pid" 2>/dev/null
        fi
    done
    sleep 1
    rm -f "$PIDFILE"
    echo "run_stack_dev: stopped"
}

_status() {
    printf '%-14s %-8s %s\n' NAME STATE PID
    if [[ -f "$PIDFILE" ]]; then
        while read -r name pid; do
            if kill -0 "$pid" 2>/dev/null; then st="alive"; else st="dead"; fi
            printf '%-14s %-8s %s\n' "$name" "$st" "$pid"
        done < "$PIDFILE"
    fi
    echo "-- listening ports --"
    ss -ltn 2>/dev/null | grep -E ':(7449|7447|18081|18082|18083|18080)' \
        || echo "  (none of the expected ports are listening yet)"
}

case "${1:-}" in
    --stop)   _stop_all; exit 0 ;;
    --status) _status;   exit 0 ;;
esac

# Refuse a double-start: a stale stack would fight for the ports.
if [[ -f "$PIDFILE" ]] && awk '{print $2}' "$PIDFILE" | xargs -r -I{} kill -0 {} 2>/dev/null; then
    echo "a dev stack is already running (see $PIDFILE); run --stop first" >&2
    exit 1
fi
: > "$PIDFILE"

# start_bg NAME -- CMD...: launch detached, log, record the pid.
_start_bg() {
    local name="$1"; shift
    local log="$LOG_DIR/$name.log"
    # setsid so the child survives this script exiting; nohup redirect for logs.
    setsid nohup "$@" >"$log" 2>&1 < /dev/null &
    local pid=$!
    printf '%s %s\n' "$name" "$pid" >> "$PIDFILE"
    printf '  start %-14s pid %-7s log %s\n' "$name" "$pid" "$log"
}

echo "run_stack_dev: RESOLVED_DIR=$RESOLVED_DIR ROBOT_ID=$ROBOT_ID"

# Refresh the dev resolved snapshot so config is present (dev counterpart to
# Stage 0c config-freeze -- WITHOUT the assertion gate, which nulls would trip).
# Runs with a CLEAN XBRAIN_* env (see the env note above). A materialise failure
# is fatal: it wipes the snapshot, so every config-reading process would then die
# on 'MANIFEST.json does not exist' -- better to stop here with the real reason.
echo "==> refreshing dev resolved snapshot"
if python3 "$SCRIPT_DIR/materialize_resolved.py" >"$LOG_DIR/materialize.log" 2>&1; then
    echo "  materialize ok"
else
    echo "  materialize FAILED -- aborting (nothing would find its config):" >&2
    sed 's/^/    /' "$LOG_DIR/materialize.log" >&2
    rm -f "$PIDFILE"
    exit 1
fi
# NOW export the rid (whitelisted), after the snapshot is built. Every process
# that fills a Zenoh {rid} needs the SAME value: rtk_driver throws without it and
# p1's gnss bridge turns OFF ("XBRAIN_ROBOT_ID unset"), silently breaking
# rt/gnss -> state/pose. One export keeps rtk_driver + p1 + the rest on one rid.
export XBRAIN_ROBOT_ID="$ROBOT_ID"

# Turn the p5 event subsystem ON in the dev stack (SW-12): p5 reads XBRAIN_RECORD_DB
# for the record.db path until common.db.record_db is assigned (SW-6). This is a
# DIRECT env read in p5 __main__, NOT a config layer -- it never reaches freeze or
# the L5 whitelist, so exporting it globally is safe (p2..p4 ignore it).
export XBRAIN_RECORD_DB="$REPO_ROOT/data/run/record.db"

# --- routers (loopback only; gossip default-on) ---------------------------
echo "==> routers"
_start_bg zenohd-rt  "$ZENOHD" -l tcp/127.0.0.1:7449 --no-multicast-scouting
_start_bg zenohd-gen "$ZENOHD" -l tcp/127.0.0.1:7447 --no-multicast-scouting
sleep 2

# --- RT-plane: rtk_driver (real serial /dev/ttyACM0) ----------------------
echo "==> RT plane"
if [[ -x "$RTK_BIN" ]]; then
    # XBRAIN_ROBOT_ID comes from the global export above (whitelisted). Only
    # XBRAIN_RESOLVED_DIR is passed INLINE -- kept out of the global env so the
    # materialise step never sees it (L5 whitelist would abort).
    _start_bg rtk_driver env XBRAIN_RESOLVED_DIR="$RESOLVED_DIR" "$RTK_BIN"
else
    echo "  rtk_driver not built ($RTK_BIN) -- skipped"
fi

# --- cross-plane bridge: p1_motion ----------------------------------------
echo "==> p1_motion (cross-plane bridge rt/gnss -> state/pose)"
_start_bg p1_motion python3 -m xbrain.p1_motion --voice-loop --resolved-root "$RESOLVED_DIR"
sleep 1

# --- general plane: p2..p5 ------------------------------------------------
echo "==> general plane p2..p5"
for proc in p2_core p3_task p4_agent p5_gateway; do
    _start_bg "$proc" python3 -m "xbrain.$proc" --voice-loop --resolved-root "$RESOLVED_DIR"
done
sleep 1

# --- AI services (decoupled; slow model load must not block the above) -----
echo "==> AI services"
for svc in asr llm payload; do
    s="$REPO_ROOT/services/$svc"
    case "$svc" in
        asr)     sh="$s/asr_server.sh" ;;
        llm)     sh="$s/llm_server.sh" ;;
        payload) sh="$s/payload_server.sh" ;;
    esac
    if [[ -x "$sh" ]]; then _start_bg "ai_$svc" bash "$sh"
    else echo "  ai_$svc script missing ($sh) -- skipped"; fi
done

echo
echo "run_stack_dev: all launched. Waiting 4s then reporting status..."
sleep 4
_status
echo
echo "HMI (if p5 up): http://<this-host>:18083   |   stop: $0 --stop"
