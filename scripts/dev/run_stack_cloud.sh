#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: run_stack_cloud.sh
# Brief: DEV-ONLY launcher for the CLOUD Qt integration test (v2.0 17-key face)
#
# Description:
# What this solves. scripts/dev/run_stack_dev.sh brings up the runnable software
# subset for the VOICE / RTK / HMI chain, and it binds both Zenoh routers to
# LOOPBACK ONLY. That is correct for a dev box and wrong for the cloud test: the
# customer's Qt client is an off-board Zenoh CLIENT that must reach the GENERAL
# router over the LAN (任务枚举_qt端v2.0 S1.1 -- "连接该机器人通用面
# tcp/<robot-ip>:7447"). A loopback-only router shows up as "Qt connects to
# nothing" and is indistinguishable from a firewall problem.
#
# This script is the CLOUD path. It is a SEPARATE file rather than a flag on
# run_stack_dev.sh on purpose: the difference is an EXPOSED LAN SOCKET carrying
# estop and task authority, and a mode flag on the everyday dev launcher is how
# that exposure gets turned on by accident. Two names, two intents.
#
# What it does NOT change: on-board participants still connect to
# tcp/127.0.0.1:7447 / :7449 -- those two endpoints are CONSTANTS in
# xbrain/common/zenoh/session_factory.py per 11 S1.1.4 / S1.1.7, NOT config. So
# adding a LAN listener to the GEN router is purely additive: internal traffic
# keeps riding loopback, Qt rides the LAN address, both reach the same router.
#
# The RT router stays loopback-only and MUST stay that way (RT-C4; Qt never
# touches 7449 -- v2.0 S1.1 逐字 "Qt 不运行 router, 不连接实时面 7449").
#
# NET-C9 -- the GEN router gets TWO explicit -l endpoints, never 0.0.0.0. A
# wildcard bind would put the estop-carrying bus on every segment at once,
# including the chassis and PTZ segments (11 S1.1.9.2).
#
# Startup order and WHY each step sits where it does:
#   0  preflight   -- rid shape, LAN ip really exists on an interface, ports
#                     free, resolved snapshot refreshed. Every one of these
#                     fails LOUDLY here instead of as a confusing symptom later.
#   1  chassis_stub-- the CHS-A observation point (tcp/30004). Started BEFORE p1
#                     so the first APDU lands in the log rather than in a
#                     reconnect warning. This is the acceptance-evidence sink for
#                     motion commands while quadruped is unbuilt.
#   2  routers     -- RT first then GEN, mirroring 10 S3.3 stage 0z ordering.
#   3  rtk_driver  -- RT plane; skipped when the binary or /dev/ttyACM0 is absent
#                     (GATED-HW, and its absence must not stop the cloud test).
#   4  p1_motion   -- cross-plane; also the process that owns the chassis exit.
#   5  p2..p5      -- general plane, p5 LAST: the cloud bridge lives inside it
#                     and its first observations should find the others already
#                     publishing.
#   6  AI services -- slow model load, decoupled, must not delay the cloud face.
#   7  verify      -- assert the things the test actually depends on: the LAN
#                     socket is listening and p5 logged the cloud bridge with the
#                     agreed rid. A launcher that only prints "started" hides
#                     exactly the two failures that matter here.
#
# Usage:
#   XBRAIN_ROBOT_ID=<rid> scripts/dev/run_stack_cloud.sh
#   scripts/dev/run_stack_cloud.sh --stop
#   scripts/dev/run_stack_cloud.sh --status
# Env: XBRAIN_ROBOT_ID (REQUIRED -- must match the rid the customer's Qt uses)
#      CLOUD_LAN_IP    (default: auto-detected primary non-loopback IPv4)
#      RESOLVED_DIR / LOG_DIR / ZENOHD  (same meaning as run_stack_dev.sh)

set -uo pipefail   # NO -e: one dead dev process must not abort the rest; the
                   # stage-7 verify is what decides whether the run is usable.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"

RESOLVED_DIR="${RESOLVED_DIR:-$REPO_ROOT/data/run/resolved}"
LOG_DIR="${LOG_DIR:-$REPO_ROOT/data/run/cloud-logs}"
PIDFILE="$LOG_DIR/cloud-stack.pids"
ZENOHD="${ZENOHD:-/usr/local/bin/zenohd}"
RTK_BIN="$REPO_ROOT/ros2_ws/sensor/build/rtk_driver"
CHASSIS_PORT="${CHASSIS_PORT:-30004}"

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

mkdir -p "$LOG_DIR"

_stop_all() {
    if [[ ! -f "$PIDFILE" ]]; then
        echo "no pidfile ($PIDFILE); nothing this script tracked is running"
        return 0
    fi
    tac "$PIDFILE" | while read -r name pid; do
        if kill -0 "$pid" 2>/dev/null; then
            printf '  stop %-14s pid %s\n' "$name" "$pid"
            kill "$pid" 2>/dev/null
        fi
    done
    sleep 1
    rm -f "$PIDFILE"
    echo "run_stack_cloud: stopped"
}

_status() {
    printf '%-14s %-8s %s\n' NAME STATE PID
    if [[ -f "$PIDFILE" ]]; then
        while read -r name pid; do
            if kill -0 "$pid" 2>/dev/null; then st="alive"; else st="dead"; fi
            printf '%-14s %-8s %s\n' "$name" "$st" "$pid"
        done < "$PIDFILE"
    fi
    echo "-- listening --"
    ss -ltn 2>/dev/null | grep -E ':(7449|7447|18080|18081|18082|18083|30004)' \
        || echo "  (nothing expected is listening)"
}

case "${1:-}" in
    --stop)   _stop_all; exit 0 ;;
    --status) _status;   exit 0 ;;
esac

# ---------------------------------------------------------------- stage 0
echo "==> [stage 0] preflight"

# rid: v2.0 S1.3 -- must match [a-z0-9_-]{1,32} and be byte-identical to the
# second key segment Qt publishes on. An empty rid makes maybe_wire() skip the
# bridge entirely and the symptom is a silent no-op, so refuse here instead.
ROBOT_ID="${XBRAIN_ROBOT_ID:-}"
if [[ -z "$ROBOT_ID" ]]; then
    echo "  FATAL XBRAIN_ROBOT_ID is unset. The cloud bridge would be skipped" >&2
    echo "        and Qt would see a connected session with zero response." >&2
    echo "        Set it to the rid agreed with the customer." >&2
    exit 1
fi
if ! [[ "$ROBOT_ID" =~ ^[a-z0-9_-]{1,32}$ ]]; then
    echo "  FATAL rid '$ROBOT_ID' violates v2.0 S1.3 [a-z0-9_-]{1,32}" >&2
    exit 1
fi
echo "  rid: $ROBOT_ID"

# Take the rid OUT of the environment for the materialise step below. This is
# the trap run_stack_dev.sh documents from the other side: it never needs the
# unset because its rid comes from a default, but THIS script requires the
# caller to pass one, which puts it in the env from the first line. With it set
# during materialise, freeze resolves per-process references against that rid
# and aborts with 'per_proc_ref_unresolved' on p2_core. The value is kept in
# ROBOT_ID and re-exported after the snapshot exists.
unset XBRAIN_ROBOT_ID
# Same reason, different mechanism: XBRAIN_RESOLVED_DIR is not on the L5
# whitelist (11 S1.1.9.7), so freeze aborts on it outright and wipes the
# snapshot. It is a dev-only rtk_driver override, passed inline at stage 3.
unset XBRAIN_RESOLVED_DIR

# LAN ip: must actually exist on an interface. zenohd fails to bind an address
# the host does not own, and the resulting Errno 99 reads like a zenoh problem.
if [[ -n "${CLOUD_LAN_IP:-}" ]]; then
    LAN_IP="$CLOUD_LAN_IP"
else
    LAN_IP="$(ip -4 -o addr show scope global | awk '{print $4}' | cut -d/ -f1 | head -1)"
fi
if [[ -z "$LAN_IP" ]]; then
    echo "  FATAL no global IPv4 found and CLOUD_LAN_IP unset" >&2
    exit 1
fi
if ! ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1 | grep -qx "$LAN_IP"; then
    echo "  FATAL $LAN_IP is not on any interface of this host" >&2
    ip -4 -o addr show | awk '{print "        have: "$2"  "$4}' >&2
    exit 1
fi
if [[ "$LAN_IP" == "0.0.0.0" ]]; then
    echo "  FATAL 0.0.0.0 is forbidden (NET-C9): the estop-carrying bus would" >&2
    echo "        be reachable from the chassis and PTZ segments too." >&2
    exit 1
fi
echo "  LAN ip: $LAN_IP  (Qt connects to tcp/$LAN_IP:7447)"

# Ports free. A stale router still holding 7447 makes the new one exit and the
# stack then runs with no bus at all.
for p in 7447 7449 "$CHASSIS_PORT"; do
    if ss -ltn 2>/dev/null | grep -q ":$p "; then
        echo "  FATAL port $p already in use; run --stop first" >&2
        exit 1
    fi
done
echo "  ports 7447 / 7449 / $CHASSIS_PORT free"

if [[ -f "$PIDFILE" ]] && awk '{print $2}' "$PIDFILE" | xargs -r -I{} kill -0 {} 2>/dev/null; then
    echo "  FATAL a cloud stack is already running (see $PIDFILE)" >&2
    exit 1
fi
: > "$PIDFILE"

# Resolved snapshot. Same dev counterpart to stage 0c as run_stack_dev.sh: the
# real freeze refuses on the still-null safety params (assertion A, by design),
# so the fixture-backed materialiser builds a complete snapshot instead. A
# failure here is fatal -- every config-reading process would die on a missing
# MANIFEST.json with a much less obvious message.
echo "  refreshing resolved snapshot"
if python3 "$SCRIPT_DIR/materialize_resolved.py" >"$LOG_DIR/materialize.log" 2>&1; then
    echo "  materialize ok -> $RESOLVED_DIR"
else
    echo "  FATAL materialize failed:" >&2
    sed 's/^/    /' "$LOG_DIR/materialize.log" >&2
    rm -f "$PIDFILE"
    exit 1
fi

# Exported only AFTER materialise: with the rid set, freeze resolves per-proc
# refs against it and aborts on p2_core (per_proc_ref_unresolved). Same trap
# run_stack_dev.sh documents.
export XBRAIN_ROBOT_ID="$ROBOT_ID"
export XBRAIN_RECORD_DB="$REPO_ROOT/data/run/record.db"

_start_bg() {
    local name="$1"; shift
    local log="$LOG_DIR/$name.log"
    setsid nohup "$@" >"$log" 2>&1 < /dev/null &
    local pid=$!
    printf '%s %s\n' "$name" "$pid" >> "$PIDFILE"
    printf '  start %-14s pid %-7s log %s\n' "$name" "$pid" "$log"
}

# ---------------------------------------------------------------- stage 1
echo "==> [stage 1] chassis observation point"
_start_bg chassis_stub python3 "$SCRIPT_DIR/chassis_stub.py" \
    --host 127.0.0.1 --port "$CHASSIS_PORT"
sleep 1

# ---------------------------------------------------------------- stage 2
echo "==> [stage 2] routers (RT loopback-only; GEN loopback + LAN)"
_start_bg zenohd-rt  "$ZENOHD" -l tcp/127.0.0.1:7449 --no-multicast-scouting
_start_bg zenohd-gen "$ZENOHD" \
    -l "tcp/127.0.0.1:7447" -l "tcp/$LAN_IP:7447" --no-multicast-scouting
sleep 2

# ---------------------------------------------------------------- stage 3
echo "==> [stage 3] RT plane"
if [[ -x "$RTK_BIN" ]] && [[ -e /dev/ttyACM0 ]]; then
    _start_bg rtk_driver env XBRAIN_RESOLVED_DIR="$RESOLVED_DIR" "$RTK_BIN"
else
    echo "  rtk_driver skipped (binary or /dev/ttyACM0 absent -- GATED-HW)"
fi

# ---------------------------------------------------------------- stage 4
echo "==> [stage 4] p1_motion (cross-plane; owns the CHS-A exit)"
_start_bg p1_motion python3 -m xbrain.p1_motion --voice-loop \
    --resolved-root "$RESOLVED_DIR" \
    --chassis-host 127.0.0.1 --chassis-port "$CHASSIS_PORT"
sleep 1

# ---------------------------------------------------------------- stage 5
echo "==> [stage 5] general plane p2..p5 (p5 last: it carries the cloud bridge)"
for proc in p2_core p3_task p4_agent p5_gateway; do
    _start_bg "$proc" python3 -m "xbrain.$proc" --voice-loop \
        --resolved-root "$RESOLVED_DIR"
    sleep 1
done

# ---------------------------------------------------------------- stage 6
echo "==> [stage 6] AI services"
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

# ---------------------------------------------------------------- stage 7
echo
echo "==> [stage 7] verify (waiting 6s for the bridge to settle)"
sleep 6

rc=0

# The LAN socket is the whole point of this script. Without it Qt cannot connect
# and every downstream symptom is misleading.
if ss -ltn 2>/dev/null | grep -q "$LAN_IP:7447"; then
    echo "  OK   GEN router listening on $LAN_IP:7447 (Qt reachable)"
else
    echo "  FAIL GEN router is NOT listening on $LAN_IP:7447" >&2
    rc=1
fi
if ss -ltn 2>/dev/null | grep -q "127.0.0.1:7447"; then
    echo "  OK   GEN router listening on 127.0.0.1:7447 (on-board participants)"
else
    echo "  FAIL GEN router is NOT on loopback; on-board processes cannot attach" >&2
    rc=1
fi
if ss -ltn 2>/dev/null | grep -qE "^.*$LAN_IP:7449|0.0.0.0:7449"; then
    echo "  FAIL RT router is exposed beyond loopback (RT-C4 violation)" >&2
    rc=1
else
    echo "  OK   RT router loopback-only (RT-C4)"
fi

# The bridge logs its rid and its sub/pub counts. A bridge that silently skipped
# looks identical to a healthy p5 from the outside.
if grep -q "cloud bridge wired: rid=$ROBOT_ID" "$LOG_DIR/p5_gateway.log" 2>/dev/null; then
    echo "  OK   $(grep -o 'cloud bridge wired: rid=.*' "$LOG_DIR/p5_gateway.log" | tail -1)"
else
    echo "  FAIL p5 did not log a cloud bridge for rid=$ROBOT_ID" >&2
    tail -5 "$LOG_DIR/p5_gateway.log" 2>/dev/null | sed 's/^/       /' >&2
    rc=1
fi

if grep -q "entering full mode" "$LOG_DIR/p5_gateway.log" 2>/dev/null; then
    echo "  OK   p5 in FULL mode (minimal mode may publish only 3 keys)"
else
    echo "  WARN p5 not confirmed in full mode -- check $LOG_DIR/p5_gateway.log" >&2
fi

echo
_status
echo
echo "cloud face : xbrain/$ROBOT_ID/**  on  tcp/$LAN_IP:7447"
echo "chassis evidence : $LOG_DIR/chassis_stub.log"
echo "stop : $0 --stop"
exit $rc
