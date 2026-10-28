#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: start_basic_motion.sh
# Brief: Bring up the delivery processes for the basic-motion test (plan B) and open the console
#
# Description:
# User ruling 2026-10-26 (plan B, "ignore the freeze line for now"):
#   * systemd (formal units): run-xbrain.mount, xbrain-zenohd-rt, xbrain-zenohd-gen,
#     xbrain-payload, xbrain-ai-asr, xbrain-llm. None of them Requires the freeze line.
#   * installed binaries / modules, same commands as their units, but reading the
#     existing snapshot data/run/resolved instead of a fresh freeze (their units
#     Require xbrain-config-freeze, which refuses 42 null keys by design):
#     quadruped, rtk_driver, chassis_relay, p2_core, p3_task, p4_agent, p5_gateway.
#   * perception (RT plane, 10 S3.3 Stage 1) as a transient systemd unit in
#     perception.slice (cores 1,2,3,7), as nvidia with --init-groups (the video
#     group opens GPU/DLA) and a clean env with the ROS humble runtime -- the
#     perception team's own launch recipe (19 S1.2, deploy/.../10-perception-r6.conf).
#   * p1_motion after chassis_relay (10 S3.3 GATE-4). Commands come from commx.
#
# Usage:
#   scripts/start_basic_motion.sh          start the stack
#   scripts/start_basic_motion.sh --stop   stop what it started
#   scripts/start_basic_motion.sh --no-perception
#                                          start everything except perception
#                                          (2026-10-27: perception is being tested
#                                          on its own and must not be started)
# Run on the robot as nvidia; it asks for sudo for the systemd steps.
#
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO="$( cd "$SCRIPT_DIR/.." && pwd )"
RESOLVED="$REPO/data/run/resolved"
PIDFILE="$REPO/data/run/basic_motion.pids"
SYSTEMD_UNITS=(xbrain-zenohd-rt xbrain-zenohd-gen xbrain-payload xbrain-ai-asr xbrain-llm)
PERCEPTION_UNIT=xbrain-perception-dev
PERCEPTION_BIN="$REPO/data/install/perception/lib/perception/perception_node"

start_perception() {
    # Clean environment on purpose: perception refuses any inherited PERCEPTION_* /
    # RNS_* variable (exit 78), and this script exports XBRAIN_BASIC_MOTION_*.
    sudo systemctl stop "$PERCEPTION_UNIT" 2>/dev/null || true
    sudo systemctl reset-failed "$PERCEPTION_UNIT" 2>/dev/null || true
    # One camera, one perception: a perception_node still running after OUR unit
    # stopped belongs to someone else (2026-10-27: the perception team's golive
    # harness, scope perception-manual). A second instance would fight it for
    # the 338Le stream, so leave it running and do not start ours.
    local other
    other=$(pgrep -ax perception_node | head -1 || true)
    if [[ -n "$other" ]]; then
        echo "  perception already running, not started again: $other"
        echo "    (cgroup $(cut -d: -f3 "/proc/${other%% *}/cgroup" 2>/dev/null))"
        return 0
    fi
    sudo systemd-run --quiet --unit="$PERCEPTION_UNIT" --slice=perception.slice \
        -p CPUAffinity="1 2 3 7" -p OOMScoreAdjust=-900 \
        --setenv=HOME=/home/nvidia --setenv=PATH=/usr/local/bin:/usr/bin:/bin \
        --setenv=LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib \
        --setenv=AMENT_PREFIX_PATH=/opt/ros/humble --setenv=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        --setenv=ROS_DOMAIN_ID=42 \
        /usr/bin/setpriv --reuid=nvidia --regid=nvidia --init-groups "$PERCEPTION_BIN"
}

stop_ours() {
    if [[ -f "$PIDFILE" ]]; then
        while read -r pid name; do
            kill "$pid" 2>/dev/null && echo "  stopped $name ($pid)" || true
        done < "$PIDFILE"
        rm -f "$PIDFILE"
    fi
}

stop_legacy() {
    # Whatever the old dev launcher left running (exact patterns, no globs on pids).
    local pats=(
        "scripts/dev/chassis_stub.py" "zenohd -l tcp/127.0.0.1:7449" "zenohd -l tcp/127.0.0.1:7447"
        "python3 -m xbrain.p1_motion" "python3 -m xbrain.p2_core" "python3 -m xbrain.p3_task"
        "python3 -m xbrain.p4_agent" "python3 -m xbrain.p5_gateway"
        "python3 -m services.asr.app" "python3 -m services.payload.app" "services/llm/bin/llama-server"
        "ros2_ws/sensor/build/rtk_driver" "data/install/rtk_driver/lib/rtk_driver/rtk_driver"
        "chassis_relay/build/chassis_relay" "data/install/chassis_relay/lib/chassis_relay/chassis_relay"
    )
    local p
    for p in "${pats[@]}"; do pkill -f -- "$p" 2>/dev/null && echo "  stopped: $p" || true; done
    pkill -x quadruped_m20 2>/dev/null && echo "  stopped: quadruped_m20" || true
    sleep 3
}

WITH_PERCEPTION=1
if [[ "${1:-}" == "--no-perception" ]]; then
    WITH_PERCEPTION=0
fi
if [[ "${1:-}" == "--stop" ]]; then
    echo "==> stopping basic-motion stack"
    stop_ours
    sudo systemctl stop "$PERCEPTION_UNIT" 2>/dev/null || true
    sudo systemctl stop "${SYSTEMD_UNITS[@]}" || true
    exit 0
fi

[[ -f "$RESOLVED/MANIFEST.json" ]] || { echo "no snapshot at $RESOLVED" >&2; exit 1; }
set -a; . "$REPO/deploy/env/robot.env"; set +a
[[ -n "${XBRAIN_ROBOT_ID:-}" ]] || { echo "XBRAIN_ROBOT_ID unset in deploy/env/robot.env" >&2; exit 1; }

STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="$REPO/data/run/basic_motion/$STAMP"
mkdir -p "$LOGDIR"
echo "logs: $LOGDIR"

echo "==> [0] stop previous stack"
stop_ours
stop_legacy

echo "==> [0b] config snapshot for this boot (freeze line ignored; sim-variant materialise)"
# The loaders refuse a snapshot whose MANIFEST.boot_id is from another boot, so it
# is regenerated on every start. It is built WITH XBRAIN_ROBOT_ID (gj-001 from
# robot.env): perception takes its rid only from MANIFEST.robot_id, so a "dev"
# snapshot would put its keys under xbrain/dev/ where p1 never looks. This needs
# configs/calib/gj-001.yaml (h_camera_m, measured 2026-10-26); without it the
# freeze aborts with per_proc_ref_unresolved on p2_core.
# A failed materialise empties the snapshot directory, so the previous one is
# kept aside and put back on failure.
rm -rf "$REPO/data/run/resolved.prev" && cp -a "$RESOLVED" "$REPO/data/run/resolved.prev"
if ( unset XBRAIN_RESOLVED_DIR; python3 "$REPO/scripts/dev/materialize_resolved.py" ) > "$LOGDIR/materialize.log" 2>&1; then
    echo "  snapshot ok -> $RESOLVED"
else
    echo "  snapshot FAILED:"; cat "$LOGDIR/materialize.log"
    rm -rf "$RESOLVED" && cp -a "$REPO/data/run/resolved.prev" "$RESOLVED" && echo "  previous snapshot restored"
    exit 1
fi

echo "==> [1] install systemd units (sudo)"
sudo "$REPO/scripts/install_units.sh" > "$LOGDIR/install_units.log" 2>&1 || { cat "$LOGDIR/install_units.log"; exit 1; }

echo "==> [2] systemd: zenoh routers + AI services"
sudo systemctl restart xbrain-zenohd-rt xbrain-zenohd-gen
sleep 2
for u in xbrain-zenohd-rt xbrain-zenohd-gen; do
    systemctl is-active --quiet "$u" || { echo "  $u NOT active"; sudo journalctl -u "$u" -n 20 --no-pager; exit 1; }
    echo "  $u active"
done
sudo systemctl restart xbrain-payload xbrain-ai-asr xbrain-llm || true

: > "$PIDFILE"
start_bg() {  # name, then the command
    local name="$1"; shift
    setsid nohup "$@" > "$LOGDIR/$name.log" 2>&1 < /dev/null &
    echo "$! $name" >> "$PIDFILE"
    echo "  started $name ($!)"
}

# TEMPORARY basic-motion test (user ruling 2026-10-26): every gate that would
# block the six basic motions is off (p2 G-4/5/6, p1 RNS + speed gates; relmove
# runs closed-loop on chassis odom). Soft estop still stops. Remove after the test.
export XBRAIN_BASIC_MOTION_TEST=1
# Test speeds (change here): forward/back m/s, strafe m/s, turn rad/s.
export XBRAIN_BASIC_MOTION_LIN=0.2
export XBRAIN_BASIC_MOTION_LAT=0.15
export XBRAIN_BASIC_MOTION_ANG=0.3
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export PYTHONUNBUFFERED=1
INST="$REPO/data/install"

echo "==> [3] RT plane: quadruped, perception, rtk_driver"
start_bg quadruped bash -c "set +u; source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; exec $INST/quadruped/lib/quadruped/quadruped_m20"
if [[ "$WITH_PERCEPTION" == 0 ]]; then
    echo "  perception NOT started (--no-perception)"
elif [[ -x "$PERCEPTION_BIN" ]]; then
    start_perception && systemctl is-active --quiet "$PERCEPTION_UNIT" \
        && echo "  started perception ($PERCEPTION_UNIT, journalctl -u $PERCEPTION_UNIT)"
else
    echo "  perception skipped (no binary at $PERCEPTION_BIN)"
fi
if [[ -e /dev/ttyACM0 ]]; then
    start_bg rtk_driver env XBRAIN_RESOLVED_DIR="$RESOLVED" "$INST/rtk_driver/lib/rtk_driver/rtk_driver"
fi
sleep 4

echo "==> [4] cross plane: chassis_relay"
start_bg chassis_relay "$INST/chassis_relay/lib/chassis_relay/chassis_relay"
sleep 2

echo "==> [5] p1_motion (after chassis_relay, GATE-4)"
cd "$REPO"
start_bg p1_motion python3 -m xbrain.p1_motion --voice-loop --resolved-root "$RESOLVED"
sleep 2

echo "==> [6] general plane: p2 p3 p4 p5"
for proc in p2_core p3_task p4_agent p5_gateway; do
    if [[ "$proc" == "p5_gateway" ]]; then
        start_bg "$proc" env XBRAIN_RECORD_DB="$REPO/data/run/record.db" python3 -m "xbrain.$proc" --voice-loop --resolved-root "$RESOLVED"
    else
        start_bg "$proc" python3 -m "xbrain.$proc" --voice-loop --resolved-root "$RESOLVED"
    fi
    sleep 1
done
sleep 6

echo "==> [7] check"
grep -a "chassis link" "$LOGDIR/quadruped.log" | tail -1 || echo "  quadruped: no chassis link line yet"
grep -a "forwarding up" "$LOGDIR/chassis_relay.log" | tail -1 || echo "  chassis_relay: not forwarding"
while read -r pid name; do
    kill -0 "$pid" 2>/dev/null && echo "  alive  $name" || echo "  DEAD   $name (see $LOGDIR/$name.log)"
done < "$PIDFILE"
for u in "${SYSTEMD_UNITS[@]}" "$PERCEPTION_UNIT"; do echo "  $(systemctl is-active "$u")  $u"; done
# Broadcast / alarm mode framework (99 U97 + U99 C1-C3), printed unconditionally
# -- a check that only speaks on failure cannot show it ran:
#   * payload device mode: p2 moves payload-service out of its boot `idle` into
#     func1 within one 5 s reconcile; `idle` refuses /play (the 2026-10-27 403s).
#   * p2's startup cmd/motion/behavior (resume_task in idle) clears any hold a
#     previous p2 left in p1.
#   * p2 mic counters: published must grow, not muted only -- a speaker hold that
#     is never released leaves the mic muted for good (2026-10-28, C1 lease).
echo "  payload device mode: $(curl -s -m 2 http://127.0.0.1:18080/status | python3 -c 'import json,sys; print(json.load(sys.stdin).get("mode"))' 2>/dev/null || echo unreachable) (expect func1)"
echo "  p2 behavior: $(grep -a 'cmd/motion/behavior ->' "$LOGDIR/p2_core.log" | tail -1 | sed 's/.*INFO //' || true)"
echo "  p2 mic: $(grep -a 'p2 alive' "$LOGDIR/p2_core.log" | tail -1 | grep -oE 'published=[0-9]+ muted=[0-9]+' || echo 'no alive line yet')"

echo "stack up; logs in $LOGDIR; stop with $0 --stop"
