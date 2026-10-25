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
#   * p1_motion after chassis_relay (10 S3.3 GATE-4). Commands come from commx.
#
# Usage:
#   scripts/start_basic_motion.sh          start the stack
#   scripts/start_basic_motion.sh --stop   stop what it started
# Run on the robot as nvidia; it asks for sudo for the systemd steps.
#
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO="$( cd "$SCRIPT_DIR/.." && pwd )"
RESOLVED="$REPO/data/run/resolved"
PIDFILE="$REPO/data/run/basic_motion.pids"
SYSTEMD_UNITS=(xbrain-zenohd-rt xbrain-zenohd-gen xbrain-payload xbrain-ai-asr xbrain-llm)

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

if [[ "${1:-}" == "--stop" ]]; then
    echo "==> stopping basic-motion stack"
    stop_ours
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
export XBRAIN_BASIC_MOTION_ANG=0.2
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export PYTHONUNBUFFERED=1
INST="$REPO/data/install"

echo "==> [3] RT plane: quadruped, rtk_driver"
start_bg quadruped bash -c "set +u; source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; exec $INST/quadruped/lib/quadruped/quadruped_m20"
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
for u in "${SYSTEMD_UNITS[@]}"; do echo "  $(systemctl is-active "$u")  $u"; done

echo "stack up; logs in $LOGDIR; stop with $0 --stop"
