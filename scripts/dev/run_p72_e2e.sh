#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: run_p72_e2e.sh
# Brief: P7.2 end-to-end -- routers + real p1_motion + Zenoh SIL world, one route, one verdict
#
# Description:
# Starts the two zenohd routers (unless already listening), the REAL
# p1_motion (--voice-loop, resolved snapshot from data/run/resolved, so
# scripts/dev/materialize_resolved.py must have run), then runs
# scripts/dev/pose_stub.py in the foreground with --grant (p2's Stage-D
# factor stand-in) and the requested route. The stub's exit code is the
# verdict (0 arrived / 2 failed / 3 timeout); p1's nav-loop heartbeat lines
# (period p99 / max, CLAUDE.md 4.4) are printed at the end. Everything this
# script started is stopped on exit; a router that was already up is left up.
#
# Usage:
#   bash scripts/dev/run_p72_e2e.sh [fwd|rev|none] [max_s] [--goto X,Y]
# Logs: data/run/e2e-logs/{zenohd-rt,zenohd-gen,p1_motion,pose_stub}.log
#
set -uo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"
RESOLVED_DIR="${RESOLVED_DIR:-$REPO_ROOT/data/run/resolved}"
LOG_DIR="${LOG_DIR:-$REPO_ROOT/data/run/e2e-logs}"
ZENOHD="${ZENOHD:-/usr/local/bin/zenohd}"
ROUTE="${1:-fwd}"
MAX_S="${2:-300}"
shift $(( $# > 2 ? 2 : $# ))
export XBRAIN_ROBOT_ID="${XBRAIN_ROBOT_ID:-dev}"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$LOG_DIR"
PIDS=()

_cleanup() {
    for pid in "${PIDS[@]:-}"; do
        [[ -n "$pid" ]] && kill "$pid" 2>/dev/null
    done
    sleep 1
    for pid in "${PIDS[@]:-}"; do
        [[ -n "$pid" ]] && kill -9 "$pid" 2>/dev/null
    done
}
trap _cleanup EXIT

_start() {
    local name="$1"; shift
    setsid nohup "$@" >"$LOG_DIR/$name.log" 2>&1 < /dev/null &
    PIDS+=("$!")
    echo "  start $name pid $!"
}

cd "$REPO_ROOT"
if [[ ! -f "$RESOLVED_DIR/p1_motion.yaml" || ! -f "$RESOLVED_DIR/rns.yaml" ]]; then
    echo "resolved snapshot missing under $RESOLVED_DIR -- run scripts/dev/materialize_resolved.py" >&2
    exit 4
fi
if ! ss -ltn 2>/dev/null | grep -q ':7449 '; then
    _start zenohd-rt "$ZENOHD" -l tcp/127.0.0.1:7449 --no-multicast-scouting
fi
if ! ss -ltn 2>/dev/null | grep -q ':7447 '; then
    _start zenohd-gen "$ZENOHD" -l tcp/127.0.0.1:7447 --no-multicast-scouting
fi
sleep 2
find xbrain -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
_start p1_motion python3 -m xbrain.p1_motion --voice-loop --resolved-root "$RESOLVED_DIR"
sleep 3
if ! grep -q "p1 nav loop wired" "$LOG_DIR/p1_motion.log"; then
    echo "p1 nav loop did not come up; last log lines:" >&2
    tail -20 "$LOG_DIR/p1_motion.log" >&2
    exit 5
fi
echo "==> pose stub (route=$ROUTE max_s=$MAX_S)"
# -u: the stub prints through a pipe (tee); block buffering would hold the trace until exit.
python3 -u scripts/dev/pose_stub.py --rid "$XBRAIN_ROBOT_ID" --resolved "$RESOLVED_DIR" \
    --grant --route "$ROUTE" --max-s "$MAX_S" "$@" 2>&1 | tee "$LOG_DIR/pose_stub.log"
RC=${PIPESTATUS[0]}
echo "==> p1 nav loop heartbeat (last 3):"
grep "p1 nav loop:" "$LOG_DIR/p1_motion.log" | tail -3
grep -c "p1 nav tick failed" "$LOG_DIR/p1_motion.log" | sed 's/^/  tick errors: /'
exit "$RC"
