#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: run_sil_e2e.sh
# Brief: browser SIL in E2E mode -- routers + real p1_motion + sil_server --mode e2e, left running
#
# Description:
# Brings up the two zenohd routers (unless already listening), the REAL
# p1_motion (--voice-loop, resolved snapshot from data/run/resolved; run
# scripts/dev/materialize_resolved.py first), waits for its nav loop, then
# starts scripts/sil/sil_server.py --mode e2e on :8890 and leaves everything
# running for the browser. In this mode the web page's "执行导航测试" /
# click-goto send cmd/motion/route / cmd/motion/relative_move to p1 and the
# robot moves on p1's rt/motion/cmd_vel -- the same chain a real robot uses,
# only the three peripherals are simulated (zenoh_world.py).
#
# The rns-mode bench (plain `python3 scripts/sil/sil_server.py`) uses the same
# port; this script refuses to start while something else holds :8890 unless
# --takeover is given (then it stops that listener first).
#
# Usage:
#   bash scripts/dev/run_sil_e2e.sh [--takeover]     start
#   bash scripts/dev/run_sil_e2e.sh --stop            stop what this script started
#   bash scripts/dev/run_sil_e2e.sh --status
# Logs: data/run/sil-e2e-logs/{zenohd-rt,zenohd-gen,p1_motion,sil_server}.log
#
set -uo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"
RESOLVED_DIR="${RESOLVED_DIR:-$REPO_ROOT/data/run/resolved}"
LOG_DIR="${LOG_DIR:-$REPO_ROOT/data/run/sil-e2e-logs}"
PIDFILE="$LOG_DIR/sil-e2e.pids"
ZENOHD="${ZENOHD:-/usr/local/bin/zenohd}"
PORT="${SIL_PORT:-8890}"
export XBRAIN_ROBOT_ID="${XBRAIN_ROBOT_ID:-dev}"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$LOG_DIR"

_stop_all() {
    if [[ ! -f "$PIDFILE" ]]; then
        echo "no pidfile ($PIDFILE); nothing this script tracked is running"
        return 0
    fi
    tac "$PIDFILE" | while read -r name pid; do
        if kill -0 "$pid" 2>/dev/null; then
            printf '  stop %-12s pid %s\n' "$name" "$pid"
            kill "$pid" 2>/dev/null
        fi
    done
    sleep 1
    rm -f "$PIDFILE"
    echo "run_sil_e2e: stopped"
}
_status() {
    printf '%-12s %-6s %s\n' NAME STATE PID
    if [[ -f "$PIDFILE" ]]; then
        while read -r name pid; do
            if kill -0 "$pid" 2>/dev/null; then st="alive"; else st="dead"; fi
            printf '%-12s %-6s %s\n' "$name" "$st" "$pid"
        done < "$PIDFILE"
    fi
    ss -ltn 2>/dev/null | grep -E ":(7449|7447|$PORT) " || echo "  (no expected port listening)"
}
case "${1:-}" in
    --stop)   _stop_all; exit 0 ;;
    --status) _status;   exit 0 ;;
esac
TAKEOVER=0
[[ "${1:-}" == "--takeover" ]] && TAKEOVER=1

_start() {
    local name="$1"; shift
    setsid nohup "$@" >"$LOG_DIR/$name.log" 2>&1 < /dev/null &
    printf '%s %s\n' "$name" "$!" >> "$PIDFILE"
    echo "  start $name pid $!"
}

cd "$REPO_ROOT"
if [[ -f "$PIDFILE" ]] && awk '{print $2}' "$PIDFILE" | xargs -r -I{} kill -0 {} 2>/dev/null; then
    echo "an e2e SIL stack is already running (see $PIDFILE); run --stop first" >&2
    exit 1
fi
if [[ ! -f "$RESOLVED_DIR/p1_motion.yaml" || ! -f "$RESOLVED_DIR/rns.yaml" ]]; then
    echo "resolved snapshot missing under $RESOLVED_DIR -- run scripts/dev/materialize_resolved.py" >&2
    exit 4
fi
if ss -ltnp 2>/dev/null | grep -q ":$PORT "; then
    if [[ "$TAKEOVER" == "1" ]]; then
        pid=$(ss -ltnp 2>/dev/null | grep ":$PORT " | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | head -1)
        echo "  --takeover: stopping the listener on :$PORT (pid $pid)"
        [[ -n "$pid" ]] && kill "$pid" 2>/dev/null
        sleep 1
    else
        echo ":$PORT is busy (probably the rns-mode bench). Stop it or pass --takeover" >&2
        exit 6
    fi
fi
: > "$PIDFILE"
if ! ss -ltn 2>/dev/null | grep -q ':7449 '; then
    _start zenohd-rt "$ZENOHD" -l tcp/127.0.0.1:7449 --no-multicast-scouting
fi
if ! ss -ltn 2>/dev/null | grep -q ':7447 '; then
    _start zenohd-gen "$ZENOHD" -l tcp/127.0.0.1:7447 --no-multicast-scouting
fi
sleep 2
find xbrain -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
_start p1_motion python3 -m xbrain.p1_motion --voice-loop --resolved-root "$RESOLVED_DIR"
for _i in $(seq 1 30); do
    grep -q "p1 nav loop wired" "$LOG_DIR/p1_motion.log" 2>/dev/null && break
    sleep 0.5
done
if ! grep -q "p1 nav loop wired" "$LOG_DIR/p1_motion.log"; then
    echo "p1 nav loop did not come up; last log lines:" >&2
    tail -20 "$LOG_DIR/p1_motion.log" >&2
    _stop_all
    exit 5
fi
_start sil_server python3 -u scripts/sil/sil_server.py --mode e2e --rid "$XBRAIN_ROBOT_ID" \
    --resolved "$RESOLVED_DIR" --port "$PORT"
sleep 2
_status
echo
echo "browser: http://$(hostname -I 2>/dev/null | awk '{print $1}'):$PORT   (E2E: the real p1 drives)"
echo "stop:    bash $0 --stop"
