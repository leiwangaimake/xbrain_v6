#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: stop_voice_loop.sh
# Brief: Stop the voice-loop stack started by start_voice_loop.sh
#
# Description:
# Reads /opt/xbrain_v6/data/run/voice_loop.pids and sends SIGTERM to
# each recorded pid. Waits up to 5 s for graceful shutdown; escalates
# to SIGKILL for anything still alive. Preserves the log dir so
# post-mortem is possible. Path defaults follow the V6 rule 'all
# runtime files under /opt/xbrain_v6/' -- override with
# XBRAIN_PID_FILE / XBRAIN_LOG_DIR only for isolated dev sandboxes.

set -euo pipefail

PID_FILE="${XBRAIN_PID_FILE:-/opt/xbrain_v6/data/run/voice_loop.pids}"
LOG_DIR="${XBRAIN_LOG_DIR:-/opt/xbrain_v6/logs/voice_loop}"

if [[ ! -f "${PID_FILE}" ]]; then
    echo "[stop_voice_loop] no pid file at ${PID_FILE}; nothing to do"
    exit 0
fi

echo "[stop_voice_loop] sending SIGTERM to recorded pids"
declare -a PIDS=()
declare -a NAMES=()
while read -r pid name; do
    [[ -z "${pid}" ]] && continue
    if kill -0 "${pid}" 2>/dev/null; then
        PIDS+=("${pid}")
        NAMES+=("${name}")
        kill -TERM "${pid}" 2>/dev/null || true
        echo "  sent SIGTERM to ${name} (pid ${pid})"
    else
        echo "  ${name} (pid ${pid}) already gone"
    fi
done < "${PID_FILE}"

# Wait up to 5 s.
for _ in $(seq 1 50); do
    all_gone=1
    for pid in "${PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            all_gone=0
            break
        fi
    done
    if [[ "${all_gone}" == "1" ]]; then
        break
    fi
    sleep 0.1
done

# SIGKILL survivors.
survivors=0
for i in "${!PIDS[@]}"; do
    pid="${PIDS[$i]}"
    if kill -0 "${pid}" 2>/dev/null; then
        kill -KILL "${pid}" 2>/dev/null || true
        echo "[stop_voice_loop] SIGKILL ${NAMES[$i]} (pid ${pid})"
        survivors=$((survivors + 1))
    fi
done

rm -f "${PID_FILE}"
echo "[stop_voice_loop] done. Logs preserved at ${LOG_DIR}"
[[ "${survivors}" -gt 0 ]] && echo "[stop_voice_loop] WARN ${survivors} process(es) required SIGKILL"
exit 0
