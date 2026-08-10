#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: start_voice_loop.sh
# Brief: Launch the full voice-loop stack on ORIN (zenohd x2 + AI x3 + p1..p5 x5 + chassis_stub)
#
# Description:
# Starts the whole voice-loop test stack in the background. Each process
# has its own stdout+stderr log under /tmp/xbrain_v6/voice_loop_logs/.
# A tag file /tmp/xbrain_v6/voice_loop.pids records the pgids so
# stop_voice_loop.sh can clean up.
#
# Startup order (no waits, just ordering):
#   1. zenohd-gen (tcp/7447)
#   2. zenohd-rt  (tcp/lo:7449)
#   3. services/asr (127.0.0.1:18081)
#   4. services/llm (127.0.0.1:18082)
#   5. services/payload (127.0.0.1:18080) -- forwards to GZH-2
#   6. chassis_stub (0.0.0.0:30004) -- CHS-A frame receiver
#   7. p2_core --voice-loop  (owns MIC + speaker + gate)
#   8. p3_task --voice-loop  (task queue observer)
#   9. p5_gateway --voice-loop (state/link + speak/ack observer)
#   10. p4_agent --voice-loop (turn loop: MIC->ASR->intent->cmd/*)
#   11. p1_motion --voice-loop (cmd/motion/intent -> chassis_stub)
#
# What this DOES NOT do:
#   * config-freeze -- if /run/xbrain/resolved is missing p2/p3/p4/p1
#     exit with code 3; run `python -m xbrain.boot.freeze` first
#   * NTP sync
#   * device permissions -- USB MIC needs the audio group; GZH-2 IP
#     must be reachable
#
# Stop with scripts/dev/stop_voice_loop.sh

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." && pwd )"

LOG_DIR="/tmp/xbrain_v6/voice_loop_logs"
PID_FILE="/tmp/xbrain_v6/voice_loop.pids"

mkdir -p "${LOG_DIR}"
: > "${PID_FILE}"

# Config: allow override via env, default to on-ORIN topology.
ASR_URL="${ASR_URL:-http://127.0.0.1:18081}"
LLM_URL="${LLM_URL:-http://127.0.0.1:18082}"
PAYLOAD_URL="${PAYLOAD_URL:-http://127.0.0.1:18080}"
CHASSIS_HOST="${CHASSIS_HOST:-127.0.0.1}"
CHASSIS_PORT="${CHASSIS_PORT:-30004}"
ARECORD_DEVICE="${ARECORD_DEVICE:-hw:0,0}"

ZENOHD_BIN="${ZENOHD_BIN:-zenohd}"
ZENOHD_GEN_CONFIG="${ZENOHD_GEN_CONFIG:-${REPO_ROOT}/configs/zenoh/router_gen.json5}"
ZENOHD_RT_CONFIG="${ZENOHD_RT_CONFIG:-${REPO_ROOT}/configs/zenoh/router_rt.json5}"

# Dev-only resolved-config root. Not /run/xbrain/resolved (systemd
# freeze writes there and needs root). materialize_resolved.py
# populates this from the CHK-0-56 fixture pattern.
RESOLVED_ROOT="${XBRAIN_RESOLVED_ROOT:-/tmp/xbrain_v6/resolved}"

_spawn() {
    local name="$1"; shift
    local logfile="${LOG_DIR}/${name}.log"
    echo "[start_voice_loop] launching ${name} -> ${logfile}"
    "$@" >"${logfile}" 2>&1 &
    local pid=$!
    echo "${pid} ${name}" >> "${PID_FILE}"
    # Small stagger so the router is up before clients start.
    sleep 0.15
}

# 1-2. Zenoh routers -- REQUIRED. Fail-loud on missing binary OR config.
# Without either, downstream P-processes cannot connect and will crash
# 30 s later with 'connect timeout' messages that hide the real cause.
if ! command -v "${ZENOHD_BIN}" >/dev/null 2>&1; then
    echo "[start_voice_loop] ERROR ${ZENOHD_BIN} not on PATH; aborting"
    exit 2
fi
if [[ ! -f "${ZENOHD_GEN_CONFIG}" ]]; then
    echo "[start_voice_loop] ERROR zenoh-gen config missing: ${ZENOHD_GEN_CONFIG}"
    exit 2
fi
if [[ ! -f "${ZENOHD_RT_CONFIG}" ]]; then
    echo "[start_voice_loop] ERROR zenoh-rt config missing: ${ZENOHD_RT_CONFIG}"
    exit 2
fi
_spawn zenohd-gen "${ZENOHD_BIN}" --config "${ZENOHD_GEN_CONFIG}"
_spawn zenohd-rt "${ZENOHD_BIN}" --config "${ZENOHD_RT_CONFIG}"
# Give routers 0.5 s to bind their listen ports before clients connect.
sleep 0.5

# 2.5. Materialise resolved config products so P-processes can start
# without config-freeze systemd unit + root privileges.
echo "[start_voice_loop] materialising resolved products at ${RESOLVED_ROOT}"
if ! python3 "${REPO_ROOT}/scripts/dev/materialize_resolved.py" \
        --root "${RESOLVED_ROOT}" > "${LOG_DIR}/materialize_resolved.log" 2>&1; then
    echo "[start_voice_loop] ERROR materialize_resolved failed; see ${LOG_DIR}/materialize_resolved.log"
    cat "${LOG_DIR}/materialize_resolved.log" | tail -20
    exit 2
fi

# 3-5. AI services (background). Only start if not already listening.
_port_free() {
    ! ss -ln 2>/dev/null | grep -q ":$1 " || return 1
}

if _port_free 18081; then
    _spawn asr-service bash "${REPO_ROOT}/services/asr/asr_server.sh"
else
    echo "[start_voice_loop] asr already on :18081, skipping"
fi
if _port_free 18082; then
    _spawn llm-service bash "${REPO_ROOT}/services/llm/llm_server.sh"
else
    echo "[start_voice_loop] llm already on :18082, skipping"
fi
if _port_free 18080; then
    # payload/app.py uses relative imports (from .api.rest ...) so it
    # must be invoked as a module from the repo root, not as a script.
    _spawn payload-service bash -c \
        "cd '${REPO_ROOT}' && python3 -m services.payload.app"
else
    echo "[start_voice_loop] payload already on :18080, skipping"
fi

# 6. chassis_stub (offline-chassis fallback receiver).
_spawn chassis_stub python3 "${REPO_ROOT}/scripts/dev/chassis_stub.py" \
    --port "${CHASSIS_PORT}" --host 0.0.0.0

# 7-11. P-processes in dependency order.
# p2_core owns MIC; must be up before p4 subscribes rt/audio/mic.
_spawn p2_core python3 -m xbrain.p2_core --voice-loop \
    --resolved-root "${RESOLVED_ROOT}" \
    --payload-base-url "${PAYLOAD_URL}" \
    --arecord-device "${ARECORD_DEVICE}"

# p3/p5 pure observers; start before p4 so we don't miss early frames.
_spawn p3_task python3 -m xbrain.p3_task --voice-loop \
    --resolved-root "${RESOLVED_ROOT}"
_spawn p5_gateway python3 -m xbrain.p5_gateway --voice-loop \
    --resolved-root "${RESOLVED_ROOT}"

_spawn p4_agent python3 -m xbrain.p4_agent --voice-loop \
    --resolved-root "${RESOLVED_ROOT}" \
    --asr-base-url "${ASR_URL}"

_spawn p1_motion python3 -m xbrain.p1_motion --voice-loop \
    --resolved-root "${RESOLVED_ROOT}" \
    --chassis-host "${CHASSIS_HOST}" \
    --chassis-port "${CHASSIS_PORT}"

echo ""
echo "[start_voice_loop] all processes launched. Log dir: ${LOG_DIR}"
echo "[start_voice_loop] PID file: ${PID_FILE}"
echo "[start_voice_loop] Tail one:  tail -f ${LOG_DIR}/p4_agent.log"
echo "[start_voice_loop] Stop all:  bash ${SCRIPT_DIR}/stop_voice_loop.sh"
