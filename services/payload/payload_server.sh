#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: payload_server.sh
# Brief: Launch payload-service, the sole owner of the GZH-2 device sockets
#
# Description:
# Starts services/payload as an HTTP endpoint on 127.0.0.1:18080. It is the ONLY
# process permitted to hold the GZH-2 三合一载荷's two TCP sockets -- 8519 audio and
# 8529 lights -- so speech, siren, lighting and the microphone downlink all pass
# through it. Sibling of llm_server.sh and asr_server.sh, deliberately the same
# shape: one wrapper that pins the deploy, validates it, and hands off.
#
# ★ Why single ownership matters (11 PAY-L2): the device accepts one TCP connection
# per port. A second process opening 8519 does not get an error, it gets the stream,
# and whatever the first process was in the middle of sending is lost in an RST. So
# the socket owner is a designed singleton, not an accident of who started first --
# and that is why TTS reaches the device through this service rather than by any
# caller connecting directly.
#
# ★ TTS is DEVICE-INTERNAL. This service sends text on the [31] frame and the GZH-2
# synthesizes and speaks it; there is no TTS model, no GPU use and no audio buffer on
# this box (99 U52). The consequence that shapes the whole voice loop: the device
# emits no "finished speaking" event, so the half-duplex gate that keeps the robot
# from transcribing its own voice can only run on an ESTIMATE, which POST /tts
# returns as est_ms.
#
# WHY THE TUNABLES LIVE HERE AND NOT IN A YAML: config.py already makes from_env()
# the single boundary between the environment and its typed config, and every field
# is overridable, so exports plus exec are the whole configuration story. A yaml
# would add a second source of truth and a parser to express what an export already
# expresses. If the three-layer configs/ scheme of 10 §5.4 later becomes the
# authority, this script is where it would be read and turned into these same
# exports; nothing downstream changes.
#
# Failure philosophy, matching its siblings: preconditions that can be checked are
# checked up front with actionable messages. Device REACHABILITY is deliberately not
# one of them -- see the note above the launch line.
#
# Shell strictness: fail fast on the first error, on any unset variable, and on a
# failure anywhere in a pipeline.
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." && pwd )"

PYTHON="${PAYLOAD_PYTHON:-python3}"

# -- Runtime knobs, all environment-overridable ------------------------------------
#
# Names match services/payload/config.py's env keys exactly; that file is the
# authority on what each means. Repeated here so `cat payload_server.sh` answers
# "what is this box running" without reading Python.

# The GZH-2 module and its two ports. The Orin reaches it over enP8p1s0
# (192.168.144.100/24); measured LAN RTT is about 1 ms.
export DEVICE_HOST="${DEVICE_HOST:-192.168.144.38}"
export PORT_AUDIO="${PORT_AUDIO:-8519}"    # speech, siren, microphone uplink
export PORT_LIGHTS="${PORT_LIGHTS:-8529}"  # searchlight, brightness, red/blue

# Listen port. 99 U52 allocation: HMI 8080, payload 18080, asr 18081, llama 18082.
# The bind address is pinned to 127.0.0.1 in app.py as a module constant (PAY-L5),
# so it is not settable here: the general plane carries no authentication, and a
# LAN-reachable /lights would let any host in the compound run the siren.
export PORT_PAYLOAD="${PORT_PAYLOAD:-18080}"

# ★ TTS playback-time estimate: est_ms = max(base, chars x per_char) + tail.
# CALIBRATED 2026-08-03 against the real device by recording on the Orin's USB
# microphone while firing POST /tts -- speech runs 232-240 ms/char and the device
# stays silent for 357-557 ms after the request while it synthesizes. The planned
# 800/180/500 covered only 80-83% of the window the gate actually needs, so on a
# long sentence the microphone reopened while the robot was still speaking. 250 and
# 900 cover every measured case with 13-25% to spare.
# ⚠️ 16 §3.0.4.1 permits adjusting these UPWARD only until calibrated: estimating
# long costs response time, estimating short costs the half-duplex property itself.
export TTS_EST_BASE_MS="${TTS_EST_BASE_MS:-800}"
export PER_CHAR_MS="${PER_CHAR_MS:-250}"
export TAIL_MS="${TAIL_MS:-900}"

# Device socket behaviour.
export SOCKET_TIMEOUT_S="${SOCKET_TIMEOUT_S:-5}"
export CLOSE_FLUSH_MS="${CLOSE_FLUSH_MS:-200}"

# -- Preconditions -----------------------------------------------------------------

if ! command -v "${PYTHON}" >/dev/null 2>&1; then
  echo "[payload_server] ERROR: python interpreter not found: ${PYTHON}" >&2
  exit 1
fi

# opuslib is needed for the microphone uplink codec; its absence would otherwise
# surface only when a /mic session is opened, long after startup looked fine.
if ! "${PYTHON}" -c "import opuslib" >/dev/null 2>&1; then
  echo "[payload_server] ERROR: opuslib is not importable by ${PYTHON}" >&2
  echo "[payload_server]        install it: ${PYTHON} -m pip install --user opuslib" >&2
  echo "[payload_server]        (it also needs the system library: apt install libopus0)" >&2
  exit 1
fi

echo "[payload_server] starting payload-service: device=${DEVICE_HOST}" \
     "audio=${PORT_AUDIO} lights=${PORT_LIGHTS} listen=127.0.0.1:${PORT_PAYLOAD}" \
     "tts_est=max(${TTS_EST_BASE_MS},n*${PER_CHAR_MS})+${TAIL_MS}"

# ⚠️ Device reachability is NOT checked here, and that is deliberate. The service is
# designed to come up with the device absent and reconnect -- GET /status reports
# audio_connected / lights_connected so an operator and the health rollup can see the
# link state. Refusing to start on an unreachable device would make a momentary cable
# or power-up ordering problem into a service that has to be manually restarted, and
# would also make it impossible to bring the stack up before the payload is powered.
cd "${REPO_ROOT}"
export PYTHONPATH="${REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
exec "${PYTHON}" -m services.payload.app
