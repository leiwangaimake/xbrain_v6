#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: asr_server.sh
# Brief: Launch asr-service, the CPU speech-recognition endpoint of the AI Runtime
#
# Description:
# Starts services/asr as an OpenAI-style HTTP endpoint
# (POST /v1/audio/transcriptions) on 127.0.0.1:18081, which the AI_runtime
# orchestrator calls once per dialogue turn. Sibling of services/llm/llm_server.sh
# and deliberately the same shape: one wrapper that pins the deploy, validates it,
# and hands off, so systemd, a developer at a terminal and a test harness all start
# the service identically without knowing its layout or argument list.
#
# WHY THE TUNABLES LIVE HERE AND NOT IN A YAML: services/asr/config.py already makes
# from_env() the single boundary between the environment and its typed config, and
# every field is overridable. So a launcher that exports variables and execs is the
# whole configuration story -- adding a yaml would introduce a second source of
# truth and a parser, to express exactly what an export already expresses. If the
# three-layer configs/ scheme of 10 §5.4 later becomes the authority, this script is
# the natural place to read it and turn it into the same exports; nothing downstream
# changes.
#
# ★ Resource shape (relevant to 10 §7.3 and the 11 §11A.2 memory ledger): this
# service is CPU-only ONNX and never touches the GPU. core/families.py pins the
# execution provider rather than letting the engine default choose, because the
# whole AI Runtime plan is built on asr-service holding no CUDA context (AIR-M3),
# and V5's engine defaulted the other way. Measured on the Orin: RSS ≈ 350 MB,
# RTF ≈ 0.043 against a requirement of RTF < 0.5.
#
# Failure philosophy, matching llm_server.sh: every precondition this script can
# check -- interpreter present, package importable, model directory populated -- is
# checked up front and fails loud with an actionable message, so a misconfigured
# deploy stops here rather than binding a port and looking healthy while being
# unable to transcribe anything.
#
# Shell strictness: fail fast on the first error, on any unset variable, and on a
# failure anywhere in a pipeline.
set -euo pipefail

# Resolve the deploy root from this script's location, never a hardcoded cd, so
# moving the tree does not break the launch. BASH_SOURCE (not $0) is correct whether
# the script is executed or sourced; the cd/pwd pair canonicalises the path.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "${SCRIPT_DIR}/../.." && pwd )"

PYTHON="${ASR_PYTHON:-python3}"

# -- Runtime knobs, all environment-overridable ------------------------------------
#
# Names match services/asr/config.py's env keys exactly; that file is the authority
# on what each one means and why it holds the value it does. Repeated here only as
# the deploy's declared configuration, so `cat asr_server.sh` answers "what is this
# box running" without reading Python.

# ★ Model family and weights. paraformer-zh-2023-09 was selected on 2026-08-03 over
# eight alternatives: on 59 human recordings it reached 93.2% exact and 92.6% recall
# on the estop keywords, against 74.6% / 66.7% for the previously deployed zipformer.
# The transducer export is still on disk and reachable by setting ASR_MODEL_FAMILY.
export ASR_MODEL_FAMILY="${ASR_MODEL_FAMILY:-paraformer}"

# Listen port. 99 U52 allocation: HMI 8080, payload 18080, asr 18081, llama 18082.
# The BIND ADDRESS is not settable here on purpose -- app.py pins 127.0.0.1 as a
# module constant, per 11 AS-13 / AIR-P1, so it cannot be misconfigured at all.
export PORT_ASR="${PORT_ASR:-18081}"

# ONNX intra-op threads. Below the core count so the co-hosted services keep theirs.
# ★ 1, matching config.py's default -- see the long note there. Short version: AIR-P3 pins
# this service to one core, and on one core four threads contend rather than parallelise
# (measured rtf 0.452 at 4 threads vs 0.119 at 1). More threads here is strictly slower.
export ASR_NUM_THREADS="${ASR_NUM_THREADS:-1}"

# int8 weights: the fast path, and what the RTF budget assumes. 0 selects fp32, used
# only to A/B a suspicious transcription.
export ASR_INT8="${ASR_INT8:-1}"

# Shortest upload the engine can decode. An engine capability floor, NOT a minimum
# utterance length -- a real spoken 停 is about 190 ms and passes comfortably.
export ASR_MIN_AUDIO_MS="${ASR_MIN_AUDIO_MS:-100}"

# Required input rate (11 AS-2). The capture path produces 16 kHz by construction, so
# anything else means a misconfigured chain; 0 disables the check for a bench feeding
# archived audio at its original rate.
export ASR_REQUIRE_SAMPLE_RATE="${ASR_REQUIRE_SAMPLE_RATE:-16000}"

# Hotword vocabulary. ⚠️ Has effect ONLY under the transducer family: sherpa-onnx
# offers contextual biasing on no other architecture, so under the default paraformer
# the file is not even read and GET /status reports hotwords_supported=false.
export ASR_HOTWORDS_FILE="${ASR_HOTWORDS_FILE:-${SCRIPT_DIR}/hotwords.txt}"
export ASR_HOTWORDS_SCORE="${ASR_HOTWORDS_SCORE:-2.0}"

# -- Preconditions -----------------------------------------------------------------

if ! command -v "${PYTHON}" >/dev/null 2>&1; then
  echo "[asr_server] ERROR: python interpreter not found: ${PYTHON}" >&2
  exit 1
fi

# The engine is the one dependency that is heavy, external, and whose absence would
# otherwise surface as a stack trace during startup rather than as a deploy problem.
if ! "${PYTHON}" -c "import sherpa_onnx" >/dev/null 2>&1; then
  echo "[asr_server] ERROR: sherpa-onnx is not importable by ${PYTHON}" >&2
  echo "[asr_server]        install it: ${PYTHON} -m pip install --user sherpa-onnx" >&2
  exit 1
fi

# Resolve the model directory the same way config.py will, so a missing export is
# reported here with a path rather than inside the service.
if [[ -n "${ASR_MODEL_DIR:-}" ]]; then
  MODEL_DIR="${ASR_MODEL_DIR}"
else
  case "${ASR_MODEL_FAMILY}" in
    # ★ Through `current`, not through a version directory. 11 §11A.4.1 defines rollback as
    # 「改软链 + 重启单元」, so every path that names a model must go via the symlink or the
    # flip would need this script edited too. Must agree with config.py's _FAMILY_MODEL_DIRS
    # -- the pre-flight below is worthless if it checks a different tree than the service.
    paraformer) MODEL_DIR="${SCRIPT_DIR}/models/paraformer-zh-2023-09/current" ;;
    transducer) MODEL_DIR="${SCRIPT_DIR}/models/zipformer-multi-zh-hans/current" ;;
    *)
      echo "[asr_server] ERROR: no default model directory for family '${ASR_MODEL_FAMILY}';" >&2
      echo "[asr_server]        set ASR_MODEL_DIR to point at its export" >&2
      exit 1
      ;;
  esac
fi

# -d follows symlinks, so a `current` pointing at a version directory passes and a dangling
# one fails here rather than inside the engine -- which is the whole point of a pre-flight.
if [[ ! -d "${MODEL_DIR}" ]]; then
  echo "[asr_server] ERROR: model directory missing: ${MODEL_DIR}" >&2
  exit 1
fi
# tokens.txt is the one file every family needs; its absence means the directory is
# not an export at all, which is worth catching before a multi-second model load.
if [[ ! -f "${MODEL_DIR}/tokens.txt" && -z "$(find "${MODEL_DIR}" -name tokens.txt -print -quit)" ]]; then
  echo "[asr_server] ERROR: no tokens.txt under ${MODEL_DIR} -- not a model export?" >&2
  exit 1
fi

echo "[asr_server] starting asr-service: family=${ASR_MODEL_FAMILY} model=${MODEL_DIR}" \
     "port=${PORT_ASR} threads=${ASR_NUM_THREADS} int8=${ASR_INT8}" \
     "min_audio_ms=${ASR_MIN_AUDIO_MS} require_rate=${ASR_REQUIRE_SAMPLE_RATE}"

# Run from the repo root so "services.asr.app" resolves, and exec so the service
# inherits this PID -- systemd then supervises the server itself rather than a shell
# that happens to have spawned it, and signals reach the right process.
cd "${REPO_ROOT}"
export PYTHONPATH="${REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
# ★ AIR-P3 (11 §11A.1.4): 三个服务 CPUAffinity=7，OMP_NUM_THREADS=1. The affinity half is a
# systemd directive (see deploy/systemd/), but the thread half belongs here, and it is the
# half that bites without it: onnxruntime already runs ASR_NUM_THREADS workers, and the
# BLAS layer underneath starts its OWN pool per worker. The two multiply. 10 §3.2 puts it
# directly -- 「numpy/scipy 的 OpenMP 多线程在绑核场景下会抢核，反而恶化周期 P99」 -- so once
# core 7 is shared with P4/P5/HMI, an unpinned BLAS pool degrades the processes beside it
# rather than speeding this one up. Measured RTF is 0.043 against a 0.5 requirement, so
# there is over 10x of headroom to spend on being a good neighbour.
#
# ★ ASR_NUM_THREADS is now 1 as well (see its export above and config.py's note): under the
# same pin, four onnxruntime threads contend on one core and measured 3.7x SLOWER than one.
# The two settings are separate constraints that happen to point the same way -- OMP=1 is
# written into AIR-P3 itself (frozen with AIR-P1~P4), while the thread count sits in F-23's
# NOT-frozen column and was changed on measurement.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1

exec "${PYTHON}" -m services.asr.app
