#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: llm_server.sh
# Brief: Launch the bundled llama-server as an OpenAI-compatible LLM service
#
# Description:
# Starts services/llm/bin/llama-server as an OpenAI-style HTTP endpoint
# (POST /v1/chat/completions) that the AI_runtime orchestrator calls during
# 功能1 near-field dialogue. Only the prebuilt binary, its shared libs and the
# gguf model live under services/llm/ (bin/ lib/ model/); the llama.cpp source
# is intentionally NOT deployed. Every path is derived from this script's own
# location so the whole services/llm/ tree stays relocatable. All tunables are
# environment-overridable so the same script works on the Orin and elsewhere.
# See docs GZH-2_测试系统开发方案 section 16.
#
# Why a shell wrapper rather than launching the binary directly: the wrapper is
# the single place that pins the model path, the network bind, and the library
# search path, and that validates the deploy before handing off. That means the
# service is started the same way by systemd, by a developer at a terminal, or by
# a test harness -- none of them needs to know the argument list or the layout.
#
# Contract with AI_runtime: the orchestrator's LLM_BASE_URL must point at
# http://HOST:PORT here, and it speaks the OpenAI chat-completions schema, which
# llama-server serves natively. Because llama-server IS prebuilt infrastructure,
# reusing it is allowed under the from-scratch rule (R0) -- only our own glue
# (this launcher and the runtime that calls it) must be written from scratch.
#
# Failure philosophy: every precondition this script can check (binary present and
# executable, a model file resolvable) is checked up front and made to fail loud
# with an actionable message, because a misconfigured deploy should stop here with
# a clear cause rather than surface as an opaque error from deep inside the binary
# or, worse, as a silently unreachable endpoint that AI_runtime only discovers at
# first request time.
#
# Shell strictness: fail fast on the first error, on any unset variable, and on a
# failure anywhere in a pipeline -- so a broken launch aborts immediately instead
# of limping forward with half its configuration missing.
set -euo pipefail

# Resolve the deploy root from this script's location, never a hardcoded cd,
# so moving services/llm/ around does not break the launch. Using BASH_SOURCE
# (not $0) makes this correct whether the script is executed directly or sourced,
# and the cd/pwd pair canonicalises the path so every derived path below is
# absolute regardless of the caller's working directory.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Fixed layout under SCRIPT_DIR; BIN is overridable in case a differently named
# binary is dropped in bin/ (e.g. a CUDA vs CPU build).
BIN="${LLM_BIN:-${SCRIPT_DIR}/bin/llama-server}"
LIB_DIR="${SCRIPT_DIR}/lib"       # bundled .so dependencies for the binary
MODEL_DIR="${SCRIPT_DIR}/model"   # gguf model(s) live here

# Runtime knobs, all env-overridable.
#
# * HOST is loopback, not 0.0.0.0, and that is a contract requirement rather than a
# preference: 11 AIR-P1 says every AI Runtime port binds 127.0.0.1 only. The reason is that
# the general plane carries no authentication at all (U23), so a LAN-reachable
# /v1/chat/completions lets any host in the compound submit an arbitrary grammar and an
# arbitrarily long prompt -- both a resource-exhaustion surface and a way around the intent
# closed set that 11 §11.2 relies on. A remote caller that genuinely needs this should be
# tunnelled, not served.
#
# * PORT is 18082 from the allocation in 99 U52: HMI 8080, payload 18080, asr 18081,
# llama 18082. The historical default here was 8080, which U52 assigns to the HMI.
HOST="${LLM_HOST:-127.0.0.1}"      # * loopback ONLY -- 11 AIR-P1. The general plane is unauthenticated (U23),
                                   # so 0.0.0.0 lets any host on the camp net POST arbitrary grammar to
                                   # /v1/chat/completions -- a resource-exhaustion surface AND a way around the
                                   # §11.2 intent closed set. Override only for a deliberate, reviewed exception.
PORT="${LLM_PORT:-18082}"          # OpenAI endpoint port (99 U52 allocation; AI_runtime llm_url must match)
# ! Contested value -- see the note above the exec line. 11 §11A.3.1/.2/.3 writes 2048 and
# derives its memory ledger from it; 99 T-LLM-1 says keep the measured 32768. 11 §14.2 F-23
# lists n_ctx in the NOT-frozen column, so this is a PR-level change either way.
CTX_SIZE="${LLM_CTX:-2048}"        # context window (11 §11A.3.2); prompt <=900 tok + predict 256 fits with 2x headroom
THREADS="${LLM_THREADS:-2}"        # * 11 §11A.3.2 fixes this at 2 (AIR-P3). 10 §3.2 gives the AI services core 7
                                   # only; llama.cpp defaulting to all cores preempts the pinned P1/P2 processes.
NGL="${LLM_NGL:-999}"              # GPU layers to offload; Orin has CUDA so default offloads all, set 0 for a CPU-only build
MODEL="${LLM_MODEL:-}"             # explicit gguf path; if empty we auto-pick the first one in model/

# * Server slots. 11 AS-3 sets the AI Runtime concurrency limit at 1, and llama-server's
# default (-1 = auto) picked 4 on this box, which silently exceeded it.
#
# The deployment makes 1 the honest number as well as the contractual one: this binds
# loopback on a quadruped patrol robot, where the only client is the on-box AI_runtime and
# it issues one utterance per dialogue turn. Slots reserve KV cache whether or not they are
# used, so four of them spend memory on concurrency that cannot occur.
#
# ! Note what this does and does not do. A second request is QUEUED server-side, not
# refused -- unlike asr-service, which answers 429. 11 §11A.3.4 flags exactly that: once
# llama-server queues, the gateway's own priority ordering is overridden by the server's
# FIFO, so the gateway must take its GPU token BEFORE issuing the HTTP call rather than
# relying on the server to push back. The /slots endpoint stays enabled so queue_depth can
# be watched; 11 §11A.7 treats a non-zero depth as a defect indicator, not a load metric.
PARALLEL="${LLM_PARALLEL:-1}"      # server slots -- 11 AS-3 concurrency limit is 1

# * Server-side generation cap and prefix reuse (11 §11A.3.2). --predict is a DEFAULT, not a
# ceiling: a client can still override n_predict per request, so the hard limit stays the
# gateway's job (16 §14 n_predict_max). --cache-reuse lets the KV suffix be reused when the
# context layer changes mid-prompt, which is what makes the five-layer prompt affordable.
PREDICT="${LLM_PREDICT:-256}"
CACHE_REUSE="${LLM_CACHE_REUSE:-256}"

# Throughput/memory knobs, pinned to the values field-tuned for this box. They are
# stated explicitly rather than left to llama-server's built-in defaults so that a
# future llama.cpp version bump cannot silently change the service's performance
# profile underneath us -- a launcher's job is to make the deploy reproducible.
BATCH="${LLM_BATCH:-512}"          # logical batch (11 §11A.3.2): prompt <=900 tok prefills in 2 passes at 512;
                                   # larger only adds compute-buffer bytes to the AIR-M1 ledger for no gain.
UBATCH="${LLM_UBATCH:-512}"        # physical micro-batch actually handed to the GPU
KV_TYPE="${LLM_KV_TYPE:-f16}"      # KV cache precision, used for both K and V
FLASH_ATTN="${LLM_FLASH_ATTN:-on}" # on|off|auto; forced on since the Orin's CUDA build supports it

# Prefer the bundled libs over system ones so the service is self-contained and
# does not depend on whatever libstdc++/CUDA happens to be installed globally.
# LIB_DIR is prepended (not appended) so the bundled versions win over any system
# copy of the same soname, and the existing LD_LIBRARY_PATH is preserved after it
# via the ":-" default so we extend the search path rather than clobber it.
export LD_LIBRARY_PATH="${LIB_DIR}:${LD_LIBRARY_PATH:-}"

# Pre-flight: the binary must exist and be executable, otherwise fail loud early
# instead of emitting a confusing error deep inside llama-server.
if [[ ! -x "${BIN}" ]]; then
  echo "[llm_server] ERROR: llama-server binary missing or not executable: ${BIN}" >&2
  echo "[llm_server] hint: place the binary at services/llm/bin/llama-server and chmod +x" >&2
  exit 1
fi

# When no model is pinned via LLM_MODEL, deterministically take the first *.gguf
# found in model/ (sorted) so a single-model deploy just works with no config.
# The sort makes the choice reproducible: without it the pick would depend on
# filesystem enumeration order and could differ between boxes or after a copy.
# maxdepth 1 keeps the search to model/ itself, and the trailing "|| true" stops
# a find that matches nothing from tripping the "set -e" abort -- the genuinely
# empty case is handled explicitly by the guard immediately below, with a message.
if [[ -z "${MODEL}" ]]; then
  # * -L and the */current/ segment are what make 11 §11A.4.1's rollback work: the layout
  # is <name>/<ver>/model.gguf with `current` a symlink to the live version, so rollback is
  # "改软链 + 重启单元" and never a code change. Without -L, find would refuse to descend
  # through the symlink and auto-pick would silently find nothing -- the failure would look
  # like a missing model rather than a traversal flag, so it is called out here.
  MODEL="$( find -L "${MODEL_DIR}" -mindepth 2 -type f -name '*.gguf' -path '*/current/*' \
            | sort | head -n 1 || true )"
fi
# Whether the model was auto-picked or set via LLM_MODEL, confirm it actually
# resolves to a real file before launching, so a typo'd LLM_MODEL or an empty
# model/ fails here with a clear cause rather than inside llama-server.
if [[ -z "${MODEL}" || ! -f "${MODEL}" ]]; then
  echo "[llm_server] ERROR: no gguf model under ${MODEL_DIR} (or set LLM_MODEL=/path/to/model.gguf)" >&2
  exit 1
fi

# Echo the fully-resolved configuration once at launch. This is the record of what
# the service actually started with (which model file won auto-pick, which port and
# thread count are in effect), so a log or a terminal shows the effective settings
# without the reader having to reconstruct them from the environment and defaults.
echo "[llm_server] starting llama-server: model=${MODEL} host=${HOST} port=${PORT} ctx=${CTX_SIZE} threads=${THREADS} ngl=${NGL} parallel=${PARALLEL} batch=${BATCH}/${UBATCH} kv=${KV_TYPE} fa=${FLASH_ATTN}"

# exec so llama-server replaces this shell as PID: SIGTERM/SIGINT from systemd or
# the terminal reach the server directly for a clean shutdown (no orphan wrapper).
# Without exec, this shell would remain the parent, signals would land on the
# wrapper instead of the server, and process managers would have to chase a second
# PID to stop the service cleanly. llama-server serves the OpenAI
# /v1/chat/completions route by default, so no route flag is needed here.
#
# --context-shift keeps generation alive past the context window by evicting the
# oldest tokens instead of erroring out, which matters for 功能1 because a dialogue
# session accumulates turns and must not die mid-conversation. --jinja selects the
# chat template embedded in the GGUF rather than a built-in guess, so Qwen's turn
# markers are emitted exactly as the model was trained on -- getting this wrong
# degrades reply quality in ways that are hard to attribute later.
# * 11 §11A.3.1 opens with these exports and AIR-P3 requires them: without OMP_NUM_THREADS=1
# the BLAS layer spawns its own pool underneath --threads, and the two multiply into far more
# runnable threads than core 7 can hold.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1

# * --context-shift is deliberately NOT passed. 11 F-AI-4 requires a context overflow to be
# a VISIBLE failure -- the request errors and P4 records a ctx_overflow event -- because the
# correct repair is trimming the history layer in prompt assembly, which F-AI-4 calls a P4
# defect. Silently sliding the window would hide exactly the defect the contract wants seen.

exec "${BIN}" \
  --model "${MODEL}" \
  --host "${HOST}" \
  --port "${PORT}" \
  --ctx-size "${CTX_SIZE}" \
  --threads "${THREADS}" \
  --n-gpu-layers "${NGL}" \
  --parallel "${PARALLEL}" \
  --batch-size "${BATCH}" \
  --ubatch-size "${UBATCH}" \
  --cache-type-k "${KV_TYPE}" \
  --cache-type-v "${KV_TYPE}" \
  --flash-attn "${FLASH_ATTN}" \
  --predict "${PREDICT}" \
  --cache-reuse "${CACHE_REUSE}" \
  --metrics \
  --slots \
  --no-webui \
  --jinja
