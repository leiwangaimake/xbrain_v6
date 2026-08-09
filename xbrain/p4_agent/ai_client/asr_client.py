"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: asr_client.py
Brief: GWY-P4-01 -- P4's ONLY authorized ASR call site (11 S8.13.1)

Description:
Wraps asr-service's OpenAI-compatible /v1/audio/transcriptions endpoint
into one callable, `transcribe(base_url, pcm_16k_mono, *, timeout_s,
model, language)`. Every module in xbrain/p4_agent/ that needs speech
recognition goes through this file; other modules must not import
requests directly (CLAUDE.md 4.1 enforces this via a CI grep, and the
`import requests` line below is the ONLY authorized instance in P4).

Why a fresh, dependency-free wrapper instead of promoting
tests/ai_runtime/asr_client.py directly:
  * tests/ai_runtime carries an AiRuntimeConfig frozen dataclass with
    env-var overrides -- that's fine for a harness, not for a runtime
    entry-point which reads /run/xbrain/resolved/p4_agent.yaml
  * P4's config loader (xbrain/p4_agent/config/loader.py) already
    returns the frozen p4_agent config; this wrapper takes just the
    URL/timeout that loader emits, so the coupling is one arrow
  * Test coverage sits in tests/p4_agent/ai_client/, not tests/ai_runtime/

Wave-envelope discipline (kept from tests/ai_runtime/asr_client.py):
  raw PCM upload requires a sample_rate form field that can silently
  disagree with the actual audio. Wrapping in a WAV means the RIFF
  header carries rate + width, so the two processes cannot get out of
  sync. Costs 44 bytes; pays off in unambiguous failure semantics.

Error semantics: three failure classes (connect / non-200 / bad shape)
all map to one AsrClientError. The intent router's response to all
three is identical -- log, drop this turn, keep listening -- so
splitting the types would invite handling that does not differ.

★ paraformer specific: the recognizer returns `conf` as null; any
downstream comparison like `if conf > 0.8:` must guard the null case
(GWY-P4-01 judgeria #6). This wrapper does not filter on conf; that
policy lives in intent_router.
"""

from __future__ import annotations

import io
import wave
from typing import Optional

import requests  # ECODE-OK(ai_client): CLAUDE.md 4.1 authorizes requests HERE


# --- Audio format constants (VOI-10a) --------------------------------
# The mic plane pins 16 kHz s16le mono AFTER downsample from USB MIC's
# native 48 kHz. If any of these three change, asr-service's `-r 16000`
# audio-in loader and this wrapper must move together.
_SAMPLE_RATE = 16_000
_CHANNELS = 1
_SAMPLE_WIDTH = 2

# OpenAI-compatible endpoint path.
_TRANSCRIBE_PATH = "/v1/audio/transcriptions"

# Multipart field names, fixed by the OpenAI schema.
_FILE_FIELD = "file"
_MODEL_FIELD = "model"
_LANGUAGE_FIELD = "language"

# Sent as the multipart filename. The service ignores it, but it shows
# up in access logs / packet captures, so a name that says which
# process emitted the audio is more useful than "audio.wav".
_UPLOAD_NAME = "xbrain_p4_agent_utterance.wav"


class AsrClientError(RuntimeError):
    """Raised when an utterance could not be transcribed.

    CLAUDE.md 4.5 bans bare Exception. One type covers connect
    failure, non-200 response, and unrecognized body shape because the
    intent router treats all three the same. RuntimeError is the base
    because these are operational faults, not bad arguments.
    """


def pcm16_to_wav(pcm: bytes) -> bytes:
    """Wrap raw 16 kHz mono s16le PCM in a WAV container.

    stdlib wave.open writes the RIFF header; hand-packed RIFF headers
    are a classic source of off-by-one chunk sizes that produce a file
    most readers accept and one rejects. The from-scratch part of P4
    is the router, not the container format.

    Built in memory: the WAV exists only as the body of one HTTP POST;
    writing to disk would add I/O + cleanup per utterance."""
    buf = io.BytesIO()
    # The context manager close is what backpatches chunk sizes into
    # the header -- read buf before that and sizes are still zero.
    with wave.open(buf, "wb") as writer:
        writer.setnchannels(_CHANNELS)
        writer.setsampwidth(_SAMPLE_WIDTH)
        writer.setframerate(_SAMPLE_RATE)
        writer.writeframes(pcm)
    return buf.getvalue()


def transcribe(
    base_url: str,
    pcm_16k_mono: bytes,
    *,
    timeout_s: float,
    model: str = "paraformer",
    language: str = "zh",
) -> str:
    """Send one utterance to asr-service; return recognized text.

    Args:
        base_url: e.g. "http://127.0.0.1:18081" (from p4_agent.yaml
            `gateway.asr.base_url` after config-freeze resolution).
        pcm_16k_mono: raw 16 kHz mono s16le PCM for one utterance.
        timeout_s: HTTP client timeout. Bounded, never None -- an
            unbounded timeout on the ASR call would let a wedged
            service hang an entire turn (AS-7 upper bound = 5 s per
            11 S8.13.1, but the actual value comes from config).
        model: asr-service ignores this for local models; kept for
            OpenAI compat so pointing at OpenAI needs no code change.
        language: same. For Chinese "zh" measurably improves accuracy
            on OpenAI Whisper; local paraformer ignores it.

    Returns:
        The recognized text. Empty string is legal (VAD false trigger
        or genuine silence-only segment) -- callers must not treat it
        as an error.

    Raises:
        AsrClientError: connect fail / non-200 / unexpected body shape.

    Blocking: this makes a network call. Run under asyncio.to_thread
    if the caller is async.
    """
    url = base_url.rstrip("/") + _TRANSCRIBE_PATH
    wav_body = pcm16_to_wav(pcm_16k_mono)
    files = {_FILE_FIELD: (_UPLOAD_NAME, wav_body, "audio/wav")}
    data = {_MODEL_FIELD: model, _LANGUAGE_FIELD: language}
    try:
        r = requests.post(url, files=files, data=data, timeout=timeout_s)
    except requests.RequestException as exc:
        raise AsrClientError(
            "asr request to %s failed: %s" % (url, exc)) from exc
    if r.status_code != 200:
        # Body included: asr-service puts the reason there (a malformed
        # upload yields 400 with the specific complaint). Truncate so a
        # multi-KB error page doesn't blow up the log.
        raise AsrClientError(
            "asr returned %d: %s" % (r.status_code, r.text[:200]))
    try:
        body = r.json()
        text = body["text"]
    except (ValueError, KeyError, TypeError) as exc:
        # 200 with unexpected shape means the wrapper is pointed at
        # something that is not asr-service (wrong port is the usual
        # cause). Saying so beats KeyError deep in the router.
        raise AsrClientError(
            "asr response not {'text': ...}: %s" % r.text[:200]
        ) from exc
    if not isinstance(text, str):
        raise AsrClientError(
            "asr text field is %s not str" % type(text).__name__)
    return text
