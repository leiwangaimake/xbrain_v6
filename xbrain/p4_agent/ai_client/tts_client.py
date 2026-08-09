"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: tts_client.py
Brief: GWY-P4-01 -- P4's ONLY authorized TTS speak call (via payload-service)

Description:
Emits one TTS utterance via payload-service's REST /tts endpoint.

Where TTS actually runs. There is NO on-Orin TTS engine.
payload-service forwards the text to the GZH-2 payload device, which
synthesizes and plays it over its integrated speaker (VOI-* series in
00). U52 pins this: no on-machine TTS process, no VRAM used; adding
an xbrain-ai-tts.service unit is banned (SEC-06 variant catches it).

Half-duplex is client-side. GZH-2 emits NO playback-complete event.
payload-service ESTIMATES speech duration from text length and
returns it as `est_ms`. The intent router uses est_ms to keep the
mic gate closed for that duration; if the mic reopened during
playback, the ASR would pick up the robot's own voice and produce a
feedback loop (AEC is structurally impossible per 00 VOI-*).

4.0 s per sentence is the U62 default; the config-loaded value
overrides. Sentence count = number of CJK / ASCII terminators in
the text. Zero terminators -> count = 1 (whole reply is one utterance).
"""

from __future__ import annotations

import logging
import re
from typing import Optional

import requests  # ECODE-OK(ai_client): CLAUDE.md 4.1 authorizes requests HERE


_logger = logging.getLogger("xbrain.ai_client.tts")

_TTS_PATH = "/tts"

# CJK + ASCII sentence terminators. Runtime chars covered:
#   U+3002 IDEOGRAPHIC FULL STOP        (CJK period)
#   U+FF1F FULLWIDTH QUESTION MARK
#   U+FF01 FULLWIDTH EXCLAMATION MARK
#   ASCII ? ! .  (period requires trailing whitespace / EOL to avoid
#                 counting an in-word abbreviation like "u.s." as three)
# CJK code points expressed as \u escapes so this file stays ASCII-
# clean per CLAUDE.md 2.2.
_SENTENCE_END = re.compile("[\u3002\uff1f\uff01?!]|(?:\\.(?=\\s|$))")


class TtsClientError(RuntimeError):
    """Raised when TTS could not be spoken.

    Same rationale as asr / llm client errors: one type covers connect
    failure, non-200, and unexpected body shape because the router's
    response to all three is the same (log, skip audio for this turn,
    keep listening).
    """


def _count_sentences(text: str) -> int:
    """Count sentence-ending punctuation; minimum 1.

    An overcount here is safer than undercount: the caller uses the
    number to KEEP THE MIC GATE CLOSED for est duration -- overcounting
    keeps the gate closed slightly too long (a benign artifact), while
    undercounting would let the mic pick up the robot's tail audio."""
    n = len(_SENTENCE_END.findall(text))
    return max(1, n)


def speak(
    base_url: str,
    text: str,
    *,
    timeout_s: float,
    est_ms_per_sentence: float = 4000.0,
) -> float:
    """POST /tts to payload-service; return estimated playback ms.

    Args:
        base_url: payload-service, typically http://127.0.0.1:18080
        text: what to speak; non-empty
        timeout_s: HTTP timeout for the POST itself, NOT the estimated
            playback duration (which the caller waits out separately)
        est_ms_per_sentence: fallback estimate if payload-service's
            response does not include an `est_ms` field; 4000 =
            4 s / sentence, U62 default. The config-loaded value
            should override.

    Returns:
        Estimated playback duration in ms. Router uses this to time
        the mic-gate close window.

    Raises:
        TtsClientError: request fail / non-200 / unexpected body / empty text.

    Blocking: makes a network call. Run under asyncio.to_thread for
    async callers.
    """
    if not text or not text.strip():
        raise TtsClientError("tts text is empty; refusing to POST")

    url = base_url.rstrip("/") + _TTS_PATH
    try:
        r = requests.post(url, json={"text": text}, timeout=timeout_s)
    except requests.RequestException as exc:
        raise TtsClientError(
            "tts request to %s failed: %s" % (url, exc)) from exc
    if r.status_code != 200:
        raise TtsClientError(
            "tts returned %d: %s" % (r.status_code, r.text[:200]))
    try:
        body = r.json()
    except ValueError as exc:
        raise TtsClientError(
            "tts response not JSON: %s" % r.text[:200]) from exc

    est_ms = body.get("est_ms")
    if est_ms is None:
        # Fallback: count sentences, multiply by per-sentence budget.
        # This code path is exercised whenever payload-service is not
        # yet returning est_ms; it lets voice-loop MVP work end-to-end
        # even before T-MIC-3 is calibrated.
        est_ms = _count_sentences(text) * est_ms_per_sentence
        _logger.info(
            "tts: est_ms not in response, using fallback: %.0f ms", est_ms)
    else:
        try:
            est_ms = float(est_ms)
        except (ValueError, TypeError) as exc:
            raise TtsClientError(
                "tts est_ms field not a number: %r" % est_ms) from exc

    _logger.info("tts: chars=%d est_ms=%.0f", len(text), est_ms)
    return est_ms
