"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: llm_client.py
Brief: GWY-P4-01 -- P4's ONLY authorized LLM call site (11 S8.13.2)

Description:
Wraps llama-server's OpenAI-compatible /v1/chat/completions endpoint
as a synchronous streaming client. Every P4 module that needs open
dialog goes through `complete(base_url, user_text, ...)`; nothing
else in xbrain/p4_agent/ may `import requests` (CLAUDE.md 4.1 + CI
grep). This file is one of the two authorized instances (the other
is asr_client.py).

Why the stream is consumed even when the caller only wants finished
text: reading incrementally lets the client CUT the generation off
once it passes reply_max_chars, instead of waiting for a paragraph
that would then be discarded. It also makes time-to-first-token
measurable, which is the number that decides whether a turn feels
responsive.

Why the reply is nevertheless spoken as ONE utterance (not sentence
by sentence): the payload's TTS is fire-and-forget with no completion
event; payload-service ESTIMATES when speech ends. Chaining one TTS
per sentence would chain one estimate per sentence and every error
would either cut the previous sentence off mid-word or leave an
audible hole. One utterance -> one estimate.

Failure semantics match asr_client: one exception type covers all
three failure classes because the router treats them identically.

★ SSE encoding pin: llama-server sends `text/event-stream` without a
charset; requests falls back to latin-1 which mojibakes every Chinese
reply. `response.encoding = 'utf-8'` is not optional -- removing it
silently reintroduces the bug and there is no error path to hit.
"""

from __future__ import annotations

import json
import logging
import time
from typing import Dict, List, Optional

import requests  # ECODE-OK(ai_client): CLAUDE.md 4.1 authorizes requests HERE


_logger = logging.getLogger("xbrain.ai_client.llm")

# llama-server OpenAI-compatible route.
_CHAT_PATH = "/v1/chat/completions"

# W3C SSE format: payload lines prefixed 'data: '; stream terminated
# by literal '[DONE]' sentinel (not by connection close).
_SSE_DATA_PREFIX = "data:"
_SSE_DONE = "[DONE]"


class LlmClientError(RuntimeError):
    """Raised when a reply could not be generated.

    Covers connect failure, non-200 response, and stream that ends
    without content. The router's response to all three is the same
    (log, drop turn, keep listening), so splitting the types would
    invite handling that does not differ.
    """


def _build_messages(system_prompt: str, user_text: str) -> List[dict]:
    """Compose the OpenAI messages array for one turn.

    Empty system_prompt is OMITTED (not sent as empty system message).
    Some chat templates render an empty system message as a blank
    instruction block that measurably degrades the reply. Sending
    nothing is the faithful representation of "not decided yet"."""
    msgs: List[dict] = []
    if system_prompt:
        msgs.append({"role": "system", "content": system_prompt})
    # Order is contract: chat templates render positionally; a system
    # turn placed AFTER the user turn is read as an instruction about
    # a question already asked, not as standing persona.
    msgs.append({"role": "user", "content": user_text})
    return msgs


def _delta_text(line: str) -> str:
    """Extract token text from one SSE data line; '' if none.

    Returning '' rather than raising on unrecognized lines is
    deliberate: SSE allows comments and keep-alives, and llama-server
    chunk shape has gained fields across versions. A parser that
    rejected anything it did not recognize would turn a harmless
    protocol addition into a dead turn."""
    if not line.startswith(_SSE_DATA_PREFIX):
        return ""
    payload = line[len(_SSE_DATA_PREFIX):].strip()
    if not payload or payload == _SSE_DONE:
        return ""
    try:
        chunk = json.loads(payload)
        return (chunk.get("choices", [{}])[0]
                .get("delta", {})
                .get("content", "") or "")
    except (ValueError, IndexError, AttributeError, TypeError):
        # Malformed chunk skipped: losing one token is recoverable;
        # losing the whole reply is not.
        return ""


def complete(
    base_url: str,
    user_text: str,
    *,
    timeout_s: float,
    model: str,
    system_prompt: str = "",
    max_tokens: int = 512,
    reply_max_chars: int = 200,
    temperature: float = 0.7,
) -> str:
    """Generate the spoken reply for one recognized utterance.

    Args:
        base_url: e.g. "http://127.0.0.1:18082"
        user_text: recognized user utterance (already through ASR)
        timeout_s: HTTP timeout; upper bound per 11 S8.13.1 AS-7 is
            5 s but the actual value comes from p4_agent.yaml
        model: LLM model name (llama-server's -m arg)
        system_prompt: '' = do NOT send a system turn (see _build_messages)
        max_tokens: bounds what the SERVER produces (frees the GPU slot);
            NOT the same as reply_max_chars because Chinese chars/token
            is not fixed
        reply_max_chars: caps what the DEVICE will be allowed to speak,
            in the unit the length policy is expressed in; stream is
            cut once accumulated content exceeds this
        temperature: 0.7 is a reasonable default; caller supplies

    Returns:
        Reply text, capped at reply_max_chars.

    Raises:
        LlmClientError: request fail / non-200 / empty stream.

    Blocking: makes a network call. Run under asyncio.to_thread for
    async callers.
    """
    url = base_url.rstrip("/") + _CHAT_PATH
    body = {
        "model": model,
        "messages": _build_messages(system_prompt, user_text),
        "max_tokens": max_tokens,
        "temperature": temperature,
        "stream": True,
    }

    # Clock STARTS BEFORE the request, not at first byte: the number
    # being measured is the silence the person sits through. Connect
    # + prompt-process time is part of that gap even though the model
    # has not started writing.
    started = time.perf_counter()
    first_token_ms = 0.0
    pieces: List[str] = []
    length = 0

    try:
        with requests.post(url, json=body, stream=True,
                           timeout=timeout_s) as r:
            if r.status_code != 200:
                # Read body BEFORE leaving the with-block (unavailable
                # after context close).
                raise LlmClientError(
                    "llm returned %d: %s" % (r.status_code, r.text[:200]))
            # ★ SSE UTF-8 pin -- see module docstring. Without this
            # every Chinese reply reaches the device as mojibake.
            r.encoding = "utf-8"
            for raw in r.iter_lines(decode_unicode=True):
                if not raw:
                    continue
                piece = _delta_text(raw)
                if not piece:
                    continue
                if not pieces:
                    first_token_ms = (time.perf_counter() - started) * 1000.0
                pieces.append(piece)
                length += len(piece)
                if length >= reply_max_chars:
                    # Stop as soon as accumulated > cap. Everything
                    # after would be discarded; continuing to receive
                    # only delays the reply.
                    break
    except requests.RequestException as exc:
        raise LlmClientError(
            "llm request to %s failed: %s" % (url, exc)) from exc

    reply = "".join(pieces).strip()
    if not reply:
        # Empty stream = model produced nothing to say. Speaking a
        # canned apology would hide a broken deploy behind plausible
        # behaviour -- so this is an error.
        raise LlmClientError("llm stream produced no text")

    total_ms = (time.perf_counter() - started) * 1000.0
    _logger.info(
        "llm reply: chars=%d first_token_ms=%.0f total_ms=%.0f",
        len(reply), first_token_ms, total_ms)

    # Apply cap once more: the loop stops AFTER the piece that
    # crossed the limit is appended, so accumulated text can overshoot
    # by up to one token's worth of characters.
    return reply[:reply_max_chars]
