"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: llm_client.py
Brief: Client for llama-server's OpenAI chat-completions endpoint, consumed as an SSE stream.

Description:
  Turns the user's recognized sentence into the reply the device will speak. llama-server
  is prebuilt inference infrastructure and outside R0 (plan section 11); written from
  scratch here are the request shaping, the SSE decoding, and the length policy.

  Why the stream is consumed token by token when the caller only wants the finished text:
  a stream can be STOPPED. The reply is spoken aloud by the device, and speech is far
  slower than generation, so the binding limit is not how fast the model writes but how
  long the robot talks. Reading incrementally lets the client cut the generation off the
  moment it passes reply_max_chars, instead of waiting for a paragraph it would then throw
  away. It also makes time-to-first-token measurable, which is the number that decides
  whether the turn feels responsive.

  Why the reply is nonetheless spoken as ONE utterance rather than sentence by sentence:
  the device's TTS ([31]) is fire-and-forget with no completion event, so payload-service
  can only ESTIMATE when speech ends (plan section 7). Chaining one TTS per sentence would
  chain one estimate per sentence, and every error would either cut the previous sentence
  off mid-word or leave an audible hole. One utterance means one estimate, which is also
  the single number the on-device timing calibration has to establish. Speaking earlier is
  worth revisiting once the device gives a real done event -- not before.

  Failure policy matches asr_client: one exception type, because the turn loop's answer to
  every LLM fault is the same -- log it, drop the turn, keep listening.
"""
from __future__ import annotations

# json decodes each SSE chunk, and time measures the two numbers this module reports:
# time-to-first-token, which decides whether the turn feels responsive, and the total,
# which is what the length policy trades against.
import json
import logging
import time
from typing import Dict, List

# The whitelisted HTTP client (plan section 11). Its stream=True mode is what makes the
# early cut-off in complete() possible at all.
import requests

from .config import AiRuntimeConfig

# Namespaced under "ai_runtime." so the generation phase can be traced independently of
# the recognition and playback phases around it.
logger = logging.getLogger("ai_runtime.llm")

# llama-server's OpenAI-compatible route (plan section 5.4).
_CHAT_PATH = "/v1/chat/completions"

# SSE wire format (W3C server-sent events, which is what the OpenAI streaming API speaks):
# payload lines are prefixed "data: ", and the stream is terminated by the literal
# sentinel "[DONE]" rather than by the connection closing.
_SSE_DATA_PREFIX = "data:"
_SSE_DONE = "[DONE]"

# Chat message roles.
_ROLE_SYSTEM = "system"
_ROLE_USER = "user"


class LlmClientError(RuntimeError):
    """Raised when a reply could not be generated.

    House rule bans bare Exception. Covers the connection failing, a non-200 answer and a
    stream that ends without any content, since the turn loop treats all three the same
    way. RuntimeError is the base because these are operational faults.
    """


def _build_messages(config: AiRuntimeConfig, user_text: str) -> List[Dict[str, str]]:
    """Assemble the OpenAI messages array for one turn.

    Args:
        config: supplies the system prompt, which is empty by default.
        user_text: what asr-service recognized the person as having said.

    Returns:
        The messages list: the user turn, optionally preceded by a system turn.

    An empty system prompt is OMITTED rather than sent as an empty system message. The
    development plan deliberately leaves the prompt unwritten for now (section 16), and an
    empty system message is not the same thing as no system message -- some templates
    render it as a blank instruction block that measurably degrades the reply. Sending
    nothing is the faithful representation of "not decided yet".

    No conversation history is carried. 功能1 turns are robot commands and short questions,
    each self-contained; accumulating history would grow the prompt every turn and make the
    reply depend on how long the session had been running, which is exactly the property
    that makes a bring-up test unreproducible.
    """
    messages: List[Dict[str, str]] = []
    if config.llm_system_prompt:
        messages.append({"role": _ROLE_SYSTEM, "content": config.llm_system_prompt})
    # Order is part of the contract, not an implementation detail: chat templates render
    # the array positionally, and a system turn placed after the user turn is read as an
    # instruction about a question already asked rather than as the standing persona.
    messages.append({"role": _ROLE_USER, "content": user_text})
    return messages


def _delta_text(line: str) -> str:
    """Extract the token text from one SSE data line, or "" if it carries none.

    Args:
        line: one line of the response body, already stripped of its newline.

    Returns:
        The incremental content this line adds, or "" for anything that is not a content
        delta -- the blank separator lines, the [DONE] sentinel, the role-only first chunk,
        and the final chunk that carries only a finish_reason.

    Returning "" rather than raising for an unrecognized line is deliberate: SSE allows
    comments and keep-alives, and llama-server's chunk shape has gained fields across
    versions. A parser that rejected anything it did not recognize would turn a harmless
    protocol addition into a dead turn, whereas one that ignores what it cannot use keeps
    working. Anything genuinely broken still surfaces, as a stream that produced no text.
    """
    if not line.startswith(_SSE_DATA_PREFIX):
        return ""
    # lstrip rather than a fixed slice: the space after the colon is optional in SSE.
    payload = line[len(_SSE_DATA_PREFIX):].strip()
    if not payload or payload == _SSE_DONE:
        return ""
    try:
        chunk = json.loads(payload)
        # Structure: {"choices": [{"delta": {"content": "..."}}]}. Every level is fetched
        # with a default so a chunk that omits one (the role-only opener, the finish-reason
        # closer) yields "" instead of raising.
        return chunk.get("choices", [{}])[0].get("delta", {}).get("content", "") or ""
    except (ValueError, IndexError, AttributeError, TypeError):
        # A malformed chunk is skipped rather than killing the stream: losing one token is
        # recoverable, losing the whole reply is not.
        return ""


def complete(config: AiRuntimeConfig, user_text: str) -> str:
    """Generate the spoken reply for one recognized utterance.

    Args:
        config: supplies the service URL, model name, sampling knobs and length cap.
        user_text: the recognized user utterance.

    Returns:
        The reply text, already capped at config.reply_max_chars.

    Raises:
        LlmClientError: if the request fails, the service answers non-200, or the stream
            completes without producing any text.

    Blocking: this makes a network call and must be run on a worker thread, never on the
    event loop.
    """
    url = config.llm_url.rstrip("/") + _CHAT_PATH
    # max_tokens and reply_max_chars are two different limits and both are needed.
    # max_tokens bounds what the SERVER is asked to produce, which is what frees the
    # generation slot; reply_max_chars bounds what the DEVICE will be allowed to say, in
    # the unit the length policy is actually expressed in. Tokens do not map to Chinese
    # characters at a fixed rate, so neither one can stand in for the other.
    body = {
        "model": config.llm_model,
        "messages": _build_messages(config, user_text),
        "max_tokens": config.llm_max_tokens,
        "temperature": config.llm_temperature,
        # Streaming is what makes the early cut-off below possible (module docstring).
        "stream": True,
    }
    # The clock starts BEFORE the request, not at the first byte of the response, because
    # the number being measured is the silence the person sits through: connecting and
    # prompt processing are part of that gap even though the model has not started writing.
    started = time.perf_counter()
    first_token_ms = 0.0
    # Accumulated as pieces rather than string concatenation: a reply arrives as dozens of
    # tokens, and joining once at the end avoids rebuilding the whole string per token.
    pieces: List[str] = []
    length = 0
    try:
        # stream=True holds the body open so iter_lines yields chunks as they arrive; with
        # it False, requests would buffer the entire reply and the early stop would be
        # meaningless. The response is closed by the with-block on every path, including
        # the early break, which is what releases the server-side generation slot.
        with requests.post(
            url, json=body, stream=True, timeout=config.http_timeout_s
        ) as response:
            if response.status_code != 200:
                # Read the body before leaving the block, since it is unavailable after.
                raise LlmClientError(f"llm returned {response.status_code}: {response.text[:200]}")
            # Pinned because requests falls back to ISO-8859-1 for any text/* content type
            # that does not name a charset, and llama-server sends bare
            # "text/event-stream". Left alone, decode_unicode below would latin-1 the
            # bytes and every Chinese reply would reach the device as mojibake -- which
            # the device's TTS would dutifully read aloud. SSE is UTF-8 by specification,
            # so this asserts the encoding the wire actually uses.
            response.encoding = "utf-8"
            for raw in response.iter_lines(decode_unicode=True):
                # iter_lines yields the empty SSE separator lines too; skipping them here
                # keeps the parser below dealing only with real data lines.
                if not raw:
                    continue
                piece = _delta_text(raw)
                if not piece:
                    continue
                if not pieces:
                    # Time to first token: the part of the gap the model is responsible
                    # for, as opposed to the ASR and TTS phases around it.
                    first_token_ms = (time.perf_counter() - started) * 1000.0
                pieces.append(piece)
                length += len(piece)
                # Stop as soon as there is more text than the device will be allowed to
                # speak. Everything after this point would be discarded by the cap anyway,
                # so continuing to receive it only delays the reply.
                if length >= config.reply_max_chars:
                    break
    except requests.RequestException as exc:
        # Chain so the underlying socket or timeout error survives into the log.
        raise LlmClientError(f"llm request to {url} failed: {exc}") from exc

    # Stripped because chat templates commonly open the reply with a newline. That
    # whitespace is silent, but it is not free: it counts against the cap below, and
    # payload-service derives est_ms from the character count, so leaving it in would
    # lengthen the mic gate for speech the device never makes.
    reply = "".join(pieces).strip()
    if not reply:
        # A stream that carried no content means the model produced nothing to say. There
        # is no sensible utterance to fall back on -- speaking a canned apology would hide
        # a broken deploy behind plausible behaviour -- so this is an error.
        raise LlmClientError("llm stream produced no text")
    total_ms = (time.perf_counter() - started) * 1000.0
    logger.info(
        "llm reply: chars=%d first_token_ms=%.0f total_ms=%.0f",
        len(reply),
        first_token_ms,
        total_ms,
    )
    # The cap is applied once more here because the loop stops only AFTER the piece that
    # crossed the limit has been appended, so the accumulated text can overshoot by up to
    # one token's worth of characters.
    return reply[: config.reply_max_chars]
