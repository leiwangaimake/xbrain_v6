"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_llm_client.py
Brief: ai_client tests -- llm client

Description:
GWY-P4-01 -- llm_client wrapper tests + variants.
"""


import json
from typing import Iterator, List

import pytest
import requests

from xbrain.p4_agent.ai_client import llm_client


pytestmark = pytest.mark.no_device


# --- _delta_text: SSE line parser (pure function) --------------------

def test_delta_text_extracts_content_from_data_line():
    line = 'data: {"choices":[{"delta":{"content":"你好"}}]}'
    assert llm_client._delta_text(line) == "你好"


def test_delta_text_ignores_non_data_lines():
    # SSE comment; keep-alive; separator; whatever.
    assert llm_client._delta_text("event: something") == ""
    assert llm_client._delta_text("") == ""
    assert llm_client._delta_text(": keep-alive") == ""


def test_delta_text_ignores_done_sentinel():
    assert llm_client._delta_text("data: [DONE]") == ""


def test_delta_text_role_only_chunk_yields_empty():
    """Role-only opening chunk: {'choices':[{'delta':{'role':'assistant'}}]}.
    No content -> return empty, do NOT raise."""
    line = 'data: {"choices":[{"delta":{"role":"assistant"}}]}'
    assert llm_client._delta_text(line) == ""


def test_delta_text_malformed_json_is_swallowed():
    """A malformed chunk mid-stream must not kill the whole reply.
    Losing one token is recoverable; raising would drop everything."""
    assert llm_client._delta_text("data: {broken}") == ""


# --- _build_messages ------------------------------------------------

def test_build_messages_no_system_prompt_omits_system_turn():
    msgs = llm_client._build_messages("", "你好")
    assert msgs == [{"role": "user", "content": "你好"}]


def test_build_messages_system_prompt_precedes_user():
    msgs = llm_client._build_messages("You are a robot.", "hello")
    assert msgs == [
        {"role": "system", "content": "You are a robot."},
        {"role": "user", "content": "hello"},
    ]


# --- complete: full SSE roundtrip (fake requests.post) --------------

class _FakeStreamResponse:
    """Fakes requests.post(stream=True).__enter__(). Yields the fake
    line list from iter_lines and exposes status_code + text."""

    def __init__(self, status_code=200, lines=None, text=""):
        self.status_code = status_code
        self._lines: List[str] = lines or []
        self.text = text
        self.encoding = None  # requests exposes this attribute

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def iter_lines(self, decode_unicode=False):
        for line in self._lines:
            yield line


def _build_sse_lines(chunks: List[str]) -> List[str]:
    """Compose an SSE line list where each chunk becomes one 'data: '
    payload with content=chunk. Terminates with '[DONE]'."""
    out = []
    for c in chunks:
        payload = {"choices": [{"delta": {"content": c}}]}
        out.append("data: " + json.dumps(payload, ensure_ascii=False))
    out.append("data: [DONE]")
    return out


def test_complete_streams_and_returns_full_text(monkeypatch):
    lines = _build_sse_lines(["机", "器", "人", "你", "好"])

    def _fake_post(url, **kw):
        return _FakeStreamResponse(200, lines)

    monkeypatch.setattr(llm_client.requests, "post", _fake_post)
    out = llm_client.complete(
        "http://x", "hi",
        timeout_s=5.0, model="qwen-2.5-3b",
        max_tokens=50, reply_max_chars=100,
    )
    assert out == "机器人你好"


def test_complete_caps_at_reply_max_chars(monkeypatch):
    """VARIANT: caller sets reply_max_chars=3; stream should be cut
    once accumulated content >= 3 chars, and the return value must be
    <= 3 chars even if the LAST piece pushed past the cap."""
    lines = _build_sse_lines(["ab", "cd", "ef", "gh"])

    def _fake_post(url, **kw):
        return _FakeStreamResponse(200, lines)

    monkeypatch.setattr(llm_client.requests, "post", _fake_post)
    out = llm_client.complete(
        "http://x", "hi",
        timeout_s=5.0, model="m",
        max_tokens=50, reply_max_chars=3,
    )
    assert len(out) <= 3


def test_complete_utf8_encoding_pin(monkeypatch):
    """* SSE UTF-8 pin -- llama-server sends text/event-stream without
    a charset; requests would default to latin-1 and mojibake Chinese.
    The wrapper sets encoding='utf-8' explicitly. This test verifies."""
    lines = _build_sse_lines(["中文回复"])

    captured = {}

    def _fake_post(url, **kw):
        r = _FakeStreamResponse(200, lines)
        captured["response"] = r
        return r

    monkeypatch.setattr(llm_client.requests, "post", _fake_post)
    out = llm_client.complete(
        "http://x", "hi",
        timeout_s=5.0, model="m",
        reply_max_chars=100,
    )
    assert out == "中文回复"
    assert captured["response"].encoding == "utf-8", \
        "wrapper must set encoding='utf-8' explicitly"


# --- Variants: connect / non-200 / empty stream ---------------------

def test_complete_connect_failure_maps_to_llm_error(monkeypatch):
    def _fake_post(url, **kw):
        raise requests.ConnectionError("refused")

    monkeypatch.setattr(llm_client.requests, "post", _fake_post)
    with pytest.raises(llm_client.LlmClientError):
        llm_client.complete("http://x", "hi", timeout_s=1.0, model="m")


def test_complete_non_200_maps_to_llm_error(monkeypatch):
    def _fake_post(url, **kw):
        return _FakeStreamResponse(500, [], text="internal error")

    monkeypatch.setattr(llm_client.requests, "post", _fake_post)
    with pytest.raises(llm_client.LlmClientError) as ei:
        llm_client.complete("http://x", "hi", timeout_s=1.0, model="m")
    assert "500" in str(ei.value)


def test_complete_empty_stream_raises(monkeypatch):
    """VARIANT: stream terminates without any content -> LlmClientError.
    A stub that returned "" would hide a broken deploy behind
    plausible behaviour (silent robot); the wrapper must FAIL LOUDLY."""
    lines = ["data: [DONE]"]

    def _fake_post(url, **kw):
        return _FakeStreamResponse(200, lines)

    monkeypatch.setattr(llm_client.requests, "post", _fake_post)
    with pytest.raises(llm_client.LlmClientError) as ei:
        llm_client.complete("http://x", "hi", timeout_s=1.0, model="m")
    assert "no text" in str(ei.value)
