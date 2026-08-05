"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_clients.py
Brief: Unit tests for the asr, llm and payload REST clients against a local http.server.

Description:
  All three clients are synchronous by design, so they can be exercised against a real
  socket with no async harness at all: a stdlib http.server on an ephemeral port answers
  canned bodies, and the tests assert both what the client sent and what it made of the
  answer. Using a real server rather than a mocked requests keeps the multipart encoding,
  the SSE line framing and the status handling honestly in scope -- those are precisely
  the parts a mock would assume correct.
"""
from __future__ import annotations

import dataclasses
import io
import json
import threading
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from tests.ai_runtime.asr_client import AsrClientError, pcm_to_wav, transcribe
from tests.ai_runtime.config import AiRuntimeConfig
from tests.ai_runtime.llm_client import LlmClientError, _build_messages, _delta_text, complete
from tests.ai_runtime.payload_client import (
    MODE_FUNC1,
    PayloadClientError,
    _check_mic_header,
    _ws_url,
    ensure_mode,
    get_status,
    speak,
)


class _Handler(BaseHTTPRequestHandler):
    """Answers whatever the test put in server.routes, and records what it received."""

    def do_GET(self) -> None:
        self._serve()

    def do_POST(self) -> None:
        self._serve()

    def _serve(self) -> None:
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length) if length else b""
        self.server.received.append((self.path, body))
        route = self.server.routes.get(self.path, (404, b"no such route"))
        status, payload = route[0], route[1]
        # The content type is part of what a route declares, not an incidental header:
        # requests picks its fallback charset from it, so a test that served everything as
        # application/json would decode UTF-8 correctly for the wrong reason and hide the
        # latin-1 fallback that a real text/event-stream triggers.
        content_type = route[2] if len(route) > 2 else "application/json"
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt: str, *args) -> None:
        # Silence the stderr access log; the tests assert on server.received instead.
        pass


@pytest.fixture
def service():
    """A throwaway HTTP server on an ephemeral port, torn down after each test."""
    server = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
    server.routes = {}
    server.received = []
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


def _config(service, **overrides) -> AiRuntimeConfig:
    """A config whose three service URLs all point at the fake server."""
    url = f"http://127.0.0.1:{service.server_address[1]}"
    return dataclasses.replace(
        AiRuntimeConfig(), payload_url=url, asr_url=url, llm_url=url, **overrides
    )


def _json_route(obj) -> tuple:
    return (200, json.dumps(obj).encode("utf-8"))


def _sse(*pieces: str) -> bytes:
    """Build an OpenAI-style streaming body: one content delta per piece, then [DONE].

    ensure_ascii is off so non-ASCII content goes on the wire as raw UTF-8, which is what
    llama-server actually sends. The default would emit \\uXXXX escapes -- pure ASCII, and
    therefore immune to a wrong charset -- so a body built that way cannot detect a
    decoding fault no matter what content type it is served under.
    """
    lines = []
    for piece in pieces:
        lines.append(
            "data: "
            + json.dumps({"choices": [{"delta": {"content": piece}}]}, ensure_ascii=False)
        )
        lines.append("")
    lines.append("data: [DONE]")
    lines.append("")
    return "\n".join(lines).encode("utf-8")


def _sse_route(*pieces: str) -> tuple:
    """An SSE route labelled the way llama-server labels it: no charset parameter.

    Reproducing the bare "text/event-stream" matters because that is precisely what makes
    requests fall back to ISO-8859-1, so this is what puts the client's own utf-8 pin
    under test rather than assuming it.
    """
    return (200, _sse(*pieces), "text/event-stream")


# -- asr_client -------------------------------------------------------------


def test_pcm_to_wav_declares_the_mic_geometry():
    pcm = b"\x01\x02" * 160
    with wave.open(io.BytesIO(pcm_to_wav(pcm)), "rb") as reader:
        assert reader.getnchannels() == 1
        assert reader.getsampwidth() == 2
        assert reader.getframerate() == 16000
        assert reader.readframes(reader.getnframes()) == pcm


def test_transcribe_uploads_a_wav_and_returns_the_text(service):
    service.routes["/v1/audio/transcriptions"] = _json_route({"text": "前进"})

    assert transcribe(_config(service), b"\x00\x00" * 160) == "前进"

    path, body = service.received[0]
    assert path == "/v1/audio/transcriptions"
    # The upload is a wav, not raw PCM, so the two processes cannot disagree on the rate.
    assert b"RIFF" in body
    assert b'name="file"' in body


def test_transcribe_accepts_an_empty_transcript(service):
    # A VAD false trigger legitimately transcribes to nothing; that is not a failure.
    service.routes["/v1/audio/transcriptions"] = _json_route({"text": ""})
    assert transcribe(_config(service), b"\x00\x00" * 160) == ""


def test_transcribe_reports_a_non_200(service):
    service.routes["/v1/audio/transcriptions"] = (400, b"bad upload")
    with pytest.raises(AsrClientError, match="400"):
        transcribe(_config(service), b"\x00\x00" * 160)


def test_transcribe_reports_a_body_of_the_wrong_shape(service):
    service.routes["/v1/audio/transcriptions"] = _json_route({"result": "前进"})
    with pytest.raises(AsrClientError, match="text"):
        transcribe(_config(service), b"\x00\x00" * 160)


def test_transcribe_reports_an_unreachable_service():
    # Port 1 is reserved and nothing listens on it, so this is a connect failure.
    config = dataclasses.replace(AiRuntimeConfig(), asr_url="http://127.0.0.1:1", http_timeout_s=2.0)
    with pytest.raises(AsrClientError, match="failed"):
        transcribe(config, b"\x00\x00" * 160)


# -- llm_client -------------------------------------------------------------


def test_an_empty_system_prompt_is_omitted_entirely():
    # The empty prompt is constructed EXPLICITLY rather than taken from the default. The
    # behaviour under test is "empty means no system message at all, not an empty one" --
    # some chat templates render an empty system turn as a visible block -- and that is a
    # property of _build_messages, not of whatever the shipped default happens to be. The
    # earlier version read the default and so broke the moment a default prompt was added,
    # reporting a failure in a behaviour that had not changed.
    config = dataclasses.replace(AiRuntimeConfig(), llm_system_prompt="")
    assert _build_messages(config, "你好") == [{"role": "user", "content": "你好"}]


def test_the_default_system_prompt_is_sent_and_constrains_form():
    # The shipped default is not empty, and what it must contain is a length bound and a
    # ban on markup -- the reply is read aloud verbatim by the device, so asterisks and
    # numbered lists are spoken, and an unbounded reply becomes an utterance that cannot be
    # interrupted. Asserted by property rather than by exact wording so the text can be
    # improved without breaking the test.
    prompt = AiRuntimeConfig().llm_system_prompt
    assert prompt, "the default system prompt must not be empty"
    assert _build_messages(AiRuntimeConfig(), "你好")[0]["role"] == "system"
    assert "40" in prompt, "the reply length bound must be stated to the model"
    assert "星号" in prompt or "标记" in prompt, "spoken output must ban markup"


def test_a_configured_system_prompt_leads_the_messages():
    config = dataclasses.replace(AiRuntimeConfig(), llm_system_prompt="be brief")
    assert _build_messages(config, "你好")[0] == {"role": "system", "content": "be brief"}


@pytest.mark.parametrize(
    "line",
    [
        "",
        ": keep-alive",
        "data: [DONE]",
        'data: {"choices": [{"delta": {"role": "assistant"}}]}',
        'data: {"choices": [{"finish_reason": "stop"}]}',
        "data: not json at all",
        "data: {}",
    ],
)
def test_non_content_sse_lines_yield_no_text(line):
    # Anything the parser cannot use must be skipped, never raised on: an unrecognized
    # line would otherwise turn a harmless protocol addition into a dead turn.
    assert _delta_text(line) == ""


def test_a_content_delta_line_yields_its_text():
    assert _delta_text('data: {"choices": [{"delta": {"content": "好"}}]}') == "好"
    # The space after the colon is optional in SSE.
    assert _delta_text('data:{"choices": [{"delta": {"content": "好"}}]}') == "好"


def test_complete_assembles_the_streamed_pieces(service):
    service.routes["/v1/chat/completions"] = _sse_route("你", "好", "呀")

    assert complete(_config(service), "在吗") == "你好呀"

    path, body = service.received[0]
    assert path == "/v1/chat/completions"
    assert json.loads(body)["stream"] is True


def test_a_multibyte_reply_survives_a_stream_with_no_declared_charset(service):
    # Regression: llama-server labels the stream "text/event-stream" with no charset, and
    # requests answers that with an ISO-8859-1 fallback. Undetected, every Chinese reply
    # reached the device as mojibake -- and the device's TTS reads whatever it is given
    # aloud, so the corruption was audible rather than merely ugly. A long line is used
    # because the damage is per byte: a short one can survive by luck.
    reply = "你好我是船舶智能助手请问需要我做什么"
    service.routes["/v1/chat/completions"] = _sse_route(*reply)

    assert complete(_config(service, reply_max_chars=200), "你是谁") == reply


def test_complete_stops_at_the_reply_cap(service):
    service.routes["/v1/chat/completions"] = _sse_route("abc", "def", "ghi")
    # The loop stops only after the piece that crossed the cap, so the accumulated text
    # overshoots and the final slice is what enforces the limit.
    assert complete(_config(service, reply_max_chars=5), "hi") == "abcde"


def test_complete_reports_a_stream_that_produced_no_text(service):
    service.routes["/v1/chat/completions"] = _sse_route()
    with pytest.raises(LlmClientError, match="no text"):
        complete(_config(service), "hi")


def test_complete_reports_a_non_200(service):
    service.routes["/v1/chat/completions"] = (503, b"model loading")
    with pytest.raises(LlmClientError, match="503"):
        complete(_config(service), "hi")


# -- payload_client ---------------------------------------------------------


def test_ws_url_takes_the_websocket_scheme_from_the_base_url():
    assert _ws_url(dataclasses.replace(AiRuntimeConfig(), payload_url="http://h:1/"), "/mic") == "ws://h:1/mic"
    assert _ws_url(dataclasses.replace(AiRuntimeConfig(), payload_url="https://h:1"), "/mic") == "wss://h:1/mic"


def test_ws_url_serves_both_audio_routes_off_one_base():
    # /mic and /play are the same host and scheme with a different path, so a base URL that
    # is right for one must be right for the other -- the failure this guards against is a
    # 功能2 session that reaches the microphone and not the loudspeaker.
    config = dataclasses.replace(AiRuntimeConfig(), payload_url="http://h:1")
    assert _ws_url(config, "/play") == "ws://h:1/play"


def test_ws_url_rejects_a_base_url_with_no_scheme():
    with pytest.raises(PayloadClientError, match="scheme"):
        _ws_url(dataclasses.replace(AiRuntimeConfig(), payload_url="127.0.0.1:18080"), "/mic")


def test_the_expected_mic_header_is_accepted():
    _check_mic_header(
        json.dumps(
            {
                "encoding": "s16le",
                "sample_rate": 16000,
                "channels": 1,
                "frame_ms": 20,
                "frame_bytes": 640,
            }
        )
    )


@pytest.mark.parametrize(
    "field, value",
    [
        ("encoding", "f32le"),
        ("sample_rate", 8000),
        ("channels", 2),
        ("frame_bytes", 320),
    ],
)
def test_a_mic_header_that_differs_in_any_field_is_refused(field, value):
    # Each of these silently breaks a different downstream stage rather than raising on
    # its own, which is why every field is checked and not just the frame size.
    header = {"encoding": "s16le", "sample_rate": 16000, "channels": 1, "frame_bytes": 640}
    header[field] = value
    with pytest.raises(PayloadClientError, match=field):
        _check_mic_header(json.dumps(header))


@pytest.mark.parametrize("raw", [b"binary frame", "not json", "[1, 2]"])
def test_a_mic_header_that_is_not_a_json_object_is_refused(raw):
    with pytest.raises(PayloadClientError):
        _check_mic_header(raw)


def test_ensure_mode_reports_an_actual_switch(service):
    service.routes["/mode"] = _json_route({"ok": True, "mode": "func1", "previous": "idle"})

    assert ensure_mode(_config(service), MODE_FUNC1) is True
    assert json.loads(service.received[0][1]) == {"mode": "func1"}


def test_ensure_mode_treats_a_409_as_already_there(service):
    # POST /mode raises its only 409 for re-entry of the active mode, so on this route
    # that status means success for a caller whose goal is to BE in the mode.
    service.routes["/mode"] = (409, b"already in func1")
    assert ensure_mode(_config(service), MODE_FUNC1) is False


def test_ensure_mode_reports_any_other_status(service):
    service.routes["/mode"] = (503, b"device down")
    with pytest.raises(PayloadClientError, match="503"):
        ensure_mode(_config(service), MODE_FUNC1)


def test_speak_returns_the_playback_estimate(service):
    service.routes["/tts"] = _json_route({"ok": True, "est_ms": 1340})

    assert speak(_config(service), "你好") == 1340
    assert json.loads(service.received[0][1]) == {"voice": 0, "text": "你好"}


def test_speak_refuses_an_answer_with_no_estimate(service):
    # A missing est_ms defaulted to zero would reopen the mic instantly and feed the
    # device's own speech back into the VAD, so it has to be an error.
    service.routes["/tts"] = _json_route({"ok": True})
    with pytest.raises(PayloadClientError, match="est_ms"):
        speak(_config(service), "你好")


def test_speak_reports_a_mode_refusal(service):
    service.routes["/tts"] = (409, b"tts not allowed in mode deter")
    with pytest.raises(PayloadClientError, match="409"):
        speak(_config(service), "你好")


def test_get_status_returns_the_document(service):
    body = {"mode": "func1", "device": {"audio_connected": True, "lights_connected": True}}
    service.routes["/status"] = _json_route(body)
    assert get_status(_config(service)) == body


def test_get_status_reports_a_body_that_is_not_an_object(service):
    service.routes["/status"] = (200, b"[]")
    with pytest.raises(PayloadClientError, match="object"):
        get_status(_config(service))
