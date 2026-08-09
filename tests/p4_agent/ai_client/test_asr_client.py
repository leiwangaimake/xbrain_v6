"""GWY-P4-01 -- asr_client wrapper tests + variants."""

import io
import wave

import pytest
import requests

from xbrain.p4_agent.ai_client import asr_client


pytestmark = pytest.mark.no_device


# --- pcm16_to_wav ----------------------------------------------------

def test_pcm16_to_wav_produces_riff_header():
    """POSITIVE: 200 ms of silence -> a valid WAV whose header
    correctly describes 16 kHz mono s16le."""
    pcm = b"\x00" * (16_000 * 2 * 200 // 1000)   # 200 ms silence
    wav = asr_client.pcm16_to_wav(pcm)
    assert wav[:4] == b"RIFF"
    assert wav[8:12] == b"WAVE"
    # Read it back with the stdlib wave module to prove it round-trips.
    with wave.open(io.BytesIO(wav), "rb") as r:
        assert r.getnchannels() == 1
        assert r.getsampwidth() == 2
        assert r.getframerate() == 16_000
        # Frames = 200 ms * 16 kHz = 3200 frames.
        assert r.getnframes() == 3200


def test_pcm16_to_wav_empty_pcm_is_valid_empty_wav():
    """Edge case: empty PCM produces a valid WAV with zero frames.
    The wrapper does NOT reject empty input -- empty utterances are
    a valid VAD false-trigger outcome and the caller decides what to
    do about them."""
    wav = asr_client.pcm16_to_wav(b"")
    assert wav[:4] == b"RIFF"
    with wave.open(io.BytesIO(wav), "rb") as r:
        assert r.getnframes() == 0


# --- transcribe (against a stubbed asr-service) ---------------------
# We do NOT depend on a real asr-service being up. All tests below
# monkey-patch requests.post so they run offline and deterministically.

class _FakeResponse:
    def __init__(self, status_code=200, body=None, text=""):
        self.status_code = status_code
        self._body = body
        self.text = text if text else (str(body) if body else "")

    def json(self):
        if self._body is None:
            raise ValueError("not json")
        return self._body


def test_transcribe_returns_text_on_200(monkeypatch):
    captured = {}

    def _fake_post(url, **kw):
        captured["url"] = url
        captured["timeout"] = kw.get("timeout")
        return _FakeResponse(200, {"text": "你好机器人"})

    monkeypatch.setattr(asr_client.requests, "post", _fake_post)
    out = asr_client.transcribe(
        "http://127.0.0.1:18081",
        b"\x00" * 3200,
        timeout_s=5.0,
    )
    assert out == "你好机器人"
    assert captured["url"] == "http://127.0.0.1:18081/v1/audio/transcriptions"
    assert captured["timeout"] == 5.0


def test_transcribe_empty_text_is_valid_return(monkeypatch):
    """POSITIVE: an empty text field is a legal return (VAD false
    trigger). Callers must NOT treat it as an error, so the wrapper
    must return it unchanged."""
    def _fake_post(url, **kw):
        return _FakeResponse(200, {"text": ""})
    monkeypatch.setattr(asr_client.requests, "post", _fake_post)
    out = asr_client.transcribe("http://x", b"\x00" * 320, timeout_s=1.0)
    assert out == ""


# --- Variant: connect failure -> AsrClientError ---------------------

def test_transcribe_maps_connect_failure_to_asr_error(monkeypatch):
    def _fake_post(url, **kw):
        raise requests.ConnectionError("connection refused")
    monkeypatch.setattr(asr_client.requests, "post", _fake_post)
    with pytest.raises(asr_client.AsrClientError) as ei:
        asr_client.transcribe("http://x", b"\x00", timeout_s=1.0)
    assert "asr request" in str(ei.value)


def test_transcribe_maps_timeout_to_asr_error(monkeypatch):
    def _fake_post(url, **kw):
        raise requests.Timeout("read timeout")
    monkeypatch.setattr(asr_client.requests, "post", _fake_post)
    with pytest.raises(asr_client.AsrClientError):
        asr_client.transcribe("http://x", b"\x00", timeout_s=1.0)


# --- Variant: non-200 response -> AsrClientError with body ---------

def test_transcribe_non_200_error_contains_status_and_body(monkeypatch):
    def _fake_post(url, **kw):
        return _FakeResponse(400, None, text="bad wav header")
    monkeypatch.setattr(asr_client.requests, "post", _fake_post)
    with pytest.raises(asr_client.AsrClientError) as ei:
        asr_client.transcribe("http://x", b"\x00", timeout_s=1.0)
    assert "400" in str(ei.value)
    assert "bad wav header" in str(ei.value)


# --- Variant: 200 but wrong body shape (wrong port?) ---------------

def test_transcribe_wrong_body_shape_raises(monkeypatch):
    """VARIANT: pointing at a wrong port that returns HTML must
    raise AsrClientError, NOT a KeyError deep in the router."""
    def _fake_post(url, **kw):
        return _FakeResponse(200, {"unrelated": "html body"},
                             text="<html>...</html>")
    monkeypatch.setattr(asr_client.requests, "post", _fake_post)
    with pytest.raises(asr_client.AsrClientError) as ei:
        asr_client.transcribe("http://x", b"\x00", timeout_s=1.0)
    assert "not {'text'" in str(ei.value)


def test_transcribe_text_is_non_string_raises(monkeypatch):
    """VARIANT: 200 with {'text': 12345} (int) must raise. A stub that
    returns str(text) would pass this; the point is to catch a body
    where 'text' isn't a str -- an implementation bug."""
    def _fake_post(url, **kw):
        return _FakeResponse(200, {"text": 12345})
    monkeypatch.setattr(asr_client.requests, "post", _fake_post)
    with pytest.raises(asr_client.AsrClientError):
        asr_client.transcribe("http://x", b"\x00", timeout_s=1.0)


# --- Meta: this file is ai_client's only requests entry point ------

def test_requests_only_imported_by_ai_client(monkeypatch):
    """CLAUDE.md 4.1 forbids `import requests` outside
    xbrain/p4_agent/ai_client/. Static grep is done by CI; this test
    is a runtime canary that catches accidental sibling imports."""
    import pathlib
    p4 = pathlib.Path(__file__).parent.parent.parent.parent / "xbrain" / "p4_agent"
    for py in p4.rglob("*.py"):
        # ai_client/ is the allowed location.
        if "ai_client" in py.parts:
            continue
        src = py.read_text(encoding="utf-8")
        # Strip line-leading '#' comments so a comment mentioning the
        # anti-pattern (like this test's own docstring) does not fire.
        for line in src.splitlines():
            stripped = line.strip()
            if stripped.startswith("#"):
                continue
            assert "import requests" not in stripped, \
                "%s imports requests outside ai_client/" % py
