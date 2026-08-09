"""GWY-P4-01 -- tts_client wrapper tests + variants."""

import pytest
import requests

from xbrain.p4_agent.ai_client import tts_client


pytestmark = pytest.mark.no_device


# --- _count_sentences (pure) ----------------------------------------

def test_count_sentences_minimum_is_one():
    assert tts_client._count_sentences("你好") == 1
    assert tts_client._count_sentences("") == 1  # even empty -> 1


def test_count_sentences_chinese_terminators():
    assert tts_client._count_sentences("你好。再见。") == 2
    assert tts_client._count_sentences("好吗？好的！走。") == 3


# --- speak: fake payload-service ------------------------------------

class _FakeResponse:
    def __init__(self, status_code=200, body=None, text=""):
        self.status_code = status_code
        self._body = body
        self.text = text or (str(body) if body else "")

    def json(self):
        if self._body is None:
            raise ValueError("not json")
        return self._body


def test_speak_returns_est_ms_from_response(monkeypatch):
    captured = {}

    def _fake_post(url, **kw):
        captured["url"] = url
        captured["json"] = kw.get("json")
        return _FakeResponse(200, {"est_ms": 2500})

    monkeypatch.setattr(tts_client.requests, "post", _fake_post)
    est = tts_client.speak(
        "http://127.0.0.1:18080", "你好机器人",
        timeout_s=5.0,
    )
    assert est == 2500.0
    assert captured["url"] == "http://127.0.0.1:18080/tts"
    assert captured["json"] == {"text": "你好机器人"}


def test_speak_falls_back_to_sentence_estimate_when_no_est_ms(monkeypatch):
    """VARIANT: if payload-service does not include est_ms in response,
    speak() falls back to sentence-count * per_sentence budget."""
    def _fake_post(url, **kw):
        # Note: no est_ms field.
        return _FakeResponse(200, {"ok": True})

    monkeypatch.setattr(tts_client.requests, "post", _fake_post)
    est = tts_client.speak(
        "http://x", "第一句。第二句。第三句。",
        timeout_s=1.0,
        est_ms_per_sentence=1000.0,
    )
    assert est == 3000.0


# --- Variants: empty text / connect / non-200 / bad body -----------

def test_speak_rejects_empty_text(monkeypatch):
    """VARIANT: empty text is a caller bug (why POST /tts with nothing
    to say?). The wrapper refuses BEFORE the network call so a wedged
    payload-service does not delay the error."""
    monkeypatch.setattr(tts_client.requests, "post",
                        lambda *a, **k: pytest.fail("should not POST"))
    with pytest.raises(tts_client.TtsClientError):
        tts_client.speak("http://x", "", timeout_s=1.0)
    with pytest.raises(tts_client.TtsClientError):
        tts_client.speak("http://x", "   ", timeout_s=1.0)


def test_speak_connect_failure_maps_to_tts_error(monkeypatch):
    def _fake_post(url, **kw):
        raise requests.ConnectionError("refused")

    monkeypatch.setattr(tts_client.requests, "post", _fake_post)
    with pytest.raises(tts_client.TtsClientError):
        tts_client.speak("http://x", "hi", timeout_s=1.0)


def test_speak_non_200_error_contains_status(monkeypatch):
    def _fake_post(url, **kw):
        return _FakeResponse(503, None, text="service unavailable")

    monkeypatch.setattr(tts_client.requests, "post", _fake_post)
    with pytest.raises(tts_client.TtsClientError) as ei:
        tts_client.speak("http://x", "hi", timeout_s=1.0)
    assert "503" in str(ei.value)


def test_speak_bad_est_ms_type_raises(monkeypatch):
    """VARIANT: est_ms is a string (impl bug in payload). Must raise
    with a clear message; NOT silently coerce."""
    def _fake_post(url, **kw):
        return _FakeResponse(200, {"est_ms": "not-a-number"})

    monkeypatch.setattr(tts_client.requests, "post", _fake_post)
    with pytest.raises(tts_client.TtsClientError) as ei:
        tts_client.speak("http://x", "hi", timeout_s=1.0)
    assert "est_ms" in str(ei.value)
