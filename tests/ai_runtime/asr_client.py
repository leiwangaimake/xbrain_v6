"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: asr_client.py
Brief: Client for asr-service's OpenAI-compatible transcription endpoint.

Description:
  Turns one VAD-segmented utterance into text by calling asr-service. The plan's whitelist
  admits requests as an HTTP client (section 11); what is written from scratch is this
  wrapper -- shaping the multipart body, wrapping raw PCM as a wav so the service does not
  have to be told the geometry, and normalizing every failure into one type the turn loop
  can act on.

  Why a wav envelope rather than the raw-PCM path: asr-service accepts both (it sniffs the
  RIFF magic and otherwise reads a sample_rate form field), but a wav carries its own rate
  and width, so the two processes cannot silently disagree about them. A raw upload whose
  declared rate is wrong is not an error anywhere -- it decodes, and it transcribes to
  nonsense at the wrong speed. Paying 44 bytes to make that unrepresentable is trivially
  worth it, and the header is built here with the stdlib wave module (whitelisted).

  Why synchronous: requests is sync, and the turn loop is async. Rather than pull in an
  async HTTP client, the caller runs this on a worker thread with asyncio.to_thread -- the
  same discipline payload-service uses for device writes and recognizer.py for decodes.
  This module therefore stays a plain function with no event-loop coupling at all, which
  is also what makes it testable against a local http.server with no async test harness.

  A failed transcription is not fatal to the session: the turn loop is expected to log it,
  skip the turn and go back to listening, because the person can simply repeat themselves.
  That is why every failure is one exception type rather than several.
"""
from __future__ import annotations

import io
import logging
import wave

import requests

from .config import AiRuntimeConfig

logger = logging.getLogger("ai_runtime.asr")

# The mic plane's format, matching payload-service's WS /mic header and the VAD's frames.
_SAMPLE_RATE = 16000
_CHANNELS = 1
_SAMPLE_WIDTH = 2

# asr-service's OpenAI-compatible route (plan section 5.3).
_TRANSCRIBE_PATH = "/v1/audio/transcriptions"

# Sent as the multipart filename. The service ignores it, but it is what appears in an
# access log or a packet capture, so a name that says which process produced the audio is
# worth more than a generic one.
_UPLOAD_NAME = "utterance.wav"

# Field name the endpoint reads the audio from; fixed by the OpenAI schema.
_FILE_FIELD = "file"

# The endpoint accepts and ignores these for OpenAI compatibility. They are sent anyway so
# that pointing this client at a real OpenAI-compatible server needs no code change --
# there, the language hint measurably improves Chinese recognition.
_MODEL_FIELD = "model"
_LANGUAGE_FIELD = "language"
_LANGUAGE = "zh"


class AsrClientError(RuntimeError):
    """Raised when an utterance could not be transcribed.

    House rule bans bare Exception. One type covers the connection failing, the service
    answering non-200 and the body not being the expected json, because the turn loop's
    response to all three is identical -- log it, abandon this turn, keep listening -- and
    splitting them would invite handling that does not differ. RuntimeError is the base
    because these are operational faults, not bad arguments.
    """


def pcm_to_wav(pcm: bytes) -> bytes:
    """Wrap raw 16 kHz mono s16le PCM in a wav container.

    Args:
        pcm: the raw sample bytes, as captured from WS /mic.

    Returns:
        A complete in-memory wav file with a correct RIFF header.

    The stdlib wave module writes the header rather than this module packing one by hand:
    it is on the plan's whitelist, and a hand-packed RIFF header is a classic source of
    off-by-one chunk sizes that produce a file most readers accept and one rejects. The
    from-scratch part of this service is the audio pipeline, not the container format.

    The file is built in memory because it exists only to be the body of one HTTP request;
    writing it to disk would add an I/O round trip and a temporary file to clean up for
    every utterance of every turn.
    """
    buffer = io.BytesIO()
    # The context manager closes the writer, which is what patches the RIFF and data chunk
    # sizes into the header -- read the buffer before that and the sizes are still zero.
    with wave.open(buffer, "wb") as writer:
        writer.setnchannels(_CHANNELS)
        writer.setsampwidth(_SAMPLE_WIDTH)
        writer.setframerate(_SAMPLE_RATE)
        writer.writeframes(pcm)
    return buffer.getvalue()


def transcribe(config: AiRuntimeConfig, pcm: bytes) -> str:
    """Send one utterance to asr-service and return the recognized text.

    Args:
        config: supplies the service base URL and the HTTP timeout.
        pcm: raw 16 kHz mono s16le PCM for exactly one utterance.

    Returns:
        The recognized text, which is legitimately an empty string when the segment held
        no speech -- a VAD false trigger is a normal event, not a failure.

    Raises:
        AsrClientError: if the request fails, the service answers non-200, or the response
            body is not the documented {"text": ...} shape.

    Blocking: this makes a network call and must be run on a worker thread, never directly
    on the event loop (module docstring).
    """
    url = config.asr_url.rstrip("/") + _TRANSCRIBE_PATH
    # requests builds the multipart body from this triple: (filename, content, mime type).
    files = {_FILE_FIELD: (_UPLOAD_NAME, pcm_to_wav(pcm), "audio/wav")}
    data = {_MODEL_FIELD: config.llm_model, _LANGUAGE_FIELD: _LANGUAGE}
    try:
        response = requests.post(url, files=files, data=data, timeout=config.http_timeout_s)
    except requests.RequestException as exc:
        # Covers connect, read and timeout failures alike; the cause is chained so the
        # underlying socket error is still in the traceback.
        raise AsrClientError(f"asr request to {url} failed: {exc}") from exc
    if response.status_code != 200:
        # The body is included because asr-service puts its reason there (a malformed
        # upload answers 400 with the specific complaint), and without it the operator sees
        # only a status code.
        raise AsrClientError(f"asr returned {response.status_code}: {response.text[:200]}")
    try:
        payload = response.json()
        text = payload["text"]
    except (ValueError, KeyError, TypeError) as exc:
        # A 200 whose body is not the agreed shape means this client is pointed at
        # something that is not asr-service -- a wrong port is the usual cause, and saying
        # so beats a KeyError from deep in the turn loop.
        raise AsrClientError(f"asr response was not {{'text': ...}}: {response.text[:200]}") from exc
    return text
