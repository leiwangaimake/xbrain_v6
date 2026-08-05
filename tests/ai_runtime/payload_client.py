"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: payload_client.py
Brief: Client for payload-service -- REST control (/mode /tts /status) and WS /mic audio.

Description:
  AI_runtime's only door to the hardware. Invariant R1 makes payload-service the sole
  owner of the device sockets, so nothing here speaks 8519/8529: this module talks the
  loopback HTTP and WebSocket surface that payload-service publishes (plan sections 5.1
  and 5.2) and lets that process own the wire. Written from scratch per R0; requests and
  websockets are the whitelisted transport underneath.

  Why the control calls are synchronous and the mic is async: they are different shapes
  of traffic, not a style inconsistency. /mode, /tts and /status are one-shot
  request/response actions that happen a handful of times per turn, so they stay plain
  blocking functions run with asyncio.to_thread -- the same discipline asr_client and
  llm_client use. WS /mic is a continuous 50-frame-per-second stream that must live on
  the event loop for its whole session, so it is a native async iterator. Forcing either
  into the other's shape would cost more than it saved.

  Why /play arrives only now: the 功能1 reply is spoken by the DEVICE's own TTS via
  POST /tts, not streamed as PCM (plan section 6), so /play had no caller until M5 added
  the 功能2 intercom. It appears here with that caller, not before it.

  Mode handling: POST /mode answers 409 when asked to re-enter the mode already active.
  For a startup path that means "already where you wanted to be", not a failure, so
  ensure_mode treats that one status specially and reports whether a switch actually
  happened. Doing it by catching the 409 rather than by reading /status first keeps it to
  one round trip and leaves no window in which another client could change the mode
  between the check and the switch.

  Failure policy matches the other two clients: one exception type, because the turn
  loop's answer to a payload fault is the same wherever it comes from.
"""
from __future__ import annotations

import json
import logging
from typing import Any, Dict

import requests
from websockets.asyncio.client import connect
from websockets.exceptions import (
    ConnectionClosed,
    ConnectionClosedOK,
    InvalidStatus,
    WebSocketException,
)

from .config import AiRuntimeConfig

logger = logging.getLogger("ai_runtime.payload")

# payload-service's control-plane routes (plan section 5.1) and its mic data plane
# (section 5.2). Named once here so a route rename is a one-line edit rather than a
# search through string literals.
_MODE_PATH = "/mode"
_TTS_PATH = "/tts"
_STATUS_PATH = "/status"
_MIC_PATH = "/mic"
_PLAY_PATH = "/play"

# The three modes AI_runtime drives. deter is deliberately absent: that mode is armed by an
# operator calling POST /deter directly and its loop runs inside payload-service, so this
# process never sets it and a constant for it would be a reserved name with no caller.
MODE_IDLE = "idle"
MODE_FUNC1 = "func1"
MODE_FUNC2 = "func2"

# Only two status codes need to be distinguished by name: everything that is not 200 is
# an error, except the 409 from POST /mode, which means the target mode was already
# active (see the module docstring).
_HTTP_OK = 200
_HTTP_CONFLICT = 409

# The mic format this process is built around (plan section 7). The VAD's millisecond
# knobs are converted to frame counts on the assumption of 20 ms frames at 16 kHz, and
# vad.FRAME_BYTES hard-codes the resulting 640-byte frame, so a server that declared any
# other geometry would silently make every VAD timing wrong. These constants exist to
# turn that into a loud failure at connect time instead.
_MIC_ENCODING = "s16le"
_MIC_SAMPLE_RATE = 16000
_MIC_CHANNELS = 1
_MIC_FRAME_BYTES = 640


class PayloadClientError(RuntimeError):
    """Raised when payload-service could not carry out a request.

    House rule bans bare Exception. Covers the connection failing, a non-200 answer, a
    body that is not the documented shape, and a mic socket that is refused or drops --
    all of which leave the turn loop with the same options (log, and either abandon the
    turn or end the session), so splitting them would invite handling that does not
    differ. RuntimeError is the base because these are operational faults.
    """


def _url(config: AiRuntimeConfig, path: str) -> str:
    """Join the configured payload base URL with a route path."""
    return config.payload_url.rstrip("/") + path


def _post(config: AiRuntimeConfig, path: str, body: Dict[str, Any]) -> requests.Response:
    """POST json to payload-service and return the raw response.

    Args:
        config: supplies the base URL and the HTTP timeout.
        path: the route path, e.g. "/tts".
        body: the json body to send.

    Returns:
        The response, whatever its status -- status interpretation is left to the caller
        because it genuinely differs per route (a 409 is fatal on /tts but expected on
        /mode), and a helper that collapsed every non-200 into one error would take that
        distinction away from the only place that can make it.

    Raises:
        PayloadClientError: if the request never got an answer at all.
    """
    url = _url(config, path)
    try:
        return requests.post(url, json=body, timeout=config.http_timeout_s)
    except requests.RequestException as exc:
        # Transport-level failure: payload-service is not running, or the loopback call
        # timed out. Chained so the underlying socket error survives into the log.
        raise PayloadClientError(f"payload request to {url} failed: {exc}") from exc


def _json_body(response: requests.Response, what: str) -> Dict[str, Any]:
    """Parse a 200 response body as a json object, or raise.

    Args:
        response: a response already known to be 200.
        what: the operation name, used only in the error message.

    Returns:
        The decoded json object.

    Raises:
        PayloadClientError: if the body is not a json object. A 200 whose body is not the
            agreed shape means this client is pointed at something that is not
            payload-service -- a wrong port is the usual cause, and saying so beats a
            TypeError from deep in the turn loop.
    """
    try:
        payload = response.json()
    except ValueError as exc:
        raise PayloadClientError(f"{what} response was not json: {response.text[:200]}") from exc
    if not isinstance(payload, dict):
        raise PayloadClientError(f"{what} response was not a json object: {response.text[:200]}")
    return payload


def ensure_mode(config: AiRuntimeConfig, mode: str) -> bool:
    """Put payload-service into the given mode, tolerating it already being there.

    Args:
        config: supplies the base URL and the HTTP timeout.
        mode: the target mode, one of the MODE_* constants.

    Returns:
        True if this call performed the switch, False if the mode was already active.
        The caller uses that to decide whether it is responsible for restoring the
        previous mode on the way out -- a process that did not change the mode should
        not reset it either.

    Raises:
        PayloadClientError: if the request fails or is answered with anything other than
            200 or the expected 409.

    Blocking: this makes a network call and must be run on a worker thread.
    """
    response = _post(config, _MODE_PATH, {"mode": mode})
    if response.status_code == _HTTP_CONFLICT:
        # POST /mode raises its only 409 for re-entry of the active mode, so this status
        # on this route unambiguously means "already in {mode}" -- which is success for a
        # caller whose goal is to BE in that mode, not to have changed it.
        logger.info("payload already in mode %s", mode)
        return False
    if response.status_code != _HTTP_OK:
        raise PayloadClientError(
            f"payload mode {mode} returned {response.status_code}: {response.text[:200]}"
        )
    body = _json_body(response, "mode")
    logger.info("payload mode %s -> %s", body.get("previous"), body.get("mode"))
    return True


def speak(config: AiRuntimeConfig, text: str) -> int:
    """Make the device speak a line, and return its estimated playback duration.

    Args:
        config: supplies the base URL, the HTTP timeout and the TTS voice selector.
        text: the sentence to speak.

    Returns:
        est_ms, payload-service's estimate of how long the device will be talking.

    Raises:
        PayloadClientError: if the request fails, is refused, or the answer carries no
            usable est_ms.

    The estimate is the entire reason this returns a value. The device emits no
    "TTS finished" event (plan section 9), so the caller cannot wait for completion; it
    waits est_ms plus its own margin before reopening the mic gate. A missing or
    non-integer est_ms is therefore treated as an error rather than defaulted to zero: a
    zero would reopen the mic instantly and feed the device's own speech back into the
    VAD, which is precisely the failure the gate exists to prevent.

    Blocking: this makes a network call and must be run on a worker thread.
    """
    response = _post(config, _TTS_PATH, {"voice": config.tts_voice, "text": text})
    if response.status_code != _HTTP_OK:
        # 409 here means the mode does not allow a one-shot line, 503 that the audio link
        # is down; the body carries payload-service's own reason, so it is passed through
        # rather than replaced with a generic message.
        raise PayloadClientError(
            f"payload tts returned {response.status_code}: {response.text[:200]}"
        )
    body = _json_body(response, "tts")
    est_ms = body.get("est_ms")
    if not isinstance(est_ms, int):
        raise PayloadClientError(f"payload tts returned no est_ms: {response.text[:200]}")
    logger.info("tts spoken: chars=%d est_ms=%d", len(text), est_ms)
    return est_ms


def get_status(config: AiRuntimeConfig) -> Dict[str, Any]:
    """Fetch payload-service's status document.

    Args:
        config: supplies the base URL and the HTTP timeout.

    Returns:
        The decoded /status body (mode, device connectivity, lights, and the nulls this
        unit reports for volume and temperature). Returned as the raw dict rather than a
        typed DTO because the only use here is a startup precondition check -- reading
        two keys -- and a mirror class would add a second place to update every time
        payload-service's status grows a field.

    Raises:
        PayloadClientError: if the request fails or the answer is not a 200 json object.

    Blocking: this makes a network call and must be run on a worker thread.
    """
    url = _url(config, _STATUS_PATH)
    try:
        response = requests.get(url, timeout=config.http_timeout_s)
    except requests.RequestException as exc:
        raise PayloadClientError(f"payload request to {url} failed: {exc}") from exc
    if response.status_code != _HTTP_OK:
        raise PayloadClientError(
            f"payload status returned {response.status_code}: {response.text[:200]}"
        )
    return _json_body(response, "status")


def _ws_url(config: AiRuntimeConfig, path: str) -> str:
    """Derive a WebSocket URL on payload-service from the configured http base URL.

    Args:
        config: supplies the payload base URL.
        path: the websocket route, _MIC_PATH or _PLAY_PATH.

    Returns:
        The same host and port with the websocket scheme and the given path.

    Raises:
        PayloadClientError: if the base URL has no recognized http scheme.

    The scheme is rewritten rather than configured separately so there is ONE payload
    address to set. Two settings could disagree, and an audio stream pointed at a different
    process than the control calls is a fault that would show up only as strange audio.
    """
    base = config.payload_url.rstrip("/")
    for http_scheme, ws_scheme in (("https://", "wss://"), ("http://", "ws://")):
        if base.startswith(http_scheme):
            return ws_scheme + base[len(http_scheme):] + path
    raise PayloadClientError(f"payload url has no http scheme: {config.payload_url}")


def _check_mic_header(raw: Any) -> None:
    """Verify the server's opening /mic header describes the format this process assumes.

    Args:
        raw: the first message received on the socket; expected to be the json text frame
            payload-service sends before any audio (plan section 5.2).

    Raises:
        PayloadClientError: if the message is not that header, or declares any field
            differently from the _MIC_* constants.

    Every geometry field is checked, not just the frame size, because each one silently
    breaks a different downstream stage if it differs: the sample rate scales all the
    VAD's millisecond thresholds and would make the wav sent to ASR play at the wrong
    speed, the channel count and encoding would make the samples themselves nonsense, and
    the frame size would desynchronise the VAD's frame accounting. None of those raise on
    their own -- they just produce a system that mishears everything -- so this is the one
    place they can be caught, and it costs one comparison per session.
    """
    if not isinstance(raw, str):
        raise PayloadClientError("mic stream did not open with a text format header")
    try:
        header = json.loads(raw)
    except ValueError as exc:
        raise PayloadClientError(f"mic header is not valid json: {raw[:200]}") from exc
    if not isinstance(header, dict):
        raise PayloadClientError(f"mic header is not a json object: {raw[:200]}")
    expected = {
        "encoding": _MIC_ENCODING,
        "sample_rate": _MIC_SAMPLE_RATE,
        "channels": _MIC_CHANNELS,
        "frame_bytes": _MIC_FRAME_BYTES,
    }
    for field, want in expected.items():
        got = header.get(field)
        if got != want:
            raise PayloadClientError(
                f"mic header {field} is {got!r}, this client requires {want!r}"
            )


class MicStream:
    """Async iterator over payload-service's WS /mic plane, yielding 20 ms PCM frames.

    Used as an async context manager so the socket -- and therefore the device's
    recording state, which payload-service ties to this socket's lifetime -- is closed on
    every exit path including an exception. Iterating yields the raw 640-byte frames
    straight through to the VAD with no copy or reframing, because payload-service has
    already normalized them to a fixed cadence.

    Iteration ends (StopAsyncIteration) on a clean server close and raises
    PayloadClientError on an error close. That split is the client side of the server's
    close-code taxonomy: 1011 means the device audio link went down, which the turn loop
    must report, whereas a normal close is just the session ending.

    The frames must be consumed continuously for the whole session, including while the
    device is speaking. The turn loop drops them then rather than stopping the stream,
    because payload-service disarms the device mic when this socket closes and re-arming
    costs ~98 ms of lost speech at the front of the next utterance (plan section 7).
    """

    def __init__(self, config: AiRuntimeConfig) -> None:
        """Prepare a mic stream; no connection is made until the context is entered."""
        self._config = config
        self._url = _ws_url(config, _MIC_PATH)
        # Assigned in __aenter__. Kept as None until then so using the iterator without
        # entering the context fails immediately rather than half-working.
        self._connection: Any = None

    async def __aenter__(self) -> "MicStream":
        """Open the socket and validate the server's format header.

        Returns:
            self, ready to iterate.

        Raises:
            PayloadClientError: if the handshake is refused, the connection fails, or the
                opening header does not match this client's assumed format.

        A refusal arrives as an HTTP status on the handshake rather than a WebSocket
        close, because payload-service's mode gate rejects before accepting (section 5.2).
        That case is reported separately from a transport failure since its fix is
        different: the service is running and reachable, it is simply not in a mode where
        the mic is available.
        """
        try:
            # compression is disabled because this is a loopback stream of s16le PCM:
            # deflate would spend CPU on every 640-byte frame for a saving that does not
            # matter over the loopback interface.
            self._connection = await connect(
                self._url, compression=None, open_timeout=self._config.http_timeout_s
            )
        except InvalidStatus as exc:
            raise PayloadClientError(
                f"payload refused WS /mic with HTTP {exc.response.status_code}; "
                f"the service must be in mode {MODE_FUNC1} with no other mic client"
            ) from exc
        except (OSError, WebSocketException) as exc:
            raise PayloadClientError(f"cannot open {self._url}: {exc}") from exc
        try:
            _check_mic_header(await self._connection.recv())
        except ConnectionClosed as exc:
            raise PayloadClientError(f"mic stream closed before its header: {exc}") from exc
        except PayloadClientError:
            # A header this client cannot work with makes the session useless, so close
            # the socket now rather than leaving the device recording into nothing while
            # the error propagates.
            await self._connection.close()
            raise
        logger.info("mic stream open: %s", self._url)
        return self

    async def __aexit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        """Close the socket, which is also what disarms the device's microphone."""
        await self._connection.close()
        logger.info("mic stream closed")

    def __aiter__(self) -> "MicStream":
        """Return self: the stream is its own iterator, since it has one cursor."""
        return self

    async def __anext__(self) -> bytes:
        """Return the next 20 ms PCM frame.

        Returns:
            One 640-byte s16le frame at 16 kHz.

        Raises:
            StopAsyncIteration: on a clean close -- the session ended normally.
            PayloadClientError: on an error close, which means the device audio link
                dropped and no further audio will arrive on this socket.

        Frame size is not re-checked here: the opening header already established the
        geometry once per session, and the VAD raises on a wrong-sized frame anyway, so a
        third check on every one of the fifty frames a second would buy nothing.
        """
        while True:
            try:
                message = await self._connection.recv()
            except ConnectionClosedOK:
                # Normal close: the session is over, which is what ends an async for.
                raise StopAsyncIteration
            except ConnectionClosed as exc:
                # Any other close code is a fault the caller must hear about; 1011 in
                # particular is payload-service reporting the device audio link is down.
                raise PayloadClientError(f"mic stream closed: {exc}") from exc
            if isinstance(message, bytes):
                return message
            # The server sends exactly one text frame (the header, already consumed), so
            # a later one is unexpected. It is skipped rather than fatal: losing a live
            # dialogue over a stray non-audio frame would be a worse outcome than ignoring
            # something this client has no use for.
            logger.warning("mic stream sent unexpected text frame, ignoring")


class PlayStream:
    """Writable side of payload-service's WS /play plane: PCM in, device loudspeaker out.

    The mirror image of MicStream, and used the same way -- as an async context manager, so
    the socket closes on every exit path. Closing matters more here than on /mic: the
    device has no "hail finished" signal, so payload-service flushes the encoder tail and
    sends [11] when this socket closes (api/ws.py _finish_play). A stream merely abandoned
    would leave the loudspeaker parked on the last frame it was handed.

    That teardown is exactly why the 功能2 intercom opens one of these per transmission
    rather than holding one open for the whole session: closing it IS the end of the
    transmission, so press-to-talk and release map onto open and close with nothing extra
    to invent. Holding one socket open across a whole intercom session would instead leave
    a hail permanently running, fed with silence between turns.

    Frames are sent at the pipeline's own 16 kHz; payload-service resamples to the device's
    8 kHz itself. The optional opening header declares that rate explicitly rather than
    relying on the server's default, so the two ends cannot silently disagree if that
    default ever changes -- the same reasoning that makes MicStream check the header it is
    sent instead of assuming it.
    """

    def __init__(self, config: AiRuntimeConfig) -> None:
        """Prepare a play stream; no connection is made until the context is entered."""
        self._config = config
        self._url = _ws_url(config, _PLAY_PATH)
        # Assigned in __aenter__. None until then so sending without entering the context
        # fails immediately rather than half-working.
        self._connection: Any = None

    async def __aenter__(self) -> "PlayStream":
        """Open the socket and declare the sample rate of the PCM that will follow.

        Returns:
            self, ready for send().

        Raises:
            PayloadClientError: if the handshake is refused (the mode gate, or another
                play client already holding the session under R2) or the connection fails.

        The header is sent immediately and before any audio because payload-service only
        honours a rate declaration in the FIRST frame it sees -- once binary audio starts,
        its encoder is mid-stream and a late header is ignored.
        """
        try:
            # compression off for the same reason as /mic: deflate on loopback s16le PCM
            # spends CPU per frame for a saving that does not matter here.
            self._connection = await connect(
                self._url, compression=None, open_timeout=self._config.http_timeout_s
            )
        except InvalidStatus as exc:
            raise PayloadClientError(
                f"payload refused WS /play with HTTP {exc.response.status_code}; "
                f"the service must be in mode {MODE_FUNC1} or {MODE_FUNC2} "
                "with no other play client"
            ) from exc
        except (OSError, WebSocketException) as exc:
            raise PayloadClientError(f"cannot open {self._url}: {exc}") from exc
        try:
            await self._connection.send(json.dumps({"sample_rate": _MIC_SAMPLE_RATE}))
        except (ConnectionClosed, WebSocketException) as exc:
            # A socket that dies between the handshake and the header is not usable, so
            # close it rather than leaving a half-open session registered on the server.
            await self._connection.close()
            raise PayloadClientError(f"play stream closed before its header: {exc}") from exc
        logger.info("play stream open: %s", self._url)
        return self

    async def __aexit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        """Close the socket, which is what flushes the tail and stops the hail ([11])."""
        await self._connection.close()
        logger.info("play stream closed")

    async def send(self, frame: bytes) -> None:
        """Push one PCM frame towards the device loudspeaker.

        Args:
            frame: s16le PCM at 16 kHz mono. Any length is accepted -- payload-service
                buffers and re-cuts to the device's 480-sample Opus frames -- so the
                caller passes its frames straight through with no reblocking.

        Raises:
            PayloadClientError: if the socket has closed, which means the far end stopped
                accepting audio (a mode transition, or the device audio link dropping).

        A closed socket is an error rather than a silent no-op: it means the words being
        spoken are not reaching the loudspeaker, and an intercom that swallows the
        operator's voice without telling them is worse than one that stops.
        """
        try:
            await self._connection.send(frame)
        except ConnectionClosed as exc:
            raise PayloadClientError(f"play stream closed: {exc}") from exc
