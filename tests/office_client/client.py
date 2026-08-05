"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: client.py
Brief: Office end of the 功能2 intercom -- PTT console, uplink audio, downlink playback.

Description:
  The operator's half of 功能2 (plan sections 5.5 and 6). It connects to AI_runtime's
  intercom server on the robot, takes the format header it is handed, and from then on
  moves voice in whichever direction the operator has selected:

    talk   office microphone (or a wav file) -> intercom -> robot loudspeaker
    listen microphone on the robot -> intercom -> office loudspeaker

  Push-to-talk is driven from typed lines rather than a held key. A real PTT switch reports
  press and release, which a terminal cannot do without putting the tty in raw mode and
  taking over the operator's keyboard -- a cost worth paying in a product and not in a
  bring-up tool. Typed states are also the honest shape for a THREE-state control: talk,
  listen and idle, where idle means holding the channel while routing neither way.

  Two orderings matter, and they are opposites:

    pressing  send the control message FIRST, then start streaming. The server opens the
              device loudspeaker path when it sees the press, and PCM that arrives before
              it has nowhere to go and is dropped.
    releasing stop streaming FIRST, then send the control message. The server closes that
              path the moment it sees the release, so a control message sent early would
              cut the tail off the operator's own sentence.

  --talk-wav exists because the office PC this was written for has no microphone, and
  without it the talk direction could not be exercised at all. It is not a lesser path: a
  fixed wav is a REPEATABLE utterance, which is what a timing measurement wants and what a
  live voice can never be. Each press replays it from the start.

  The two sources have deliberately different lifecycles, because they are different kinds
  of thing. A microphone runs whether or not anyone is listening, so it is opened once for
  the session and drained continuously with unwanted frames discarded -- the same discipline
  the server uses, and for the same reason: a source that is stopped loses the front of the
  next transmission, and one that is left unread falls behind realtime. A file has neither
  problem: it is read only during a burst, from the top.

  Written from scratch per R0; websockets and alsa-utils are the whitelisted transport and
  audio stack underneath.
"""
from __future__ import annotations

import asyncio
import json
import logging
import sys
import threading
import wave
from typing import Any, AsyncIterator, Dict, Optional

from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed, WebSocketException

from .audio_io import AudioIoError, MicCapture, SpeakerPlayback

logger = logging.getLogger("office_client.client")

# The PTT states as they travel on the wire. Duplicated from the server rather than
# imported for the same reason audio_io does not import local_mic: importing anything from
# tests/ai_runtime would drag the AI stack (VAD, sherpa-onnx, the silero model) onto an
# office PC that has no use for it. Three short strings are the cheaper coupling.
_PTT_TALK = "talk"
_PTT_LISTEN = "listen"
_PTT_IDLE = "idle"

# The PCM geometry this client is built for; the server's header is checked against it.
_ENCODING = "s16le"
_SAMPLE_RATE = 16000
_CHANNELS = 1
_BYTES_PER_SAMPLE = 2

# Console commands. Both the single letter and the full word are accepted: the letter is
# what gets used once the operator knows the tool, the word is what they type before that.
_CMD_QUIT = "q"
_COMMANDS = {
    "t": _PTT_TALK,
    "talk": _PTT_TALK,
    "l": _PTT_LISTEN,
    "listen": _PTT_LISTEN,
    "i": _PTT_IDLE,
    "idle": _PTT_IDLE,
    _CMD_QUIT: None,
    "quit": None,
}
_HELP = "commands: t=talk  l=listen  i=idle  q=quit"

# Close codes the server uses (mirrored from intercom.py, see the note on the PTT strings).
# 1013 is "try again later" and means another operator holds the channel, which needs a
# different message from a fault: the operator's remedy is to wait, not to investigate.
_WS_CLOSE_BUSY = 1013
_WS_CLOSE_FAULT = 1011
# What a connection that vanished without a close handshake reports: a network failure
# rather than a decision by either end.
_WS_CLOSE_ABNORMAL = 1006

# How many microphone frames may back up while the operator is not transmitting. Small on
# purpose: this is the drop-oldest buffer that keeps arecord's pipe empty between bursts,
# and anything larger would just mean the first fraction of a second of a transmission is
# stale audio from before the press.
_MIC_BACKLOG_FRAMES = 4

# How often to report that audio is still arriving, in frames -- 250 frames is five seconds.
# Without it a listening operator who hears nothing cannot tell a quiet room from a dead
# link, and those two have completely different remedies. Five seconds is slow enough not to
# scroll the console away from whatever else it is saying.
_DOWNLINK_LOG_FRAMES = 250


def _closed_as(exc: ConnectionClosed):
    """Return the code and reason the robot closed with.

    Args:
        exc: the exception raised by recv or send once the connection went.

    Returns:
        A (code, reason) pair.

    Reads the received close frame rather than ConnectionClosed.code, which websockets
    deprecated in 13.1. rcvd is None when the connection dropped with no close handshake at
    all, which is the network-failure case and carries no reason to report.
    """
    if exc.rcvd is None:
        return _WS_CLOSE_ABNORMAL, ""
    return exc.rcvd.code, exc.rcvd.reason


class IntercomClientError(RuntimeError):
    """Raised when the intercom cannot be reached, or speaks a format this client does not.

    House rule bans bare Exception. Separate from AudioIoError, which is about the office
    soundcard: this one is about the link to the robot, and the two have entirely different
    remedies -- check the network and the server, versus check the sound hardware.
    """


class WavTalkSource:
    """Replays a wav file as the talk direction, paced to realtime.

    The stand-in for a microphone on a PC that has none. The file is loaded and validated
    once when the session opens, so a wrong-format file is a startup error rather than a
    surprise on the first press, and each burst is then an exact replay with no disk access
    in the middle of a sentence.
    """

    def __init__(self, path: str, frame_bytes: int) -> None:
        """Prepare a file source; nothing is read until the context is entered.

        Args:
            path: the wav file to send, which must be 16 kHz mono 16-bit.
            frame_bytes: PCM bytes per frame, taken from the server's header.
        """
        self._path = path
        self._frame_bytes = frame_bytes
        self._pcm = b""
        # How long one frame represents, which is exactly how long to wait between sending
        # them. Derived rather than hardcoded so it follows the server's frame size.
        self._frame_time_s = frame_bytes / (_BYTES_PER_SAMPLE * _CHANNELS * _SAMPLE_RATE)

    async def __aenter__(self) -> "WavTalkSource":
        """Load and validate the file.

        Returns:
            self, ready to stream.

        Raises:
            IntercomClientError: if the file cannot be read, or is not 16 kHz mono 16-bit.

        The format is refused rather than converted. Resampling here would put a second
        opinion about sample rates into a system that already has one, and the failure it
        would hide -- audio at the wrong speed -- is the single hardest audio fault to spot
        from a log, because every byte count still looks correct.
        """
        try:
            with wave.open(self._path, "rb") as handle:
                channels = handle.getnchannels()
                width = handle.getsampwidth()
                rate = handle.getframerate()
                pcm = handle.readframes(handle.getnframes())
        except (OSError, wave.Error) as exc:
            raise IntercomClientError(f"cannot read talk wav {self._path!r}: {exc}") from exc
        if (channels, width, rate) != (_CHANNELS, _BYTES_PER_SAMPLE, _SAMPLE_RATE):
            raise IntercomClientError(
                f"talk wav {self._path!r} is {rate} Hz {channels}ch {width * 8}-bit; "
                f"the intercom needs {_SAMPLE_RATE} Hz {_CHANNELS}ch {_BYTES_PER_SAMPLE * 8}-bit"
            )
        # Pad the tail to a whole frame so every slice sent is full: a short final frame
        # would leave the receiver's stream misaligned by half a sample for everything
        # after it, and the padding costs at most one frame of silence at the end.
        remainder = len(pcm) % self._frame_bytes
        if remainder:
            pcm += b"\x00" * (self._frame_bytes - remainder)
        self._pcm = pcm
        logger.info(
            "talk source: %s (%.1fs)",
            self._path,
            len(pcm) / (_BYTES_PER_SAMPLE * _CHANNELS * _SAMPLE_RATE),
        )
        return self

    async def __aexit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """Nothing to release: the file was closed as soon as it was read."""

    async def frames(self) -> AsyncIterator[bytes]:
        """Yield one burst: the whole file from the start, at realtime pace.

        Yields:
            Frames of exactly frame_bytes.

        Paced rather than sent as fast as the socket accepts it, because everything
        downstream -- the encoder in payload-service, the device's own buffer -- is sized
        for audio arriving at the speed it is spoken. A file delivered in one burst would
        be an overrun, not a fast transmission.

        The deadline accumulates instead of sleeping a frame-time per iteration: a per-frame
        sleep adds its own scheduling error every time, so a minute of audio would drift
        audibly behind. Against a fixed origin the error stays bounded.
        """
        loop = asyncio.get_running_loop()
        deadline = loop.time()
        for offset in range(0, len(self._pcm), self._frame_bytes):
            yield self._pcm[offset:offset + self._frame_bytes]
            deadline += self._frame_time_s
            delay = deadline - loop.time()
            if delay > 0:
                await asyncio.sleep(delay)


class MicTalkSource:
    """The office microphone as the talk direction, drained continuously and gated.

    Held open for the whole session with a drop-oldest backlog behind it. That is what lets
    a burst start with live audio: capture is already running when the operator presses, so
    there is no device-open delay eating the first word, and the frames produced while
    nobody was transmitting were discarded rather than queued.
    """

    def __init__(self, device: str, frame_bytes: int, open_timeout_s: float) -> None:
        """Prepare a microphone source; nothing is opened until the context is entered.

        Args:
            device: ALSA capture device (a plug-layer name, see audio_io's docstring).
            frame_bytes: PCM bytes per frame, taken from the server's header.
            open_timeout_s: how long the device may take to produce its first frame.
        """
        self._device = device
        self._frame_bytes = frame_bytes
        self._open_timeout_s = open_timeout_s
        self._capture: Optional[MicCapture] = None
        self._queue: "asyncio.Queue[Optional[bytes]]" = asyncio.Queue(_MIC_BACKLOG_FRAMES)
        self._drain: "Optional[asyncio.Task[None]]" = None
        # The error that stopped capture, re-raised into whichever burst is in progress so
        # a dead microphone surfaces to the operator instead of a transmission going quiet.
        self._failure: Optional[AudioIoError] = None

    async def __aenter__(self) -> "MicTalkSource":
        """Open the microphone and start draining it.

        Returns:
            self, ready to stream.

        Raises:
            AudioIoError: if the microphone cannot be opened or produces nothing.
        """
        capture = MicCapture(self._device, self._frame_bytes, self._open_timeout_s)
        await capture.__aenter__()
        self._capture = capture
        self._drain = asyncio.create_task(self._drain_mic())
        return self

    async def __aexit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """Stop draining and close the microphone."""
        if self._drain is not None:
            self._drain.cancel()
            self._drain = None
        if self._capture is not None:
            await self._capture.__aexit__(None, None, None)
            self._capture = None

    async def frames(self) -> AsyncIterator[bytes]:
        """Yield one burst: live audio until the caller stops iterating.

        Yields:
            Frames of exactly frame_bytes.

        Raises:
            AudioIoError: if capture stopped.

        This never ends by itself -- a microphone has no end. The burst is ended by the
        caller cancelling the task, which is exactly what releasing PTT does.
        """
        while True:
            frame = await self._queue.get()
            if frame is None:
                # Sentinel from the drain task: capture has died and there is no more audio.
                raise self._failure
            yield frame

    async def _drain_mic(self) -> None:
        """Keep the capture pipe empty, holding only the newest few frames.

        Draining unconditionally is the whole point of this class. An unread pipe fills, and
        then arecord blocks and the stream falls permanently behind realtime -- the operator
        would be transmitting what they said several seconds ago, with every byte count and
        every log line still looking perfectly healthy.
        """
        try:
            async for frame in self._capture:
                if self._queue.full():
                    # Drop the oldest: between bursts these frames are of no interest, and
                    # keeping them would only delay the live audio behind stale audio.
                    self._queue.get_nowait()
                self._queue.put_nowait(frame)
        except AudioIoError as exc:
            # MicCapture has no clean end, so an iteration that stops has failed.
            self._failure = exc
            self._queue.put_nowait(None)


class IntercomClient:
    """Runs one office-client session: console, uplink and downlink until the operator quits.

    Owns the PTT state and the socket. The audio sources and sinks are held for the length
    of the session so that pressing talk costs nothing but a control message, and the only
    per-burst object is the task streaming into the socket.
    """

    def __init__(
        self,
        url: str,
        speaker_device: str,
        mic_device: str,
        talk_wav: Optional[str],
        open_timeout_s: float,
    ) -> None:
        """Capture the settings; open nothing until run().

        Args:
            url: the intercom websocket URL on the robot.
            speaker_device: ALSA playback device for the listen direction.
            mic_device: ALSA capture device for the talk direction, used when talk_wav is None.
            talk_wav: a wav file to send instead of the microphone, or None.
            open_timeout_s: connect and device-open budget.
        """
        self._url = url
        self._speaker_device = speaker_device
        self._mic_device = mic_device
        self._talk_wav = talk_wav
        self._open_timeout_s = open_timeout_s
        # Start idle to match the server, which also starts idle: a client that assumed a
        # direction would disagree with the server about the state nobody had set yet.
        self._ptt = _PTT_IDLE
        # The task streaming audio while transmitting; None whenever PTT is not talk.
        self._uplink: "Optional[asyncio.Task[None]]" = None
        # Typed lines from the operator, plus the synthetic ones the uplink task posts when
        # it ends. A queue rather than direct calls so that the console loop is the ONLY
        # place PTT changes: the alternative would have the uplink task cancel itself.
        self._commands: "asyncio.Queue[str]" = asyncio.Queue()
        # A fault raised on a background task, re-raised from run() so the process exits
        # non-zero rather than merely printing and returning success.
        self._failure: Optional[Exception] = None

    async def run(self) -> None:
        """Connect, serve the console until the operator quits, then tear everything down.

        Raises:
            IntercomClientError: if the robot cannot be reached, refuses the connection, or
                speaks a format this client does not.
            AudioIoError: if the office soundcard fails.
        """
        try:
            async with connect(
                self._url,
                # compression off for the same reason as the server: s16le PCM does not
                # deflate usefully, and both ends would pay CPU for nothing.
                compression=None,
                open_timeout=self._open_timeout_s,
            ) as connection:
                await self._session(connection)
        except (OSError, WebSocketException) as exc:
            raise IntercomClientError(f"cannot reach intercom at {self._url}: {exc}") from exc
        if self._failure is not None:
            raise self._failure

    async def _session(self, connection: Any) -> None:
        """Run one connected session: header, devices, then console and downlink.

        Args:
            connection: the open websocket to the robot.

        The devices are opened AFTER the header because the header is what states the frame
        size, and opening the soundcard first would mean guessing it.
        """
        header = await self._receive_header(connection)
        frame_bytes = header["frame_bytes"]
        async with SpeakerPlayback(self._speaker_device, frame_bytes) as speaker:
            async with self._talk_source(frame_bytes) as talk:
                self._start_console_reader()
                print(_HELP, flush=True)
                # Two things can end the session: the operator typing quit, or the robot
                # hanging up. Raced, because neither returns while things are healthy.
                downlink = asyncio.create_task(self._pump_downlink(connection, speaker))
                console = asyncio.create_task(self._run_console(connection, talk))
                try:
                    done, pending = await asyncio.wait(
                        {downlink, console}, return_when=asyncio.FIRST_COMPLETED
                    )
                    for task in pending:
                        task.cancel()
                        try:
                            await task
                        except asyncio.CancelledError:
                            pass
                    for task in done:
                        error = task.exception()
                        if error is not None:
                            raise error
                finally:
                    # The session may be ending mid-sentence; stop transmitting before the
                    # socket goes, so the robot is not left with a half-open hail.
                    await self._stop_uplink()

    def _talk_source(self, frame_bytes: int) -> Any:
        """Build the talk source the operator asked for.

        Args:
            frame_bytes: PCM bytes per frame, from the server's header.

        Returns:
            An async context manager exposing frames(), either a file or the microphone.
        """
        if self._talk_wav is not None:
            return WavTalkSource(self._talk_wav, frame_bytes)
        return MicTalkSource(self._mic_device, frame_bytes, self._open_timeout_s)

    async def _receive_header(self, connection: Any) -> Dict[str, Any]:
        """Read and check the format header the server sends on connect.

        Args:
            connection: the open websocket.

        Returns:
            The header, with frame_bytes guaranteed to be a usable positive even integer.

        Raises:
            IntercomClientError: if the channel is busy, closed early, or announces a format
                this client does not speak.

        A mismatched format is refused rather than adapted to. This client has exactly one
        geometry wired through it -- the ALSA arguments, the pacing, the playback -- and a
        client that quietly accepted a different rate would produce audio at the wrong speed
        while reporting nothing wrong anywhere.
        """
        try:
            raw = await connection.recv()
        except ConnectionClosed as exc:
            code, reason = _closed_as(exc)
            if code == _WS_CLOSE_BUSY:
                raise IntercomClientError(
                    "the robot's intercom is already in use by another office-client"
                ) from exc
            raise IntercomClientError(
                f"intercom closed before sending its format header ({code} {reason})"
            ) from exc
        if not isinstance(raw, str):
            raise IntercomClientError("intercom sent audio before its format header")
        try:
            header = json.loads(raw)
        except ValueError as exc:
            raise IntercomClientError(f"format header is not valid json: {raw[:200]}") from exc
        if not isinstance(header, dict):
            raise IntercomClientError(f"format header is not a json object: {raw[:200]}")
        for key, want in (
            ("encoding", _ENCODING),
            ("sample_rate", _SAMPLE_RATE),
            ("channels", _CHANNELS),
        ):
            if header.get(key) != want:
                raise IntercomClientError(
                    f"intercom announces {key}={header.get(key)!r}, "
                    f"this client only speaks {want!r}"
                )
        frame_bytes = header.get("frame_bytes")
        # Even, because a 16-bit sample cannot be split across frames without shifting
        # every sample after it by one byte and turning the audio into noise.
        if not isinstance(frame_bytes, int) or frame_bytes <= 0 or frame_bytes % 2:
            raise IntercomClientError(f"intercom announces unusable frame_bytes={frame_bytes!r}")
        logger.info("intercom connected: %s", header)
        return header

    def _start_console_reader(self) -> None:
        """Feed typed lines into the command queue from a background thread.

        A plain daemon thread rather than asyncio.to_thread, because a thread parked in
        stdin.readline() never returns: asyncio.run waits for the default executor on the
        way out, so the executor version would hang the process on exit forever. A daemon
        thread simply dies with the process.
        """
        loop = asyncio.get_running_loop()

        def reader() -> None:
            for line in sys.stdin:
                _post(line.strip().lower())
            # End of input (the operator pressed Ctrl-D, or stdin was a finished file) is
            # a request to stop, the same as typing quit.
            _post(_CMD_QUIT)

        def _post(command: str) -> None:
            try:
                loop.call_soon_threadsafe(self._commands.put_nowait, command)
            except RuntimeError:
                # The session ended while a line was in flight; there is nobody to tell.
                pass

        threading.Thread(target=reader, name="office-console", daemon=True).start()

    async def _run_console(self, connection: Any, talk: Any) -> None:
        """Apply commands until the operator quits.

        Args:
            connection: the open websocket.
            talk: the talk source, iterated during a transmission.

        The single owner of the PTT state. Every change goes through here, including the
        ones the uplink task asks for when a wav file runs out, so there is never a moment
        where two places are deciding what state the channel is in.
        """
        while True:
            command = await self._commands.get()
            if not command:
                continue
            if command not in _COMMANDS:
                print(f"unknown command {command!r}; {_HELP}", flush=True)
                continue
            state = _COMMANDS[command]
            if state is None:
                return
            await self._set_ptt(connection, talk, state)

    async def _set_ptt(self, connection: Any, talk: Any, state: str) -> None:
        """Change PTT state, starting or stopping the uplink with it.

        Args:
            connection: the open websocket.
            talk: the talk source.
            state: the requested state.

        A repeat of the current state is ignored: re-entering talk would restart the
        transmission the operator is in the middle of, which on a wav source means the
        sentence starts over from the beginning.
        """
        if state == self._ptt:
            return
        if self._ptt == _PTT_TALK:
            # Release: stop the audio before the control message (module docstring), so the
            # end of the sentence reaches the loudspeaker before the server closes it.
            await self._stop_uplink()
        self._ptt = state
        await connection.send(json.dumps({"ptt": state}))
        if state == _PTT_TALK:
            # Press: the control message went first, so the loudspeaker path is opening;
            # only now is there anywhere for audio to go.
            self._uplink = asyncio.create_task(self._pump_uplink(connection, talk))
        print(f"ptt: {state}", flush=True)

    async def _pump_uplink(self, connection: Any, talk: Any) -> None:
        """Stream one transmission from the talk source into the socket.

        Args:
            connection: the open websocket.
            talk: the talk source.

        Ends when the source runs out (a wav file), when the socket closes, or when the
        console cancels it on release. In the first two cases it asks the console to change
        state rather than doing it here: cancelling this task is part of leaving talk, and
        a task cannot await its own cancellation.
        """
        try:
            async for frame in talk.frames():
                await connection.send(frame)
        except ConnectionClosed:
            # The robot hung up mid-transmission; the downlink pump is ending the session.
            return
        except AudioIoError as exc:
            logger.error("uplink stopped: %s", exc)
            self._failure = exc
            self._commands.put_nowait(_CMD_QUIT)
            return
        # A wav file played to the end. Drop back to idle so the channel is not left held
        # open by a source that has nothing left to send.
        print("talk source finished", flush=True)
        self._commands.put_nowait(_PTT_IDLE)

    async def _stop_uplink(self) -> None:
        """Cancel the transmission in progress, if there is one.

        Idempotent because it runs both on a release and in the session teardown, and a
        release followed by a quit runs it twice. Nulling the handle first means the second
        call finds nothing to do even while the first is still unwinding.
        """
        task = self._uplink
        self._uplink = None
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    async def _pump_downlink(self, connection: Any, speaker: SpeakerPlayback) -> None:
        """Play everything the robot sends.

        Args:
            connection: the open websocket.
            speaker: the office loudspeaker.

        Raises:
            IntercomClientError: if the robot ended the session with a fault.
            AudioIoError: if the office loudspeaker stops.

        Frames are played without checking the local PTT state. The server is the authority
        on what may be sent, and it already gates; re-checking here would only clip the
        frames still in flight at the instant the operator switches away from listen.
        """
        received = 0
        while True:
            try:
                message = await connection.recv()
            except ConnectionClosed as exc:
                code, reason = _closed_as(exc)
                if code == _WS_CLOSE_FAULT:
                    # The server names its own reason in the close frame; passing it on is
                    # the difference between a diagnosable failure and a silent hang-up.
                    raise IntercomClientError(f"robot ended the intercom: {reason}") from exc
                logger.info("intercom closed by the robot (%s %s)", code, reason)
                return
            if isinstance(message, str):
                # The server sends one text frame, the header, and it has already been read.
                logger.warning("ignoring unexpected control message: %s", message[:200])
                continue
            received += 1
            if received % _DOWNLINK_LOG_FRAMES == 0:
                # Counted in frames rather than timed on a clock, so the line means "audio is
                # still arriving" and not merely "this process is still alive": a stalled
                # uplink stops the count, where a timer would keep printing regardless.
                logger.info("listening: %d frames received", received)
            await speaker.play(message)
