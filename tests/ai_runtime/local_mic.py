"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: local_mic.py
Brief: Local (Orin-side) microphone capture, an alternative frame source to WS /mic.

Description:
  A second implementation of the mic stream the turn loop consumes: same 640-byte 20 ms
  s16le frames at 16 kHz, same async-context-manager and async-iterator shape as
  payload_client.MicStream, but the audio comes from a microphone attached to the Orin
  instead of from the payload device.

  Why this exists at all -- issue A1. The device's own [40] uplink is DECIMATED, not
  merely slow: measured 2.85 s of audio delivered for 12.10 s recorded (0.235x), with only
  0.10 s of tail after the [41] stop, which proves the missing three quarters are never
  transmitted rather than arriving late. Speech sampled that way is unintelligible to ASR
  and no amount of buffering recovers it, so 功能1 cannot use the device microphone until
  the vendor answers. A microphone on the Orin bypasses the device audio uplink entirely
  while leaving the downlink -- the TTS reply over [31] -- exactly as it was.

  Why arecord as a subprocess rather than a Python audio binding: alsa-utils is already on
  the Orin, while sounddevice/pyaudio are not, and this is a shared device where adding a
  pip dependency is not a decision this module gets to make. The subprocess also puts the
  ALSA read loop in another process, so a blocking driver call cannot stall the event loop
  that is meanwhile draining frames at real time.

  Why no resampler here, despite the current USB dongle being 48 kHz only: ALSA's plug
  layer converts on the way out, and its default converter was measured on this box at
  -98.2 dB alias rejection for 48k->16k, which is well past what a 16 kHz ASR front end
  can notice. The one trap is `defaults.pcm.rate_converter "linear"`, which measured
  -0.0 dB -- full-amplitude aliasing. It is unset on this box and must stay unset.

  Why there is no format check here, where MicStream validates a header: the WS source is
  told its geometry by another process and has to verify the claim, whereas this one
  REQUESTS its geometry on the command line and ALSA's plug layer is obliged to deliver
  it. That obligation is exactly why config defaults the device to "default" and warns
  against a bare hw: name -- a hw: device does no conversion, and arecord then warns on
  stderr and proceeds at the hardware's own rate rather than failing. That warning is
  logged by the stderr drain below, which is the one place such a mismatch surfaces.

  What this module deliberately does NOT do: it does not touch payload-service, the mode
  or the device. Selecting it changes where frames come from and nothing else, so the
  session still holds 功能1 mode (invariant R3) and still speaks through the device.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, List, Optional

from .config import AiRuntimeConfig

# Imported rather than redeclared: this module PRODUCES the frames the VAD consumes, so
# taking the size from the consumer is what makes it impossible for the two to disagree.
from .vad import FRAME_BYTES

logger = logging.getLogger("ai_runtime.local_mic")

# The capture format requested from ALSA. Fixed rather than configurable because 16 kHz
# mono s16le is what the zipformer behind asr-service was trained on -- any other geometry
# would have to be converted back before ASR, so there is nothing to gain by varying it.
_FORMAT = "S16_LE"
_SAMPLE_RATE = 16000
_CHANNELS = 1
_BYTES_PER_SAMPLE = 2

# ALSA period and buffer, in microseconds. The period is set to one output frame so
# arecord hands us audio on the same 20 ms cadence the VAD works in, rather than in larger
# clumps that would make the segmenter's timing granular. The buffer is five periods:
# enough that a scheduling hiccup on this box does not overrun the capture ring, small
# enough that the audio in flight stays a fraction of the VAD's 120 ms start window.
_PERIOD_US = (FRAME_BYTES // (_BYTES_PER_SAMPLE * _CHANNELS)) * 1_000_000 // _SAMPLE_RATE
_BUFFER_US = _PERIOD_US * 5

# The capture program. Named as a bare command so PATH resolves it, matching how the rest
# of the bring-up tooling invokes system utilities.
_ARECORD = "arecord"

# How long to wait for arecord to die after SIGTERM before escalating. Short because the
# process has nothing to flush -- it writes to a pipe, not a file.
_TERMINATE_TIMEOUT_S = 2.0


class LocalMicError(RuntimeError):
    """A local capture fault: arecord could not start, could not open ALSA, or died.

    Separate from PayloadClientError even though both end a session, because the two name
    completely different repairs. A payload fault means the device or its service is wrong;
    this one means the microphone attached to THIS box is wrong, and reporting it under the
    payload name would send an operator to inspect the robot instead of the USB port.
    """


def _argv(config: AiRuntimeConfig) -> List[str]:
    """Build the arecord command line for one capture session.

    Args:
        config: supplies the ALSA device name to open.

    Returns:
        An argv list ready for create_subprocess_exec.

    Returned as a list and never as a shell string: the ALSA device name is operator input
    from the environment, and a list argv means it can never be parsed as shell syntax.
    """
    return [
        _ARECORD,
        # Quiet, so the "Recording raw data ..." banner does not land on stderr every
        # session. With it suppressed, anything on stderr is a real fault worth logging.
        "-q",
        "-D", config.mic_alsa_device,
        # Raw, because a wav header would arrive as 44 bytes of non-audio at the head of
        # the stream and shift every frame boundary after it.
        "-t", "raw",
        "-f", _FORMAT,
        "-r", str(_SAMPLE_RATE),
        "-c", str(_CHANNELS),
        f"--period-time={_PERIOD_US}",
        f"--buffer-time={_BUFFER_US}",
        # Write to stdout: this is a live stream, so there is no file involved.
        "-",
    ]


class LocalMicStream:
    """Async iterator over an Orin-attached microphone, yielding 20 ms PCM frames.

    Interchangeable with payload_client.MicStream: same context-manager lifecycle, same
    frame size, same iteration contract. The turn loop is written against that shape and
    does not know which source it was handed.

    The stream is unbounded. Unlike the WS source, which ends when payload-service closes
    the socket, a microphone simply keeps producing audio until the operator stops the
    process -- so a clean end of iteration never happens, and an end of stream is always a
    fault (the process died, or the USB device was unplugged).
    """

    def __init__(self, config: AiRuntimeConfig) -> None:
        """Prepare a stream; no process is started until the context is entered."""
        self._config = config
        # All assigned in __aenter__. None until then, so using the iterator without
        # entering the context fails immediately rather than half-working.
        self._process: Any = None
        self._stderr_task: Optional[asyncio.Task] = None
        # The frame read during the open probe below, handed out by the first __anext__.
        # Without this the probe would silently drop 20 ms off the front of the session.
        self._pending: Optional[bytes] = None
        # Last line arecord wrote to stderr, kept to be quoted in the failure message.
        # ALSA's own diagnostics are far more specific than any message this module could
        # synthesise -- "No such file or directory" versus "Device or resource busy" is
        # the difference between a wrong device name and another process holding the mic.
        self._last_stderr = ""

    async def __aenter__(self) -> "LocalMicStream":
        """Start arecord and prove audio is actually flowing before returning.

        Returns:
            self, ready to iterate.

        Raises:
            LocalMicError: if arecord cannot be started, exits immediately, or produces no
                audio within the configured timeout.

        The open probe -- reading one real frame here rather than on first iteration -- is
        what makes a missing or busy microphone a startup error. Without it arecord's
        failure would surface as a turn loop that runs happily and simply never hears
        anybody, which is the single most expensive symptom to debug on a device.
        """
        argv = _argv(self._config)
        logger.info("local mic: %s", " ".join(argv))
        try:
            self._process = await asyncio.create_subprocess_exec(
                *argv,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
        except OSError as exc:
            # Raised when the binary itself is missing, before any ALSA call happens.
            raise LocalMicError(f"cannot start {_ARECORD}: {exc}") from exc
        # stderr must be drained continuously, not read at the end: a pipe nobody reads
        # fills up and blocks the writer, which would freeze capture rather than log it.
        self._stderr_task = asyncio.create_task(self._drain_stderr())
        try:
            frame = await asyncio.wait_for(
                self._read_frame(), self._config.mic_open_timeout_s
            )
        except asyncio.TimeoutError:
            await self._shutdown()
            raise LocalMicError(
                f"no audio from ALSA device {self._config.mic_alsa_device!r} within "
                f"{self._config.mic_open_timeout_s}s{self._stderr_suffix()}"
            ) from None
        if frame is None:
            # End of stream during the probe means arecord exited on the spot, which is
            # how a bad device name or a busy card presents.
            await self._shutdown()
            raise LocalMicError(
                f"{_ARECORD} exited while opening ALSA device "
                f"{self._config.mic_alsa_device!r}{self._stderr_suffix()}"
            )
        self._pending = frame
        logger.info(
            "local mic open: %s %d Hz mono, %d-byte frames",
            self._config.mic_alsa_device,
            _SAMPLE_RATE,
            FRAME_BYTES,
        )
        return self

    async def __aexit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        """Stop capture and reap the process, on every exit path including an exception."""
        await self._shutdown()
        logger.info("local mic closed")

    def __aiter__(self) -> "LocalMicStream":
        """Return self: the stream is its own iterator, since it has one cursor."""
        return self

    async def __anext__(self) -> bytes:
        """Return the next 20 ms PCM frame.

        Returns:
            One 640-byte s16le frame at 16 kHz.

        Raises:
            LocalMicError: if capture stopped -- the process died or the device went away.

        Note there is no StopAsyncIteration path. A microphone does not end its own stream,
        so anything that stops the audio is a fault the operator has to hear about; a
        session is ended from the outside instead (Ctrl-C, which app.py treats as clean).
        """
        if self._pending is not None:
            # The frame captured by the open probe. Consumed exactly once.
            frame, self._pending = self._pending, None
            return frame
        frame = await self._read_frame()
        if frame is None:
            # Reap first: the exit status and the last stderr line are the only evidence
            # of WHY capture stopped, and both are only settled once the process is gone.
            await self._shutdown()
            raise LocalMicError(
                f"{_ARECORD} stopped capturing (exit {self._process.returncode})"
                f"{self._stderr_suffix()}"
            )
        return frame

    async def _read_frame(self) -> Optional[bytes]:
        """Read exactly one frame from arecord's stdout.

        Returns:
            A frame of FRAME_BYTES, or None if the stream ended.

        readexactly does the reframing: ALSA's period sizing is a request, not a promise,
        and the pipe can split a period anywhere, so this is what guarantees the VAD only
        ever sees whole frames. A trailing partial frame at end of stream is discarded with
        the IncompleteReadError -- it is at most 20 ms, and the session is over anyway.
        """
        try:
            return await self._process.stdout.readexactly(FRAME_BYTES)
        except asyncio.IncompleteReadError:
            return None

    async def _drain_stderr(self) -> None:
        """Log arecord's stderr as it arrives, keeping the last line for error messages.

        Runs for the life of the process and ends at EOF, which the process's exit
        guarantees -- so it needs no cancellation, only to be awaited during shutdown.
        """
        while True:
            line = await self._process.stderr.readline()
            if not line:
                return
            text = line.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            self._last_stderr = text
            # WARNING rather than ERROR: with -q, ALSA still reports recoverable
            # conditions such as overruns here, and those are not fatal on their own.
            logger.warning("%s: %s", _ARECORD, text)

    def _stderr_suffix(self) -> str:
        """Format the last stderr line as a message suffix, or empty if there was none."""
        return f": {self._last_stderr}" if self._last_stderr else ""

    async def _shutdown(self) -> None:
        """Stop arecord and wait for it, tolerating a process that has already exited.

        Safe to call more than once, because every failure path calls it before raising and
        __aexit__ then calls it again on the way out.
        """
        if self._process is None:
            return
        if self._process.returncode is None:
            try:
                self._process.terminate()
            except ProcessLookupError:
                # Exited between the check and the signal; nothing left to stop.
                pass
            try:
                await asyncio.wait_for(self._process.wait(), _TERMINATE_TIMEOUT_S)
            except asyncio.TimeoutError:
                # Capture stops being read the moment iteration ends, so arecord can be
                # blocked writing into a full stdout pipe and never reach its SIGTERM
                # handler. SIGKILL cannot be blocked, and there is nothing to flush.
                logger.warning("%s ignored SIGTERM, killing", _ARECORD)
                self._process.kill()
                await self._process.wait()
        else:
            # Already dead (this is the failure paths' case); still must be awaited so the
            # transport is closed and no child is left unreaped.
            await self._process.wait()
        if self._stderr_task is not None:
            # Ends by itself at EOF on stderr, which the exit above has already caused.
            await self._stderr_task
            self._stderr_task = None
