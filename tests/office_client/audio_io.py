"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: audio_io.py
Brief: Office-side microphone capture and loudspeaker playback for the 功能2 intercom.

Description:
  The office end of 功能2 needs both halves of a soundcard: capture, so the operator can be
  heard on the robot, and playback, so the robot can be heard in the office. Both are done
  by driving alsa-utils (arecord / aplay) as child processes.

  Why child processes rather than a Python audio binding: alsa-utils is already installed
  everywhere this runs, whereas sounddevice or pyaudio would be a pip install on a machine
  we may not administer. It also puts the blocking ALSA calls in another process, so a
  driver that stalls cannot freeze this client's event loop -- which matters more here than
  usual, because a stalled loop would also stop the PTT keyboard from being read, leaving
  the operator unable to release the channel.

  Why this does NOT import tests/ai_runtime/local_mic.py, which captures the same way:
  office-client runs on an office PC, and importing that module would drag in the whole
  AI_runtime package -- including its VAD, and through it sherpa-onnx and the silero model.
  An intercom client that cannot start without a speech-recognition stack installed would
  be unusable on exactly the machines it is meant for. The duplication is deliberate and
  small; the coupling it avoids is not.

  Format is fixed at 16 kHz mono s16le because that is what the intercom socket carries in
  both directions (plan section 7), and converting at the edges would only introduce a
  place for the two ends to disagree. The frame size is a parameter rather than a constant
  so the caller states it once, from the header the server sent, instead of this module and
  the protocol each carrying their own copy of the number.

  ALSA device names: pass a plug-layer name ("default", or "plughw:1,0"), never a bare
  "hw:" name. A hw: device does no conversion, and arecord asked for a rate the hardware
  cannot do does NOT fail -- it warns on stderr and proceeds at the hardware's rate. The
  stream would then be, say, 48 kHz audio labelled 16 kHz: the operator's voice three times
  too slow on the robot, with nothing anywhere reporting an error.
"""
from __future__ import annotations

import asyncio
import logging
from typing import List, Optional

logger = logging.getLogger("office_client.audio")

# The one PCM geometry this client speaks, matching the intercom socket (plan section 7).
_FORMAT = "S16_LE"
_SAMPLE_RATE = 16000
_CHANNELS = 1
_BYTES_PER_SAMPLE = 2

# alsa-utils binaries, named once so a missing one produces one recognisable error message.
_ARECORD = "arecord"
_APLAY = "aplay"

# How long to wait for a terminated child before killing it. arecord blocks writing into a
# full stdout pipe and aplay blocks draining its buffer, so either can miss a SIGTERM for a
# moment; the kill is the backstop, not the normal path.
_TERMINATE_TIMEOUT_S = 2.0

# How long aplay may take to finish playing what it already holds once its input has ended.
# Generous compared with the buffer this module asks for (about 100 ms), because the cost of
# being wrong in each direction is not symmetric: waiting too long delays a shutdown by a
# second, while waiting too little cuts the end off every transmission the operator hears.
_DRAIN_TIMEOUT_S = 2.0


class AudioIoError(RuntimeError):
    """Raised when the office soundcard cannot be opened or stops working.

    House rule bans bare Exception. One type for both directions because the operator's
    response is the same either way -- check which ALSA device is configured, check the
    hardware is plugged in -- and splitting capture from playback would invite handling
    that does not actually differ.
    """


def _alsa_args(period_bytes: int) -> List[str]:
    """Build the format and buffering arguments shared by arecord and aplay.

    Args:
        period_bytes: the PCM frame size in bytes, used to derive the ALSA period.

    Returns:
        The argument list fragment declaring format, rate, channels and buffering.

    The period is set to one PCM frame and the buffer to a few of them so latency stays a
    few tens of milliseconds. Left at the driver's defaults, ALSA would happily buffer a
    half second, which is unusable for a conversation -- the operator would talk over
    replies they had not heard yet.
    """
    samples = period_bytes // (_BYTES_PER_SAMPLE * _CHANNELS)
    period_us = samples * 1_000_000 // _SAMPLE_RATE
    return [
        "-t", "raw",
        "-f", _FORMAT,
        "-r", str(_SAMPLE_RATE),
        "-c", str(_CHANNELS),
        f"--period-time={period_us}",
        # Five periods: enough that a scheduling hiccup does not underrun, few enough that
        # the added latency (about 100 ms at 20 ms periods) stays below what a person hears
        # as a delay in a two-way conversation.
        f"--buffer-time={period_us * 5}",
    ]


def _capture_argv(device: str, frame_bytes: int) -> List[str]:
    """Build the arecord command line for one capture stream.

    Args:
        device: ALSA capture device (a plug-layer name).
        frame_bytes: PCM bytes per frame.

    Returns:
        The full argv, writing raw PCM to stdout.

    A separate function rather than an expression inside __aenter__ so a test can replace
    the whole command with a producer it controls and still exercise the real pipe, the
    real reframing and the real shutdown. What is left untested that way is ALSA itself,
    which cannot be faked usefully and is covered on the device instead.
    """
    return [_ARECORD, "-q", "-D", device] + _alsa_args(frame_bytes) + ["-"]


def _playback_argv(device: str, frame_bytes: int) -> List[str]:
    """Build the aplay command line for one playback stream.

    Args:
        device: ALSA playback device (a plug-layer name).
        frame_bytes: PCM bytes per frame, used only to size the ALSA period.

    Returns:
        The full argv, reading raw PCM from stdin.
    """
    return [_APLAY, "-q", "-D", device] + _alsa_args(frame_bytes) + ["-"]


class MicCapture:
    """Async iterator over the office microphone, yielding fixed-size PCM frames.

    An async context manager so the child process is reaped on every exit path. Entering
    reads one real frame before returning, which turns "no microphone" into a startup
    error instead of an intercom that connects, looks healthy, and transmits silence --
    by far the more expensive symptom to diagnose from the other end of a robot.
    """

    def __init__(self, device: str, frame_bytes: int, open_timeout_s: float) -> None:
        """Prepare a capture stream; nothing is opened until the context is entered.

        Args:
            device: ALSA capture device (a plug-layer name, see the module docstring).
            frame_bytes: PCM bytes per yielded frame.
            open_timeout_s: how long the first frame may take before this is a failure.
        """
        self._device = device
        self._frame_bytes = frame_bytes
        self._open_timeout_s = open_timeout_s
        self._process: Optional[asyncio.subprocess.Process] = None
        # The frame read by the open probe, handed out by the first __anext__ so the probe
        # costs no audio. Cleared once delivered.
        self._pending: Optional[bytes] = None
        self._stderr_task: "Optional[asyncio.Task[None]]" = None
        # Last line arecord complained about, appended to errors so an ALSA message ("device
        # busy", "no such card") reaches the operator instead of just "no audio".
        self._last_stderr = ""

    async def __aenter__(self) -> "MicCapture":
        """Start arecord and prove it is delivering audio before returning.

        Returns:
            self, ready to iterate.

        Raises:
            AudioIoError: if arecord cannot be started, exits at once, or produces no
                audio within the open timeout.
        """
        argv = _capture_argv(self._device, self._frame_bytes)
        try:
            self._process = await asyncio.create_subprocess_exec(
                *argv,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
        except OSError as exc:
            raise AudioIoError(f"cannot start {_ARECORD}: {exc}") from exc
        self._stderr_task = asyncio.create_task(self._drain_stderr())
        try:
            frame = await asyncio.wait_for(self._read_frame(), self._open_timeout_s)
        except asyncio.TimeoutError:
            await self._shutdown()
            raise AudioIoError(
                f"no audio from ALSA device {self._device!r} within "
                f"{self._open_timeout_s:.1f}s{self._stderr_suffix()}"
            ) from None
        if frame is None:
            await self._shutdown()
            raise AudioIoError(
                f"{_ARECORD} exited while opening {self._device!r}{self._stderr_suffix()}"
            )
        self._pending = frame
        logger.info("office mic open: %s", self._device)
        return self

    async def __aexit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """Stop arecord and reap it."""
        await self._shutdown()

    def __aiter__(self) -> "MicCapture":
        """Return self: one process, one cursor."""
        return self

    async def __anext__(self) -> bytes:
        """Return the next PCM frame.

        Returns:
            One frame of exactly frame_bytes.

        Raises:
            AudioIoError: if capture stopped. There is no StopAsyncIteration path: arecord
                reading a live device has no natural end, so an EOF means the device went
                away or the process was killed -- always a fault, never a clean finish.
        """
        if self._pending is not None:
            frame, self._pending = self._pending, None
            return frame
        frame = await self._read_frame()
        if frame is None:
            code = self._process.returncode if self._process is not None else None
            raise AudioIoError(
                f"{_ARECORD} stopped capturing (exit {code}){self._stderr_suffix()}"
            )
        return frame

    async def _read_frame(self) -> Optional[bytes]:
        """Read exactly one whole frame, or None at end of stream.

        readexactly is what guarantees whole frames: the pipe delivers arbitrary chunk
        sizes, and a partial frame handed onward would shift every following sample by
        half a sample and turn the audio into noise.
        """
        try:
            return await self._process.stdout.readexactly(self._frame_bytes)
        except asyncio.IncompleteReadError:
            return None

    async def _drain_stderr(self) -> None:
        """Keep arecord's stderr drained, remembering the last line it wrote.

        Draining is not optional: a full stderr pipe would block arecord mid-capture. The
        remembered line is what makes an open failure say "device busy" rather than just
        reporting that no audio arrived.
        """
        while True:
            line = await self._process.stderr.readline()
            if not line:
                return
            self._last_stderr = line.decode("utf-8", "replace").strip()
            logger.warning("%s: %s", _ARECORD, self._last_stderr)

    def _stderr_suffix(self) -> str:
        """Format the remembered stderr line for appending to an error message."""
        return f" ({self._last_stderr})" if self._last_stderr else ""

    async def _shutdown(self) -> None:
        """Terminate and reap the child; safe to call more than once."""
        await _reap(self._process, _ARECORD)
        self._process = None
        if self._stderr_task is not None:
            self._stderr_task.cancel()
            self._stderr_task = None


class SpeakerPlayback:
    """Plays PCM frames on the office loudspeaker by feeding aplay.

    An async context manager so the child is reaped on every exit path. Unlike capture
    there is no open probe: aplay accepts data immediately whether or not anything audible
    comes out, so a probe could not prove anything a failed spawn does not already.
    """

    def __init__(self, device: str, frame_bytes: int) -> None:
        """Prepare a playback sink; nothing is opened until the context is entered.

        Args:
            device: ALSA playback device (a plug-layer name).
            frame_bytes: PCM bytes per frame, used only to size the ALSA period.
        """
        self._device = device
        self._frame_bytes = frame_bytes
        self._process: Optional[asyncio.subprocess.Process] = None
        self._stderr_task: "Optional[asyncio.Task[None]]" = None

    async def __aenter__(self) -> "SpeakerPlayback":
        """Start aplay reading raw PCM from its stdin.

        Returns:
            self, ready for play().

        Raises:
            AudioIoError: if aplay cannot be started.
        """
        argv = _playback_argv(self._device, self._frame_bytes)
        try:
            self._process = await asyncio.create_subprocess_exec(
                *argv,
                stdin=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
        except OSError as exc:
            raise AudioIoError(f"cannot start {_APLAY}: {exc}") from exc
        self._stderr_task = asyncio.create_task(self._drain_stderr())
        logger.info("office speaker open: %s", self._device)
        return self

    async def __aexit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """Let aplay finish what it holds, then reap it."""
        process = self._process
        self._process = None
        if process is not None:
            if process.stdin is not None:
                # End of input is aplay's signal to play out its buffer and exit by itself.
                process.stdin.close()
            try:
                # Waiting for that exit is the whole point of closing stdin: signalling it
                # and then terminating in the same breath would discard the buffer anyway,
                # which on a live intercom means the last word of every transmission.
                await asyncio.wait_for(process.wait(), _DRAIN_TIMEOUT_S)
            except asyncio.TimeoutError:
                # It did not go on its own -- most likely blocked on a soundcard that has
                # stopped consuming. _reap is the backstop.
                await _reap(process, _APLAY)
        if self._stderr_task is not None:
            self._stderr_task.cancel()
            self._stderr_task = None

    async def play(self, frame: bytes) -> None:
        """Write one PCM frame to the loudspeaker.

        Args:
            frame: s16le PCM at 16 kHz mono.

        Raises:
            AudioIoError: if aplay has exited, which means nothing is being heard any more.

        drain() applies real backpressure: if the soundcard is behind, this waits rather
        than buffering without bound. That is the correct behaviour for a stream that
        arrives at exactly realtime -- the sender is the robot's microphone, which produces
        one frame per frame-time, so a persistently full pipe means the office soundcard is
        running slow, and buffering it away would only grow the delay silently.
        """
        if self._process is None or self._process.stdin is None:
            raise AudioIoError(f"{_APLAY} is not running")
        try:
            self._process.stdin.write(frame)
            await self._process.stdin.drain()
        except (BrokenPipeError, ConnectionResetError) as exc:
            raise AudioIoError(f"{_APLAY} stopped playing: {exc}") from exc

    async def _drain_stderr(self) -> None:
        """Keep aplay's stderr drained so a full pipe cannot block playback."""
        while True:
            line = await self._process.stderr.readline()
            if not line:
                return
            logger.warning("%s: %s", _APLAY, line.decode("utf-8", "replace").strip())


async def _reap(process: Optional[asyncio.subprocess.Process], name: str) -> None:
    """Terminate a child if it is still running and wait for it to go.

    Args:
        process: the child, or None if it was never started.
        name: the binary name, for the log line when it has to be killed.

    Shared by both classes because the sequence is identical and easy to get subtly wrong:
    a process already gone raises ProcessLookupError from terminate(), and one blocked on a
    full pipe will not act on SIGTERM at all, so the timeout-then-kill is what guarantees
    this returns instead of hanging a shutdown.
    """
    if process is None or process.returncode is not None:
        return
    try:
        process.terminate()
    except ProcessLookupError:
        # Already exited between the check and the signal; nothing left to do.
        return
    try:
        await asyncio.wait_for(process.wait(), _TERMINATE_TIMEOUT_S)
    except asyncio.TimeoutError:
        logger.warning("%s did not exit on terminate, killing", name)
        process.kill()
        await process.wait()
