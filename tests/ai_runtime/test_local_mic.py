"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_local_mic.py
Brief: Unit tests for the Orin-side microphone capture source.

Description:
  Exercises LocalMicStream against a REAL child process and a REAL pipe, with arecord
  replaced by a small python producer. That substitution is the whole design of this file:
  everything worth testing here -- reframing across pipe boundaries, the open probe, the
  stderr capture, the shutdown -- is subprocess and stream behaviour, and a mocked
  subprocess would test the mock. What is left out is ALSA itself, which cannot be faked
  usefully and is covered by the on-device acceptance run instead.

  The producers are spelled as python -c programs because the Orin has no fixture audio
  tooling and this keeps the tests dependent on nothing but the interpreter already
  running them. Chunk sizes in the streaming producer are deliberately NOT multiples of
  the frame size, so a reframing bug cannot pass by accident of alignment.
"""
from __future__ import annotations

import asyncio
import sys

import pytest

from tests.ai_runtime.config import AiRuntimeConfig
from tests.ai_runtime.local_mic import LocalMicError, LocalMicStream, _argv
from tests.ai_runtime.vad import FRAME_BYTES

# Eight frames of a repeating, position-dependent pattern. Position-dependent so a test can
# tell not just that bytes arrived but that they arrived in order and unshifted.
_PATTERN = bytes(range(256)) * 20
# Written in 97-byte pieces: coprime with 640, so every frame boundary in the test falls
# inside a write rather than on one.
_CHUNK = 97


def _config(**overrides) -> AiRuntimeConfig:
    """Build a config for these tests, with a short open timeout so failures are quick.

    The shortened timeout is a default the caller can still replace, so a test about the
    timeout itself can set its own without colliding with this one.
    """
    return AiRuntimeConfig(**{"mic_open_timeout_s": 1.0, **overrides})


def _producer(body: str):
    """Return an _argv replacement that runs `body` as a python program.

    Args:
        body: python source for the fake capture process.

    Returns:
        A callable with _argv's signature, ignoring the config the way a patched
        function must still accept it.
    """
    return lambda config: [sys.executable, "-c", body]


# Streams the pattern in misaligned pieces, then stays alive so the stream does not end.
_STREAMING = f"""
import sys, time
buf = {_PATTERN!r}
out = sys.stdout.buffer
for i in range(0, len(buf), {_CHUNK}):
    out.write(buf[i:i + {_CHUNK}])
    out.flush()
time.sleep(30)
"""

# Writes two frames and exits cleanly: the mid-stream end-of-audio case.
_TWO_FRAMES_THEN_EXIT = f"""
import sys
sys.stdout.buffer.write(bytes({FRAME_BYTES * 2}))
"""

# Fails the way a wrong or busy ALSA device does: a diagnostic on stderr, then exit.
_FAILS_AT_ONCE = """
import sys
sys.stderr.write("audio open error: No such file or directory\\n")
sys.exit(1)
"""

# Opens successfully but never produces audio -- a mic that is present and silent at the
# driver level, which is what the open timeout exists to catch.
_SILENT = """
import time
time.sleep(30)
"""


def test_argv_requests_the_geometry_the_pipeline_requires():
    """The command line must name the ASR-side format, not merely some capture."""
    argv = _argv(_config(mic_alsa_device="hw:Device,0"))
    # Paired flags are checked as pairs: -f S16_LE and -r 16000 mean nothing apart.
    assert argv[argv.index("-D") + 1] == "hw:Device,0"
    assert argv[argv.index("-f") + 1] == "S16_LE"
    assert argv[argv.index("-r") + 1] == "16000"
    assert argv[argv.index("-c") + 1] == "1"
    # raw, because a wav header would shift every frame boundary in the session.
    assert argv[argv.index("-t") + 1] == "raw"
    # A period of exactly one output frame, so capture arrives on the VAD's own cadence.
    assert "--period-time=20000" in argv


def test_frames_are_whole_and_in_order_across_misaligned_writes(monkeypatch):
    """The pipe splits audio anywhere; the VAD must still see whole 640-byte frames."""
    monkeypatch.setattr("tests.ai_runtime.local_mic._argv", _producer(_STREAMING))

    async def _read_four():
        async with LocalMicStream(_config()) as mic:
            return [await mic.__anext__() for _ in range(4)]

    frames = asyncio.run(_read_four())
    assert [len(f) for f in frames] == [FRAME_BYTES] * 4
    # Concatenating must reproduce the source exactly -- which fails if the open probe's
    # frame were dropped, or if any frame were split or duplicated.
    assert b"".join(frames) == _PATTERN[: FRAME_BYTES * 4]


def test_capture_stopping_mid_session_is_a_fault(monkeypatch):
    """A microphone does not end its own stream, so an end of audio is never routine."""
    monkeypatch.setattr("tests.ai_runtime.local_mic._argv", _producer(_TWO_FRAMES_THEN_EXIT))

    async def _read_past_the_end():
        async with LocalMicStream(_config()) as mic:
            # The two frames the producer wrote arrive normally...
            await mic.__anext__()
            await mic.__anext__()
            # ...and the third read finds the stream gone.
            await mic.__anext__()

    with pytest.raises(LocalMicError) as caught:
        asyncio.run(_read_past_the_end())
    # The exit status is quoted because it is the operator's first clue about the cause.
    assert "stopped capturing" in str(caught.value)


def test_a_device_that_cannot_be_opened_fails_at_open_with_alsas_own_message(monkeypatch):
    """The failure must arrive on entry, carrying the driver's diagnostic verbatim."""
    monkeypatch.setattr("tests.ai_runtime.local_mic._argv", _producer(_FAILS_AT_ONCE))

    async def _open():
        async with LocalMicStream(_config(mic_alsa_device="hw:Nope,0")):
            pass

    with pytest.raises(LocalMicError) as caught:
        asyncio.run(_open())
    message = str(caught.value)
    # The device name, so the operator knows which one was refused...
    assert "hw:Nope,0" in message
    # ...and the driver's own words, which are far more specific than anything this module
    # could synthesise. This also proves stderr was drained before the message was built.
    assert "No such file or directory" in message


def test_a_silent_device_fails_at_open_rather_than_hanging(monkeypatch):
    """No audio must be reported in seconds, not appear as a robot that hears nobody."""
    monkeypatch.setattr("tests.ai_runtime.local_mic._argv", _producer(_SILENT))

    async def _open():
        async with LocalMicStream(_config(mic_open_timeout_s=0.3)):
            pass

    with pytest.raises(LocalMicError) as caught:
        asyncio.run(_open())
    assert "no audio" in str(caught.value)


def test_a_missing_capture_binary_is_named_as_such(monkeypatch):
    """Distinguished from an ALSA fault: the repair is installing alsa-utils, not a mic."""
    monkeypatch.setattr(
        "tests.ai_runtime.local_mic._argv",
        lambda config: ["/nonexistent/xbrain-no-such-capture-binary"],
    )

    async def _open():
        async with LocalMicStream(_config()):
            pass

    with pytest.raises(LocalMicError) as caught:
        asyncio.run(_open())
    assert "cannot start" in str(caught.value)


def test_leaving_the_context_stops_the_capture_process(monkeypatch):
    """A capture left running would hold the card and lock out the next session."""
    monkeypatch.setattr("tests.ai_runtime.local_mic._argv", _producer(_STREAMING))

    async def _open_and_leave():
        stream = LocalMicStream(_config())
        async with stream:
            await stream.__anext__()
        # Returned after __aexit__, so the process must already be reaped -- a returncode
        # of None here would mean a child left behind for every session the harness runs.
        return stream._process.returncode

    assert asyncio.run(_open_and_leave()) is not None
