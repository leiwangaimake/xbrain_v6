"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_audio_io.py
Brief: Unit tests for the office-side microphone capture and loudspeaker playback.

Description:
  Exercises MicCapture and SpeakerPlayback against REAL child processes and REAL pipes,
  with arecord and aplay replaced by small python programs. That substitution is the design
  of this file: everything worth testing here -- reframing across pipe boundaries, the open
  probe, the stderr capture, the drain-before-kill on shutdown -- is subprocess and stream
  behaviour, and a mocked subprocess would test the mock. What is left out is ALSA itself,
  which cannot be faked usefully.

  The producers and consumers are spelled as python -c programs because an office PC has no
  fixture audio tooling, and this keeps the tests dependent on nothing but the interpreter
  already running them. Chunk sizes in the streaming producer are deliberately NOT multiples
  of the frame size, so a reframing bug cannot pass by accident of alignment.
"""
from __future__ import annotations

import asyncio
import sys

import pytest

from tests.office_client.audio_io import (
    AudioIoError,
    MicCapture,
    SpeakerPlayback,
    _capture_argv,
    _playback_argv,
)

# The frame size the intercom announces (plan section 7): 20 ms of 16 kHz mono 16-bit.
_FRAME_BYTES = 640

# Eight frames of a repeating, position-dependent pattern. Position-dependent so a test can
# tell not just that bytes arrived but that they arrived in order and unshifted.
_PATTERN = bytes(range(256)) * 20
# Written in 97-byte pieces: coprime with 640, so every frame boundary in the test falls
# inside a write rather than on one.
_CHUNK = 97


def _program(body: str):
    """Return an argv-builder replacement that runs `body` as a python program.

    Args:
        body: python source for the fake capture or playback process.

    Returns:
        A callable with the argv builders' signature, ignoring the arguments the way a
        patched function must still accept them.
    """
    return lambda device, frame_bytes: [sys.executable, "-c", body]


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
sys.stdout.buffer.write(bytes({_FRAME_BYTES * 2}))
"""

# Fails the way a wrong or busy ALSA device does: a diagnostic on stderr, then exit.
_FAILS_AT_ONCE = """
import sys
sys.stderr.write("audio open error: Device or resource busy\\n")
sys.exit(1)
"""

# Opens successfully but never produces audio -- a microphone that is present and silent at
# the driver level, which is what the open timeout exists to catch.
_SILENT = """
import time
time.sleep(30)
"""


def _consumer(path) -> str:
    """Return a python program that copies stdin to `path` and exits at end of input.

    Used as the fake aplay. Writing to a file is what lets a test prove the bytes actually
    reached the child, and exiting only at EOF is what lets it prove that leaving the
    context closes stdin instead of killing a process that still holds audio.
    """
    return f"""
import sys
data = sys.stdin.buffer.read()
open({str(path)!r}, "wb").write(data)
"""


def test_capture_asks_for_the_geometry_the_intercom_carries():
    """The command line must name the intercom's format, not merely some capture."""
    argv = _capture_argv("plughw:1,0", _FRAME_BYTES)
    # Paired flags are checked as pairs: -f S16_LE and -r 16000 mean nothing apart.
    assert argv[argv.index("-D") + 1] == "plughw:1,0"
    assert argv[argv.index("-f") + 1] == "S16_LE"
    assert argv[argv.index("-r") + 1] == "16000"
    assert argv[argv.index("-c") + 1] == "1"
    # raw, because a wav header would shift every frame boundary in the session.
    assert argv[argv.index("-t") + 1] == "raw"
    # A period of exactly one intercom frame, so capture arrives on the socket's cadence.
    assert "--period-time=20000" in argv


def test_playback_asks_for_the_same_geometry_as_capture():
    """The two directions carry identical PCM; a mismatch would be audible only on hardware."""
    capture = _capture_argv("default", _FRAME_BYTES)
    playback = _playback_argv("default", _FRAME_BYTES)
    # Everything but the binary name must match, which is stronger than checking the same
    # flags twice: it also catches a period or buffer size that drifted apart.
    assert capture[1:] == playback[1:]


def test_the_buffer_stays_short_enough_for_a_conversation():
    """ALSA's own default would buffer a half second, and a person hears that as a delay."""
    argv = _playback_argv("default", _FRAME_BYTES)
    buffer_us = int(next(a for a in argv if a.startswith("--buffer-time=")).split("=")[1])
    assert buffer_us <= 150_000


def test_frames_are_whole_and_in_order_across_misaligned_writes(monkeypatch):
    """The pipe splits audio anywhere; the intercom must still see whole 640-byte frames."""
    monkeypatch.setattr(
        "tests.office_client.audio_io._capture_argv", _program(_STREAMING)
    )

    async def _read_four():
        async with MicCapture("default", _FRAME_BYTES, 1.0) as mic:
            return [await mic.__anext__() for _ in range(4)]

    frames = asyncio.run(_read_four())
    assert [len(f) for f in frames] == [_FRAME_BYTES] * 4
    # Concatenating must reproduce the source exactly -- which fails if the open probe's
    # frame were dropped, or if any frame were split or duplicated.
    assert b"".join(frames) == _PATTERN[: _FRAME_BYTES * 4]


def test_capture_stopping_mid_session_is_a_fault(monkeypatch):
    """A microphone does not end its own stream, so an end of audio is never routine."""
    monkeypatch.setattr(
        "tests.office_client.audio_io._capture_argv", _program(_TWO_FRAMES_THEN_EXIT)
    )

    async def _read_past_the_end():
        async with MicCapture("default", _FRAME_BYTES, 1.0) as mic:
            # The two frames the producer wrote arrive normally...
            await mic.__anext__()
            await mic.__anext__()
            # ...and the third read finds the stream gone.
            await mic.__anext__()

    with pytest.raises(AudioIoError) as caught:
        asyncio.run(_read_past_the_end())
    assert "stopped capturing" in str(caught.value)


def test_a_device_that_cannot_be_opened_fails_at_open_with_alsas_own_message(monkeypatch):
    """The failure must arrive on entry, carrying the driver's diagnostic verbatim."""
    monkeypatch.setattr(
        "tests.office_client.audio_io._capture_argv", _program(_FAILS_AT_ONCE)
    )

    async def _open():
        async with MicCapture("plughw:9,0", _FRAME_BYTES, 1.0):
            pass

    with pytest.raises(AudioIoError) as caught:
        asyncio.run(_open())
    message = str(caught.value)
    # The device name, so the operator knows which one was refused...
    assert "plughw:9,0" in message
    # ...and the driver's own words, which are far more specific than anything this module
    # could synthesise. This also proves stderr was drained before the message was built.
    assert "Device or resource busy" in message


def test_a_silent_device_fails_at_open_rather_than_hanging(monkeypatch):
    """No audio must be reported in seconds, not appear as an intercom nobody talks on."""
    monkeypatch.setattr("tests.office_client.audio_io._capture_argv", _program(_SILENT))

    async def _open():
        async with MicCapture("default", _FRAME_BYTES, 0.3):
            pass

    with pytest.raises(AudioIoError) as caught:
        asyncio.run(_open())
    assert "no audio" in str(caught.value)


def test_a_missing_alsa_binary_is_named_as_such(monkeypatch):
    """Distinguished from a device fault: the repair is installing alsa-utils, not a mic."""
    monkeypatch.setattr(
        "tests.office_client.audio_io._capture_argv",
        lambda device, frame_bytes: ["/nonexistent/xbrain-no-such-capture-binary"],
    )

    async def _open():
        async with MicCapture("default", _FRAME_BYTES, 1.0):
            pass

    with pytest.raises(AudioIoError) as caught:
        asyncio.run(_open())
    assert "cannot start" in str(caught.value)


def test_leaving_the_context_stops_the_capture_process(monkeypatch):
    """A capture left running would hold the card and lock out the next session."""
    monkeypatch.setattr(
        "tests.office_client.audio_io._capture_argv", _program(_STREAMING)
    )

    async def _open_and_leave():
        capture = MicCapture("default", _FRAME_BYTES, 1.0)
        async with capture:
            await capture.__anext__()
        # Returned after __aexit__, so the process must already be reaped -- a returncode
        # of None here would mean a child left behind for every session the operator runs.
        return capture._process

    assert asyncio.run(_open_and_leave()) is None


def test_played_audio_reaches_the_soundcard_intact(monkeypatch, tmp_path):
    """Playback is the listen direction; a byte lost here is a word the operator misses."""
    sink = tmp_path / "played.raw"
    monkeypatch.setattr(
        "tests.office_client.audio_io._playback_argv", _program(_consumer(sink))
    )

    async def _play_four():
        async with SpeakerPlayback("default", _FRAME_BYTES) as speaker:
            for offset in range(0, _FRAME_BYTES * 4, _FRAME_BYTES):
                await speaker.play(_PATTERN[offset:offset + _FRAME_BYTES])

    asyncio.run(_play_four())
    # Written by the child only after it saw end of input, so this passing also proves the
    # context closed stdin and waited, rather than killing a process still holding audio --
    # which on real hardware is the difference between hearing the last word and not.
    assert sink.read_bytes() == _PATTERN[: _FRAME_BYTES * 4]


def test_playing_before_the_device_is_open_is_refused():
    """Silently accepting audio with nowhere to send it would be an intercom that looks fine."""

    async def _play_unopened():
        await SpeakerPlayback("default", _FRAME_BYTES).play(b"\x00" * _FRAME_BYTES)

    with pytest.raises(AudioIoError) as caught:
        asyncio.run(_play_unopened())
    assert "not running" in str(caught.value)


def test_a_missing_playback_binary_is_named_as_such(monkeypatch):
    """Same reasoning as capture: the operator's repair is a package, not the hardware."""
    monkeypatch.setattr(
        "tests.office_client.audio_io._playback_argv",
        lambda device, frame_bytes: ["/nonexistent/xbrain-no-such-playback-binary"],
    )

    async def _open():
        async with SpeakerPlayback("default", _FRAME_BYTES):
            pass

    with pytest.raises(AudioIoError) as caught:
        asyncio.run(_open())
    assert "cannot start" in str(caught.value)
