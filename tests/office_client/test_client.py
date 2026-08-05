"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_client.py
Brief: Unit tests for the office end of the 功能2 intercom.

Description:
  Three things in client.py can be wrong in ways nothing else would catch, and they are
  what this file tests.

  The ORDER of a press and a release. The server opens the device loudspeaker path when it
  sees a press and closes it when it sees a release, so a control message that overtakes
  the audio around it either loses the start of a sentence or cuts off its end. The socket
  is faked, which is exactly what makes the order observable: the fake records what it was
  handed, in the order it was handed it.

  The FORMAT check on the header. This client has one PCM geometry wired through it, and
  the failure it is guarding against -- audio at the wrong speed -- is the one audio fault
  that produces no error anywhere, because every byte count is still correct.

  The wav TALK SOURCE, which on a PC with no microphone is the only way the talk direction
  exists at all. Its pacing and its replay-from-the-start behaviour are what make a timing
  measurement repeatable.

  The office soundcard is not tested here; it has its own file. This one fakes it, so a
  routing test cannot fail because a machine running the suite has no audio hardware.
"""
from __future__ import annotations

import asyncio
import json
import time
import wave

import pytest
from websockets.exceptions import ConnectionClosed
from websockets.frames import Close

from tests.office_client.audio_io import AudioIoError
from tests.office_client.client import (
    IntercomClient,
    IntercomClientError,
    WavTalkSource,
    _DOWNLINK_LOG_FRAMES,
    _PTT_IDLE,
    _PTT_LISTEN,
    _PTT_TALK,
)

_FRAME_BYTES = 640
# A header exactly as intercom.py sends it, used as the baseline the malformed cases vary.
_HEADER = {
    "encoding": "s16le",
    "sample_rate": 16000,
    "channels": 1,
    "frame_bytes": _FRAME_BYTES,
}
# Position-dependent audio, so a test can tell which frame arrived and not merely that one did.
_PATTERN = bytes(range(256)) * 20


class _FakeConnection:
    """Stands in for the intercom websocket, replaying a script and recording sends.

    Recording rather than checking is deliberate: the properties under test are about the
    ORDER of control messages and audio, which only a record of everything can show.
    """

    def __init__(self, inbound=()) -> None:
        """Take the messages the robot will send, in order.

        Args:
            inbound: str control messages, bytes audio, or an exception to raise instead.
                When exhausted, recv reports the connection dropped.
        """
        self._inbound = list(inbound)
        self.sent = []

    async def recv(self):
        """Return the next scripted message, or report the robot hung up."""
        if not self._inbound:
            raise ConnectionClosed(None, None)
        item = self._inbound.pop(0)
        if isinstance(item, BaseException):
            raise item
        return item

    async def send(self, message) -> None:
        """Record what the client put on the wire."""
        self.sent.append(message)

    def controls(self):
        """Return only the PTT states sent, in order."""
        return [json.loads(m)["ptt"] for m in self.sent if isinstance(m, str)]

    def audio(self):
        """Return only the audio frames sent, in order."""
        return [m for m in self.sent if isinstance(m, bytes)]


class _FakeSpeaker:
    """Stands in for the office loudspeaker, recording what it was asked to play."""

    def __init__(self) -> None:
        """Start with nothing played."""
        self.played = []

    async def play(self, frame: bytes) -> None:
        """Record one frame."""
        self.played.append(frame)


class _FakeTalk:
    """A talk source that never runs out, so a burst ends only when the client stops it.

    Counting bursts is what makes the "pressing twice does not restart the sentence" test
    meaningful: a restart is invisible in the audio if every frame is identical, but it is
    unmistakable in the number of times the source was asked to begin.
    """

    def __init__(self) -> None:
        """Start with no bursts requested."""
        self.bursts = 0

    async def frames(self):
        """Yield audio until the caller stops iterating."""
        self.bursts += 1
        while True:
            yield b"\x07" * _FRAME_BYTES
            # A real source produces frames over time; without this the burst would spin
            # the loop and starve the very task that is meant to cancel it.
            await asyncio.sleep(0.005)


def _client(talk_wav=None) -> IntercomClient:
    """Build a client with the audio devices left at defaults; they are always faked here."""
    return IntercomClient(
        url="ws://127.0.0.1:18082",
        speaker_device="default",
        mic_device="default",
        talk_wav=talk_wav,
        open_timeout_s=1.0,
    )


def _write_wav(path, pcm: bytes, rate: int = 16000, channels: int = 1, width: int = 2) -> str:
    """Write a wav file for the talk-source tests and return its path."""
    with wave.open(str(path), "wb") as handle:
        handle.setnchannels(channels)
        handle.setsampwidth(width)
        handle.setframerate(rate)
        handle.writeframes(pcm)
    return str(path)


def _header_error(raw) -> str:
    """Run the header check against one message and return the complaint it produced."""

    async def _drive():
        await _client()._receive_header(_FakeConnection([raw]))

    with pytest.raises(IntercomClientError) as caught:
        asyncio.run(_drive())
    return str(caught.value)


def test_a_matching_header_is_accepted_and_supplies_the_frame_size():
    """The frame size comes from the server, so the two ends cannot disagree about it."""

    async def _drive():
        return await _client()._receive_header(_FakeConnection([json.dumps(_HEADER)]))

    assert _drive is not None
    assert asyncio.run(_drive())["frame_bytes"] == _FRAME_BYTES


@pytest.mark.parametrize(
    "field,value",
    [
        # The one that matters most: wrong-speed audio is the audio fault that reports
        # nothing wrong anywhere, because every byte count still adds up.
        ("sample_rate", 48000),
        ("channels", 2),
        ("encoding", "f32le"),
    ],
)
def test_a_format_this_client_cannot_speak_is_refused_rather_than_adapted_to(field, value):
    """Adapting would hide the mismatch; refusing puts it in front of the operator."""
    message = _header_error(json.dumps({**_HEADER, field: value}))
    assert field in message
    assert str(value) in message


@pytest.mark.parametrize("frame_bytes", [0, -640, 641, "640", None])
def test_an_unusable_frame_size_is_refused(frame_bytes):
    """Odd sizes split a 16-bit sample, which shifts every sample after it into noise."""
    assert "frame_bytes" in _header_error(json.dumps({**_HEADER, "frame_bytes": frame_bytes}))


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("not json at all", "not valid json"),
        # Valid json, wrong shape.
        ('"s16le"', "not a json object"),
        # Audio before the header: the server would have to be badly broken, and guessing
        # a format from it is exactly the adaptation this client refuses to do.
        (b"\x00" * 640, "before its format header"),
    ],
)
def test_a_header_that_is_not_a_header_is_refused(raw, expected):
    """A client that carried on here would be guessing at the format of live audio."""
    assert expected in _header_error(raw)


def test_a_busy_channel_says_so_instead_of_reporting_a_broken_link():
    """1013 means another operator holds the channel: the remedy is to wait, not to debug."""
    closed = ConnectionClosed(Close(1013, "intercom already in use"), None)
    assert "already in use" in _header_error(closed)


def test_pressing_sends_the_control_message_before_any_audio():
    """The server opens the loudspeaker path on the press; audio sent earlier is discarded."""

    async def _drive():
        connection = _FakeConnection()
        client = _client()
        talk = _FakeTalk()
        await client._set_ptt(connection, talk, _PTT_TALK)
        # Let the uplink run so there is audio to be out of order with.
        await asyncio.sleep(0.05)
        await client._stop_uplink()
        return connection.sent

    sent = asyncio.run(_drive())
    assert json.loads(sent[0])["ptt"] == _PTT_TALK
    # Everything after the press is audio -- nothing slipped in front of it.
    assert all(isinstance(m, bytes) for m in sent[1:])
    assert len(sent) > 1


def test_releasing_sends_the_last_audio_before_the_control_message():
    """The server closes the loudspeaker path on the release, cutting anything sent after."""

    async def _drive():
        connection = _FakeConnection()
        client = _client()
        talk = _FakeTalk()
        await client._set_ptt(connection, talk, _PTT_TALK)
        await asyncio.sleep(0.05)
        await client._set_ptt(connection, talk, _PTT_IDLE)
        return connection.sent

    sent = asyncio.run(_drive())
    # The release is last, so no frame of the sentence was stranded behind it.
    assert json.loads(sent[-1])["ptt"] == _PTT_IDLE
    assert isinstance(sent[-2], bytes)


def test_pressing_twice_does_not_restart_the_sentence():
    """A second press mid-sentence would replay a wav source from the top."""

    async def _drive():
        connection = _FakeConnection()
        client = _client()
        talk = _FakeTalk()
        await client._set_ptt(connection, talk, _PTT_TALK)
        await asyncio.sleep(0.02)
        await client._set_ptt(connection, talk, _PTT_TALK)
        await asyncio.sleep(0.02)
        await client._stop_uplink()
        return connection.controls(), talk.bursts

    controls, bursts = asyncio.run(_drive())
    assert controls == [_PTT_TALK]
    assert bursts == 1


def test_the_console_drives_the_channel_and_quit_ends_it():
    """The console loop is the single owner of PTT state; every change goes through it."""

    async def _drive():
        connection = _FakeConnection()
        client = _client()
        for command in ("t", "listen", "i", "q"):
            client._commands.put_nowait(command)
        await client._run_console(connection, _FakeTalk())
        return connection.controls()

    # Both the letter and the whole word are accepted, and quit returns rather than being
    # sent to the robot as a fourth state.
    assert asyncio.run(_drive()) == [_PTT_TALK, _PTT_LISTEN, _PTT_IDLE]


def test_an_unknown_command_changes_nothing():
    """A typo must not put the channel in a state the operator did not ask for."""

    async def _drive():
        connection = _FakeConnection()
        client = _client()
        for command in ("talkk", "", "q"):
            client._commands.put_nowait(command)
        await client._run_console(connection, _FakeTalk())
        return connection.sent

    assert asyncio.run(_drive()) == []


def test_a_finished_talk_source_releases_the_channel():
    """A wav that has played out must not leave the robot's loudspeaker held open."""

    class _OneFrame:
        """A source with exactly one frame, standing in for a wav that ends."""

        async def frames(self):
            """Yield once and stop."""
            yield b"\x08" * _FRAME_BYTES

    async def _drive():
        client = _client()
        await client._pump_uplink(_FakeConnection(), _OneFrame())
        return client._commands.get_nowait()

    # Asked for through the queue rather than applied here, because leaving talk cancels
    # this very task and a task cannot await its own cancellation.
    assert asyncio.run(_drive()) == _PTT_IDLE


def test_everything_the_robot_sends_is_played():
    """The server is the authority on what may be sent; re-gating here would clip frames."""
    frames = [_PATTERN[i * _FRAME_BYTES:(i + 1) * _FRAME_BYTES] for i in range(3)]

    async def _drive():
        speaker = _FakeSpeaker()
        # The local state is idle throughout: audio still plays, because the decision about
        # whether to send it was already made at the end that owns the microphone.
        await _client()._pump_downlink(_FakeConnection(frames), speaker)
        return speaker.played

    assert asyncio.run(_drive()) == frames


def test_arriving_audio_is_reported_periodically_so_silence_can_be_told_from_a_dead_link(caplog):
    """A listening operator who hears nothing needs to know whether frames are still coming.

    A quiet room and a stopped uplink sound identical on the loudspeaker and have completely
    different remedies, so the count is the only thing that separates them. The report is
    driven by frames rather than by a clock precisely so that it STOPS when the audio does.
    """
    frame = b"\x00" * _FRAME_BYTES
    # One frame short of the third report, so a wrong comparison in either direction shows up.
    frames = [frame] * (_DOWNLINK_LOG_FRAMES * 2 + _DOWNLINK_LOG_FRAMES // 2)

    async def _drive():
        await _client()._pump_downlink(_FakeConnection(frames), _FakeSpeaker())

    with caplog.at_level("INFO", logger="office_client.client"):
        asyncio.run(_drive())
    reports = [r.getMessage() for r in caplog.records if "frames received" in r.getMessage()]
    assert reports == [
        f"listening: {_DOWNLINK_LOG_FRAMES} frames received",
        f"listening: {_DOWNLINK_LOG_FRAMES * 2} frames received",
    ]


def test_a_fault_close_from_the_robot_is_reported_with_its_reason():
    """The server names its own cause; passing it on is what makes the failure diagnosable."""

    async def _drive():
        closed = ConnectionClosed(Close(1011, "no audio from ALSA device 'default'"), None)
        await _client()._pump_downlink(_FakeConnection([closed]), _FakeSpeaker())

    with pytest.raises(IntercomClientError) as caught:
        asyncio.run(_drive())
    assert "no audio from ALSA device" in str(caught.value)


def test_a_normal_close_ends_the_session_without_an_error():
    """The operator on the robot end stopping AI_runtime is not a client fault."""

    async def _drive():
        closed = ConnectionClosed(Close(1000, ""), None)
        await _client()._pump_downlink(_FakeConnection([closed]), _FakeSpeaker())

    # Returns rather than raising: the exit status of the process depends on this.
    asyncio.run(_drive())


def test_a_wav_at_the_wrong_rate_is_refused_with_both_rates_named(tmp_path):
    """The operator's fix is a conversion, and they need to know from what to what."""
    path = _write_wav(tmp_path / "wrong.wav", b"\x00" * 1024, rate=44100)

    async def _drive():
        async with WavTalkSource(path, _FRAME_BYTES):
            pass

    with pytest.raises(IntercomClientError) as caught:
        asyncio.run(_drive())
    message = str(caught.value)
    assert "44100" in message and "16000" in message


def test_a_stereo_wav_is_refused(tmp_path):
    """Stereo at the right rate would play at half speed, which no byte count would reveal."""
    path = _write_wav(tmp_path / "stereo.wav", b"\x00" * 2048, channels=2)

    async def _drive():
        async with WavTalkSource(path, _FRAME_BYTES):
            pass

    with pytest.raises(IntercomClientError):
        asyncio.run(_drive())


def test_a_missing_wav_names_the_file(tmp_path):
    """A mistyped path must not look like a microphone problem."""

    async def _drive():
        async with WavTalkSource(str(tmp_path / "absent.wav"), _FRAME_BYTES):
            pass

    with pytest.raises(IntercomClientError) as caught:
        asyncio.run(_drive())
    assert "absent.wav" in str(caught.value)


def test_a_short_final_frame_is_padded_rather_than_dropped(tmp_path):
    """Silence at the end costs 20 ms; a short frame misaligns everything after it."""
    # Two whole frames plus a fragment.
    path = _write_wav(tmp_path / "ragged.wav", _PATTERN[: _FRAME_BYTES * 2 + 100])

    async def _drive():
        async with WavTalkSource(path, _FRAME_BYTES) as talk:
            return [frame async for frame in talk.frames()]

    frames = asyncio.run(_drive())
    assert [len(f) for f in frames] == [_FRAME_BYTES] * 3
    # The audio itself is untouched; only the tail was padded.
    assert b"".join(frames)[: _FRAME_BYTES * 2 + 100] == _PATTERN[: _FRAME_BYTES * 2 + 100]


def test_each_press_replays_the_wav_from_the_start(tmp_path):
    """A repeatable utterance is the point of a file source; a stateful one would not be."""
    path = _write_wav(tmp_path / "hello.wav", _PATTERN[: _FRAME_BYTES * 3])

    async def _drive():
        async with WavTalkSource(path, _FRAME_BYTES) as talk:
            first = [frame async for frame in talk.frames()]
            second = [frame async for frame in talk.frames()]
            return first, second

    first, second = asyncio.run(_drive())
    assert first == second
    assert len(first) == 3


def test_the_wav_is_sent_at_the_speed_it_would_be_spoken(tmp_path):
    """Everything downstream is sized for audio arriving as fast as a person says it."""
    # Ten frames, which is 200 ms of audio.
    path = _write_wav(tmp_path / "paced.wav", b"\x00" * (_FRAME_BYTES * 10))

    async def _drive():
        started = time.monotonic()
        async with WavTalkSource(path, _FRAME_BYTES) as talk:
            async for _ in talk.frames():
                pass
        return time.monotonic() - started

    # Well under the 200 ms the audio represents, because the assertion that matters is
    # that it was paced at all: an unpaced source finishes in microseconds.
    assert asyncio.run(_drive()) >= 0.15


def test_the_uplink_reports_a_dead_microphone_instead_of_going_quiet(tmp_path):
    """A transmission that silently stops is the hardest fault to notice from the far end."""

    class _DeadMic:
        """A talk source whose device fails part way through a burst."""

        async def frames(self):
            """Yield once, then fail the way capture does when the card goes away."""
            yield b"\x09" * _FRAME_BYTES
            raise AudioIoError("arecord stopped capturing (exit 1)")

    async def _drive():
        client = _client()
        await client._pump_uplink(_FakeConnection(), _DeadMic())
        return client._failure, client._commands.get_nowait()

    failure, command = asyncio.run(_drive())
    # Stored so run() can re-raise it and the process exits non-zero...
    assert isinstance(failure, AudioIoError)
    # ...and the console is asked to quit, so the operator is not left holding a dead channel.
    assert command == "q"
