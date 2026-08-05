"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_intercom.py
Brief: Unit tests for the 功能2 half-duplex intercom routing.

Description:
  What is worth testing in intercom.py is the GATE: which frames move, in which direction,
  in which PTT state, and when. Everything else it does -- the websocket transport, ALSA
  capture, the device loudspeaker -- belongs to modules that already have their own tests
  or can only be exercised on hardware.

  So the microphone and the loudspeaker path are replaced by fakes that record what they
  were given, and the tests assert on the routing decisions. The fakes are deliberately
  thin: they answer the same calls the real classes do and nothing more, so a test cannot
  pass because the fake was more forgiving than payload-service.

  The half-duplex property is the one that has to hold on real hardware, because the
  microphone and the loudspeaker are on the same robot a short distance apart with no AEC
  anywhere (issue A5). A leak in either direction is not a cosmetic bug there -- it is
  audible feedback. That is why several tests here check that audio was DROPPED, which is
  otherwise an easy thing to leave untested: nothing arriving looks like nothing happening.
"""
from __future__ import annotations

import asyncio
import json

import pytest
from websockets.exceptions import ConnectionClosed

from tests.ai_runtime.config import AiRuntimeConfig
from tests.ai_runtime.intercom import (
    PTT_IDLE,
    PTT_LISTEN,
    PTT_TALK,
    IntercomError,
    IntercomServer,
    IntercomSession,
    _parse_control,
)
from tests.ai_runtime.local_mic import LocalMicError
from tests.ai_runtime.payload_client import MODE_FUNC2, MODE_IDLE

# Position-dependent frames, so a test can tell not just that audio arrived but which
# audio: a gate that forwarded the wrong frame would otherwise look identical to one that
# forwarded the right one.
_MIC_FRAMES = [bytes([i]) * 640 for i in range(1, 6)]


def _config(**overrides) -> AiRuntimeConfig:
    """Build a config for these tests, with the turnaround off unless a test wants it.

    Zero turnaround is the right default here because it is one specific behaviour with one
    test of its own; leaving it on would make every OTHER test's assertions depend on wall
    clock timing they are not about.
    """
    return AiRuntimeConfig(**{"intercom_turnaround_ms": 0, **overrides})


class _FakeConnection:
    """Stands in for the office-client websocket, replaying a script and recording sends."""

    def __init__(self, inbound) -> None:
        """Take the messages the client will send, in order.

        Args:
            inbound: str control messages and bytes audio frames. When exhausted, recv
                raises ConnectionClosed, which is how a real client disconnecting looks.
        """
        self._inbound = list(inbound)
        self.sent = []
        self.closed = None

    async def recv(self):
        """Return the next scripted message, or report the client hung up."""
        if not self._inbound:
            raise ConnectionClosed(None, None)
        # Yield to the loop on every message so the microphone pump gets a turn between
        # them; without this the office pump would drain its whole script before the robot
        # pump ran once, and no test of the gate would mean anything.
        await asyncio.sleep(0)
        return self._inbound.pop(0)

    async def send(self, message) -> None:
        """Record what the server routed to the office."""
        self.sent.append(message)

    async def close(self, code: int = 1000, reason: str = "") -> None:
        """Record the close, as the busy and fault paths do."""
        self.closed = (code, reason)

    def audio(self):
        """Return only the audio frames sent, dropping the format header."""
        return [item for item in self.sent if isinstance(item, bytes)]


class _FakeMic:
    """Stands in for the microphone on the robot, yielding a fixed script then idling."""

    def __init__(self, config) -> None:
        """Accept the config the real stream takes, and ignore it."""

    async def __aenter__(self):
        """Open: nothing to do."""
        return self

    async def __aexit__(self, exc_type, exc, traceback) -> None:
        """Close: nothing to do."""

    def __aiter__(self):
        """Yield the scripted frames, then stay open.

        Staying open rather than ending matters: a real microphone has no end, and a fake
        that stopped would end the session early and hide whatever the test came to check.
        """

        async def _frames():
            for frame in _MIC_FRAMES:
                yield frame
                # One scheduling turn per frame, so the office pump can interleave its
                # control messages between them the way a real operator's would arrive.
                await asyncio.sleep(0)
            await asyncio.sleep(30)

        return _frames()


class _FakePlay:
    """Stands in for the device loudspeaker path, recording its own lifecycle.

    Records opens and closes as well as audio because the lifecycle IS the behaviour under
    test: payload-service ends a hail when the socket closes, so an extra open or a missing
    close is a real fault on the device, not just an internal detail.
    """

    instances = []

    def __init__(self, config) -> None:
        """Register this instance so a test can count how many were opened."""
        self.frames = []
        self.entered = False
        self.exited = False
        _FakePlay.instances.append(self)

    async def __aenter__(self):
        """Open the loudspeaker path."""
        self.entered = True
        return self

    async def __aexit__(self, exc_type, exc, traceback) -> None:
        """Close it, which is what ends the hail on the real device."""
        self.exited = True

    async def send(self, frame: bytes) -> None:
        """Record audio routed to the loudspeaker."""
        self.frames.append(frame)


@pytest.fixture(autouse=True)
def _fakes(monkeypatch):
    """Replace the microphone and the loudspeaker path for every test in this file."""
    _FakePlay.instances = []
    monkeypatch.setattr("tests.ai_runtime.intercom.LocalMicStream", _FakeMic)
    monkeypatch.setattr("tests.ai_runtime.intercom.PlayStream", _FakePlay)


def _run_session(inbound, config=None) -> _FakeConnection:
    """Run one session against a scripted client and return the connection it used.

    Args:
        inbound: the messages the fake office-client sends.
        config: an optional config, defaulting to zero turnaround.

    Returns:
        The connection, holding everything the server sent back.
    """
    connection = _FakeConnection(inbound)
    asyncio.run(IntercomSession(config or _config(), connection).run())
    return connection


def _ptt(state: str) -> str:
    """Format a PTT control message the way office-client does."""
    return json.dumps({"ptt": state})


async def _finite_mic():
    """Yield the scripted frames and stop.

    The listen-direction tests drive _pump_robot directly with this instead of running a
    whole session, because a session ends when whichever pump finishes first does, and how
    many microphone frames got through before that is a scheduling accident. A test that
    asserted on it would be asserting on the event loop, and would eventually fail for
    reasons that have nothing to do with the gate.
    """
    for frame in _MIC_FRAMES:
        yield frame


def _listen_after(states, config=None):
    """Apply a sequence of PTT changes, then push one microphone script through the gate.

    Args:
        states: the PTT states to move through, in order.
        config: an optional config, defaulting to zero turnaround.

    Returns:
        The audio frames that reached the office.

    Everything happens inside ONE asyncio.run because the turnaround deadline is measured
    against the running loop's clock: split across two runs, the deadline would be compared
    with a different loop's time origin and the gate would appear to work by accident.
    """

    async def _drive():
        connection = _FakeConnection([])
        session = IntercomSession(config or _config(), connection)
        for state in states:
            await session._set_ptt(state)
        await session._pump_robot(_finite_mic())
        return connection.audio()

    return asyncio.run(_drive())


def test_control_messages_accept_exactly_the_three_states():
    """The wire protocol is three states; anything else is a client bug, not a default."""
    assert _parse_control(_ptt(PTT_TALK)) == PTT_TALK
    assert _parse_control(_ptt(PTT_LISTEN)) == PTT_LISTEN
    assert _parse_control(_ptt(PTT_IDLE)) == PTT_IDLE


@pytest.mark.parametrize(
    "raw",
    [
        "not json at all",
        # A bare string is valid json but not the object the protocol specifies.
        '"talk"',
        # The right shape with a state that does not exist -- the typo case, which must not
        # be silently mapped onto idle: that would look connected and carry nothing.
        '{"ptt": "tak"}',
        # The key missing entirely.
        "{}",
    ],
)
def test_malformed_control_messages_are_refused(raw):
    """A control message that cannot be honoured must say so rather than be guessed at."""
    with pytest.raises(IntercomError):
        _parse_control(raw)


def test_the_client_is_told_the_audio_format_before_anything_else():
    """office-client sizes its soundcard from this header, so it must arrive first."""
    connection = _run_session([])
    header = json.loads(connection.sent[0])
    assert header["encoding"] == "s16le"
    assert header["sample_rate"] == 16000
    assert header["channels"] == 1
    # Even, because a 16-bit sample split across two frames would shift every sample after
    # it by one byte and turn the audio into noise.
    assert header["frame_bytes"] > 0 and header["frame_bytes"] % 2 == 0


def test_the_robot_is_not_heard_until_the_operator_asks_to_listen():
    """Idle is a real state: the channel is held, and nothing is routed either way."""
    assert _listen_after([]) == []


def test_listening_forwards_the_robots_microphone_intact():
    """The listen direction is the whole point of the robot end; it must actually flow.

    Every frame, in order, byte for byte. Reblocking or reordering here would be inaudible
    in a log and unmistakable in a loudspeaker.
    """
    assert _listen_after([PTT_LISTEN]) == _MIC_FRAMES


def test_the_microphone_is_muted_again_when_the_operator_stops_listening():
    """Leaving listen must stop the uplink, or the operator hears a channel they closed."""
    assert _listen_after([PTT_LISTEN, PTT_IDLE]) == []


def test_talking_opens_the_loudspeaker_and_releasing_closes_it():
    """Closing the /play socket is what makes payload-service end the hail cleanly."""
    _run_session([_ptt(PTT_TALK), b"\x01" * 640, _ptt(PTT_IDLE)])
    assert len(_FakePlay.instances) == 1
    play = _FakePlay.instances[0]
    assert play.entered and play.exited
    assert play.frames == [b"\x01" * 640]


def test_a_session_that_ends_mid_transmission_still_closes_the_loudspeaker():
    """A dropped client must not leave the robot's speaker parked on its last frame."""
    # No release: the script simply runs out, which is a client disconnecting while talking.
    _run_session([_ptt(PTT_TALK), b"\x02" * 640])
    assert _FakePlay.instances[0].exited


def test_office_audio_is_dropped_unless_the_operator_is_talking():
    """The server owns the loudspeaker, so it enforces half-duplex regardless of the client.

    A client whose microphone keeps streaming after release must not be able to keep the
    robot talking: the enforcement that matters is the one at the end that owns the device.
    """
    _run_session([b"\x03" * 640, _ptt(PTT_LISTEN), b"\x04" * 640])
    # Not merely "no frames delivered" -- no loudspeaker path was opened at all.
    assert _FakePlay.instances == []


def test_pressing_talk_twice_does_not_restart_the_transmission():
    """Re-entering talk would tear down a working /play socket in mid-sentence."""
    _run_session([_ptt(PTT_TALK), b"\x05" * 640, _ptt(PTT_TALK), b"\x06" * 640])
    assert len(_FakePlay.instances) == 1
    # Both frames went to the same socket, so the sentence was not cut in half.
    assert _FakePlay.instances[0].frames == [b"\x05" * 640, b"\x06" * 640]


def test_the_operators_own_voice_does_not_come_back_after_release():
    """The turnaround guard exists because releasing PTT does not silence the robot at once.

    The loudspeaker and the microphone are on the same dog with no AEC, so the sound already
    in the air still reaches the microphone after the socket closes. Forwarding immediately
    would send the operator the tail of their own sentence.
    """
    # A turnaround longer than the whole test, so any frame forwarded after the release is
    # a gate failure and not a timing margin that happened to expire.
    assert _listen_after(
        [PTT_TALK, PTT_LISTEN], _config(intercom_turnaround_ms=30_000)
    ) == []


def test_the_turnaround_guard_is_a_delay_and_not_a_mute():
    """With the guard expired, talk-then-listen must carry audio like any other listen.

    The companion to the test above: a gate that dropped everything after a transmission
    would satisfy that one perfectly and leave the operator with a dead channel for the
    rest of the session.
    """
    assert _listen_after([PTT_TALK, PTT_LISTEN]) == _MIC_FRAMES


def test_the_intercom_holds_the_payload_in_func2_and_hands_it_back(monkeypatch):
    """func2 is what makes payload-service accept /play at all, and R3 wants it returned."""
    modes = []

    def _ensure(config, mode):
        modes.append(mode)
        # True means this process performed the switch, so it owes the restore.
        return True

    monkeypatch.setattr("tests.ai_runtime.intercom.ensure_mode", _ensure)
    # serve() is replaced rather than actually bound: this test is about the mode bracket,
    # and binding a real port would make it fail on a machine already running the intercom.
    monkeypatch.setattr(
        "tests.ai_runtime.intercom.serve", lambda *args, **kwargs: _StubServer()
    )
    asyncio.run(IntercomServer(_config()).run())
    assert modes == [MODE_FUNC2, MODE_IDLE]


def test_a_payload_already_in_func2_is_left_that_way(monkeypatch):
    """A process that did not take the device must not give it back: something else has it."""
    modes = []

    def _ensure(config, mode):
        modes.append(mode)
        # False means the payload was already in the requested mode.
        return False

    monkeypatch.setattr("tests.ai_runtime.intercom.ensure_mode", _ensure)
    monkeypatch.setattr(
        "tests.ai_runtime.intercom.serve", lambda *args, **kwargs: _StubServer()
    )
    asyncio.run(IntercomServer(_config()).run())
    assert modes == [MODE_FUNC2]


def test_a_second_office_client_is_refused_rather_than_mixed_in():
    """R2 gives payload one audio client, and two operators on one speaker is worse than one."""

    async def _drive():
        server = IntercomServer(_config())
        # Mark the channel held, exactly as serving a first client does.
        server._busy = True
        connection = _FakeConnection([])
        await server._handle(connection)
        return connection

    connection = asyncio.run(_drive())
    # 1013 is "try again later": the channel is held, not broken, so retrying is correct
    # client behaviour and a permanent error code would be a lie.
    assert connection.closed[0] == 1013


def test_a_session_fault_tells_the_client_why_and_leaves_the_server_listening(monkeypatch):
    """One operator's dead microphone must not take down the listener for the next one."""

    class _DeadMic:
        """A microphone that refuses to open, the way a missing or busy card does."""

        def __init__(self, config) -> None:
            """Accept the config the real stream takes, and ignore it."""

        async def __aenter__(self):
            """Fail on open, which is where a real capture fault surfaces."""
            raise LocalMicError("no audio from ALSA device 'default'")

        async def __aexit__(self, exc_type, exc, traceback) -> None:
            """Never reached: the open failed."""

    monkeypatch.setattr("tests.ai_runtime.intercom.LocalMicStream", _DeadMic)

    async def _drive():
        server = IntercomServer(_config())
        connection = _FakeConnection([])
        await server._handle(connection)
        return server, connection

    server, connection = asyncio.run(_drive())
    code, reason = connection.closed
    assert code == 1011
    # The reason travels to the operator: a silent disconnection would leave them with
    # nothing to act on at the far end of a robot.
    assert "no audio" in reason
    # Released, so the operator can fix the microphone and reconnect.
    assert server._busy is False


class _StubServer:
    """Stands in for websockets' serve() so the mode bracket can be tested without a port."""

    async def __aenter__(self):
        """Enter the listener context."""
        return self

    async def __aexit__(self, exc_type, exc, traceback) -> None:
        """Leave it."""

    async def serve_forever(self) -> None:
        """Return at once: the tests here are about what brackets the serving, not it."""
