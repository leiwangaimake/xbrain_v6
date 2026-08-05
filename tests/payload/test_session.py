"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_session.py
Brief: Unit tests for the M4A mode state machine and R2/R3 audio gate (core/session).

Description:
  Exercises SessionManager end to end without hardware: mode transitions, the /mic + /play
  + /tts gate tables, the R2 single-client-per-kind rule, the cooperative audio-session
  teardown handshake (close/done Events), and the deter start/stop lifecycle. The deter
  controller is injected as a fake through SessionManager's deter_factory seam, and the
  device link is a bare stand-in, so nothing here touches a socket -- the manager's own
  state logic is what is under test.

  Async without a plugin: each test drives one coroutine through asyncio.run, so the suite
  needs no pytest-asyncio config and runs the same on the dev box and the Orin. The manager
  (and its asyncio.Lock) is built INSIDE each scenario so it binds to that run's loop.

  Import reach: core/session imports core/deter, which pulls in numpy (core/siren) and
  opuslib (codec) at module load, so this module is collected/run on the Orin's python3.10
  where those are installed. Run from the /opt/xbrain_v6 root:
      python3 -m pytest tests/payload/test_session.py
"""
from __future__ import annotations

import asyncio

import pytest

from services.payload.config import PayloadConfig
from services.payload.core.device_link import DeviceLinkError
from services.payload.core.deter import DeterParams
import services.payload.core.session as session_mod
from services.payload.core.session import (
    Mode,
    ModeStateError,
    ModeTransitionError,
    SessionManager,
)


class _FakeDeter:
    """Stand-in for DeterController that records start/stop without any device I/O.

    It mirrors the real controller's contract the manager depends on: it validates its
    params at construction (so an illegal request would still fail here) and exposes async
    start()/stop(). start() can be made to fail so the manager's start-failure path -- reset
    then propagate -- is testable. The three public flags let a test assert what happened.
    """

    def __init__(self, link, config, params, *, fail_start: bool = False) -> None:
        params.validate()  # mirror DeterController.__init__: bad params raise before I/O
        self.link = link
        self.config = config
        self.params = params
        self._fail_start = fail_start
        self.started = False
        self.stopped = False

    async def start(self) -> None:
        # Simulate an arm that fails partway (link down) when the test asked for it, so the
        # manager's "reset then re-raise" branch runs; otherwise just record the arm.
        if self._fail_start:
            raise DeviceLinkError("fake deter start failed")
        self.started = True

    async def stop(self) -> None:
        # Always succeeds and is idempotent, like the real controller's best-effort reset.
        self.stopped = True


def _make_factory(*, fail_start: bool = False, record: list | None = None):
    # Build a deter_factory closure matching DeterFactory's (link, config, params) shape.
    # The optional record list captures every controller built, so a test can inspect the
    # start/stop flags the manager drove; fail_start makes the built controller's start()
    # raise, to drive the manager's failure path.
    def factory(link, config, params):
        controller = _FakeDeter(link, config, params, fail_start=fail_start)
        if record is not None:
            record.append(controller)
        return controller

    return factory


def _manager(factory=None) -> SessionManager:
    # Construct a manager with a dummy link (the fake controller ignores it) and a real
    # config (cheap, no I/O). MUST be called inside a running loop so its Lock binds there.
    return SessionManager(object(), PayloadConfig(), deter_factory=factory or _make_factory())


def test_initial_mode_is_idle() -> None:
    # A fresh manager starts in the IDLE hub with nothing armed.
    async def scenario() -> None:
        mgr = _manager()
        assert mgr.current_mode() is Mode.IDLE

    asyncio.run(scenario())


def test_gate_predicates_per_mode() -> None:
    # The three lock-free predicates must reflect the section-6 gate tables exactly, so the
    # REST/WS layers gate on the same rule the manager enforces under lock.
    async def scenario() -> None:
        mgr = _manager()
        # IDLE: no live audio; a one-shot TTS announcement is still allowed.
        assert (mgr.allows_mic(), mgr.allows_play(), mgr.allows_tts()) == (False, False, True)
        await mgr.transition(Mode.FUNC1)
        # FUNC1 (AI dialogue): both audio streams and TTS (the AI's spoken reply) allowed.
        assert (mgr.allows_mic(), mgr.allows_play(), mgr.allows_tts()) == (True, True, True)
        await mgr.transition(Mode.FUNC2)
        # FUNC2 (live intercom): audio yes, but injected TTS must not interrupt a human.
        assert (mgr.allows_mic(), mgr.allows_play(), mgr.allows_tts()) == (True, True, False)
        await mgr.transition(Mode.DETER)
        # DETER: the loop owns the audio path exclusively, so nothing external is allowed.
        assert (mgr.allows_mic(), mgr.allows_play(), mgr.allows_tts()) == (False, False, False)

    asyncio.run(scenario())


def test_transition_returns_previous_mode() -> None:
    # transition reports the mode it left, which POST /mode echoes back as `previous`.
    async def scenario() -> None:
        mgr = _manager()
        assert await mgr.transition(Mode.FUNC1) is Mode.IDLE
        assert await mgr.transition(Mode.DETER) is Mode.FUNC1

    asyncio.run(scenario())


def test_reentering_active_mode_raises() -> None:
    # Re-entering the current mode is a 409 conflict, not a silent reset.
    async def scenario() -> None:
        mgr = _manager()
        await mgr.transition(Mode.FUNC1)
        with pytest.raises(ModeTransitionError):
            await mgr.transition(Mode.FUNC1)

    asyncio.run(scenario())


def test_open_audio_refused_in_idle() -> None:
    # /mic and /play are invalid in IDLE (nothing running), so both are refused.
    async def scenario() -> None:
        mgr = _manager()
        with pytest.raises(ModeStateError):
            await mgr.open_audio("mic")
        with pytest.raises(ModeStateError):
            await mgr.open_audio("play")

    asyncio.run(scenario())


def test_open_audio_refused_in_deter() -> None:
    # In DETER the loop owns the audio path, so an external /mic is refused.
    async def scenario() -> None:
        mgr = _manager()
        await mgr.transition(Mode.DETER)
        with pytest.raises(ModeStateError):
            await mgr.open_audio("mic")

    asyncio.run(scenario())


def test_open_audio_allows_both_kinds_in_func1() -> None:
    # FUNC1 permits one mic and one play session concurrently (the two-way AI dialogue).
    async def scenario() -> None:
        mgr = _manager()
        await mgr.transition(Mode.FUNC1)
        mic = await mgr.open_audio("mic")
        play = await mgr.open_audio("play")
        assert mic.kind == "mic"
        assert play.kind == "play"

    asyncio.run(scenario())


def test_open_audio_duplicate_kind_refused() -> None:
    # R2: a second session of the same kind is refused so it cannot hijack the first.
    async def scenario() -> None:
        mgr = _manager()
        await mgr.transition(Mode.FUNC1)
        await mgr.open_audio("mic")
        with pytest.raises(ModeStateError):
            await mgr.open_audio("mic")

    asyncio.run(scenario())


def test_open_audio_unknown_kind_raises_valueerror() -> None:
    # An unknown kind is a programming error in a caller, surfaced as ValueError not a mode
    # fault -- only ws.py calls this, with the two canonical kind constants.
    async def scenario() -> None:
        mgr = _manager()
        await mgr.transition(Mode.FUNC1)
        with pytest.raises(ValueError):
            await mgr.open_audio("bogus")

    asyncio.run(scenario())


def test_finish_audio_deregisters_and_is_idempotent() -> None:
    # finish_audio sets the session's done Event and frees its R2 slot; calling it twice, or
    # after the slot is reused, is harmless.
    async def scenario() -> None:
        mgr = _manager()
        await mgr.transition(Mode.FUNC1)
        sess = await mgr.open_audio("mic")
        mgr.finish_audio(sess)
        assert sess.done.is_set()
        # Slot freed: a fresh mic session opens rather than being refused as a duplicate.
        again = await mgr.open_audio("mic")
        assert again is not sess
        # Second finish on the original is a no-op (already discarded, Event already set).
        mgr.finish_audio(sess)

    asyncio.run(scenario())


def test_transition_tears_down_open_audio_cooperatively() -> None:
    # Section 13: switching modes closes an open audio session. A well-behaved handler acks
    # the close by calling finish_audio, which releases the waiting transition promptly.
    async def scenario() -> None:
        mgr = _manager()
        await mgr.transition(Mode.FUNC1)
        sess = await mgr.open_audio("mic")

        async def handler() -> None:
            # Stand in for the WS route: wait for the close signal, then acknowledge.
            await sess.close.wait()
            mgr.finish_audio(sess)

        task = asyncio.create_task(handler())
        previous = await mgr.transition(Mode.FUNC2)
        assert previous is Mode.FUNC1
        assert mgr.current_mode() is Mode.FUNC2
        assert sess.close.is_set()  # manager asked it to close
        assert sess.done.is_set()  # handler acknowledged
        await task

    asyncio.run(scenario())


def test_transition_drops_unacknowledged_session_after_timeout(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # A wedged handler that never acks must not block a mode change forever: teardown waits
    # the bounded timeout, drops the session, and completes. Shorten the timeout so the test
    # is fast rather than the production 5 s.
    monkeypatch.setattr(session_mod, "_AUDIO_TEARDOWN_TIMEOUT_S", 0.05)

    async def scenario() -> None:
        mgr = _manager()
        await mgr.transition(Mode.FUNC1)
        sess = await mgr.open_audio("mic")  # no handler will ever ack this one
        previous = await mgr.transition(Mode.FUNC2)
        assert previous is Mode.FUNC1
        assert mgr.current_mode() is Mode.FUNC2
        assert sess.close.is_set()
        # The stuck session was dropped from the registry, so its R2 slot is free again: a
        # new mic session opens in FUNC2 instead of being refused as a duplicate.
        fresh = await mgr.open_audio("mic")
        assert fresh is not sess

    asyncio.run(scenario())


def test_start_deter_requires_deter_mode() -> None:
    # start_deter outside DETER is a 409: the caller must POST /mode deter first.
    async def scenario() -> None:
        mgr = _manager()
        with pytest.raises(ModeStateError):
            await mgr.start_deter(DeterParams())

    asyncio.run(scenario())


def test_start_deter_arms_and_stores_controller() -> None:
    # In DETER, start_deter builds via the factory, arms it, and stores it -- proven by a
    # following stop_deter reaching the same controller.
    async def scenario() -> None:
        record: list = []
        mgr = _manager(_make_factory(record=record))
        await mgr.transition(Mode.DETER)
        await mgr.start_deter(DeterParams(redblue_mode=3))
        assert len(record) == 1
        assert record[0].started is True
        assert record[0].params.redblue_mode == 3
        await mgr.stop_deter()
        assert record[0].stopped is True

    asyncio.run(scenario())


def test_start_deter_restart_stops_previous() -> None:
    # A second start_deter stops the running loop before arming the new one, so fresh params
    # cleanly replace it rather than two loops stacking.
    async def scenario() -> None:
        record: list = []
        mgr = _manager(_make_factory(record=record))
        await mgr.transition(Mode.DETER)
        await mgr.start_deter(DeterParams())
        await mgr.start_deter(DeterParams(redblue_mode=5))
        assert len(record) == 2
        assert record[0].stopped is True  # the first run was torn down
        assert record[1].started is True  # the second run is live

    asyncio.run(scenario())


def test_start_deter_failure_resets_and_propagates() -> None:
    # If arming fails (link down), the manager resets the half-armed controller and re-raises
    # DeviceLinkError, leaving nothing stored behind.
    async def scenario() -> None:
        record: list = []
        mgr = _manager(_make_factory(fail_start=True, record=record))
        await mgr.transition(Mode.DETER)
        with pytest.raises(DeviceLinkError):
            await mgr.start_deter(DeterParams())
        assert record[0].started is False  # start() raised before recording the arm
        assert record[0].stopped is True  # ...but the reset (stop) still ran
        assert mgr._deter is None  # nothing half-live is stored

    asyncio.run(scenario())


def test_stop_deter_requires_deter_mode() -> None:
    # stop_deter outside DETER is a 409, symmetric with start_deter.
    async def scenario() -> None:
        mgr = _manager()
        with pytest.raises(ModeStateError):
            await mgr.stop_deter()

    asyncio.run(scenario())


def test_stop_deter_is_idempotent_within_deter() -> None:
    # Stopping a running loop works; stopping again (nothing running) is a harmless success,
    # so a client can POST /deter {on:false} twice without error.
    async def scenario() -> None:
        record: list = []
        mgr = _manager(_make_factory(record=record))
        await mgr.transition(Mode.DETER)
        await mgr.start_deter(DeterParams())
        await mgr.stop_deter()
        assert record[0].stopped is True
        await mgr.stop_deter()  # no run active now: must not raise

    asyncio.run(scenario())


def test_transition_out_of_deter_stops_loop() -> None:
    # Leaving DETER by any transition tears the loop down through the shared _teardown path.
    async def scenario() -> None:
        record: list = []
        mgr = _manager(_make_factory(record=record))
        await mgr.transition(Mode.DETER)
        await mgr.start_deter(DeterParams())
        await mgr.transition(Mode.IDLE)
        assert record[0].stopped is True
        assert mgr.current_mode() is Mode.IDLE

    asyncio.run(scenario())


def test_aclose_tears_down_and_resets_to_idle() -> None:
    # Shutdown stops a running deter loop and returns the manager to IDLE, so nothing
    # outlives the app.
    async def scenario() -> None:
        record: list = []
        mgr = _manager(_make_factory(record=record))
        await mgr.transition(Mode.DETER)
        await mgr.start_deter(DeterParams())
        await mgr.aclose()
        assert record[0].stopped is True
        assert mgr.current_mode() is Mode.IDLE

    asyncio.run(scenario())
