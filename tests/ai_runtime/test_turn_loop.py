"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_turn_loop.py
Brief: Unit tests for the 功能1 turn loop -- the Speak gate and the per-turn fault policy.

Description:
  The turn loop is driven here with a scripted mic stream and stubbed clients, so the two
  behaviours that cannot be checked any other way are asserted without a device: that
  frames arriving while a turn is in flight are DROPPED rather than queued into a second
  turn, and that an operational client fault ends one turn instead of the session.

  Determinism comes from the fake mic stream: it yields the scripted frames, then filler
  silence until the test's own stop event fires, so the session never ends in the middle
  of a turn by accident of scheduling. asyncio.run drives each case, which keeps these as
  ordinary sync tests with no async plugin required.
"""
from __future__ import annotations

import asyncio
import dataclasses

import numpy as np
import pytest

from tests.ai_runtime import turn_loop
from tests.ai_runtime.asr_client import AsrClientError
from tests.ai_runtime.config import AiRuntimeConfig
from tests.ai_runtime.local_mic import LocalMicStream
from tests.ai_runtime.payload_client import MicStream, PayloadClientError
from tests.ai_runtime.turn_loop import TurnLoop
from tests.ai_runtime.vad import BACKEND_ENERGY, FRAME_BYTES, Utterance

_QUIET = 5
_LOUD = 900

# Stops a broken test from running forever if its stop event is never set (2 s at the
# 1 ms filler cadence below).
_MAX_FILLER_FRAMES = 2000


def _frame(amplitude: int) -> bytes:
    return np.full(FRAME_BYTES // 2, amplitude, dtype="<i2").tobytes()


def _frames(amplitude: int, count: int):
    return [_frame(amplitude)] * count


def _config(**overrides) -> AiRuntimeConfig:
    return dataclasses.replace(
        AiRuntimeConfig(),
        # The energy backend, because these tests are about the LOOP -- the gate, the fault
        # policy -- and they drive it with constant-amplitude PCM that no trained detector
        # would ever call speech. Running silero here would test the model instead, and
        # would make these cases depend on a deployed model file they have no use for.
        vad_backend=BACKEND_ENERGY,
        vad_threshold=40.0,
        vad_start_ms=40,
        vad_stop_ms=100,
        vad_preroll_ms=60,
        vad_min_utterance_ms=200,
        vad_max_utterance_ms=400,
        tts_gate_margin_ms=0,
        **overrides,
    )


class _FakeMic:
    """Yields a scripted frame list, then silence, until the test says to stop."""

    def __init__(self, script, stop: asyncio.Event) -> None:
        self._script = list(script)
        self._stop = stop
        self._filler = 0

    async def __aenter__(self) -> "_FakeMic":
        return self

    async def __aexit__(self, *exc) -> None:
        return None

    def __aiter__(self) -> "_FakeMic":
        return self

    async def __anext__(self) -> bytes:
        if self._script:
            # Yield to the loop so an in-flight turn task gets to run between frames.
            await asyncio.sleep(0)
            return self._script.pop(0)
        if self._stop.is_set():
            raise StopAsyncIteration
        self._filler += 1
        if self._filler > _MAX_FILLER_FRAMES:
            raise StopAsyncIteration
        # Filler frames pass REAL time, not just a scheduling tick: a tight sleep(0) loop
        # outruns the turn's asyncio.to_thread hops and would end the stream mid-turn.
        await asyncio.sleep(0.001)
        return _frame(_QUIET)


def _install(monkeypatch, mic, *, transcribed="前进", reply="好的", est_ms=100000):
    """Point the loop at the fake mic and record what each stubbed client was called with."""
    calls = {"transcribe": [], "complete": [], "speak": [], "mode": []}
    # Patched at open_mic rather than at either stream class, because that is the seam the
    # loop actually uses -- and patching it means these tests stay source-agnostic instead
    # of silently only covering whichever mic_source happens to be the default.
    monkeypatch.setattr(turn_loop, "open_mic", lambda config: mic)
    monkeypatch.setattr(
        turn_loop.payload_client,
        "get_status",
        lambda config: {"mode": "idle", "device": {"audio_connected": True}},
    )
    monkeypatch.setattr(
        turn_loop.payload_client,
        "ensure_mode",
        lambda config, mode: calls["mode"].append(mode) or True,
    )
    monkeypatch.setattr(
        turn_loop.asr_client,
        "transcribe",
        lambda config, pcm: calls["transcribe"].append(pcm) or transcribed,
    )
    monkeypatch.setattr(
        turn_loop.llm_client,
        "complete",
        lambda config, text: calls["complete"].append(text) or reply,
    )
    return calls


def test_one_utterance_produces_one_transcribed_answered_spoken_turn(monkeypatch):
    stop = asyncio.Event()
    # One utterance, then a SECOND full burst that must be swallowed by the Speak gate.
    script = (
        _frames(_QUIET, 10) + _frames(_LOUD, 10) + _frames(_QUIET, 5)
        + _frames(_LOUD, 10) + _frames(_QUIET, 5)
    )
    mic = _FakeMic(script, stop)
    calls = _install(monkeypatch, mic)

    def _speak(config, text):
        calls["speak"].append(text)
        # Ending the stream only once the device has been told to speak guarantees the
        # session never stops mid-turn by accident of scheduling.
        stop.set()
        return 100000

    monkeypatch.setattr(turn_loop.payload_client, "speak", _speak)

    asyncio.run(TurnLoop(_config()).run())

    # Exactly one turn: the second burst arrived while the gate was shut and was dropped.
    assert len(calls["transcribe"]) == 1
    assert len(calls["transcribe"][0]) == 16 * FRAME_BYTES
    assert calls["complete"] == ["前进"]
    assert calls["speak"] == ["好的"]
    # The loop switched into func1 and, because it was the one that switched, back out.
    assert calls["mode"] == ["func1", "idle"]


def test_a_mode_this_process_did_not_set_is_left_alone(monkeypatch):
    stop = asyncio.Event()
    stop.set()
    mic = _FakeMic([], stop)
    calls = _install(monkeypatch, mic)
    # ensure_mode reporting False means the service was already in func1, so this session
    # is sharing one somebody else set up and must not reset it on the way out.
    monkeypatch.setattr(
        turn_loop.payload_client,
        "ensure_mode",
        lambda config, mode: calls["mode"].append(mode) or False,
    )

    asyncio.run(TurnLoop(_config()).run())

    assert calls["mode"] == ["func1"]


def test_a_disconnected_audio_link_is_reported_before_the_mic_opens(monkeypatch):
    monkeypatch.setattr(
        turn_loop.payload_client,
        "get_status",
        lambda config: {"mode": "idle", "device": {"audio_connected": False}},
    )

    with pytest.raises(PayloadClientError, match="audio link"):
        asyncio.run(TurnLoop(_config()).run())


def test_mic_source_selects_which_stream_the_loop_opens():
    """Both sources must be reachable from configuration alone, with no other change."""
    # Constructed, never entered: this is about which class is chosen, and entering would
    # open a real ALSA device or a real socket.
    assert isinstance(turn_loop.open_mic(_config(mic_source="local")), LocalMicStream)
    assert isinstance(turn_loop.open_mic(_config(mic_source="device")), MicStream)


def test_an_unknown_mic_source_is_refused_rather_than_defaulted():
    """A silent fallback would run a whole session listening in the wrong place."""
    with pytest.raises(PayloadClientError, match="unknown mic_source"):
        turn_loop.open_mic(_config(mic_source="usb"))


def _utterance() -> Utterance:
    return Utterance(pcm=b"\x00\x00" * 160 * 16, frames=16)


def test_an_operational_fault_ends_only_that_turn(monkeypatch):
    monkeypatch.setattr(
        turn_loop.asr_client,
        "transcribe",
        lambda config, pcm: (_ for _ in ()).throw(AsrClientError("asr is down")),
    )
    loop = TurnLoop(_config())

    # The person can simply repeat themselves, so this must not raise out of the turn.
    asyncio.run(loop._run_turn(_utterance()))


def test_an_unexpected_fault_is_allowed_to_propagate(monkeypatch):
    # Not one of the three client error types, so it is a bug rather than an operational
    # fault -- stopping is more useful than retrying it on every utterance forever.
    monkeypatch.setattr(
        turn_loop.asr_client,
        "transcribe",
        lambda config, pcm: (_ for _ in ()).throw(ValueError("bug")),
    )

    with pytest.raises(ValueError):
        asyncio.run(TurnLoop(_config())._run_turn(_utterance()))


def test_an_empty_transcript_speaks_nothing(monkeypatch):
    spoken = []
    monkeypatch.setattr(turn_loop.asr_client, "transcribe", lambda config, pcm: "   ")
    monkeypatch.setattr(
        turn_loop.llm_client, "complete", lambda config, text: spoken.append(text) or "x"
    )
    monkeypatch.setattr(
        turn_loop.payload_client, "speak", lambda config, text: spoken.append(text) or 0
    )

    asyncio.run(TurnLoop(_config())._run_turn(_utterance()))

    # A VAD false trigger -- a door, a cough -- must not make the robot answer it.
    assert spoken == []
