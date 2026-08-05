"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_deter.py
Brief: Unit tests for the 功能3 deter loop -- params, light-once, teardown, resilience (core/deter).

Description:
  Exercises DeterParams validation and DeterController's lifecycle without hardware. A fake
  link records the exact frame batches the controller writes, so the tests can assert the
  four device contracts that matter: params are validated at construction, the deter lights
  are set exactly ONCE at start (the firmware animates them thereafter), stop() resets every
  aspect deter touched, and a dropped audio link does NOT kill the loop while the lights keep
  flashing. The siren itself is rendered for real (cheap CPU work); only the socket writes are
  faked, so the frame-level assertions check the true builder output.

  Async without a plugin: each lifecycle test drives one coroutine through asyncio.run, so the
  suite needs no pytest-asyncio config and runs the same on the dev box and the Orin. The
  controller (and any task it spawns) is built INSIDE each scenario so it binds to that loop.

  Import reach: core/deter pulls in numpy (core/siren) and opuslib (codec) at module load, so
  this module is collected/run on the Orin's python3.10 where those are installed. Run from
  the /opt/xbrain_v6 root:
      python3 -m pytest tests/payload/test_deter.py
"""
from __future__ import annotations

import asyncio

import pytest

from services.payload.config import PayloadConfig
from services.payload.core.device_link import DeviceLinkError
from services.payload.core.deter import DeterController, DeterParamError, DeterParams
from services.payload.protocol.audio_8519 import build_hail_stop
from services.payload.protocol.lights_8529 import (
    BRIGHT_MAX,
    REDBLUE_MAX,
    REDBLUE_MIN,
    build_brightness,
    build_redblue,
    build_searchlight,
    build_strobe,
)


class _FakeLink:
    """Records the frame batches the deter loop writes, without opening a socket.

    Mirrors the two DeviceLink methods DeterController uses -- send_lights_control and
    send_audio -- as plain synchronous recorders (the controller always calls them through
    asyncio.to_thread, so they stay sync here). Either direction can be made to fail with a
    DeviceLinkError so the loop's audio-hiccup resilience and stop()'s best-effort reset are
    both testable. Batches are stored as lists so a test can assert the exact builder frames.
    """

    def __init__(self, *, fail_lights: bool = False, fail_audio: bool = False) -> None:
        self._fail_lights = fail_lights
        self._fail_audio = fail_audio
        self.lights_batches: list = []
        self.audio_batches: list = []

    def send_lights_control(self, frames) -> None:
        # Blow up like a dropped 8529 link when asked; otherwise snapshot the batch.
        if self._fail_lights:
            raise DeviceLinkError("fake lights link down")
        self.lights_batches.append(list(frames))

    def send_audio(self, frames) -> None:
        # Blow up like a dropped 8519 link when asked; otherwise snapshot the batch.
        if self._fail_audio:
            raise DeviceLinkError("fake audio link down")
        self.audio_batches.append(list(frames))


def test_params_defaults_valid() -> None:
    # The documented defaults (redblue 1, level 0.45, reps 2) must pass their own validate().
    DeterParams().validate()


def test_params_redblue_mode_out_of_range_rejected() -> None:
    # redblue floor is 1, not 0: a deter run with the warning light off is not a deter run.
    with pytest.raises(DeterParamError):
        DeterParams(redblue_mode=0).validate()
    with pytest.raises(DeterParamError):
        DeterParams(redblue_mode=REDBLUE_MAX + 1).validate()
    # Both inclusive bounds are accepted.
    DeterParams(redblue_mode=1).validate()
    DeterParams(redblue_mode=REDBLUE_MAX).validate()


def test_params_siren_level_out_of_range_rejected() -> None:
    # siren_level scales the waveform and must stay within the 0..1 amplitude range.
    with pytest.raises(DeterParamError):
        DeterParams(siren_level=-0.01).validate()
    with pytest.raises(DeterParamError):
        DeterParams(siren_level=1.01).validate()
    # Both inclusive bounds are accepted.
    DeterParams(siren_level=0.0).validate()
    DeterParams(siren_level=1.0).validate()


def test_params_tts_reps_negative_rejected() -> None:
    # 0 repeats is a valid siren-and-lights-only deterrent; a negative count is a caller bug.
    with pytest.raises(DeterParamError):
        DeterParams(tts_reps=-1).validate()
    DeterParams(tts_reps=0).validate()


def test_construct_validates_params_and_does_no_io() -> None:
    # __init__ calls validate(), so an illegal request fails at construction -- before any
    # siren render or device write. A valid construct arms no task and touches no socket.
    link = _FakeLink()
    with pytest.raises(DeterParamError):
        DeterController(link, PayloadConfig(), DeterParams(redblue_mode=0))
    ctl = DeterController(link, PayloadConfig(), DeterParams())
    assert ctl._task is None
    assert link.lights_batches == []
    assert link.audio_batches == []


def test_start_sets_deter_lights_once() -> None:
    # start() raises the full visual deterrent in ONE batched control group and never re-sends
    # it (the firmware animates the pattern), so exactly one lights batch exists after start.
    async def scenario() -> None:
        link = _FakeLink()
        ctl = DeterController(link, PayloadConfig(), DeterParams(redblue_mode=3, tts_reps=0))
        await ctl.start()
        # The loop only drives audio, so lights_batches stays at exactly the one arm batch.
        assert link.lights_batches == [
            [
                # ★ Strobe explicitly OFF and first: 14 GL-2 forbids MSG_STROBE[1] on the
                # searchlight, and 14 §4.3.0 requires deter to send [0] rather than merely
                # not send [1], so a lamp left flashing by something else is cleared.
                build_strobe(False),
                build_searchlight(True),
                build_brightness(BRIGHT_MAX),
                build_redblue(3),
            ]
        ]
        await ctl.stop()  # clean up the spawned loop task before the loop closes

    asyncio.run(scenario())


def test_stop_resets_device() -> None:
    # stop() is the acceptance contract (灯全灭收尾): strobe off, searchlight off, red/blue
    # reset, and a final hail-stop so no siren tail is left ringing.
    async def scenario() -> None:
        link = _FakeLink()
        ctl = DeterController(link, PayloadConfig(), DeterParams(tts_reps=0))
        await ctl.start()
        await ctl.stop()
        # Last lights batch is the all-off reset group.
        assert link.lights_batches[-1] == [
            build_strobe(False),
            build_searchlight(False),
            build_redblue(REDBLUE_MIN),
        ]
        # Last audio write is the reset hail-stop (no loop send follows a cancelled task).
        assert link.audio_batches[-1] == [build_hail_stop()]
        assert ctl._task is None

    asyncio.run(scenario())


def test_start_lights_failure_propagates() -> None:
    # If the lights arm fails (8529 link down), start() raises before spawning the loop, so no
    # task is left running; a following stop() still runs its best-effort reset without raising.
    async def scenario() -> None:
        link = _FakeLink(fail_lights=True)
        ctl = DeterController(link, PayloadConfig(), DeterParams())
        with pytest.raises(DeviceLinkError):
            await ctl.start()
        assert ctl._task is None  # the failure happened before create_task
        # stop() must not raise: the reset swallows the (still-failing) lights-off, and the
        # audio hail-stop link is up, so that reset write goes through.
        await ctl.stop()
        assert link.lights_batches == []  # both the arm and the reset lights-off failed
        assert link.audio_batches == [[build_hail_stop()]]  # reset hail-stop still landed

    asyncio.run(scenario())


def test_audio_failure_keeps_loop_running() -> None:
    # A dropped audio link must NOT end the deterrent: the lights (a separate socket, set once)
    # keep flashing from firmware, so the loop logs the fault, paces, and stays alive.
    async def scenario() -> None:
        link = _FakeLink(fail_audio=True)
        ctl = DeterController(link, PayloadConfig(), DeterParams())
        await ctl.start()
        # Let one cycle attempt (and fail) an audio send, then park in the retry pace.
        await asyncio.sleep(0.05)
        assert not ctl._task.done()  # the audio error was swallowed, the loop survives
        assert link.lights_batches == [
            [
                build_strobe(False),
                build_searchlight(True),
                build_brightness(BRIGHT_MAX),
                build_redblue(DeterParams().redblue_mode),
            ]
        ]
        # Teardown still resets the lights even though every audio write fails.
        await ctl.stop()
        assert link.lights_batches[-1] == [
            build_strobe(False),
            build_searchlight(False),
            build_redblue(REDBLUE_MIN),
        ]

    asyncio.run(scenario())


def test_deter_never_strobes_the_searchlight():
    """14 GL-2: the searchlight must never receive MSG_STROBE[1], in any mode.

    Asserted as a property over every frame deter emits rather than by matching one
    expected batch, so it keeps holding if the light sequence is ever reordered or
    extended -- the prohibition is on the VALUE, not on a position in a list.

    Why it is worth its own test: the searchlight is the fill light night perception
    depends on (PER-42), and a strobing lamp only images during its flashes. The failure
    is silent and looks like success -- the robot appears to be alarming vigorously while
    having just blinded itself to whatever it is alarming about -- so nothing downstream
    would catch it. An earlier version of this loop did exactly that.
    """

    async def scenario() -> None:
        link = _FakeLink()
        ctl = DeterController(link, PayloadConfig(), DeterParams(tts_reps=0))
        await ctl.start()
        await ctl.stop()
        emitted = [frame for batch in link.lights_batches for frame in batch]
        assert emitted, "deter emitted no light frames at all"
        assert build_strobe(True) not in emitted, (
            "deter sent MSG_STROBE[1] to the searchlight, which 14 GL-2 forbids"
        )
        # And the positive half: it must actively turn the strobe off, not merely omit it.
        assert build_strobe(False) in emitted

    asyncio.run(scenario())
