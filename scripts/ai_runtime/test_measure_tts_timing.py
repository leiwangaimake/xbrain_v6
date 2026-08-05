"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_measure_tts_timing.py
Brief: Unit tests for the tts_gate_margin_ms calibration tool's analysis.

Description:
  The tool this covers can only be run with a person standing next to a hailing robot, so
  a session with it is scarce and a bug in the analysis wastes one. Everything here is the
  part that can be checked without the device: given a run of frame levels, does it find
  the right start and end, and does it turn those into the right recommendation.

  Speech detection is where a confident wrong answer comes from, so the tests are built
  around the three ways it can be wrong rather than the one way it can be right: a
  transient counted as the start, an in-sentence pause counted as the end, and -- the one
  issue E8 makes real -- nothing heard at all being reported as a measurement instead of a
  failure.

  No microphone and no device are involved. The levels are written directly, because the
  question is what the analysis concludes from a recording, not whether a recording can be
  made; the capture path has its own tests in tests/ai_runtime/test_local_mic.py.
"""
from __future__ import annotations

import pytest

from scripts.ai_runtime.measure_tts_timing import (
    PhraseTiming,
    _FRAME_MS,
    _MIN_MARGIN_MS,
    _ONSET_FRAMES,
    _SILENCE_FRAMES,
    _find_speech,
    _report,
    _round_up,
)

# Well clear of the trigger in both directions, so no test turns on rounding at the
# boundary -- the boundary itself is not what any of these are about.
_LOUD = 900.0
_QUIET = 10.0
_TRIGGER = 100.0


def _levels(*runs) -> list:
    """Build a level track from (value, frame_count) pairs."""
    track = []
    for value, count in runs:
        track.extend([value] * count)
    return track


def test_an_utterance_is_bounded_by_its_first_loud_frame_and_its_trailing_silence():
    """The onset is where speech began, not where it had been proved to have begun."""
    track = _levels((_QUIET, 5), (_LOUD, 50), (_QUIET, _SILENCE_FRAMES + 10))

    onset, end = _find_speech(track, _TRIGGER)

    # 5, not 5 + _ONSET_FRAMES - 1: the frames that confirmed the run were speech too.
    assert onset == 5
    assert end == 55


def test_nothing_heard_is_reported_as_no_utterance_rather_than_a_zero_length_one():
    """This is issue E8's exact shape: a dead microphone input records only its own floor.

    Returning a degenerate utterance here would flow straight through to a recommended
    margin computed from a recording of a room the loudspeaker is not in.
    """
    assert _find_speech(_levels((_QUIET, 400)), _TRIGGER) is None


def test_speech_that_never_stops_is_not_reported_as_stopping_at_the_end_of_the_window():
    """A device still talking when the window closed has an unmeasured end, not a late one.

    The window length is this tool's own parameter, so reporting it as the end of speech
    would be measuring the tool instead of the device.
    """
    assert _find_speech(_levels((_QUIET, 5), (_LOUD, 300)), _TRIGGER) is None


def test_a_transient_shorter_than_the_onset_run_does_not_start_an_utterance():
    """The amplifier clicks when it switches on, and that click precedes the first syllable.

    Taken as the onset it would stretch the measured speech backwards past the moment the
    device actually began speaking.
    """
    track = _levels(
        (_QUIET, 5),
        (_LOUD, _ONSET_FRAMES - 1),
        (_QUIET, 30),
        (_LOUD, 40),
        (_QUIET, _SILENCE_FRAMES + 5),
    )

    onset, _ = _find_speech(track, _TRIGGER)

    assert onset == 5 + (_ONSET_FRAMES - 1) + 30


def test_a_pause_between_words_does_not_end_the_utterance():
    """Ending on the first quiet frame would cut every multi-word line at its first comma.

    That is the failure that matters most: a short measured utterance yields a short
    recommended margin, and the gate then reopens while the device is still talking.
    """
    track = _levels(
        (_LOUD, 20),
        (_QUIET, _SILENCE_FRAMES - 1),
        (_LOUD, 20),
        (_QUIET, _SILENCE_FRAMES + 5),
    )

    onset, end = _find_speech(track, _TRIGGER)

    assert onset == 0
    assert end == 20 + (_SILENCE_FRAMES - 1) + 20


def test_overrun_is_measured_from_the_same_zero_the_gate_starts_from():
    """turn_loop starts its timer when POST /tts returns and sleeps est_ms + margin.

    So the margin has to cover end_ms - est_ms exactly; measuring the overrun from the
    start of speech instead would under-report it by the device's start-up delay.
    """
    timing = PhraseTiming(text="你好", est_ms=1160, lead_ms=400, end_ms=1500)

    assert timing.overrun_ms == 1500 - 1160
    assert timing.spoken_ms == 1500 - 400
    assert timing.chars == 2


@pytest.mark.parametrize(
    "value,expected", [(1.0, 50), (50.0, 50), (51.0, 100), (743.0, 750), (0.0, 0)]
)
def test_recommendations_round_up_never_down(value, expected):
    """Rounding a safety allowance down would spend the safety it was rounding."""
    assert _round_up(value) == expected


def _flat(chars: int, overrun: int) -> PhraseTiming:
    """A phrase whose overrun is what the test wants, with a plausible est_ms."""
    est_ms = chars * 180 + 500
    return PhraseTiming(text="x" * chars, est_ms=est_ms, lead_ms=300, end_ms=est_ms + overrun)


def test_a_constant_overrun_yields_a_margin_that_covers_the_worst_of_it(capsys):
    """A device that merely starts late is exactly what a fixed margin is the right fix for."""
    timings = [_flat(2, 250), _flat(24, 300), _flat(114, 280)]

    assert _report(timings) == 0
    recommended = int(capsys.readouterr().out.split("AI_TTS_GATE_MARGIN_MS = ")[1])
    # Above the worst overrun, not merely equal to it: equal leaves no room for the spread
    # a handful of samples cannot show.
    assert recommended > 300


def test_an_overrun_that_grows_with_length_is_refused_rather_than_averaged(capsys):
    """A wrong per-character rate cannot be repaired by adding a constant to it.

    A margin fitted to these numbers would pass every short reply and reopen the gate
    mid-sentence on every long one, which is worse than reporting no margin at all.
    """
    timings = [_flat(2, 100), _flat(24, 900), _flat(114, 4000)]

    assert _report(timings) == 1
    # The operator is pointed at the knob that actually controls the slope.
    assert "PER_CHAR_MS" in capsys.readouterr().err


def test_a_device_that_finishes_early_still_gets_a_usable_margin(capsys):
    """The margin also absorbs start-up jitter, so it cannot follow the overrun to zero."""
    timings = [_flat(2, -800), _flat(24, -900), _flat(114, -850)]

    assert _report(timings) == 0
    recommended = int(capsys.readouterr().out.split("AI_TTS_GATE_MARGIN_MS = ")[1])
    assert recommended == _MIN_MARGIN_MS


def test_frame_indices_convert_to_milliseconds_at_the_capture_rate():
    """The frame count IS the clock, so a wrong frame duration silently scales every result."""
    # 20 ms frames at 16 kHz mono s16le is what local_mic produces and what the tool assumes.
    assert _FRAME_MS == 20
