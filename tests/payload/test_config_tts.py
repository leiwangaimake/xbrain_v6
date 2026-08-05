"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_config_tts.py
Brief: Unit tests for the M3 TTS playback-time estimate on PayloadConfig.

Description:
  Exercises PayloadConfig.estimate_tts_ms and the three env-overridable TTS knobs that
  feed it (development plan sections 7 and 10). The estimate is what POST /tts returns so
  AI_runtime can time its mic gate without a device "TTS finished" event, so the formula
  -- max(base, char_count * per_char) + tail -- is pinned here at the boundaries that
  matter: the base floor, the crossover into per-char cost, and character (not UTF-8
  byte) counting. from_env wiring is checked too, since a knob that silently ignored its
  env var would ship a wrong estimate. config.py has no fastapi dependency, so this whole
  module runs on the dev box. Run from the /opt/xbrain_v6 root:
      python3 -m pytest tests/payload/test_config_tts.py
"""
from __future__ import annotations

import pytest

from services.payload.config import ConfigError, PayloadConfig

# The shipped defaults; every expectation below is DERIVED from these rather than written
# as a literal, so a recalibration shows up in one place. per_char and tail were measured
# against the real device on 2026-08-03 (see the note in services/payload/config.py) --
# 250 ms/char from the observed 232-240, and a tail that also absorbs the ~400 ms the
# device stays silent while synthesizing.
_BASE = 800
_PER_CHAR = 250
_TAIL = 900

# Where the base floor stops winning: ceil(_BASE / _PER_CHAR). Computed rather than
# hardcoded because recalibrating per_char MOVES this boundary, and the two tests either
# side of it are only meaningful if they sit on the correct side.
_CROSSOVER = -(-_BASE // _PER_CHAR)


def test_empty_text_is_base_plus_tail() -> None:
    # Zero characters: the per-char term is 0, so the base floor applies and tail is added.
    assert PayloadConfig().estimate_tts_ms("") == _BASE + _TAIL


def test_short_text_floored_by_base() -> None:
    # Just under the crossover, the per-char sum is smaller than the base, so the floor
    # wins -- this is what reserves a sane minimum for a one- or two-word utterance.
    short = "a" * (_CROSSOVER - 1)
    assert (_CROSSOVER - 1) * _PER_CHAR < _BASE
    assert PayloadConfig().estimate_tts_ms(short) == _BASE + _TAIL


def test_crossover_switches_to_per_char() -> None:
    # At the crossover the per-char sum overtakes the base floor and starts driving the
    # estimate; past this point the gate window grows with what is actually being said.
    text = "a" * _CROSSOVER
    assert _CROSSOVER * _PER_CHAR >= _BASE
    assert PayloadConfig().estimate_tts_ms(text) == _CROSSOVER * _PER_CHAR + _TAIL


def test_long_text_scales_with_per_char() -> None:
    # Ten chars: well past the crossover, the estimate tracks char_count * per_char.
    assert PayloadConfig().estimate_tts_ms("x" * 10) == 10 * _PER_CHAR + _TAIL


def test_counts_characters_not_utf8_bytes() -> None:
    # Six Chinese characters are 18 UTF-8 bytes; the estimate must use the 6-character
    # count (speaking time tracks characters), not the byte length that would inflate it.
    text = "你好世界你好"
    assert len(text) == 6
    assert PayloadConfig().estimate_tts_ms(text) == 6 * _PER_CHAR + _TAIL


def test_custom_knobs_are_honoured() -> None:
    # A config built with explicit knobs must use them, proving the formula reads the
    # instance fields rather than the module defaults.
    config = PayloadConfig(tts_est_base_ms=100, tts_est_per_char_ms=10, tts_est_tail_ms=0)
    assert config.estimate_tts_ms("abc") == 100  # max(100, 30) + 0
    assert config.estimate_tts_ms("a" * 50) == 500  # max(100, 500) + 0


def test_from_env_overrides_tts_knobs(monkeypatch: pytest.MonkeyPatch) -> None:
    # The three section-10 env vars must land on the matching fields (note the bare
    # PER_CHAR_MS / TAIL_MS names, per the plan) so an operator override actually changes
    # the estimate rather than being silently ignored.
    monkeypatch.setenv("TTS_EST_BASE_MS", "1000")
    monkeypatch.setenv("PER_CHAR_MS", "200")
    monkeypatch.setenv("TAIL_MS", "300")
    config = PayloadConfig.from_env()
    assert config.tts_est_base_ms == 1000
    assert config.tts_est_per_char_ms == 200
    assert config.tts_est_tail_ms == 300
    # And the override flows through the formula: 5 chars * 200 = 1000 == base, +300 tail.
    assert config.estimate_tts_ms("abcde") == 1000 + 300


def test_estimate_covers_measured_playback() -> None:
    # ★ The property the whole half-duplex gate rests on: the estimate must EXCEED the
    # time the device is actually audible, counted from the moment the request is sent.
    # These four cases are real measurements from 2026-08-03 -- (text, first-audio ms,
    # playback ms) -- and the previous 180/500 defaults failed every multi-word one,
    # covering only 80-83% and reopening the microphone while the robot was still talking.
    measured = [
        ("急停", 404, 290),
        ("哈船智能，语音链路测试", 447, 2640),
        ("您已进入管制区域，请立即离开", 357, 3260),
        ("已切换到巡逻档，当前感知距离限制实际速度零点五米每秒", 397, 6030),
    ]
    config = PayloadConfig()
    for text, first_ms, playback_ms in measured:
        needed = first_ms + playback_ms
        estimate = config.estimate_tts_ms(text)
        assert estimate > needed, (
            f"{text!r}: estimate {estimate} ms does not cover the measured "
            f"{needed} ms ({first_ms} silent + {playback_ms} audible)"
        )


def test_from_env_defaults_when_unset(monkeypatch: pytest.MonkeyPatch) -> None:
    # With the vars cleared, from_env must fall back to the documented section-7 defaults.
    monkeypatch.delenv("TTS_EST_BASE_MS", raising=False)
    monkeypatch.delenv("PER_CHAR_MS", raising=False)
    monkeypatch.delenv("TAIL_MS", raising=False)
    config = PayloadConfig.from_env()
    assert config.tts_est_base_ms == _BASE
    assert config.tts_est_per_char_ms == _PER_CHAR
    assert config.tts_est_tail_ms == _TAIL


def test_bad_env_knob_raises_configerror(monkeypatch: pytest.MonkeyPatch) -> None:
    # A non-integer override must fail loudly at startup (naming the key), not silently
    # revert to the default -- the same fail-fast contract the other numeric knobs have.
    monkeypatch.setenv("TTS_EST_BASE_MS", "not-a-number")
    with pytest.raises(ConfigError):
        PayloadConfig.from_env()


def test_the_warning_line_splits_into_speakable_segments() -> None:
    # The pause after 警告 is produced by sending two [31] utterances, not by hoping the
    # device's TTS infers timing from punctuation -- nothing observed says it does. So the
    # configured line must actually split, and both halves must be speakable.
    segments = [part for part in PayloadConfig().deter_tts_text.split("|") if part.strip()]
    assert len(segments) == 2
    assert segments[0] == "警告"
    assert "管控区域" in segments[1] and "请立即离开" in segments[1]


def test_the_mid_sentence_gap_is_shorter_than_the_mic_gate_estimate() -> None:
    # ★ The regression this exists to catch. Reusing estimate_tts_ms for the mid-sentence
    # gap is the obvious-looking thing to do and it is wrong: that formula floors at
    # tts_est_base_ms and adds tts_est_tail_ms because it must never reopen the microphone
    # early, and on a two-character segment those fixed terms swamp the ~500 ms of speech.
    # Measured on the real unit, that produced a 1.66 s hole in the middle of one sentence.
    config = PayloadConfig()
    assert config.estimate_segment_gap_ms("警告") < config.estimate_tts_ms("警告")


def test_the_gap_is_speech_plus_the_configured_beat() -> None:
    # The device's start latency cancels between two consecutive segments -- both are heard
    # `start` after their send -- so the audible silence is gap - speech, and the gap must
    # therefore be exactly speech + the beat. If a tail term crept back in, this fails.
    config = PayloadConfig()
    for text in ("警告", "你已进入管控区域 请立即离开"):
        expected = len(text) * config.tts_est_per_char_ms + config.deter_tts_pause_ms
        assert config.estimate_segment_gap_ms(text) == expected


def test_the_beat_is_long_enough_to_hear_and_short_enough_to_stay_one_sentence() -> None:
    # Tuned by ear on the real unit: measured 0.66-0.75 s of silence after 警告 at this
    # setting. Below ~200 ms the two halves run together; much above 1 s they stop reading
    # as one warning. The bounds are wide because this is a taste judgement, but a zero or
    # a stray multiplication would be a real defect and both are caught here.
    assert 200 <= PayloadConfig().deter_tts_pause_ms <= 900
