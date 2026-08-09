"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_presets.py
Brief: speech tests -- presets

Description:
BIZ-P2-30 -- speech_presets loader + BIT announce sequence.
"""


import pytest

from xbrain.p2_core.speech.presets import (
    PresetSet, SpeechPreset, SpeechPresetError,
    build_announce_sequence, parse_presets,
)


pytestmark = pytest.mark.no_device


def test_empty_yaml_produces_empty_set():
    ps = parse_presets("")
    assert ps.presets == {}


def test_valid_yaml_parses():
    yml = """
presets:
  - id: bit_head
    text: "开机自检"
    voice: male
  - id: bit_tail_continue
    text: "自检完成"
"""
    ps = parse_presets(yml)
    assert "bit_head" in ps.presets
    assert ps.presets["bit_head"].voice == "male"


def test_missing_id_raises():
    with pytest.raises(SpeechPresetError):
        parse_presets("presets:\n  - text: hi\n")


def test_bad_voice_rejected():
    with pytest.raises(SpeechPresetError):
        parse_presets(
            "presets:\n  - id: a\n    text: t\n    voice: robot\n")


def test_duplicate_id_rejected():
    yml = """
presets:
  - id: a
    text: t1
  - id: a
    text: t2
"""
    with pytest.raises(SpeechPresetError):
        parse_presets(yml)


# --- Announce sequence ---

def test_announce_sequence_assembles_head_fails_tail():
    ps = PresetSet(presets={
        "bit_head": SpeechPreset(id="bit_head", text="开机自检"),
        "bit_fail_ptz": SpeechPreset(id="bit_fail_ptz", text="云台故障"),
        "bit_fail_mic": SpeechPreset(id="bit_fail_mic", text="麦克风故障"),
        "bit_tail_continue": SpeechPreset(
            id="bit_tail_continue", text="继续出勤"),
    }, version=1)
    seq = build_announce_sequence(ps, fail_item_names=["ptz", "mic"],
                                   max_items=5)
    assert [p.id for p in seq] == ["bit_head", "bit_fail_ptz",
                                     "bit_fail_mic", "bit_tail_continue"]


def test_announce_sequence_caps_at_max_items():
    """max_items = 2; six fails -> only first 2 fail presets played."""
    ps = PresetSet(presets={
        "bit_fail_a": SpeechPreset(id="bit_fail_a", text="A 故障"),
        "bit_fail_b": SpeechPreset(id="bit_fail_b", text="B 故障"),
        "bit_fail_c": SpeechPreset(id="bit_fail_c", text="C 故障"),
    }, version=1)
    seq = build_announce_sequence(
        ps, fail_item_names=["a", "b", "c"], max_items=2)
    assert [p.id for p in seq] == ["bit_fail_a", "bit_fail_b"]


def test_announce_sequence_silent_when_no_matching_presets():
    """Missing presets produce a shorter but valid sequence."""
    ps = PresetSet(presets={}, version=1)
    seq = build_announce_sequence(ps, fail_item_names=["chassis"],
                                   max_items=5)
    assert seq == []
