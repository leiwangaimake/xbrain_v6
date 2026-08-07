"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_speech_presets.py
Brief: GWY-P4-29 -- the preset library loads and validates, voice lives on the
       preset and NOT in D08's slots, each rule with its mutant

Description:
Two halves of GWY-P4-29:
  (1) configs/speech_presets.yaml holds the library (preset_id p- form / verbatim
      14 S7.3.3 text / device voice), and the loader refuses every malformed
      shape by name;
  (2) D08 speak_preset's slots must NOT contain voice -- the gender rides on the
      preset. The criterion's own mutation: add a voice slot to D08 => red.
"""

import copy
import os

import pytest
import yaml

from xbrain.p4_agent.registry.speech_presets import (
    SpeechPresetError, load_speech_presets, VOICES,
)

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
PRESETS_YAML = os.path.join(ROOT, "configs", "speech_presets.yaml")
INTENTS_YAML = os.path.join(ROOT, "configs", "intents.yaml")


def real_mapping():
    """The committed speech_presets.yaml, parsed."""
    with open(PRESETS_YAML, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def mutate(fn):
    """A deep copy of the real mapping with fn applied -- same pattern as the
    intent-registry tests, so a mutation cannot leak between cases."""
    m = copy.deepcopy(real_mapping())
    fn(m["presets"])
    return m


# --------------------------------------------------------------------------
# (1) the library itself
# --------------------------------------------------------------------------

def test_real_library_loads_with_the_14_s7_3_3_texts():
    """The committed file loads; ids are the p- forms mapped from warn_01/02/03
    and the texts are the 14 S7.3.3 sentences verbatim."""
    rows = load_speech_presets(real_mapping())
    by_id = {r.preset_id: r for r in rows}
    assert set(by_id) == {"p-warn_leave", "p-warn_no_photo", "p-warn_wait"}
    assert by_id["p-warn_leave"].text == "您已进入管制区域，请立即离开"
    assert by_id["p-warn_no_photo"].text == "此处为管制区域，禁止拍摄"
    assert by_id["p-warn_wait"].text == "请出示证件并原地等待"
    # voice on every preset (GWY-P4-29), and within the device's two genders.
    assert all(r.voice in VOICES for r in rows)


def test_missing_field_is_named():
    """*** Mutation: drop `voice` from one entry => the error names entry+field."""
    with pytest.raises(SpeechPresetError, match="voice"):
        load_speech_presets(mutate(lambda p: p[0].pop("voice")))
    with pytest.raises(SpeechPresetError, match="text"):
        load_speech_presets(mutate(lambda p: p[1].pop("text")))


def test_warn_nn_form_id_is_rejected():
    """*** The 14-sketch key form (warn_01) must NOT load: ID-2 requires p-<slug>.

    This is the cross-volume rule the yaml header documents; accepting the old
    form would let the library and the few-shot enumerations teach two shapes.
    """
    with pytest.raises(Exception):                   # geo-id validator raises its own type
        load_speech_presets(mutate(
            lambda p: p[0].__setitem__("preset_id", "warn_01")))


def test_non_preset_geo_id_is_rejected():
    """A VALID geo id with the wrong prefix (w- waypoint) is still not a preset."""
    with pytest.raises(SpeechPresetError, match="p-"):
        load_speech_presets(mutate(
            lambda p: p[0].__setitem__("preset_id", "w-gate_east")))


def test_duplicate_id_is_rejected():
    """Two rows, one id: the second would shadow the first silently."""
    with pytest.raises(SpeechPresetError, match="duplicate"):
        load_speech_presets(mutate(
            lambda p: p[1].__setitem__("preset_id", p[0]["preset_id"])))


def test_off_device_voice_is_rejected_not_coerced():
    """*** voice outside {male, female} raises -- never coerced to a default
    gender (the device has exactly two, [31] byte)."""
    with pytest.raises(SpeechPresetError, match="voice"):
        load_speech_presets(mutate(
            lambda p: p[0].__setitem__("voice", "robot")))


def test_empty_text_is_rejected():
    """An empty sentence would 'play' silence and read as a dead speaker."""
    with pytest.raises(SpeechPresetError, match="empty text"):
        load_speech_presets(mutate(lambda p: p[2].__setitem__("text", "  ")))


def test_empty_or_missing_library_is_rejected():
    """A comment-only skeleton (parses to None) and an empty list both refuse:
    a vacuous library must not look configured."""
    with pytest.raises(SpeechPresetError):
        load_speech_presets(None)
    with pytest.raises(SpeechPresetError):
        load_speech_presets({"presets": []})


# --------------------------------------------------------------------------
# (2) voice does NOT enter D08's slots (the criterion's verbatim mutation)
# --------------------------------------------------------------------------

def test_d08_slots_do_not_carry_voice():
    """*** GWY-P4-29 (2): D08 speak_preset's slots must not contain `voice`.

    The gender rides on the preset entry; a voice slot on D08 would pay GBNF
    closed-set + mission few-shot tokens AND hand the LLM a slot it can fill
    wrong. Mutation (verbatim from the criterion): add `voice` to D08's slots in
    intents.yaml => this goes red.
    """
    with open(INTENTS_YAML, encoding="utf-8") as fh:
        intents = yaml.safe_load(fh)["intents"]
    d08 = [v for v in intents.values() if v["id"] == "D08"]
    assert len(d08) == 1                             # speak_preset exists, once
    assert "voice" not in d08[0]["slots"], (
        "D08 must not carry a voice slot; the voice lives on the preset "
        "(GWY-P4-29)")
    assert d08[0]["slots"] == ["preset_id"]          # and only preset_id
