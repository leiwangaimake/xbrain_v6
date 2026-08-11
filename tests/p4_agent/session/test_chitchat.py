"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chitchat.py
Brief: GWY-P4-36 (32.D) -- zero-LLM chitchat preset responder

Description:
Tests the chitchat preset responder against the real configs/chitchat.yaml
(16 S11.5 / 18 S12). Each criterion carries a mutation that must turn red
per CLAUDE.md 3.3: preset (not LLM, not echo), out_of_scope threshold read
from config (not hardcoded).
"""
from __future__ import annotations

import copy

import pytest
import yaml

from xbrain.p4_agent.session.chitchat import (
    ChitchatPresetError, ChitchatResponder, ChitchatState,
)

pytestmark = pytest.mark.no_device

_CHITCHAT_PATH = "/opt/xbrain_v6/configs/chitchat.yaml"


def _presets():
    with open(_CHITCHAT_PATH, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# -- criterion 1: greeting returns a preset, not LLM, not echo -----------

def test_greeting_returns_preset_not_echo():
    r = ChitchatResponder(_presets())
    st = ChitchatState()
    out = r.respond("greeting", st)
    # It is one of the configured presets, and NOT an echo of user text.
    assert out in _presets()["greeting"]["default"]
    assert out != "你好啊机器人你在吗"    # not the (hypothetical) user utterance


def test_greeting_time_variant():
    r = ChitchatResponder(_presets())
    p = _presets()
    assert r.respond("greeting", ChitchatState(),
                     time_of_day="morning") in p["greeting"]["time_variant"]["morning"]
    assert r.respond("greeting", ChitchatState(),
                     time_of_day="evening") in p["greeting"]["time_variant"]["evening"]


def test_identity_and_help_are_presets():
    r = ChitchatResponder(_presets())
    p = _presets()
    # identity now rotates through multiple 海卫 variants (party-A ask).
    ident = r.respond("identity", ChitchatState())
    assert ident in p["identity"]["replies"]
    assert "海卫" in ident                       # every variant carries the name
    assert r.respond("help", ChitchatState()) == p["help"]["reply"]


def test_identity_rotates_not_rigid():
    """Party-A ask: a self-introduction should not be the same sentence
    every time. Consecutive calls on one session must cycle variants.
    MUTATION: a fixed index-0 pick would return the same string here."""
    r = ChitchatResponder(_presets())
    st = ChitchatState()
    seen = {r.respond("identity", st) for _ in range(5)}
    assert len(seen) >= 2                        # more than one distinct reply


def test_greeting_rotates():
    r = ChitchatResponder(_presets())
    st = ChitchatState()
    seen = {r.respond("greeting", st) for _ in range(5)}
    assert len(seen) >= 2


def test_zero_llm_freeform_construction_rejected():
    """MUTATION A guard: the only way greeting could 'go through LLM
    free-form' is allow_llm_freeform=true. The responder refuses to
    construct with it (Q-18-2 default not allowed, CMD-50..54), so there
    is structurally no LLM path."""
    with pytest.raises(ChitchatPresetError):
        ChitchatResponder(_presets(), allow_llm_freeform=True)


def test_responder_has_no_llm_dependency():
    """Structural zero-LLM: the chitchat module must not import any LLM
    client. MUTATION A (route greeting through an llm client) would need
    such an import; its absence is the guarantee."""
    import xbrain.p4_agent.session.chitchat as mod
    src = open(mod.__file__, encoding="utf-8").read()
    assert "llm_client" not in src
    assert "ai_client" not in src


# -- criterion 2: out_of_scope threshold read from config ----------------

def test_out_of_scope_below_threshold_plays_reply():
    r = ChitchatResponder(_presets())
    st = ChitchatState()
    reply = _presets()["out_of_scope"]["reply"]
    assert r.respond("out_of_scope", st) == reply    # 1st
    assert r.respond("out_of_scope", st) == reply    # 2nd (threshold is 3)


def test_out_of_scope_at_threshold_announces_capability():
    r = ChitchatResponder(_presets())
    st = ChitchatState()
    r.respond("out_of_scope", st)                    # 1
    r.respond("out_of_scope", st)                    # 2
    out = r.respond("out_of_scope", st)              # 3 -> capability overview
    assert out == _presets()["help"]["reply"]
    # the run resets so it only fires again after another full streak
    assert st.consecutive_out_of_scope == 0


def test_threshold_is_read_from_config_not_hardcoded():
    """MUTATION B guard: change consecutive_threshold to 2 and the
    overview must fire on the 2nd hit. A hardcoded '3' would keep playing
    the plain reply and fail here."""
    p = copy.deepcopy(_presets())
    p["out_of_scope"]["consecutive_threshold"] = 2
    r = ChitchatResponder(p)
    st = ChitchatState()
    assert r.respond("out_of_scope", st) == p["out_of_scope"]["reply"]   # 1
    assert r.respond("out_of_scope", st) == p["help"]["reply"]           # 2 -> overview


def test_success_breaks_out_of_scope_streak():
    r = ChitchatResponder(_presets())
    st = ChitchatState()
    r.respond("out_of_scope", st)                    # streak = 1
    r.respond("greeting", st)                        # success -> reset
    assert st.consecutive_out_of_scope == 0


def test_non_chitchat_intent_raises():
    r = ChitchatResponder(_presets())
    with pytest.raises(ChitchatPresetError):
        r.respond("estop", ChitchatState())          # action intent must not route here


def test_missing_preset_key_raises():
    p = copy.deepcopy(_presets())
    del p["identity"]
    with pytest.raises(ChitchatPresetError):
        ChitchatResponder(p)
