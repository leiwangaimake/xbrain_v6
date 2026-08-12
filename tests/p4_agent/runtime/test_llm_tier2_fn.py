"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_llm_tier2_fn.py
Brief: the live tier-2 classify fn -- mission select -> grammar -> parse

Description:
Tests build_tier2_fn with a MOCK llm_classify (no live server): a mission-cued
utterance runs the LLM under the right grammar and parses {intent,slots} into a
Tier2Classification; a no-mission utterance declines WITHOUT calling the LLM; a
foreign / unparseable model reply is dropped. Mutation guards per CLAUDE.md 3.3.
"""
from __future__ import annotations

import json

import pytest
import yaml

from xbrain.p4_agent.gateway.gpu_token import GpuTokenState
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.llm_tier2_fn import build_tier2_fn


pytestmark = pytest.mark.no_device

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"


def _reg():
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


class _MockLlm:
    """Records (prompt, grammar); returns a canned raw JSON."""

    def __init__(self, raw):
        self.raw = raw
        self.calls = []

    def __call__(self, prompt, grammar):
        self.calls.append((prompt, grammar))
        return self.raw


def _fn(mock, missions=None):
    return build_tier2_fn(
        _reg(),
        missions_text=missions or {"M5_speak": "MISSION M5", "M1_translate": "M1"},
        system_text="SYSTEM",
        base_url="http://x", model="m", timeout_s=1.0,
        token_state=GpuTokenState(), llm_classify=mock)


def test_speak_cue_classifies_with_slots():
    mock = _MockLlm(json.dumps(
        {"intent": "speak_custom", "slots": {"text": "前方危险"}}))
    fn = _fn(mock)
    out = fn("给大家播报前方危险", None, 1)
    assert out is not None
    assert out.name == "speak_custom" and out.slots == {"text": "前方危险"}
    # The LLM WAS called, and with a grammar (closed-set constraint).
    assert len(mock.calls) == 1 and mock.calls[0][1]      # grammar non-empty


def test_no_mission_declines_without_llm():
    """MUTATION: calling the LLM on out-of-scope text (no mission cue) would
    mislabel it -- select_mission=None must short-circuit BEFORE the call."""
    mock = _MockLlm("{}")
    fn = _fn(mock)
    assert fn("今天天气真不错", None, 1) is None
    assert mock.calls == []                               # LLM never called


def test_foreign_intent_is_dropped():
    """The model returning an intent OUTSIDE the mission's set is dropped, not
    routed. MUTATION: skipping the allowed-set check would route a stray id."""
    mock = _MockLlm(json.dumps({"intent": "estop", "slots": {}}))
    fn = _fn(mock)
    assert fn("给大家播报注意安全", None, 1) is None


def test_unparseable_reply_is_dropped():
    mock = _MockLlm("not json at all")
    fn = _fn(mock)
    assert fn("给大家播报注意安全", None, 1) is None


def test_grammar_only_exposes_mission_intents():
    """The grammar sent to the model lists ONLY M5's intents (speak_preset /
    speak_custom), never the whole registry (AI-36)."""
    mock = _MockLlm(json.dumps({"intent": "speak_preset",
                                "slots": {"preset_id": "p-x2"}}))
    fn = _fn(mock)
    fn("播报第二条", None, 1)
    grammar = mock.calls[0][1]
    assert "speak_custom" in grammar and "speak_preset" in grammar
    assert "estop" not in grammar and "goto_waypoint" not in grammar
