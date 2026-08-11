"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_layer_match.py
Brief: GWY-P4-33 (32.A) -- keyword_matcher layer2 + priority_chain feed

Description:
Tests the layer-2 keyword matcher (16 S5.2 "longest-first") and its
feed into priority_chain. Each criterion carries a mutation that must
turn red, per CLAUDE.md 3.3.
"""
from __future__ import annotations

import os

import yaml

from xbrain.p4_agent.classifier.keyword_matcher import (
    KeywordMatcher, classify_text,
)
from xbrain.p4_agent.registry.intents import load_intent_registry

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"


def _reg():
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


# -- longest-first (16 S484 / S5.2 layer 2) ---------------------------

def test_longest_first_beats_short():
    """'停止喊话' contains BOTH C04's 4-char keyword '停止喊话' AND D08's
    2-char keyword '喊话'. Longest-first (16 S484) must pick the long
    one, not substring-steal via the short one."""
    m = KeywordMatcher(_reg())
    assert "喊话" in m._by_kw          # sanity: the short keyword exists (D08)
    assert "停止喊话" in m._by_kw       # sanity: the long keyword exists (C04)
    assert m._by_kw["喊话"] != m._by_kw["停止喊话"]   # different intents
    # Longest-first picks the 4-char match.
    assert m.longest_match("停止喊话") == m._by_kw["停止喊话"]


def test_shortest_first_is_the_mutation_that_would_break_it():
    """MUTATION (guards longest-first): a shortest-first matcher would
    resolve '停止喊话' to D08's '喊话' -- the wrong intent. Simulated by
    picking the SHORTEST matching keyword; the assertion that this
    differs from the correct longest-first result must hold, proving
    longest-first is load-bearing."""
    m = KeywordMatcher(_reg())
    # shortest-first over the same index:
    text = "停止喊话"
    shortest = None
    for kw, intent in m._by_kw.items():
        if kw in text:
            if shortest is None or len(kw) < len(shortest[0]):
                shortest = (kw, intent)
    correct = m.longest_match(text)
    # The mutation (shortest) picks a DIFFERENT (wrong) intent.
    assert shortest is not None
    assert shortest[1] != correct


# -- keywords come from intents.yaml, not a hardcoded map (variant B) --

def test_index_is_built_from_registry_keywords_not_hardcoded():
    """The matcher index MUST derive from registry keywords (intents.yaml
    -> 16 S6.6 + 18), NOT a hardcoded demo map. Guard: a known
    intents.yaml keyword resolves to the registry's id for that intent.
    MUTATION: if the matcher shipped a hardcoded naive_classify-style map
    (你好->D07 etc), this cross-check against the registry would fail."""
    reg = _reg()
    m = KeywordMatcher(reg)
    # 你好 is J01 greeting in the registry (16/18), not D07 (the old
    # naive_classify demo-map bug).
    assert reg.by_id("J01").name == "greeting"
    assert "你好" in reg.by_id("J01").keywords
    assert m.longest_match("你好") == "J01"


# -- bypass + session-response intents excluded from layer 2 ----------

def test_bypass_intents_not_in_layer2_index():
    """estop/prone/stand (route=bypass) are layer-1 matched; their
    keywords must NOT be in the layer-2 index, or a normal '停止' phrase
    would resolve to estop at layer 2 instead of reaching layer 1."""
    m = KeywordMatcher(_reg())
    # estop keyword 急停 must be absent from the layer-2 index.
    assert "急停" not in m._by_kw
    # A bare '停止' therefore does NOT match A01 at layer 2.
    assert m.longest_match("停止") != "A01"


def test_session_response_intents_excluded():
    """I01 confirm / I02 deny are layer-3 session responses; their
    one-char triggers ('是'/'不') must not substring-mis-fire at layer 2
    (e.g. '天气不错' must NOT become deny)."""
    reg = _reg()
    m = KeywordMatcher(reg)
    r = classify_text("天气不错今天", reg, matcher=m)
    assert r.intent != "I02"
    # It falls through to overheard/unknown, not a spurious deny.
    assert r.layer in ("overheard", "unknown")


# -- miss -> overheard (16 S5.2.1) ------------------------------------

def test_overheard_on_ambient_speech():
    """A third-person snippet with no wake word / imperative / keyword
    is overheard (16 S5.2.1), NOT sent to the LLM."""
    reg = _reg()
    r = classify_text("队友说的悄悄话", reg)
    assert r.layer == "overheard"
    assert r.fires_llm is False


def test_directed_miss_goes_to_llm():
    """A directed phrase (imperative marker) that matches no keyword
    reaches layer 6 -> LLM (16 S5.2 step 6)."""
    reg = _reg()
    # '帮我' is an imperative marker (priority_chain _IMPERATIVE_MARKERS)
    # and the rest matches no keyword.
    r = classify_text("帮我看看那边的情况怎么样", reg)
    assert r.layer == "unknown"
    assert r.fires_llm is True
