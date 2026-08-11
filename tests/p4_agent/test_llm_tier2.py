"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_llm_tier2.py
Brief: GWY-P4-37 (32.E) -- tier-2 classify: gate + grammar + <=5 projection

Description:
Tests the tier-2 LLM classify coordinator and generate_grammar against the
real registry. Each criterion carries a mutation that must turn red per
CLAUDE.md 3.3: LLM must pass the GPU gate, grammar projects <=5 intents,
the LLM call is always grammar-constrained.
"""
from __future__ import annotations

import pytest
import yaml

from xbrain.p4_agent.gbnf.generator import (
    GbnfInvariantError, MAX_MISSION_INTENTS, generate_grammar,
    project_mission_intents,
)
from xbrain.p4_agent.gateway.gpu_token import GpuTokenState
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.llm_tier2 import (
    Tier2Error, Tier2Result, classify_unknown,
)

pytestmark = pytest.mark.no_device

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"


def _reg():
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


class _RecordingLlm:
    """Stub LLM classify call. Records how it was invoked so the test can
    assert it was (or was NOT) called and WITH what grammar."""

    def __init__(self, ret='{"intent":"move_forward"}'):
        self.calls = []
        self.ret = ret

    def __call__(self, prompt, grammar):
        self.calls.append((prompt, grammar))
        return self.ret


# -- generate_grammar: projection + <=5 (criterion 2) ---------------------

def test_project_and_generate_small_mission():
    reg = _reg()
    names = ["move_forward", "move_backward", "turn_left"]   # all route=llm
    alternation, routes = project_mission_intents(reg, names)
    g = generate_grammar(alternation, routes)
    assert "intent ::=" in g
    for n in names:
        assert '\\"%s\\"' % n in g          # each intent name is a grammar literal


def test_generate_grammar_rejects_full_registry():
    """MUTATION B: hand generate_grammar the WHOLE 128-intent registry ->
    must raise (AI-36 / S5.1: a mission exposes <=5, never all)."""
    reg = _reg()
    all_names = [e.name for e in reg.entries if e.route in ("llm", "fastpath_then_llm")]
    assert len(all_names) > MAX_MISSION_INTENTS      # sanity: there ARE more than 5
    alternation, routes = project_mission_intents(reg, all_names)
    with pytest.raises(GbnfInvariantError) as ei:
        generate_grammar(alternation, routes)
    assert str(MAX_MISSION_INTENTS) in str(ei.value)


def test_generate_grammar_rejects_fastpath_intent():
    """GB-1c: a fastpath intent must not enter a mission grammar."""
    reg = _reg()
    # set_speed_profile is fastpath; mixing it in must be rejected.
    alternation, routes = project_mission_intents(
        reg, ["move_forward", "set_speed_profile"])
    with pytest.raises(GbnfInvariantError) as ei:
        generate_grammar(alternation, routes)
    assert "GB-1c" in str(ei.value)


def test_generate_grammar_rejects_unknown_in_alternation():
    """I4: 'unknown' is the no-match fallback, not an alternation member."""
    with pytest.raises(GbnfInvariantError) as ei:
        generate_grammar(["move_forward", "unknown"],
                         {"move_forward": "llm", "unknown": "llm"})
    assert "I4" in str(ei.value)


def test_project_mission_intent_unknown_name_raises():
    reg = _reg()
    with pytest.raises(Exception):
        project_mission_intents(reg, ["not_a_real_intent"])


# -- classify_unknown: GPU gate (criterion 1) -----------------------------

def _grammar(reg):
    alt, routes = project_mission_intents(reg, ["move_forward", "turn_left"])
    return generate_grammar(alt, routes)


def test_classify_admits_and_calls_llm_with_grammar():
    reg = _reg()
    g = _grammar(reg)
    llm = _RecordingLlm()
    st = GpuTokenState()
    res = classify_unknown("prompt", g, st, now_mono_ms=1000, llm_grammar_call=llm)
    assert res.admitted is True
    assert res.raw == '{"intent":"move_forward"}'
    assert len(llm.calls) == 1
    # criterion 3: the LLM was called WITH the grammar, not free.
    assert llm.calls[0][1] == g
    # token released after the call (slot free again).
    assert st.slot_taken is False


def test_classify_denied_when_token_busy_does_not_call_llm():
    """MUTATION A guard: when the GPU slot is already taken the coordinator
    must NOT reach the LLM. A path that bypassed try_admit and called
    llm_client directly would invoke the stub here -- the 'no call' assert
    catches it ('LLM must pass token')."""
    reg = _reg()
    g = _grammar(reg)
    llm = _RecordingLlm()
    st = GpuTokenState(slot_taken=True)              # someone else holds the slot
    res = classify_unknown("prompt", g, st, now_mono_ms=1000, llm_grammar_call=llm)
    assert res.admitted is False
    assert res.reason == "slot_taken"
    assert llm.calls == []                           # LLM was NOT called


def test_classify_open_circuit_denies_with_tts():
    """16 S9: an open circuit denies with a mandatory 'unavailable' TTS
    (no silent circuit break) and does not call the LLM."""
    reg = _reg()
    g = _grammar(reg)
    llm = _RecordingLlm()
    st = GpuTokenState(open_since_millis=0, open_duration_millis=60_000)
    res = classify_unknown("prompt", g, st, now_mono_ms=1000, llm_grammar_call=llm)
    assert res.admitted is False
    assert res.must_tts is True
    assert res.tts_text
    assert llm.calls == []


def test_classify_llm_failure_releases_token_and_raises():
    reg = _reg()
    g = _grammar(reg)

    def boom(prompt, grammar):
        raise RuntimeError("server down")

    st = GpuTokenState()
    with pytest.raises(Tier2Error):
        classify_unknown("prompt", g, st, now_mono_ms=1000, llm_grammar_call=boom)
    # token released even though the call failed (slot free, breaker moved).
    assert st.slot_taken is False
    assert st.consecutive_timeouts == 1


# -- criterion 3: grammar always required ---------------------------------

def test_classify_empty_grammar_raises_before_admission():
    """MUTATION C guard: an empty grammar (an unconstrained classify) must
    be refused BEFORE the GPU slot is taken -- the LLM is never called
    without a grammar (16 S7 GB-1)."""
    llm = _RecordingLlm()
    st = GpuTokenState()
    with pytest.raises(Tier2Error):
        classify_unknown("prompt", "", st, now_mono_ms=1000, llm_grammar_call=llm)
    assert st.slot_taken is False                    # slot never burned
    assert llm.calls == []
