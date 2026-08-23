"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch4.py
Brief: GWY-P4-12/13/14 GBNF generator + IntentEnvelope EV-1..7 + validation

Description:
*** Brief 由占位串改写(2026-08-23). 原值是按路径自动生成的 "p4_agent tests -- batch4" --
既没说清本文件测什么, 也无法据以索引任务号(CLAUDE.md 2.5). 同一处理已对
P2 做过; 补 Brief 而不是绕过去, 是因为绕过去只会让下一个人再读一遍.
GWY-P4-12/13/14 batch 4 tests.
"""


import pytest

from xbrain.p4_agent.envelope.intent_envelope import (
    EnvelopeSchemaError, IntentEnvelope,
)
from xbrain.p4_agent.gbnf.generator import (
    GbnfInvariantError,
    check_gb_1c_no_fastpath_in_mission_grammar,
    check_gb_1d_ids_match_cmdset,
    check_i3_grammar_matches_few_shots,
    check_i4_no_unknown_in_alternation,
    check_r1_alternation_matches_mission_set,
    check_r4_no_empty_production,
)
from xbrain.p4_agent.validation.checks import (
    SlotSchema, ValidationResult, ValidationRule, validate,
)


pytestmark = pytest.mark.no_device


# --- P4-12 GBNF invariants ---

def test_r1_alternation_mismatch_raises():
    with pytest.raises(GbnfInvariantError):
        check_r1_alternation_matches_mission_set(
            ["a", "b"], frozenset({"a", "c"}))


def test_r4_empty_alternation_raises():
    with pytest.raises(GbnfInvariantError):
        check_r4_no_empty_production([])


def test_gb1c_fastpath_in_mission_rejected():
    routes = {"stop": "fastpath", "goto": "llm"}
    with pytest.raises(GbnfInvariantError):
        check_gb_1c_no_fastpath_in_mission_grammar(
            routes, ["stop", "goto"])


def test_gb1d_id_mismatch_raises():
    with pytest.raises(GbnfInvariantError):
        check_gb_1d_ids_match_cmdset(
            grammar_ids={"stop": "A99"},
            cmdset_ids={"stop": "A11"})


def test_gb1d_missing_from_cmdset_raises():
    with pytest.raises(GbnfInvariantError):
        check_gb_1d_ids_match_cmdset(
            grammar_ids={"ghost": "A99"},
            cmdset_ids={})


def test_i4_unknown_in_alternation_rejected():
    with pytest.raises(GbnfInvariantError):
        check_i4_no_unknown_in_alternation(["stop", "unknown", "goto"])


def test_i3_grammar_vs_few_shot_mismatch_raises():
    with pytest.raises(GbnfInvariantError):
        check_i3_grammar_matches_few_shots(
            frozenset({"a", "b"}), frozenset({"a", "c"}))


# --- P4-13 IntentEnvelope EV-1..EV-7 ---

def _env(**over):
    d = dict(
        id="A05", intent="move_forward", route="fastpath",
        auth="L1a", level="L1a",
        slots={"amount": 1.0, "unit": "m"},
        cmd_id="12345678-1234-1234-1234-123456789012",
        latency_class="fastpath",
    )
    d.update(over)
    return IntentEnvelope(**d)


def test_envelope_valid_construct():
    _env()   # no raise


def test_ev1_empty_id_rejected():
    with pytest.raises(EnvelopeSchemaError) as ei:
        _env(id="")
    assert "EV-1" in str(ei.value)


def test_ev2_bad_route_rejected():
    with pytest.raises(EnvelopeSchemaError) as ei:
        _env(route="magic")
    assert "EV-2" in str(ei.value)


def test_ev3_bad_auth_rejected():
    with pytest.raises(EnvelopeSchemaError) as ei:
        _env(auth="L4")
    assert "EV-3" in str(ei.value)


def test_ev5_bad_uuid_rejected():
    with pytest.raises(EnvelopeSchemaError) as ei:
        _env(cmd_id="not-a-uuid")
    assert "EV-5" in str(ei.value)


def test_ev6_slots_list_rejected():
    """EV-6: slots MUST be dict, never list."""
    with pytest.raises(EnvelopeSchemaError) as ei:
        _env(slots=["a", "b"])
    assert "EV-6" in str(ei.value)


def test_ev7_latency_class_route_mismatch_rejected():
    """EV-7 consistency: llm route with fastpath latency_class ->
    reject."""
    with pytest.raises(EnvelopeSchemaError) as ei:
        _env(route="llm", latency_class="fastpath")
    assert "EV-7" in str(ei.value)


def test_ev7_fastpath_then_llm_uses_fastpath_latency():
    """fastpath_then_llm route -> latency_class=fastpath (LLM leg
    is parallel, doesn't block latency-critical dispatch)."""
    _env(route="fastpath_then_llm", latency_class="fastpath")


# --- P4-14 validation ---

def _schema(**over):
    d = dict(
        required=frozenset({"amount"}),
        all_slots=frozenset({"amount", "unit"}),
        types={"amount": float, "unit": str},
        closed_sets={"unit": frozenset({"m", "cm"})},
        numeric_ranges={"amount": (0.0, 10.0)},
    )
    d.update(over)
    return SlotSchema(**d)


def test_validate_ok():
    r = validate("move_forward",
                  {"amount": 1.0, "unit": "m"},
                  _schema(), frozenset({"move_forward"}))
    assert r.accepted


def test_v7_unknown_intent():
    r = validate("magic",
                  {"amount": 1.0}, _schema(), frozenset({"stop"}))
    assert not r.accepted
    assert r.code == ValidationRule.V7


def test_v4_extra_slot():
    r = validate("move_forward",
                  {"amount": 1.0, "extra": "hi"},
                  _schema(), frozenset({"move_forward"}))
    assert r.code == ValidationRule.V4


def test_v3_missing_required():
    r = validate("move_forward",
                  {"unit": "m"},
                  _schema(), frozenset({"move_forward"}))
    assert r.code == ValidationRule.V3


def test_v5_type_mismatch():
    r = validate("move_forward",
                  {"amount": "not_number"},
                  _schema(), frozenset({"move_forward"}))
    assert r.code == ValidationRule.V5


def test_v1_closed_set_violation():
    r = validate("move_forward",
                  {"amount": 1.0, "unit": "km"},
                  _schema(), frozenset({"move_forward"}))
    assert r.code == ValidationRule.V1


def test_v2_range_violation():
    r = validate("move_forward",
                  {"amount": 100.0},
                  _schema(), frozenset({"move_forward"}))
    assert r.code == ValidationRule.V2
