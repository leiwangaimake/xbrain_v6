"""GWY-P4-02 partial -- intent_router MVP tests + variants."""

import pytest

from xbrain.p4_agent import intent_router as ir


pytestmark = pytest.mark.no_device


# --- Closed-set validation ------------------------------------------

def test_route_decision_rejects_out_of_set_route():
    with pytest.raises(ValueError):
        ir.RouteDecision(route="magic_route")


def test_validate_route_accepts_all_four():
    for r in (ir.ROUTE_FASTPATH, ir.ROUTE_LLM,
              ir.ROUTE_BYPASS, ir.ROUTE_FASTPATH_THEN_LLM):
        ir.validate_route(r)


def test_validate_route_rejects_unknown():
    with pytest.raises(ValueError):
        ir.validate_route("nonexistent")


def test_reason_defaults_to_route_name_when_empty():
    d = ir.RouteDecision(route=ir.ROUTE_LLM)
    assert d.reason == "route=llm"


# --- Empty transcript -> BYPASS -------------------------------------

def test_empty_transcript_routes_bypass():
    d = ir.classify("")
    assert d.route == ir.ROUTE_BYPASS


def test_whitespace_only_transcript_routes_bypass():
    d = ir.classify("   \n  ")
    assert d.route == ir.ROUTE_BYPASS


# --- Cancel markers -> BYPASS ---------------------------------------

@pytest.mark.parametrize("phrase", ["算了", "没事了", "取消一下", "cancel please"])
def test_cancel_marker_routes_bypass(phrase):
    d = ir.classify(phrase)
    assert d.route == ir.ROUTE_BYPASS
    assert "cancel" in d.reason or "算了" in d.reason or "没事" in d.reason or "取消" in d.reason


# --- Fastpath keyword matches ---------------------------------------

@pytest.mark.parametrize("phrase,intent", [
    ("前进",         "move_forward"),
    ("向前走",       "move_forward"),
    ("请后退一米",   "move_backward"),
    ("左转",         "turn_left"),
    ("向右转",       "turn_right"),
    ("停下来",       "stop"),
    ("stop now",     "stop"),
    ("云台向上一点", "ptz_move_up"),
    ("开灯",         "set_light_on"),
    ("关灯",         "set_light_off"),
    ("拍照",         "take_photo"),
])
def test_fastpath_keyword_matches_intent(phrase, intent):
    d = ir.classify(phrase)
    assert d.route == ir.ROUTE_FASTPATH
    assert d.matched_intent == intent


# --- LLM route (no fastpath match) ---------------------------------

@pytest.mark.parametrize("phrase", [
    "机器人你好",
    "今天天气怎么样",
    "帮我查一下三号巡逻点的状态",
])
def test_no_fastpath_match_routes_llm(phrase):
    d = ir.classify(phrase)
    assert d.route == ir.ROUTE_LLM
    assert d.matched_intent == ""


# --- Variant: cancel marker beats fastpath keyword ------------------

def test_cancel_marker_beats_fastpath_keyword():
    """VARIANT: 'stop' is a fastpath keyword AND 'cancel' is a cancel
    marker. If the phrase carries both markers, cancel wins (route
    to BYPASS, not fastpath). This is the "safer default" -- refusing
    to move on ambiguity is better than moving under it."""
    d = ir.classify("cancel stop")
    assert d.route == ir.ROUTE_BYPASS


# --- Purity: same input -> same output ------------------------------

def test_classify_is_pure_function():
    """Called twice with the same transcript, must return equal
    decisions. If a state accumulator sneaks in, this test catches it."""
    a = ir.classify("向前走")
    b = ir.classify("向前走")
    assert a == b


# --- Property: every decision.route is in the closed set ------------

def test_every_returned_decision_has_valid_route():
    """Meta: property test. For ANY transcript (in a sample), the
    returned RouteDecision.route must pass validate_route."""
    samples = [
        "", " ", "前进", "cancel", "算了", "开灯", "云台向左",
        "机器人你好", "今天天气", "the quick brown fox",
        "!@#$%^&*()", "12345", "一二三",
    ]
    for s in samples:
        d = ir.classify(s)
        ir.validate_route(d.route)   # would raise on drift


# --- Closed-set integrity vs docs ----------------------------------
# The 4 routes come from 16 S6.6 as a fixed closed set. A change to
# ROUTES must be a deliberate, documented act.

def test_route_closed_set_has_exactly_four_values():
    assert len(ir._ROUTES) == 4
