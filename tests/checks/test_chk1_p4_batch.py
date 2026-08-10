"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chk1_p4_batch.py
Brief: CHK-1-28/29/30/31/32 P4 severe items batch

Description:
Five severe P4 items covering degrade modes, safety-intent
classifier, G-query bindings, rotation refusal scripts, and
speak_stop.
"""

from __future__ import annotations

import pytest

from xbrain.common.errors import (
    E_BUSY, E_CAPABILITY,
)
from xbrain.p4_agent.dialog.speak_stop import (
    REPLY_CASE_1, REPLY_CASE_2, REPLY_CASE_3, REPLY_CASE_4,
    REPLY_STOPPED_BARE, SpeakStopDecision,
    assert_route2_no_bracket_loop, decide, replies_pairwise_distinct,
)
from xbrain.p4_agent.failsafe.degrade_modes import (
    DEGRADE_MODES, DegradeModes, IntentRoute,
    assert_no_direct_audio_bypass,
)
from xbrain.p4_agent.failsafe.rotation_reject import (
    RJ_1_TEMPLATE, RJ_2_TEMPLATE, RotationRejectResponse,
    RotationRejectShapeError,
    precheck_yaw_capable, refuse_from_ack, scripts_are_distinct,
)
from xbrain.p4_agent.query.sources_g01_g24 import (
    BINDINGS, G_QUERY_IDS, QueryBindingError,
    assert_bindings_cover_all_24, assert_zero_llm_for_g_queries,
    g01_render, g02_render, g24_render,
)
from xbrain.p4_agent.routing.safety_keyword_gate import (
    SAFETY_INTENT_IDS, SafetyRoutingViolation,
    assert_projection_matches_frozen, check_intent_is_fastpath,
    project_safety_ids_from_intents_yaml,
)


pytestmark = pytest.mark.no_device


# ---------------- CHK-1-28 degrade modes ----------------

def test_degrade_modes_closed_set_six():
    assert len(DEGRADE_MODES) == 6


def test_llm_circuit_break_fastpath_unaffected():
    dm = DegradeModes()
    dm.enter("llm_circuit_break")
    r = dm.route_intent("fastpath", "task")
    assert r.llm_used is False and r.dropped is False


def test_llm_circuit_break_llm_route_gets_fallback_utterance():
    """Silent drop is the specific bug the CHK-1-28 (a) variant
    guards against."""
    dm = DegradeModes()
    dm.enter("llm_circuit_break")
    r = dm.route_intent("llm", "task")
    assert r.llm_used is False
    assert r.fallback_uttered is True
    assert r.dropped is False


def test_p3_unreachable_task_fails_query_ok():
    dm = DegradeModes()
    dm.enter("p3_unreachable")
    r_task = dm.route_intent("fastpath", "task")
    r_query = dm.route_intent("fastpath", "query")
    assert r_task.fallback_uttered is True
    assert r_query.dropped is False


def test_tts_preset_missing_emits_single_event():
    """CHK-1-28 (b): TTS preset absent must land ONE event, never
    raise."""
    dm = DegradeModes()
    dm.enter("tts_unavailable")
    ev = dm.preset_missing_event("unknown_key", available={})
    assert ev["level"] == "warn"
    assert ev["kind"] == "tts_preset_missing"


def test_tts_preset_available_returns_content():
    dm = DegradeModes()
    dm.enter("tts_unavailable")
    got = dm.tts_preset_lookup("hello", {"hello": "hello_recorded.opus"})
    assert got == "hello_recorded.opus"


def test_direct_audio_bypass_detection_hits():
    """CHK-1-28 (c) guard: p2_unreachable must not trigger direct
    audio device fallback."""
    with pytest.raises(AssertionError, match="direct-audio bypass"):
        assert_no_direct_audio_bypass(
            "some code that imports alsaaudio for offline playback")


def test_direct_audio_bypass_clean_source_ok():
    assert_no_direct_audio_bypass("import zenoh\nimport asyncio\n")


def test_degrade_mode_enter_unknown_raises():
    dm = DegradeModes()
    with pytest.raises(ValueError, match="unknown degrade mode"):
        dm.enter("halfway")


def test_degrade_mode_exit_idempotent():
    dm = DegradeModes()
    dm.exit("tts_unavailable")   # not entered; no raise
    dm.enter("tts_unavailable")
    dm.exit("tts_unavailable")
    assert not dm.is_active("tts_unavailable")


# ---------------- CHK-1-29 safety-intent classifier ----------------

def test_safety_intent_ids_are_seven():
    """B09, C01, C03, C04, B07, D12, D13 -- exactly."""
    assert SAFETY_INTENT_IDS == frozenset({
        "B09", "C01", "C03", "C04", "B07", "D12", "D13"})


def test_projection_matches_frozen_ok():
    """Well-formed intents.yaml produces exactly the frozen set."""
    intents = [{"id": i, "route": "fastpath", "safety_critical": True}
                 for i in SAFETY_INTENT_IDS] + [
        {"id": "G01", "route": "fastpath", "safety_critical": False},
        {"id": "F01", "route": "llm", "safety_critical": False},
    ]
    assert_projection_matches_frozen(intents)   # no raise


def test_projection_missing_yaml_entry_reddens():
    """CHK-1-29 (b): drop D13 from yaml -> projection reddens."""
    intents = [{"id": i, "route": "fastpath", "safety_critical": True}
                 for i in SAFETY_INTENT_IDS - {"D13"}]
    with pytest.raises(SafetyRoutingViolation, match="D13"):
        assert_projection_matches_frozen(intents)


def test_projection_extra_yaml_entry_reddens():
    """A rogue extra safety_critical in yaml also reddens."""
    intents = [{"id": i, "route": "fastpath", "safety_critical": True}
                 for i in SAFETY_INTENT_IDS | {"E99"}]
    with pytest.raises(SafetyRoutingViolation, match="E99"):
        assert_projection_matches_frozen(intents)


def test_safety_intent_llm_route_refused():
    """CHK-1-29 (a): a safety intent must never land in llm route."""
    with pytest.raises(SafetyRoutingViolation, match="fastpath"):
        check_intent_is_fastpath("D13", "llm", llm_request_count=0)


def test_safety_intent_nonzero_llm_refused():
    with pytest.raises(SafetyRoutingViolation, match="LLM"):
        check_intent_is_fastpath("D13", "fastpath",
                                    llm_request_count=1)


def test_safety_intent_fastpath_zero_llm_ok():
    check_intent_is_fastpath("D12", "fastpath", llm_request_count=0)


def test_nonsafety_intent_no_check():
    """Non-safety intents pass through unaffected (only 7 are safety)."""
    check_intent_is_fastpath("G01", "llm", llm_request_count=3)


# ---------------- CHK-1-30 G-query bindings ----------------

def test_g_query_ids_are_24():
    assert len(G_QUERY_IDS) == 24


def test_bindings_cover_all_24():
    assert_bindings_cover_all_24()   # no raise


def test_binding_missing_template_key_reddens():
    """A binding without both ok + no_data must fail load."""
    from xbrain.p4_agent.query.sources_g01_g24 import (
        BINDINGS, QueryBinding, assert_bindings_cover_all_24,
    )
    backup = BINDINGS["G01"]
    BINDINGS["G01"] = QueryBinding(intent_id="G01", source="state/pose",
                                      templates={"ok": "..."})
    try:
        with pytest.raises(QueryBindingError, match="template key"):
            assert_bindings_cover_all_24()
    finally:
        BINDINGS["G01"] = backup


def test_g24_ts_unsynced_never_reports_time():
    """CHK-1-30 (iii) hard branch."""
    reply = g24_render(clock_iso="2026-08-10T12:34:56", ts_sync=False)
    assert "未同步" in reply or "不准" in reply
    assert "12:34:56" not in reply     # NEVER report the timestamp


def test_g24_ts_synced_reports_time():
    reply = g24_render(clock_iso="2026-08-10T12:34:56", ts_sync=True)
    assert "12:34:56" in reply


def test_g02_returns_min_of_two_packs():
    """CHK-1-30 (iv): dual-pack rule is MIN, not mean/max."""
    reply = g02_render(pack_a_soc_pct=80, pack_b_soc_pct=35)
    assert "35" in reply
    assert "80" not in reply
    assert "57" not in reply      # would be the mean


def test_g01_uses_latest_pose_not_first_syllable():
    """CHK-1-30 (v): G01 must read latest pose. Passing a distinct
    'first-syllable' snapshot must NOT show up in the reply."""
    reply = g01_render(
        pose_latest={"x_m": 10.5, "y_m": 20.3},
        pose_first_syllable={"x_m": 5.0, "y_m": 10.0})
    assert "10.5" in reply and "20.3" in reply
    assert "5.0" not in reply


def test_g_queries_zero_llm():
    assert_zero_llm_for_g_queries(0)


def test_g_queries_nonzero_llm_reddens():
    with pytest.raises(QueryBindingError, match="LLM"):
        assert_zero_llm_for_g_queries(1)


# ---------------- CHK-1-31 rotation refusal scripts ----------------

def test_rj1_rj2_scripts_distinct():
    """The whole point of a 2-way split: operator can tell them apart."""
    assert scripts_are_distinct() is True
    assert RJ_1_TEMPLATE != RJ_2_TEMPLATE


def test_rj1_on_capability():
    r = refuse_from_ack(E_CAPABILITY, {})
    assert r.script_kind == "RJ-1"
    assert r.emit_cmd_motion_intent is False   # no pointless publish
    assert r.audit_level == "warn"


def test_rj2_on_busy_quotes_detail_fields_literally():
    """RJ-2 script must include the actual occ_count + r_check_m."""
    r = refuse_from_ack(E_BUSY, {"occ_count": 3, "r_check_m": 0.45})
    assert r.script_kind == "RJ-2"
    assert "3" in r.script
    assert "0.45" in r.script


def test_rj2_missing_detail_field_raises():
    """Missing occ_count / r_check_m -> raise (never zero-fill)."""
    with pytest.raises(RotationRejectShapeError, match="occ_count"):
        refuse_from_ack(E_BUSY, {"r_check_m": 0.5})


def test_refuse_from_ack_unknown_code_raises():
    with pytest.raises(RotationRejectShapeError, match="unhandled"):
        refuse_from_ack("E_TIMEOUT", {})


def test_precheck_yaw_incapable_short_circuits():
    r = precheck_yaw_capable(yaw_capable=False)
    assert r.script_kind == "RJ-1"
    assert r.emit_cmd_motion_intent is False


def test_precheck_yaw_capable_true_raises():
    """Caller misuse: precheck should only be called when incapable."""
    with pytest.raises(RotationRejectShapeError):
        precheck_yaw_capable(yaw_capable=True)


# ---------------- CHK-1-32 speak_stop ----------------

def test_reply_case_1_only_current():
    """Pending 3 + current playing -> case 1 wording."""
    d = decide(pending_count=3, is_current_mid_play=True,
                 repeat_remaining=0)
    assert d.case == "case_1"
    assert d.reply == REPLY_CASE_1
    assert d.emit_payload_stop is True
    assert d.clear_pending is True


def test_reply_case_2_current_only_no_more():
    """Only the current utterance, no pending -> case 2."""
    d = decide(pending_count=0, is_current_mid_play=True,
                 repeat_remaining=0)
    assert d.case == "case_2"
    assert d.reply == REPLY_CASE_2
    assert d.clear_pending is False


def test_reply_case_3_nothing_playing_idempotent():
    """CHK-1-32 (c) guard: not playing -> still emit ONE stop
    (idempotent)."""
    d = decide(pending_count=0, is_current_mid_play=False,
                 repeat_remaining=0)
    assert d.case == "case_3"
    assert d.reply == REPLY_CASE_3
    assert d.emit_payload_stop is True


def test_reply_case_4_repeat_remaining_cleared():
    """CHK-1-32 (b) guard: repeat cycle must be cleared."""
    d = decide(pending_count=0, is_current_mid_play=True,
                 repeat_remaining=5)
    assert d.case == "case_4"
    assert d.reply == REPLY_CASE_4
    assert d.clear_pending is True


def test_replies_pairwise_distinct():
    assert replies_pairwise_distinct() is True


def test_no_reply_equals_bare_stopped():
    """CHK-1-32 (ii): bare '已停止喊话' would mask cases 1/2/4."""
    for r in (REPLY_CASE_1, REPLY_CASE_2, REPLY_CASE_3, REPLY_CASE_4):
        assert r != REPLY_STOPPED_BARE


def test_route2_no_bracket_loop_clean():
    assert_route2_no_bracket_loop("normal code without loop endpoint")


def test_route2_bracket_loop_forbidden():
    with pytest.raises(AssertionError, match=r"\[32\]"):
        assert_route2_no_bracket_loop("for utt in queue[32]: send(utt)")
