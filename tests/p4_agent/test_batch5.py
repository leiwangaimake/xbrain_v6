"""GWY-P4-15/16/17/18 batch 5 tests."""

import pytest

from xbrain.p4_agent.session.level_routing import (
    Level, LevelRoutingError, SessionUpgrade,
    resolve_level, try_downgrade_to_l1a, upgrade_to_l1b,
)
from xbrain.p4_agent.session.state_machines import (
    CHITCHAT_WHITELIST, L2ConfirmState, L2Slot, L3ApprovalState, L3Slot,
    RecordingSlot, RecordingState, is_chitchat_interrupt,
)
from xbrain.p4_agent.templates.query_engine import (
    TemplateSchemaError, check_qt_branches,
)
from xbrain.p4_agent.templates.restate_engine import (
    RestateSchemaError,
    check_rs1_numeric_uses_request_word,
    check_rs2_starts_with_action,
    check_rs3_placeholders_available,
    needs_rs4_correction,
)


pytestmark = pytest.mark.no_device


# --- P4-15 query template branches ---

def test_qt_battery_requires_ok_unknown_shadow():
    check_qt_branches("QT-1_battery",
                      frozenset({"ok", "unknown", "shadow"}))


def test_qt_battery_missing_shadow_raises():
    with pytest.raises(TemplateSchemaError) as ei:
        check_qt_branches("QT-1_battery", frozenset({"ok", "unknown"}))
    assert "shadow" in str(ei.value)


def test_qt_non_registered_id_no_op():
    check_qt_branches("not_a_qt_template", frozenset())   # no raise


# --- P4-16 restate ---

def test_rs1_numeric_without_request_word_raises():
    with pytest.raises(RestateSchemaError):
        check_rs1_numeric_uses_request_word("停止走 {amount} 米",
                                              has_numeric_placeholder=True)


def test_rs1_numeric_with_request_word_ok():
    check_rs1_numeric_uses_request_word("请求走 {amount} 米",
                                          has_numeric_placeholder=True)


def test_rs2_starts_with_action_ok():
    check_rs2_starts_with_action("走 5 米", frozenset({"走", "停"}))


def test_rs2_no_action_verb_raises():
    with pytest.raises(RestateSchemaError):
        check_rs2_starts_with_action("大约 5 米", frozenset({"走", "停"}))


def test_rs3_unknown_placeholder_raises():
    with pytest.raises(RestateSchemaError) as ei:
        check_rs3_placeholders_available(
            "走 {ghost} 米",
            slot_names=frozenset({"amount"}),
            state_names=frozenset({"battery"}))
    assert "ghost" in str(ei.value)


def test_rs4_correction_fires_on_mismatch():
    assert needs_rs4_correction(5, 3) is True
    assert needs_rs4_correction(5, 5) is False


# --- P4-17 level routing CL-1/2/3 ---

def test_resolve_level_default_l1a():
    s = SessionUpgrade()
    assert resolve_level(Level.L1a, s) == Level.L1a


def test_resolve_level_upgrade_to_l1b():
    s = SessionUpgrade()
    upgrade_to_l1b(s)
    assert resolve_level(Level.L1a, s) == Level.L1b


def test_l1b_stays_l1b_after_upgrade():
    s = SessionUpgrade()
    upgrade_to_l1b(s)
    assert resolve_level(Level.L1b, s) == Level.L1b


def test_downgrade_after_upgrade_raises_cl2():
    s = SessionUpgrade()
    upgrade_to_l1b(s)
    with pytest.raises(LevelRoutingError) as ei:
        try_downgrade_to_l1a(s)
    assert "CL-2" in str(ei.value)


# --- P4-18 session state machines ---

def test_l2_confirm_happy_path():
    slot = L2Slot(timeout_ms=1000)
    slot.request(now_mono_ms=0)
    slot.confirm()
    assert slot.state == L2ConfirmState.CONFIRMED


def test_l2_confirm_times_out():
    slot = L2Slot(timeout_ms=1000)
    slot.request(now_mono_ms=0)
    slot.tick(now_mono_ms=2000)
    assert slot.state == L2ConfirmState.TIMED_OUT


def test_l3_approve_happy_path():
    slot = L3Slot(timeout_ms=60_000)
    slot.request(now_mono_ms=0)
    slot.approve()
    assert slot.state == L3ApprovalState.APPROVED


def test_l3_stale_after_timeout():
    slot = L3Slot(timeout_ms=1000)
    slot.request(now_mono_ms=0)
    slot.tick(now_mono_ms=2000)
    assert slot.state == L3ApprovalState.STALE


def test_recording_captures_waypoints():
    r = RecordingSlot()
    r.start()
    r.add_waypoint("wp1")
    r.add_waypoint("wp2")
    r.finish()
    assert r.state == RecordingState.FINISHED
    assert r.waypoints == ["wp1", "wp2"]


def test_chitchat_whitelist_recognized():
    assert is_chitchat_interrupt("how_long_left")
    assert not is_chitchat_interrupt("shutdown")
