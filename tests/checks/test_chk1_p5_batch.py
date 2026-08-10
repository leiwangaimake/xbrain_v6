"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chk1_p5_batch.py
Brief: CHK-1-16/41/42/43/44 P5 severe items batch

Description:
Five severe P5 items: HMI uplink limits + degrade + sanitisation;
key surface bidirectional diff; state/media publish + credential
scan; outbound projection to Qt cmd/task/{ack,progress,result};
error-code map v6 E_* <-> Qt integer.
"""

from __future__ import annotations

import pytest

from xbrain.common.errors import E_BUSY, E_CAPABILITY, E_SCHEMA
from xbrain.p5_gateway.errormap.qt_int_codes import (
    ErrorCodeMapDivergence, NeedsDetailMissing, QT_CODE_MAP,
    QtVisibilityViolation,
    assert_bidirectional_diff_empty, assert_v6_code_not_in_qt_display,
    outbound_text_for_reason, translate,
)
from xbrain.p5_gateway.outbound.key_surface import (
    KeySurfaceDivergence, P5_EXPECTED_PUBLISHERS,
    P5_EXPECTED_SUBSCRIBERS, assert_surface_matches, diff,
)
from xbrain.p5_gateway.outbound.projection import (
    OUTBOUND_KEYS, ProjectionShapeError, TerminalSourceViolation,
    check_terminal_source, has_both_time_fields,
    outbound_keys_bidirectional_diff, project_progress,
    split_progress_by_task,
)
from xbrain.p5_gateway.outbound.state_media import (
    ENDPOINT_KINDS, Endpoint, FORBIDDEN_CREDENTIAL_KEYS,
    HEARTBEAT_PERIOD_MS, REQUIRED_QOS_PROFILE,
    StateMediaCredentialLeak, StateMediaPublisher,
    assert_qos_profile, build_payload, scan_credential_keys,
)
from xbrain.p5_gateway.uplink.rate_limit import (
    DEGRADE_ENTER_CONSECUTIVE_REJECTS,
    ForbiddenConnectionMutation, ForbiddenMessagePassthrough,
    INVALID_REQ_TYPE_PLACEHOLDER, RejectAckCoalescer,
    UPLINK_STATE_NORMAL, UPLINK_STATE_RESTRICTED, UplinkConfigError,
    UplinkDegradeState, UplinkLimits,
    can_admit_in_state, emit_ack_message,
    refuse_close_ws, refuse_ip_ban,
    sanitise_event_detail_never_echo, sanitise_req_id,
    sanitise_req_type,
)


pytestmark = pytest.mark.no_device


# ---------------- CHK-1-16 uplink limits ----------------

def test_uplink_limits_zero_refused():
    with pytest.raises(UplinkConfigError):
        UplinkLimits(non_teleop_msgs_per_sec=0,
                       teleop_msgs_per_sec_per_client_id=20,
                       payload_cap_geo_bytes=1, payload_cap_other_bytes=1,
                       reject_ack_per_sec=5)


def test_degrade_enters_after_20_consecutive_rejects():
    s = UplinkDegradeState()
    for i in range(20):
        s.note_reject(now_mono_ms=i)
    assert s.state == UPLINK_STATE_RESTRICTED
    assert s.consecutive_rejects == DEGRADE_ENTER_CONSECUTIVE_REJECTS


def test_degrade_accept_resets_counter():
    s = UplinkDegradeState()
    for i in range(19):
        s.note_reject(now_mono_ms=i)
    s.note_accept(now_mono_ms=19)
    assert s.state == UPLINK_STATE_NORMAL
    assert s.consecutive_rejects == 0


def test_degrade_auto_exit_after_60s_idle():
    s = UplinkDegradeState()
    for i in range(20):
        s.note_reject(now_mono_ms=i)
    assert s.state == UPLINK_STATE_RESTRICTED
    s.tick_clock(now_mono_ms=61_000)
    assert s.state == UPLINK_STATE_NORMAL


def test_restricted_still_admits_estop():
    assert can_admit_in_state("estop", UPLINK_STATE_RESTRICTED) is True
    assert can_admit_in_state("goto", UPLINK_STATE_RESTRICTED) is False


def test_normal_admits_everything():
    for rt in ("estop", "goto", "chat"):
        assert can_admit_in_state(rt, UPLINK_STATE_NORMAL) is True


def test_sanitise_req_type_bad_placeholder():
    """S-1: bad -> '<invalid>' (never propagate raw)."""
    assert sanitise_req_type("Bad Type With Spaces") == INVALID_REQ_TYPE_PLACEHOLDER
    assert sanitise_req_type("") == INVALID_REQ_TYPE_PLACEHOLDER
    assert sanitise_req_type(None) == INVALID_REQ_TYPE_PLACEHOLDER


def test_sanitise_req_type_good_pass_through():
    assert sanitise_req_type("goto") == "goto"


def test_sanitise_req_id_bad_null():
    """S-2: bad req_id -> None."""
    assert sanitise_req_id("with space") is None
    assert sanitise_req_id(None) is None


def test_sanitise_req_id_good_pass_through():
    assert sanitise_req_id("req-001_A") == "req-001_A"


def test_emit_ack_message_from_table_only():
    """S-3: ack message MUST come from table, never concatenated."""
    msg = emit_ack_message("thanks", {"thanks": "operation received"})
    assert msg == "operation received"


def test_emit_ack_message_unknown_key_refused():
    with pytest.raises(ForbiddenMessagePassthrough):
        emit_ack_message("halfway", {"thanks": "..."})


def test_sanitise_event_detail_strips_payload_keys():
    """S-4: rejected payload keys must not echo into event.detail."""
    detail = {"reason": "invalid", "client_content": "MALICIOUS"}
    payload = {"client_content": "MALICIOUS", "other": 1}
    clean = sanitise_event_detail_never_echo(payload, detail)
    assert "client_content" not in clean
    assert "reason" in clean


def test_refuse_close_ws_raises():
    """Never close the WebSocket -- would drop the estop button."""
    with pytest.raises(ForbiddenConnectionMutation, match="banned"):
        refuse_close_ws()


def test_refuse_ip_ban_raises():
    with pytest.raises(ForbiddenConnectionMutation, match="blacklist"):
        refuse_ip_ban("192.168.1.100")


def test_coalescer_pass_through_under_limit():
    c = RejectAckCoalescer(limit_per_sec=5)
    r = c.observe("req-1", now_mono_ms=0)
    assert r["req_id"] == "req-1" and r["suppressed"] == 0


def test_coalescer_bundles_over_limit_and_never_drops():
    """CHK-1-16 (iii): over-limit acks MUST NOT be dropped -- they
    coalesce into a synthetic ack with count."""
    c = RejectAckCoalescer(limit_per_sec=2)
    outs = [c.observe(f"req-{i}", now_mono_ms=0) for i in range(5)]
    assert [o["suppressed"] for o in outs] == [0, 0, 1, 2, 3]
    assert len(outs) == 5


def test_coalescer_new_window_resets():
    c = RejectAckCoalescer(limit_per_sec=2)
    c.observe("a", now_mono_ms=0)
    c.observe("b", now_mono_ms=0)
    c.observe("c", now_mono_ms=0)
    r = c.observe("d", now_mono_ms=1500)
    assert r["req_id"] == "d" and r["suppressed"] == 0


# ---------------- CHK-1-41 key surface ----------------

def test_key_surface_matches_expected_no_raise():
    assert_surface_matches(P5_EXPECTED_PUBLISHERS, P5_EXPECTED_SUBSCRIBERS)


def test_key_surface_missing_publisher_reddens():
    """CHK-1-41 variant 1: drop state/media publisher -> red."""
    trimmed = P5_EXPECTED_PUBLISHERS - {"state/media"}
    with pytest.raises(KeySurfaceDivergence, match="state/media"):
        assert_surface_matches(trimmed, P5_EXPECTED_SUBSCRIBERS)


def test_key_surface_missing_subscriber_reddens():
    """CHK-1-41 variant 2: drop cmd/task subscriber -> red."""
    trimmed = P5_EXPECTED_SUBSCRIBERS - {"cmd/task"}
    with pytest.raises(KeySurfaceDivergence, match="cmd/task"):
        assert_surface_matches(P5_EXPECTED_PUBLISHERS, trimmed)


def test_key_surface_extra_private_key_reddens():
    """CHK-1-41 variant 3: extra key not in 11 §2.2 -> red."""
    extra_pubs = P5_EXPECTED_PUBLISHERS | {"private/leak"}
    with pytest.raises(KeySurfaceDivergence, match="private/leak"):
        assert_surface_matches(extra_pubs, P5_EXPECTED_SUBSCRIBERS)


def test_key_surface_diff_reports_both_directions():
    d = diff(actual_pubs=P5_EXPECTED_PUBLISHERS - {"state/media"},
              actual_subs=P5_EXPECTED_SUBSCRIBERS | {"private/leak"})
    assert "state/media" in d.spec_only_publishers
    assert "private/leak" in d.impl_only_subscribers


# ---------------- CHK-1-42 state/media ----------------

def test_endpoint_kind_closed_set():
    with pytest.raises(ValueError, match="not in"):
        Endpoint(name="cam1", kind="thermal_x", reachable=True,
                    last_ok_mono_ms=0)


def test_endpoint_kinds_closed_set_size():
    assert ENDPOINT_KINDS == {"rgb", "ir", "rgbd"}


def test_payload_credential_key_leaks_refused():
    """CHK-1-42 (iii): password/secret/etc must not appear in payload."""
    with pytest.raises(StateMediaCredentialLeak, match="password"):
        scan_credential_keys({"endpoints": [{"name": "c1",
                                                "password": "p"}]})


def test_all_forbidden_credential_keys_caught():
    for key in FORBIDDEN_CREDENTIAL_KEYS:
        with pytest.raises(StateMediaCredentialLeak, match=key):
            scan_credential_keys({key: "value"})


def test_payload_clean_ok():
    payload = build_payload([
        Endpoint(name="rgb-front", kind="rgb", reachable=True,
                    last_ok_mono_ms=100),
        Endpoint(name="ir-front", kind="ir", reachable=False,
                    last_ok_mono_ms=50),
    ])
    assert len(payload["endpoints"]) == 2


def test_publisher_change_triggered():
    """CHK-1-42 (i): reachability flip -> immediate publish."""
    p = StateMediaPublisher()
    eps = [Endpoint(name="cam1", kind="rgb", reachable=True,
                      last_ok_mono_ms=100)]
    first = p.observe(eps, now_mono_ms=100)
    assert first is not None
    assert p.observe(eps, now_mono_ms=200) is None
    eps2 = [Endpoint(name="cam1", kind="rgb", reachable=False,
                       last_ok_mono_ms=100)]
    r = p.observe(eps2, now_mono_ms=300)
    assert r is not None
    assert r["endpoints"][0]["reachable"] is False


def test_publisher_heartbeat_after_10s():
    p = StateMediaPublisher()
    eps = [Endpoint(name="cam1", kind="rgb", reachable=True,
                      last_ok_mono_ms=100)]
    p.observe(eps, now_mono_ms=0)
    r = p.observe(eps, now_mono_ms=HEARTBEAT_PERIOD_MS)
    assert r is not None


def test_qos_profile_must_be_q2_state():
    assert_qos_profile("Q2_state")
    with pytest.raises(ValueError, match="Q2_state"):
        assert_qos_profile("Q0_safety")


def test_qos_required_constant():
    assert REQUIRED_QOS_PROFILE == "Q2_state"


# ---------------- CHK-1-43 outbound projection ----------------

def test_outbound_keys_three_families():
    assert OUTBOUND_KEYS == ("cmd/task/ack", "cmd/task/progress",
                                "cmd/task/result")


def test_progress_unknown_total_gives_none_percent():
    """CHK-1-43 (iii) GUARD: filling with 0 is fail-silent."""
    f = project_progress(task_id="t1", route_total_m=None,
                            distance_travelled_m=5.0, step="running",
                            timestamp=0.0, mono_ms=0)
    assert f.progress_percent is None


def test_progress_zero_total_also_gives_none():
    f = project_progress(task_id="t1", route_total_m=0.0,
                            distance_travelled_m=5.0, step="running",
                            timestamp=0.0, mono_ms=0)
    assert f.progress_percent is None


def test_progress_normal_computes_percent():
    f = project_progress(task_id="t1", route_total_m=100.0,
                            distance_travelled_m=25.0, step="running",
                            timestamp=0.0, mono_ms=0)
    assert f.progress_percent == 25.0


def test_progress_no_task_id_refused():
    with pytest.raises(ProjectionShapeError, match="task_id"):
        project_progress(task_id="", route_total_m=100.0,
                          distance_travelled_m=25.0, step="running",
                          timestamp=0.0, mono_ms=0)


def test_split_progress_by_task():
    """CHK-1-43 (ii): two concurrent tasks -> disjoint streams."""
    frames = [
        project_progress("t1", 100.0, 10.0, "running", 0.0, 0),
        project_progress("t2", 100.0, 20.0, "running", 0.0, 0),
        project_progress("t1", 100.0, 15.0, "running", 0.0, 0),
    ]
    per_task = split_progress_by_task(frames)
    assert len(per_task) == 2
    assert len(per_task["t1"]) == 2
    assert len(per_task["t2"]) == 1


def test_terminal_source_must_be_p3():
    with pytest.raises(TerminalSourceViolation, match="p3_task"):
        check_terminal_source("p5_gateway_synth")


def test_terminal_source_from_p3_ok():
    check_terminal_source("p3_task")


def test_both_time_fields_check():
    assert has_both_time_fields({"timestamp": 0.0, "mono": 0}) is True
    assert has_both_time_fields({"timestamp": 0.0}) is False
    assert has_both_time_fields({"mono": 0}) is False


def test_outbound_keys_diff_helper():
    a, b = outbound_keys_bidirectional_diff(
        expected=("cmd/task/ack", "cmd/task/progress"),
        actual=("cmd/task/ack", "cmd/task/result"))
    assert a == ("cmd/task/progress",)
    assert b == ("cmd/task/result",)


# ---------------- CHK-1-44 error map ----------------

def test_qt_code_map_has_ok_row():
    assert "OK" in QT_CODE_MAP
    assert QT_CODE_MAP["OK"][0] == 0


def test_bidirectional_diff_empty_on_current_repo():
    """The map must currently cover every v6 code."""
    assert_bidirectional_diff_empty()


def test_translate_pass_through_no_detail_ok_for_simple_codes():
    r = translate(E_SCHEMA)
    # qt_int derived deterministically from sorted-name ordering;
    # test that translate returns the mapped value AND flags
    # v6_code straight through (no detail requirement for E_SCHEMA).
    assert r["qt_int"] == QT_CODE_MAP[E_SCHEMA][0]
    assert r["v6_code"] == E_SCHEMA


def test_translate_needs_detail_for_one_to_many():
    """CHK-1-44 (ii): E_BUSY/E_CAPABILITY/E_DEGRADED/E_UNHEALTHY
    all need detail so Qt can distinguish reasons."""
    with pytest.raises(NeedsDetailMissing, match="one-to-many"):
        translate(E_BUSY)
    with pytest.raises(NeedsDetailMissing):
        translate(E_CAPABILITY)


def test_translate_with_detail_ok():
    r = translate(E_BUSY, detail={"reason": "rotation_blocked"})
    assert r["qt_int"] == QT_CODE_MAP[E_BUSY][0]
    assert r["v6_code"] == E_BUSY
    # E_BUSY is one-to-many; needs_detail flag must be True in the map.
    assert QT_CODE_MAP[E_BUSY][1] is True


def test_translate_unknown_code_raises():
    with pytest.raises(ErrorCodeMapDivergence, match="not in QT_CODE_MAP"):
        translate("E_MADE_UP")


def test_r10_2_rotation_text_not_generic_busy():
    """CHK-1-44 (iii): rotation-blocked text MUST NOT be 'robot busy'."""
    txt = outbound_text_for_reason(E_BUSY, "rotation_blocked")
    assert "rotation blocked" in txt
    assert "robot busy" not in txt


def test_r10_2_rotation_capability_distinct_from_busy():
    """CHK-1-44 (iii) merged variant: E_BUSY(rotation_blocked) and
    E_CAPABILITY(rotation_clearance) must NOT collapse."""
    a = outbound_text_for_reason(E_BUSY, "rotation_blocked")
    b = outbound_text_for_reason(E_CAPABILITY, "rotation_clearance")
    assert a != b


def test_v6_code_never_shown_to_qt():
    """R11.3: v6_code is internal-log only."""
    with pytest.raises(QtVisibilityViolation, match="v6_code"):
        assert_v6_code_not_in_qt_display({"qt_int": 5, "v6_code": "E_BUSY"})


def test_v6_code_stripped_frame_ok():
    assert_v6_code_not_in_qt_display({"qt_int": 5, "detail": {}})
