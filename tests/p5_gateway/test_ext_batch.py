"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ext_batch.py
Brief: CHK-0-38/39/40 甲方云端对接层 tests (translate + dedupe + estop)

Description:
Three fatal items covered with positive + adversarial variants each:

CHK-0-38 translate:
  * envelope shape (missing fields, extras, v!=1)
  * rid regex + rid vs key second-segment match
  * ts is JSON number (float64), NEVER ISO string
  * seq uint64 shape
  * task_type in ALLOWED, LEGACY names rejected explicitly
  * GOTO_KEYPOINT coordinate_system=='WGS84', arrival_radius in
    [0.5, 10.0], waypoints ID regex, route ID regex
  * STOP_TASK.action closed set
  * SET_ALARM_CONFIG.alarm_level in {1,2}, siren_level [0,100],
    duration_sec [1,20], cooldown_sec [0.5, 600.0]
  * regions[] kind='keep_in' HARD REJECT with E_GEO_INVALID
  * successful translate returns InboundTask with internal id
    prefixed 'ext:' (R10.4 分域)

CHK-0-39 dedupe:
  * (rid, msg_id) in window -> Duplicate
  * seq rewind within same session -> is_rewind
  * seq at watermark -> accepted (edge)
  * stale by monotonic age > threshold -> is_stale
  * future ts (age < 0) -> is_stale
  * session_reset clears watermark
  * dedupe window sweep evicts old entries

CHK-0-40 estop:
  * envelope validate: rid mismatch, action closed set
  * ack build: latency_ms from mono diff, hes closed set
  * negative latency -> raise (defensive)
  * forward budget 100 ms + e2e 300 ms boundary tests
  * EstopPathHealth: 3-miss debounce -> down; ack does NOT clear
    down mark; beat received clears it
"""

import pytest

from xbrain.common.errors import E_CONFIG_INVALID, E_GEO_INVALID, E_SCHEMA
from xbrain.p5_gateway.ext.dedupe import (
    DEFAULT_DEDUP_WINDOW_MS, InboundDedupe,
)
from xbrain.p5_gateway.ext.estop import (
    ESTOP_MAX_E2E_MS, ESTOP_MAX_FORWARD_MS,
    EstopFrame, EstopPathHealth, EstopSchemaError,
    build_ack, check_e2e_budget, check_forward_budget,
    validate_and_forward,
)
from xbrain.p5_gateway.ext.translate import (
    ALLOWED_TASK_TYPES, InboundTask, LEGACY_TASK_TYPES,
    TranslateFailure, translate,
)


pytestmark = pytest.mark.no_device


# ---------- helpers ----------

def _good_envelope(**overrides):
    base = {
        "v": 1, "rid": "robot01", "ts": 1785732000.123,
        "seq": 42, "src": "qt-console",
        "msg_id": "m-001", "task_id": "t-abc",
        "data": {
            "task_type": "GOTO_KEYPOINT",
            "task_id": "t-abc",
            "payload": {
                "coordinate_system": "WGS84",
                "arrival_radius_m": 1.5,
                "recorded_path_id": "r-morning_patrol",
                "waypoints": [{"id": "w-alpha"}, {"id": "w-beta"}],
            },
        },
    }
    base.update(overrides)
    return base


# ---------- CHK-0-38 envelope ----------

def test_translate_happy_path_returns_inbound_task():
    r = translate(_good_envelope(), key_second_segment="robot01")
    assert isinstance(r, InboundTask)
    assert r.internal_task_id == "ext:t-abc"     # R10.4 分域 prefix
    assert r.client_task_id == "t-abc"
    assert r.rid == "robot01"
    assert r.task_type == "GOTO_KEYPOINT"


def test_translate_rid_shape_bad():
    """RID must match ^[a-z0-9_-]{1,32}$."""
    msg = _good_envelope(rid="Robot 01")
    r = translate(msg, key_second_segment="Robot 01")
    assert isinstance(r, TranslateFailure)
    assert r.code == E_SCHEMA and r.detail["kind"] == "rid_shape"


def test_translate_rid_key_mismatch():
    """rid in envelope must equal key's second segment."""
    r = translate(_good_envelope(rid="robot01"),
                    key_second_segment="robot02")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "rid_key_mismatch"


def test_translate_envelope_missing_field():
    msg = _good_envelope()
    del msg["seq"]
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "envelope_missing"


def test_translate_envelope_extras_rejected():
    msg = _good_envelope(extra_field="oops")
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "envelope_extras"


def test_translate_v_mismatch():
    r = translate(_good_envelope(v=2), key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "envelope_v_mismatch"


def test_translate_ts_iso_string_rejected():
    """R11.2: ts must be JSON number, NOT ISO string."""
    r = translate(_good_envelope(ts="2026-08-09T12:00:00Z"),
                    key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "ts_not_number"


def test_translate_ts_integer_ms_accepted_as_number():
    """R11.2: an integer would also be accepted since 1785732000 is a
    valid JSON number; the receiver's dedupe layer is where age
    against monotonic clock is checked."""
    r = translate(_good_envelope(ts=1785732000),
                    key_second_segment="robot01")
    assert isinstance(r, InboundTask)


def test_translate_ts_infinite_rejected():
    r = translate(_good_envelope(ts=float("inf")),
                    key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "ts_not_finite"


def test_translate_seq_boolean_rejected():
    """Python bool is subclass of int; must be rejected explicitly."""
    r = translate(_good_envelope(seq=True),
                    key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "seq_not_int"


def test_translate_seq_negative_rejected():
    r = translate(_good_envelope(seq=-1),
                    key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "seq_out_of_range"


# ---------- task_type closed sets ----------

def test_translate_legacy_task_type_rejected_explicitly():
    """LEGACY_TASK_TYPES must produce a distinct 'legacy_task_type'
    reason so the operator sees WHY (not just 'unknown')."""
    msg = _good_envelope()
    msg["data"]["task_type"] = "INSPECTION_ROUTE"
    msg["data"]["payload"] = {}
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "legacy_task_type"


def test_translate_unknown_task_type():
    msg = _good_envelope()
    msg["data"]["task_type"] = "MAKE_COFFEE"
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "task_type_unknown"


def test_allowed_and_legacy_disjoint():
    assert ALLOWED_TASK_TYPES.isdisjoint(LEGACY_TASK_TYPES)


# ---------- GOTO_KEYPOINT payload ----------

def test_goto_wrong_coordinate_system_rejected():
    msg = _good_envelope()
    msg["data"]["payload"]["coordinate_system"] = "UTM"
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "coordinate_system_mismatch"


def test_goto_arrival_radius_out_of_range():
    for bad in (0.4, 10.1):
        msg = _good_envelope()
        msg["data"]["payload"]["arrival_radius_m"] = bad
        r = translate(msg, key_second_segment="robot01")
        assert isinstance(r, TranslateFailure)
        assert r.detail["kind"] == "arrival_radius_m_range"


def test_goto_arrival_radius_boundary_accepted():
    for edge in (0.5, 10.0):
        msg = _good_envelope()
        msg["data"]["payload"]["arrival_radius_m"] = edge
        r = translate(msg, key_second_segment="robot01")
        assert isinstance(r, InboundTask)


def test_goto_arrival_radius_missing_rejected():
    """The 'default not silently applied' rule (R3.1)."""
    msg = _good_envelope()
    msg["data"]["payload"].pop("arrival_radius_m")
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "arrival_radius_m_type"


def test_goto_waypoint_id_shape_bad():
    msg = _good_envelope()
    msg["data"]["payload"]["waypoints"] = [{"id": "wp_bad"}]
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "waypoint_id_shape"


def test_goto_route_id_shape_bad():
    msg = _good_envelope()
    msg["data"]["payload"]["recorded_path_id"] = "route_1"
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "route_id_shape"


def test_goto_waypoints_empty_rejected():
    msg = _good_envelope()
    msg["data"]["payload"]["waypoints"] = []
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "waypoints_empty_or_bad_type"


# ---------- STOP_TASK payload ----------

def test_stop_task_action_closed_set():
    msg = _good_envelope()
    msg["data"]["task_type"] = "STOP_TASK"
    msg["data"]["payload"] = {"target_task_id": "t1", "action": "abort"}
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "stop_action_closed_set"


def test_stop_task_target_required():
    msg = _good_envelope()
    msg["data"]["task_type"] = "STOP_TASK"
    msg["data"]["payload"] = {"action": "cancel"}
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "target_task_id_required"


def test_stop_task_happy_path():
    msg = _good_envelope()
    msg["data"]["task_type"] = "STOP_TASK"
    msg["data"]["payload"] = {"target_task_id": "t1", "action": "pause",
                                 "reason": "operator_stop"}
    r = translate(msg, key_second_segment="robot01")
    assert isinstance(r, InboundTask) and r.task_type == "STOP_TASK"


# ---------- SET_ALARM_CONFIG payload ----------

def _alarm_msg(**overrides):
    payload = {
        "alarm_level": 1, "siren_level": 50, "duration_sec": 10,
        "cooldown_sec": 30.0, "regions": [],
    }
    payload.update(overrides.pop("payload", {}))
    msg = _good_envelope()
    msg["data"]["task_type"] = "SET_ALARM_CONFIG"
    msg["data"]["payload"] = payload
    for k, v in overrides.items():
        msg[k] = v
    return msg


def test_alarm_level_closed_set():
    r = translate(_alarm_msg(payload={"alarm_level": 3}),
                    key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "alarm_level_closed_set"


def test_alarm_siren_out_of_range():
    r = translate(_alarm_msg(payload={"siren_level": 101}),
                    key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "siren_level_range"


def test_alarm_duration_boundary():
    for edge in (1, 20):
        r = translate(_alarm_msg(payload={"duration_sec": edge}),
                        key_second_segment="robot01")
        assert isinstance(r, InboundTask)
    for bad in (0.9, 20.1):
        r = translate(_alarm_msg(payload={"duration_sec": bad}),
                        key_second_segment="robot01")
        assert isinstance(r, TranslateFailure)


def test_alarm_cooldown_range():
    r = translate(_alarm_msg(payload={"cooldown_sec": 0.4}),
                    key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "cooldown_sec_range"


def test_alarm_keep_in_region_hard_reject():
    """R3.4: 'regions[] 只允许 alarm_region; 禁止 keep_in'.
    keep_in must return E_GEO_INVALID with distinct 'keep_in_forbidden'."""
    r = translate(_alarm_msg(payload={
        "regions": [{"kind": "keep_in", "id": "safe1"}]
    }), key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.code == E_GEO_INVALID
    assert r.detail["kind"] == "keep_in_forbidden"


def test_alarm_unknown_region_kind_rejected():
    r = translate(_alarm_msg(payload={
        "regions": [{"kind": "trap", "id": "t1"}]
    }), key_second_segment="robot01")
    assert isinstance(r, TranslateFailure)
    assert r.detail["kind"] == "region_kind_closed_set"


# ---------- CHK-0-39 dedupe ----------

def test_dedupe_fresh_accepted():
    d = InboundDedupe(stale_max_ms=5000)
    v = d.check("r1", "m1", seq=1, key="cmd/task",
                  inbound_mono_ms=100, now_mono_ms=100)
    assert v.accepted


def test_dedupe_duplicate_within_window():
    d = InboundDedupe(stale_max_ms=5000, window_ms=60_000)
    d.check("r1", "m1", 1, "cmd/task", 100, 100)
    v = d.check("r1", "m1", 2, "cmd/task", 200, 200)
    assert not v.accepted and v.is_duplicate


def test_dedupe_expires_after_window():
    """After window_ms, same (rid, msg_id) is treated as fresh again."""
    d = InboundDedupe(stale_max_ms=1_000_000, window_ms=60_000)
    d.check("r1", "m1", 1, "cmd/task", 0, 0)
    v = d.check("r1", "m1", 2, "cmd/task", 61_000, 61_000)
    assert v.accepted


def test_dedupe_stale_by_age():
    d = InboundDedupe(stale_max_ms=1000)
    v = d.check("r1", "m1", 1, "cmd/task",
                  inbound_mono_ms=0, now_mono_ms=2000)
    assert not v.accepted and v.is_stale


def test_dedupe_future_ts_treated_as_stale():
    """Negative age (inbound_mono_ms > now) means clock skew;
    conservatively refuse."""
    d = InboundDedupe(stale_max_ms=1000)
    v = d.check("r1", "m1", 1, "cmd/task",
                  inbound_mono_ms=2000, now_mono_ms=1000)
    assert not v.accepted and v.is_stale


def test_dedupe_seq_rewind_dropped():
    d = InboundDedupe(stale_max_ms=1_000_000)
    d.check("r1", "m1", seq=10, key="cmd/task",
              inbound_mono_ms=0, now_mono_ms=0)
    v = d.check("r1", "m2", seq=5, key="cmd/task",
                  inbound_mono_ms=100, now_mono_ms=100)
    assert not v.accepted and v.is_rewind


def test_dedupe_seq_equal_watermark_accepted():
    """Boundary: same seq as watermark counts as fresh (not rewind)."""
    d = InboundDedupe(stale_max_ms=1_000_000)
    d.check("r1", "m1", seq=10, key="cmd/task", inbound_mono_ms=0, now_mono_ms=0)
    v = d.check("r1", "m2", seq=10, key="cmd/task", inbound_mono_ms=1, now_mono_ms=1)
    assert v.accepted


def test_dedupe_session_reset_clears_watermark():
    d = InboundDedupe(stale_max_ms=1_000_000)
    d.check("r1", "m1", 100, "cmd/task", 0, 0)
    d.session_reset("r1", "cmd/task")
    v = d.check("r1", "m2", seq=1, key="cmd/task",
                  inbound_mono_ms=1, now_mono_ms=1)
    assert v.accepted


def test_dedupe_construct_rejects_zero_stale_max():
    with pytest.raises(ValueError, match="stale_max_ms"):
        InboundDedupe(stale_max_ms=0)


def test_dedupe_default_window_at_least_60s():
    """R1.8: 后端 rid+msg_id 统一去重窗口不少于 60 秒."""
    assert DEFAULT_DEDUP_WINDOW_MS >= 60_000


# ---------- CHK-0-40 estop ----------

def _good_estop(**overrides):
    base = {
        "v": 1, "rid": "robot01", "ts": 1.0, "seq": 1,
        "src": "qt-panel", "msg_id": "e-001",
        "data": {"action": "stop", "reason": "operator"},
    }
    base.update(overrides)
    return base


def test_estop_validate_happy_path():
    f = validate_and_forward(_good_estop(), key_second_segment="robot01")
    assert isinstance(f, EstopFrame)
    assert f.action == "stop" and f.reason == "operator"


def test_estop_rid_mismatch_raises():
    with pytest.raises(EstopSchemaError, match="rid mismatch"):
        validate_and_forward(_good_estop(rid="robot01"),
                              key_second_segment="robot02")


def test_estop_action_closed_set():
    m = _good_estop()
    m["data"]["action"] = "pause"
    with pytest.raises(EstopSchemaError, match="closed set"):
        validate_and_forward(m, key_second_segment="robot01")


def test_estop_ack_latency_positive():
    f = validate_and_forward(_good_estop(), key_second_segment="robot01")
    ack = build_ack(f, recv_mono_ms=1000, sent_mono_ms=1050,
                     estop_epoch=7, applied=True, hes="engaged",
                     timeout_lock=False)
    assert ack.latency_ms == 50


def test_estop_ack_negative_latency_raises():
    """Defensive: sent < recv shouldn't happen (monotonic clock);
    if it does, refuse rather than emit misleading latency."""
    f = validate_and_forward(_good_estop(), key_second_segment="robot01")
    with pytest.raises(EstopSchemaError, match="latency_ms negative"):
        build_ack(f, recv_mono_ms=1000, sent_mono_ms=999,
                    estop_epoch=1, applied=True, hes="engaged",
                    timeout_lock=False)


def test_estop_ack_hes_closed_set():
    f = validate_and_forward(_good_estop(), key_second_segment="robot01")
    with pytest.raises(EstopSchemaError, match="hes closed set"):
        build_ack(f, recv_mono_ms=0, sent_mono_ms=1, estop_epoch=1,
                    applied=True, hes="halfway", timeout_lock=False)


def test_forward_budget_100ms():
    assert check_forward_budget(100) is True
    assert check_forward_budget(101) is False
    assert ESTOP_MAX_FORWARD_MS == 100


def test_e2e_budget_300ms():
    assert check_e2e_budget(qt_click_mono_ms=0, ack_received_mono_ms=300) is True
    assert check_e2e_budget(qt_click_mono_ms=0, ack_received_mono_ms=301) is False
    assert ESTOP_MAX_E2E_MS == 300


def test_estop_path_health_starts_ok():
    h = EstopPathHealth()
    assert h.state == "ok" and h.consecutive_misses == 0


def test_estop_path_health_debounce_to_down():
    """R3.3: 3 consecutive missed beats -> 'down'."""
    h = EstopPathHealth()
    h.on_beat_missed()
    assert h.state == "degraded"
    h.on_beat_missed()
    assert h.state == "degraded"
    h.on_beat_missed()
    assert h.state == "down"


def test_estop_path_health_beat_restores_ok():
    h = EstopPathHealth()
    for _ in range(3):
        h.on_beat_missed()
    assert h.state == "down"
    h.on_beat_received()
    assert h.state == "ok" and h.consecutive_misses == 0


def test_estop_path_health_ack_does_not_clear_down():
    """R3.3: 'It cannot be replaced by a single ack.' A successful ack
    while beats are still missing must NOT flip the state back."""
    h = EstopPathHealth()
    for _ in range(3):
        h.on_beat_missed()
    assert h.state == "down"
    h.on_ack_received(latency_ms=50)
    assert h.state == "down"     # unchanged
