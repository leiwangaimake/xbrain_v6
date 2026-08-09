"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_f.py
Brief: BIZ-P3-18/19/27/28 fence geom + progress + geo rev + mission JSON tests

Description:
Batch F: painter algorithm (union of shapes; point-in-polygon and
point-in-circle both work; FS-4 rejects degenerate polygons at
commit), progress event kinds (closed set), content_hash idempotent
regardless of key order, SN-5 push order validator catches rev
inversion, mission_json ST-1..3 + SS-1..3 invariants (every field
enforced with a paired negative case per CLAUDE.md 3.3).
"""

import pytest

from xbrain.p3_task.fence.geom import (
    Circle, InvalidPolygon, Polygon,
    point_in_circle, point_in_composite, point_in_polygon,
    polygon_area, validate_polygon,
)
from xbrain.p3_task.state.geo_rev import (
    PushOrderViolation, content_hash, is_same_content,
    push_order_key, tombstone_delete_row, validate_push_batch,
)
from xbrain.p3_task.state.mission_json import (
    MissionJsonInvariantViolation, assert_current_in_range,
    assert_monotone, assert_ss3_terminal, parse_step_status,
)
from xbrain.p3_task.state.progress import (
    HeartbeatState, UnknownProgressKind, VALID_PROGRESS_KINDS,
    build_route_event, build_state_event, build_step_event,
    build_waypoint_event, heartbeat_snapshot,
)


pytestmark = pytest.mark.no_device


# --- BIZ-P3-18 fence geometry ---

def test_polygon_area_triangle():
    pts = [(0.0, 0.0), (2.0, 0.0), (0.0, 2.0)]
    assert polygon_area(pts) == 2.0


def test_polygon_validate_rejects_too_few_points():
    with pytest.raises(InvalidPolygon, match=">= 3"):
        validate_polygon([(0.0, 0.0), (1.0, 0.0)])


def test_polygon_validate_rejects_degenerate():
    """Three collinear points -> area 0 -> reject."""
    with pytest.raises(InvalidPolygon, match="degenerate"):
        validate_polygon([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])


def test_polygon_validate_accepts_triangle():
    validate_polygon([(0.0, 0.0), (2.0, 0.0), (0.0, 2.0)])


def test_point_in_polygon_inside():
    poly = Polygon(((0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)))
    assert point_in_polygon(2.0, 2.0, poly) is True


def test_point_in_polygon_outside():
    poly = Polygon(((0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)))
    assert point_in_polygon(5.0, 5.0, poly) is False


def test_point_in_circle():
    c = Circle(cx=0.0, cy=0.0, r=1.0)
    assert point_in_circle(0.5, 0.5, c) is True
    assert point_in_circle(2.0, 0.0, c) is False


def test_point_in_composite_union():
    """FS-1: circle + polygon; a point in EITHER counts as inside."""
    shapes = [Circle(0.0, 0.0, 1.0),
               Polygon(((10.0, 10.0), (12.0, 10.0), (12.0, 12.0),
                          (10.0, 12.0)))]
    assert point_in_composite(0.5, 0.5, shapes) is True   # in circle
    assert point_in_composite(11.0, 11.0, shapes) is True # in poly
    assert point_in_composite(5.0, 5.0, shapes) is False


def test_point_in_composite_unknown_type_raises():
    with pytest.raises(TypeError, match="unknown shape"):
        point_in_composite(0.0, 0.0, [object()])


# --- BIZ-P3-19 progress ---

def test_progress_step_event():
    e = build_step_event("t1", 3, 10)
    assert e.kind == "PP-1" and "3/10" in e.detail


def test_progress_state_event_unknown_kind_raises():
    with pytest.raises(UnknownProgressKind):
        build_state_event("t1", "coffee", "reason")


def test_progress_state_event_valid_kinds():
    for k in ("suspend", "resume", "abort"):
        e = build_state_event("t1", k, "reason")
        assert e.kind == "PP-2"


def test_progress_route_event_accepted():
    e = build_route_event("t1", True, 5)
    assert "accepted" in e.detail


def test_progress_waypoint_event():
    e = build_waypoint_event("t1", 7)
    assert e.kind == "PP-3b" and "waypoint_ix=7" in e.detail


def test_heartbeat_excludes_terminal():
    tasks = [
        HeartbeatState("t1", "running", 3, 10),
        HeartbeatState("t2", "completed", 10, 10),   # terminal - filter
        HeartbeatState("t3", "suspended", 5, 10),
    ]
    snap = heartbeat_snapshot(tasks)
    assert {t.task_id for t in snap} == {"t1", "t3"}


def test_progress_kinds_closed_set():
    """Elements of VALID_PROGRESS_KINDS are exactly the four
    expected values (CLAUDE.md 3.5)."""
    assert VALID_PROGRESS_KINDS == frozenset(
        {"PP-1", "PP-2", "PP-3", "PP-3b"})


# --- BIZ-P3-27 geo rev ---

def test_content_hash_stable_across_key_order():
    a = content_hash({"a": 1, "b": 2})
    b = content_hash({"b": 2, "a": 1})
    assert a == b and len(a) == 64


def test_content_hash_differs_on_value_change():
    assert content_hash({"a": 1}) != content_hash({"a": 2})


def test_is_same_content_helper():
    assert is_same_content("x", "x") is True
    assert is_same_content("x", "y") is False


def test_tombstone_bumps_rev_and_flags():
    row = tombstone_delete_row(5)
    assert row == {"rev": 6, "tombstone": 1}


def test_push_order_key_composition():
    assert push_order_key(3, 7) == (3, 7)


def test_validate_push_batch_ok():
    entries = [
        (1, "wp", "w1", 1),
        (2, "wp", "w1", 2),
        (3, "wp", "w2", 1),
    ]
    validate_push_batch(entries)     # no raise


def test_validate_push_batch_detects_rev_inversion():
    """SN-5: within a single object, higher push_id must NOT have
    lower rev."""
    entries = [
        (1, "wp", "w1", 5),
        (2, "wp", "w1", 3),        # rev went backwards
    ]
    with pytest.raises(PushOrderViolation, match="rev"):
        validate_push_batch(entries)


# --- BIZ-P3-28 mission_json ---

def test_current_step_in_range_ok():
    assert_current_in_range(3, 10)


def test_current_step_past_total_rejected():
    """ST-1: current_step > total_steps -> violation."""
    with pytest.raises(MissionJsonInvariantViolation, match="not in"):
        assert_current_in_range(11, 10)


def test_current_step_negative_rejected():
    with pytest.raises(MissionJsonInvariantViolation):
        assert_current_in_range(-1, 10)


def test_monotone_forward_ok():
    assert_monotone(3, 4, state="running")


def test_monotone_rewind_rejected_in_running():
    with pytest.raises(MissionJsonInvariantViolation, match="rewind"):
        assert_monotone(5, 3, state="running")


def test_monotone_rewind_allowed_in_resuming():
    """ST-2: resume with restart may reset current_step to 0."""
    assert_monotone(5, 0, state="resuming")


def test_step_status_bad_json_rejected():
    with pytest.raises(MissionJsonInvariantViolation, match="step_status_json"):
        parse_step_status("{not_json", total=3)


def test_step_status_wrong_length_rejected():
    """SS-2: length must equal total_steps."""
    with pytest.raises(MissionJsonInvariantViolation, match="total_steps"):
        parse_step_status('["ok", "ok"]', total=3)


def test_step_status_unknown_value_rejected():
    """Values are a 4-item closed set."""
    with pytest.raises(MissionJsonInvariantViolation, match=r"step\["):
        parse_step_status('["ok", "halfway", "ok"]', total=3)


def test_step_status_valid_shape():
    got = parse_step_status('["ok", "skipped", "failed"]', total=3)
    assert got == ["ok", "skipped", "failed"]


def test_ss3_completed_rejects_failed_step():
    """SS-3: completed task cannot have failed steps."""
    with pytest.raises(MissionJsonInvariantViolation):
        assert_ss3_terminal(["ok", "failed"], state="completed")


def test_ss3_completed_ok_and_skipped_ok():
    assert_ss3_terminal(["ok", "skipped", "ok"], state="completed")


def test_ss3_failed_without_failed_step_rejected():
    """SS-3: 'failed' state requires at least one 'failed' step."""
    with pytest.raises(MissionJsonInvariantViolation, match="no failed"):
        assert_ss3_terminal(["ok", "ok"], state="failed")
