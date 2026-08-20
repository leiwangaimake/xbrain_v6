"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_d.py
Brief: BIZ-P3-11/12/13/14 route push + suspend/resume + remap tests

Description:
Batch D: chunking is exact and preserves order; ack codes outside
the RA-1/2/3 closed set raise (never silently degrade); resume
policy is a 3-value closed set; direction flip on resume is
refused; arc-length remap correctly matches points and rejects
divergence past tol; GC-1..7 map correctly, especially the
'fence delete when nothing references it still triggers V-4' rule.
"""

import math

import pytest

from xbrain.p3_task.route.push import (
    AckContractViolation, CHUNK_SIZE, RouteAck, RoutePushTrigger,
    build_route_push, chunk_waypoints, classify_ack,
)
from xbrain.p3_task.route.remap import (
    RemapTooFar, cumulative_arc_lengths, remap,
)
from xbrain.p3_task.route.suspend_resume import (
    DirectionMismatch, InvalidResumePolicy,
    build_resume_snapshot, check_direction_consistency,
    resume_start_index,
)


pytestmark = pytest.mark.no_device


# --- BIZ-P3-11 route push ---

def test_chunking_below_size_single_chunk():
    parts = chunk_waypoints(list(range(10)))
    assert len(parts) == 1 and parts[0] == tuple(range(10))


def test_chunking_at_boundary():
    wp = list(range(CHUNK_SIZE * 2 + 3))
    parts = chunk_waypoints(wp)
    assert len(parts) == 3
    assert len(parts[0]) == CHUNK_SIZE
    assert len(parts[2]) == 3


def test_chunking_empty_yields_zero_chunks():
    """No waypoints -> caller should not push."""
    assert chunk_waypoints([]) == ()


def test_build_route_push_sets_chunk_ix_and_total():
    wp = [(x, 0.0, 0.0) for x in range(CHUNK_SIZE + 5)]
    parts = build_route_push("t1", 3, wp, RoutePushTrigger.RP1_DISPATCH.value)
    assert len(parts) == 2
    assert parts[0].chunk_ix == 0 and parts[0].total_chunks == 2
    assert parts[1].chunk_ix == 1 and parts[1].total_chunks == 2


def test_classify_ack_ra1():
    assert classify_ack("RA-1") == RouteAck.RA1_ACCEPTED


def test_classify_ack_unknown_raises():
    """Any ack code outside RA-1/2/3 -> abort the task (CLAUDE.md 3.5)."""
    with pytest.raises(AckContractViolation, match="unknown ack code"):
        classify_ack("RA-99")


# --- BIZ-P3-12 suspend/resume ---

def test_invalid_resume_policy_rejected():
    with pytest.raises(InvalidResumePolicy):
        build_resume_snapshot("t1", 3, 0.5, "forward", "yolo")


def test_invalid_direction_rejected():
    with pytest.raises(DirectionMismatch):
        build_resume_snapshot("t1", 3, 0.5, "sideways", "exact")


def test_progress_out_of_unit_range_rejected():
    with pytest.raises(ValueError):
        build_resume_snapshot("t1", 3, 1.5, "forward", "exact")


def test_direction_flip_on_resume_refused():
    with pytest.raises(DirectionMismatch):
        check_direction_consistency("forward", "reverse")


def test_resume_start_index_exact():
    s = build_resume_snapshot("t1", 5, 0.3, "forward", "exact")
    assert resume_start_index(s) == 5


def test_resume_start_index_nearest_wp_rounds_up():
    """within >= 0.5 rounds to next waypoint."""
    s = build_resume_snapshot("t1", 5, 0.7, "forward", "nearest_wp")
    assert resume_start_index(s) == 6


def test_resume_start_index_nearest_wp_rounds_down():
    s = build_resume_snapshot("t1", 5, 0.4, "forward", "nearest_wp")
    assert resume_start_index(s) == 5


def test_resume_start_index_restart_returns_zero():
    s = build_resume_snapshot("t1", 5, 0.9, "forward", "restart")
    assert resume_start_index(s) == 0


# --- BIZ-P3-13 remap ---

def test_arc_length_two_point_line():
    s = cumulative_arc_lengths([(0.0, 0.0), (3.0, 4.0)])
    assert s == [0.0, 5.0]


def test_arc_length_three_points():
    s = cumulative_arc_lengths([(0.0, 0.0), (0.0, 3.0), (4.0, 3.0)])
    assert s == [0.0, 3.0, 7.0]


def test_remap_identical_route_stays_put():
    """Same route in/out: remap should land at the same ix + within."""
    wp = [(x, 0.0) for x in range(0, 20, 2)]     # 10 wps, seg=2m each
    r = remap(wp, wp, wp_ix=3, within_segment=0.5, remap_tol_m=0.1)
    # arc len at wp_ix=3 = 6m + 0.5*2m = 7m; s_new[3]=6 s_new[4]=8;
    # j=4, new_ix=3, new_within = (7-6)/2 = 0.5.
    assert r.new_wp_ix == 3
    assert math.isclose(r.new_within_segment, 0.5, abs_tol=1e-6)


def test_remap_divergent_route_raises():
    old_wp = [(x, 0.0) for x in range(0, 20, 2)]
    new_wp = [(x, 0.0) for x in range(0, 40, 2)]   # longer sim same start
    # Should still succeed for a mid-route match (arc lengths line up).
    r = remap(old_wp, new_wp, wp_ix=3, within_segment=0.5, remap_tol_m=0.1)
    assert r.arc_length_delta_m < 0.1

    # Now shorten new route so that s_resume=7m exceeds new arc length.
    tiny_new = [(0.0, 0.0), (1.0, 0.0)]     # total 1m
    with pytest.raises(RemapTooFar):
        remap(old_wp, tiny_new, wp_ix=3, within_segment=0.5, remap_tol_m=0.1)


# --- BIZ-P3-14 geo linkage ---
#
# The GC-1..GC-7 cases that lived here were REMOVED on 2026-08-20, not moved:
# they asserted the previous classification, which disagreed with 15 S7.6 in the
# dangerous direction (route deleted while running -> "abort_immediately", where
# S7.6 GC-3 says in bold do NOT interrupt the motion). Keeping them would have
# meant a green suite pinning a robot that stops in the middle of a camp roadway
# when somebody tidies the map.
#
# The replacement is tests/p3_task/test_geo_delete.py, which drives the table
# with the task STATE as an input -- every row of 15 S7.6 answers differently
# for running / suspended / queued, which the old two-argument classify() could
# not express at all.
