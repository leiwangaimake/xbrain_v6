"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_progress.py
Brief: path_progress body (11 S3.5B) -- state / fail_reason coupling and indices

Description:
p3 distinguishes "navigation failed" from "aborted by estop" ONLY through
state == failed + fail_reason (11 S3.5B v2.0), so both directions of that
coupling are asserted. waypoint_index / seg_done_m / dist_done_m follow the
projection; arrived reports the last index and the route length; a goto has
no passed vertex while running.
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.nav.progress import ProgressError, build_path_progress
from xbrain.p1_motion.nav.route_intake import RouteSet
from xbrain.p1_motion.rns.route import Projection

pytestmark = pytest.mark.no_device

ROUTE = RouteSet(cmd_id="rg", route_id="r-1", route_rev=3, loop_mode="oneway",
                 total_len_m=20.0, points_xy=((0.0, 0.0), (10.0, 0.0), (10.0, 10.0)),
                 arrive_radius_m=(1.0, 1.0, 0.8))


def _b(**kw):
    args = dict(route=ROUTE, state="running", fail_reason=None, proj=None,
                now_mono_s=100.0, ts_wall_s=1.0)
    args.update(kw)
    return build_path_progress(**args)


def test_failed_requires_reason():
    """mutant: allow failed without fail_reason -> red."""
    with pytest.raises(ProgressError):
        _b(state="failed")
    assert _b(state="failed", fail_reason="watchdog_no_progress")["fail_reason"] == "watchdog_no_progress"


def test_reason_only_with_failed():
    with pytest.raises(ProgressError):
        _b(state="running", fail_reason="max_deviation")


def test_state_off_set_refused():
    with pytest.raises(ProgressError):
        _b(state="paused")


def test_indices_follow_the_projection():
    p = Projection(seg_index=1, t=0.25, foot=(10.0, 2.5), deviation_m=0.0, s_arc_m=12.5)
    b = _b(proj=p)
    assert (b["waypoint_index"], b["waypoint_total"]) == (1, 3)
    assert b["seg_done_m"] == pytest.approx(2.5) and b["dist_done_m"] == pytest.approx(12.5)
    assert b["route_id"] == "r-1" and b["route_rev"] == 3 and b["loading"] is False
    assert (b["loop_index"], b["loop_total"], b["dir_sign"]) == (0, 1, 1)


def test_arrived_reports_last_index_and_length():
    b = _b(state="arrived")
    assert b["waypoint_index"] == 2 and b["dist_done_m"] == pytest.approx(20.0)


def test_goto_has_no_passed_vertex_until_arrival():
    r = RouteSet(cmd_id="rg", route_id="r-2", route_rev=1, loop_mode="oneway",
                 total_len_m=0.0, points_xy=((5.0, 5.0),), arrive_radius_m=(1.0,))
    p = Projection(seg_index=0, t=0.5, foot=(2.0, 2.0), deviation_m=0.0, s_arc_m=3.0)
    assert _b(route=r, proj=p)["waypoint_index"] == -1
    assert _b(route=r, state="arrived")["waypoint_index"] == 0


def test_idle_without_route():
    b = _b(route=None, state="idle")
    assert b["route_id"] is None and b["waypoint_total"] == 0 and b["waypoint_index"] == -1
    assert _b(route=None, state="route_loading")["loading"] is True
