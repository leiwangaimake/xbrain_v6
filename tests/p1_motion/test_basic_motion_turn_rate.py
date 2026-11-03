"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_basic_motion_turn_rate.py
Brief: The basic-motion executor turns at the relmove's max_yaw_rate_radps (P2's turn speed) and refuses a turn whose rate is unusable
Description:
99 U104 (2026-10-30): P2 fills MO-2 max_yaw_rate_radps with the operator's
turn speed (default max 0.6, voice-adjustable). Before this the executor
always turned at XBRAIN_BASIC_MOTION_ANG = 0.3, on the chassis' twist
threshold, and many turns ended as stall. These cases drive the executor on
a simulated obedient chassis:
  * a turn with max_yaw_rate_radps runs at that rate and still ends done;
  * a missing / non-numeric / bool / non-positive / NaN rate is REFUSED
    (2026-11-03, 99 U106: the start script's fallback rate is gone -- every
    speed lives in p2_core.yaml and arrives in the relmove);
  * a straight move ignores the field and moves at max_speed_mps.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import math  # angles

import pytest  # fixtures, marks

from xbrain.p1_motion.runtime import basic_motion_test as bmt  # under test

pytestmark = pytest.mark.no_device  # pure logic against a simulated odometry


def _state(yaw=0.0):  # rt/chassis/state as the executor reads it
    return {"usage_mode": "navigation", "odom": {"valid": True, "x": 0.0, "y": 0.0, "yaw_rad": yaw}}


@pytest.fixture
def ex():
    """An executor built from a control deadzone below every rate used here."""
    return bmt.BasicMotionTest({"x": 0.08, "y": 0.08, "yaw": 0.25})


def _run_turn(ex, max_ticks=4000):  # obedient chassis; returns the wz commands and the final yaw
    yaw, t, out = 0.0, 0.0, []  # pose, time, commands
    for _ in range(max_ticks):  # bounded
        if not ex.active:  # ended
            break  # done
        _vx, _vy, wz = ex.step(t, _state(yaw), 0.0)  # one tick
        if ex.active:  # kept going
            out.append(wz)  # record
        yaw += wz * bmt.TICK_S  # the chassis does it
        t += bmt.TICK_S  # one period
    return out, yaw  # commands, end yaw


def test_turn_runs_at_the_relmove_rate(ex):
    """MUTATION: keep self.speeds[axis] for yaw -> peak 0.3 -> fails."""
    assert ex.start({"dyaw_rad": math.pi / 2, "max_yaw_rate_radps": 0.6}, _state(), 0.0) is None
    cmds, yaw = _run_turn(ex)
    assert ex.last_result == "done" and abs(yaw - math.pi / 2) < math.radians(3.0)  # arrived
    assert abs(max(cmds) - 0.6) < 1e-9  # turned at the operator's speed


@pytest.mark.parametrize("rate", [None, "0.6", True, 0.0, -0.5, float("nan"), float("inf")])
def test_unusable_rate_is_refused(ex, rate):
    """MUTATION: drop the isfinite / > 0 / bool checks -> NaN, inf or True is used -> fails."""
    body = {"dyaw_rad": -math.pi / 4}  # right 45 deg
    if rate is not None:  # the field present
        body["max_yaw_rate_radps"] = rate
    assert ex.start(body, _state(), 0.0) == "speed_missing(max_yaw_rate_radps)"  # refused
    assert not ex.active  # nothing turns


def test_straight_move_ignores_the_turn_rate(ex):
    """The field is a yaw rate; a forward metre stays at the linear speed."""
    assert ex.start({"dx_m": 0.5, "max_yaw_rate_radps": 0.6, "max_speed_mps": 0.2}, _state(), 0.0) is None
    assert ex._g.speed == 0.2  # the move speed (99 U106), not the turn rate
