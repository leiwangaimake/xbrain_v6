"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u201_route_recording_escape.py
Brief: 99 U201 -- a route recording's keyboard cannot slide out of the activity area, and stops at the line

Description:
Field 2026-11-14 (7号路径): recording a route, the cloud keyboard hit the
activity area's recorded (jagged) south-west edge at ~2.1 m/s; the route
stop kept the clip's sliding component, so the robot slid along the edge
OUTSIDE it and drifted 4.3 m out of the area, and the route was saved
there. Earlier the same day the keyboard ran 0.8 m into a forbid zone: the
0.45 s prediction was shorter than the stop at 1.6 m/s.
Pinned here, on the real NavTick:
  * past the effective line only the component into the area drives;
  * at a corner only a direction into both edges drives;
  * the line within the stopping distance of the MEASURED speed stops the
    keyboard before it crosses; far from it full speed (no band, U186);
  * the contact flag follows what really stops the keyboard;
  * a jagged edge driven at diagonally cannot be left by more than the
    inset (simulation).
"""
from __future__ import annotations  # postponed annotations

import math  # simulation
from dataclasses import replace  # frozen NavInputs

import pytest  # marker, approx

from tests.p1_motion.nav.test_nav_tick_fence import FC, _stack  # the real tick and its constants
from tests.p1_motion.test_u186_fence_contact import _kbd  # the cloud keyboard input
from xbrain.p1_motion.fence.clip import CompiledFence, CompiledPolygon, boundary_hit, d_stop_m  # geometry
from xbrain.p1_motion.nav.nav_tick import _TrackSpeed  # under test

pytestmark = pytest.mark.no_device  # pure logic

#: the square of the U186 cases: east edge x = 8, effective 7.7 on rtk_fixed


def _drive(tick, now, pose, cmd, recording="route"):  # one tick with a (vx, vy) command
    inp = _kbd(now, cmd[0], pose, recording)  # forward command
    return tick.run(replace(inp, teleop_cmd=(cmd[0], cmd[1], 0.0)))  # with strafe


def test_past_the_line_only_inward_motion():  # field: slid along the edge outside it
    """MUTATION: keep the sliding component past the line -> vy 1.0 -> red."""
    tick = _stack()  # real tick with fence constants
    slide = _drive(tick, 5000, (8.1, 0.0), (0.0, 1.0))  # past the nominal x = 8, strafing along it
    assert slide.vy == pytest.approx(0.0) and slide.vx == pytest.approx(0.0)  # nothing along it
    assert slide.limiter == "fence"  # and said why
    back = _drive(tick, 5050, (8.1, 0.0), (-1.0, 0.0))  # backing into the area
    assert back.vx == pytest.approx(-1.0)  # the way back stays open
    out = _drive(tick, 5100, (8.1, 0.0), (1.0, 1.0))  # out and along
    assert out.vx == pytest.approx(0.0) and out.vy == pytest.approx(0.0)  # neither part kept


def test_a_corner_keeps_only_a_direction_into_both_edges():
    tick = _stack()  # real tick
    out = _drive(tick, 5000, (8.1, 8.1), (1.0, -1.0))  # out of the east edge, into the north one
    assert (out.vx, out.vy) == (pytest.approx(0.0), pytest.approx(0.0))  # not into both: stop
    inn = _drive(tick, 5050, (8.1, 8.1), (-1.0, -1.0))  # into both
    assert inn.vx < -0.5 and inn.vy < -0.5  # the way back stays open


def test_inside_the_nominal_line_the_keyboard_may_slide_along_it():  # 99 U202
    """Field 2026-11-14: stopped 0.27 m inside the activity edge (in the
    0.3 m inset), parallel to it, the robot could go neither forward nor
    back for 3 min. MUTATION: inward-only in the band too -> vy 0 -> red."""
    tick = _stack()  # real tick
    along = _drive(tick, 5000, (7.9, 0.0), (0.0, 1.0))  # 0.1 m inside x = 8, strafing along
    assert along.vy == pytest.approx(1.0) and along.vx == pytest.approx(0.0)  # slides
    out = _drive(tick, 5050, (7.9, 0.0), (1.0, 1.0))  # out and along
    assert out.vx == pytest.approx(0.0) and out.vy == pytest.approx(1.0)  # only the outward part goes
    assert any(p.hard_now for p in out.fence.per_poly)  # still the contact


def test_the_line_within_the_stopping_distance_stops_before_crossing():  # field: 0.8 m past
    """MUTATION: judge with evaluate()'s 0.45 s guess -> 1.6 m/s at x = 6.0 drives on -> red."""
    tick = _stack()  # real tick
    v = 1.6  # measured, from the pose track
    assert 6.0 + d_stop_m(v, FC) > 7.7 > 5.0 + d_stop_m(v, FC)  # the two positions below straddle it
    _drive(tick, 4750, (6.0 - v * 0.25, 0.0), (1.0, 0.0))  # 0.25 s earlier, same track
    near = _drive(tick, 5000, (6.0, 0.0), (1.0, 0.0))  # 1.7 m from the line at 1.6 m/s
    assert near.vx == pytest.approx(0.0) and near.limiter == "fence"  # brakes now
    assert any(p.hard_now for p in near.fence.per_poly)  # so "已停止前进" is true

    far_tick = _stack()  # a fresh track
    _drive(far_tick, 4750, (5.0 - v * 0.25, 0.0), (1.0, 0.0))
    far = _drive(far_tick, 5000, (5.0, 0.0), (1.0, 0.0))  # 2.7 m away
    assert far.vx == pytest.approx(1.0) and far.limiter == "none"  # full speed, no band
    assert not any(p.hard_now for p in far.fence.per_poly)  # and no contact


def test_a_fence_recording_is_still_not_stopped():  # U186 unchanged
    tick = _stack()  # real tick
    rec = _drive(tick, 5000, (7.9, 0.0), (1.0, 0.0), recording="fence")  # past the line
    assert rec.vx == pytest.approx(1.0)  # only told, never stopped


#: a jagged east edge like a recorded activity area: x alternates 7.6 / 8.4
JAG = CompiledFence("fs", 1, (CompiledPolygon(
    "p-jag", "allow", "jagged", True, True,
    ((-5.0, -20.0), (8.0, -20.0)) + tuple((7.6 if i % 2 else 8.4, -18.0 + i * 1.5) for i in range(25))
    + ((8.0, 20.0), (-5.0, 20.0)), None),))


def test_a_jagged_edge_driven_at_diagonally_is_not_left():  # the 7号路径 drift
    """Two runs at 2 m/s for 15 s, diagonally out and along the jagged edge:
    from 1 m inside the robot never gets past the nominal edge (it brakes
    before it); from 0.2 m outside -- where an overshoot leaves it, as in
    the field -- it gets no farther out and makes no progress along it.
    MUTATION: keep the sliding part past the line -> the second run slides
    along the edge -> red."""
    def run(x, y):  # (worst outside distance, metres moved along y)
        tick, worst, y0 = _stack(), -1e9, y  # real tick
        for k in range(300):  # 20 Hz
            inp = replace(_kbd(5000 + 50 * k, 2.0, (x, y), "route"), fence=JAG,
                          teleop_cmd=(2.0, 2.0, 0.0))  # diagonal: out and along
            out = tick.run(inp)
            x, y = x + out.vx * 0.05, y + out.vy * 0.05  # yaw 0: body == ENU
            worst = max(worst, -boundary_hit(x, y, JAG.polygons[0]).d_nom_m)  # how far outside
        return worst, y - y0
    inside_worst, _ = run(6.5, -15.0)  # from 1 m inside
    assert inside_worst <= 0.0  # braked before the nominal edge
    start_out = -boundary_hit(8.6, -15.0, JAG.polygons[0]).d_nom_m  # 0.2 m outside
    worst, along = run(8.6, -15.0)  # from past the line
    assert worst <= start_out + 1e-6 and along < 0.05  # no farther out, no sliding


def test_the_track_speed():
    ts = _TrackSpeed()  # under test
    assert ts.update(0, (0.0, 0.0)) == 0.0  # one sample: no speed yet
    assert ts.update(100, (0.2, 0.0)) == pytest.approx(2.0)  # 0.2 m in 0.1 s
    assert ts.update(300, (0.4, 0.0)) == pytest.approx(1.0)  # the t = 0 sample left the window
    assert ts.update(350, None) == 0.0  # no position: unknown, track cleared
    assert ts.update(400, (9.0, 9.0)) == 0.0  # and nothing carried across the gap
