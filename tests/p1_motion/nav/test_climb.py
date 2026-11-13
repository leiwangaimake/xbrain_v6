"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_climb.py
Brief: 99 U151/U152 heading-deadlock climb -- the straight >= 1 m track pair, enter / hold / leave / time out, speed, pose view

Description:
The user's ruling of 2026-11-06: when both GNSS headings are unusable during
a route task (dual antenna not fixed, COG blind), climb on the heading of the
robot's last movement of at least 1 m; RNS drives at the navigation top speed
with its avoidance; a valid GNSS heading ends the climb. The outdoor case it
answers: a patrol stopped by one slow COG tick and stood 55 s until the
watchdog failed it. 99 U152 (the robot hit a wall at a corner): the track
bearing is paired with the yaw along THAT straight metre, not with the yaw at
climb entry, and a metre along which the body turned is skipped. Each case
names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import math  # angles

import pytest  # parametrize, approx

from xbrain.p1_motion.nav.nav_tick import quality_caps  # the speed table
from xbrain.p1_motion.path.climb import Climb, TrackHeading  # under test
from xbrain.p1_motion.path.heading_bridge import HeadingBridge  # 99 U152 stub field

pytestmark = pytest.mark.no_device  # pure logic


def _track_east(yaw=1.0):  # a robot that drove 3 m east, odom yaw constant (relative)
    t = TrackHeading(1.0, 20.0)  # 1 m segments, 20 deg straightness, as configured
    for x in (0.0, 0.4, 0.8, 1.2, 2.0, 2.4, 3.0):  # an eastward drive
        t.update((x, 0.0), yaw)  # one position + yaw per tick
    return t  # the track


# -- the track heading ---------------------------------------------------------------
def test_track_heading_needs_a_full_metre_and_keeps_the_last():
    """MUTATION: bearing on any step -> set after 0.4 m -> red; reset when
    standing -> None after the stop -> red."""
    t = TrackHeading(1.0, 20.0)  # as configured
    t.update((0.0, 0.0), 0.3); t.update((0.0, 0.6), 0.3)  # 0.6 m north: not yet
    assert t.heading is None
    t.update((0.0, 1.1), 0.3)  # 1.1 m north
    assert t.heading == pytest.approx(math.pi / 2)  # ENU: north = +pi/2
    assert t.heading_yaw == pytest.approx(0.3)  # paired with the yaw along it
    for _ in range(20):  # a long stand
        t.update((0.0, 1.15), 0.3)  # standing (RTK noise < 1 m)
    assert t.heading == pytest.approx(math.pi / 2)  # the last movement is kept
    t.update(None, 0.3)  # no fix: unchanged
    assert t.heading == pytest.approx(math.pi / 2)


def test_a_turning_segment_is_not_a_track_heading():
    """99 U152 (patrol t-20261106-007): a metre along which the body turned is
    a sideways arc, its bearing is not the nose. MUTATION: drop the
    straightness check -> the arc's bearing replaces the east pair -> red."""
    t = _track_east()  # east, yaw 1.0
    t.update((3.0, 1.1), 1.6)  # 1.1 m north while the body turned 0.6 rad (34 deg)
    assert t.heading == pytest.approx(0.0) and t.heading_yaw == pytest.approx(1.0)  # east pair kept
    t.update((3.0, 2.2), 1.65)  # next metre north, nearly straight (3 deg)
    assert t.heading == pytest.approx(math.pi / 2)  # a straight metre: taken
    assert t.heading_yaw == pytest.approx(1.625)  # the yaw at its middle


def test_no_yaw_no_pair():
    """MUTATION: pair a bearing with a missing yaw -> TypeError / a pair with
    nothing -> red."""
    t = TrackHeading(1.0, 20.0)  # as configured
    for y in (0.0, 0.5, 1.2):  # 1.2 m north, odometry absent
        t.update((0.0, y), None)
    assert t.heading is None and t.heading_yaw is None


# -- the climb -------------------------------------------------------------------------
def test_no_climb_with_a_heading_or_without_a_route():
    """MUTATION: climb whenever the heading is invalid -> climbs with no route
    task (the user scoped it to route tasks) -> red."""
    c, t = Climb(30.0), _track_east()  # a fresh climb over the eastward track
    assert not c.step(0.0, True, True, t, 1.0).climbing  # a heading is valid
    assert not c.step(0.1, False, False, t, 1.0).climbing  # no route running


def test_the_climb_starts_on_the_track_heading_and_follows_turns():
    """MUTATION: hold the heading constant -> a 0.5 rad RNS turn is not seen
    (the robot would spin for good) -> red; use the odom yaw directly -> the
    relative yaw (1.0 rad here) leaks into the heading -> red."""
    c, t = Climb(30.0), _track_east()  # a fresh climb over the eastward track
    h = c.step(0.0, False, True, t, 1.0)  # deadlock: odom yaw 1.0 (relative)
    assert h.climbing and h.heading_rad == pytest.approx(0.0)  # east, the last metre
    h = c.step(0.5, False, True, t, 1.5)  # RNS turned 0.5 rad left
    assert h.heading_rad == pytest.approx(0.5)  # seen through the odom change


def test_a_turn_made_before_the_climb_is_not_lost():
    """99 U152, the wall: the robot drove east, then turned 0.5 rad in place
    (no new metre), THEN lost every heading. MUTATION: pair the bearing with
    the yaw at climb entry (U151 as first written) -> east (0.0) while the
    nose points 0.5 -> red."""
    c, t = Climb(30.0), _track_east()  # east at yaw 1.0
    h = c.step(0.0, False, True, t, 1.5)  # entered after a 0.5 rad turn in place
    assert h.heading_rad == pytest.approx(0.5)


def test_a_new_straight_metre_corrects_the_climb():
    """MUTATION: keep the first pair for the whole climb -> the odom drift of
    the climb stays in the heading -> red."""
    c, t = Climb(30.0), _track_east()  # a fresh climb over the eastward track
    c.step(0.0, False, True, t, 1.0)  # climb east
    # from the open segment's start (2.4, 0): 1.27 m north-east, body yaw 1.0 -> 1.05
    t.update((3.3, 0.9), 1.05)
    h = c.step(1.0, False, True, t, 1.025)  # odom at the segment's middle yaw
    assert h.heading_rad == pytest.approx(math.pi / 4)  # positions win


def test_a_valid_heading_ends_the_climb():
    """MUTATION: keep climbing after a heading returns -> red."""
    c, t = Climb(30.0), _track_east()  # a fresh climb over the eastward track
    assert c.step(0.0, False, True, t, 1.0).climbing
    assert not c.step(0.5, True, True, t, 1.0).climbing  # heading back: normal


def test_the_climb_times_out_and_waits_for_a_heading():
    """MUTATION: no time bound -> climbs on a guess for ever -> red; re-enter
    right after the timeout -> red; never re-arm -> no climb after a valid
    heading came and went -> red."""
    c, t = Climb(30.0), _track_east()  # a fresh climb over the eastward track
    assert c.step(0.0, False, True, t, 1.0).climbing
    assert c.step(29.0, False, True, t, 1.0).climbing  # inside the bound
    assert not c.step(31.0, False, True, t, 1.0).climbing  # timed out: stop
    assert not c.step(32.0, False, True, t, 1.0).climbing  # stays out
    c.step(33.0, True, True, t, 1.0)  # a heading again
    assert c.step(34.0, False, True, t, 1.0).climbing  # re-armed


def test_no_odometry_no_climb():
    """MUTATION: climb without the odom yaw -> a heading that can never follow
    a turn -> red."""
    c, t = Climb(30.0), _track_east()  # a fresh climb over the eastward track
    assert not c.step(0.0, False, True, t, None).climbing


@pytest.mark.parametrize("fix,want", [("rtk_fixed", (None, 1.0)), ("rtk_float", (None, 1.0)),
                                      ("dgps", (None, 0.6))])
def test_the_climb_runs_at_the_cog_fixed_speed(fix, want):
    """99 U153 (2026-11-07): only the dual antenna on rtk_fixed runs at the
    2.0 top speed; a climb on rtk_fixed is held at the COG fixed speed 1.0
    (U151's \"top speed\" was 1.0 when it was ruled). float / dgps get the 0.6
    COG speed, never the 0.5 cap that would pin it at the COG threshold.
    MUTATION: no term on rtk_fixed -> the climb runs at 2.0 -> red; give it
    the 0.5 cap -> red."""
    # float pair 1.0 / 1.5 (99 U180): float is held at the bridge 1.0, never cog_float 1.5
    assert quality_caps(fix, "climb", 0.5, 2.0, 0.6, 1.0, 1.0, 1.5) == want  # COG 2.0 (U178), climb 1.0


@pytest.mark.parametrize("bad", [0.0, -1.0, None, True])
def test_the_keys_have_no_default(bad):
    """CLAUDE.md 3.1: missing / null / non-positive refuses. MUTATION: default
    1.0 / 20.0 / 30.0 -> no raise -> red."""
    with pytest.raises(ValueError):  # refused, never defaulted
        TrackHeading(bad, 20.0)  # bad segment length
    with pytest.raises(ValueError):  # refused, never defaulted
        TrackHeading(1.0, bad)  # bad straightness
    with pytest.raises(ValueError):  # refused, never defaulted
        Climb(bad)  # bad time bound


# -- the pose view -----------------------------------------------------------------
def _runtime(heading_valid, i_heading, route="running", odom=1.0, source="cog"):  # a _pose_view stub
    import time  # monotonic caches
    from xbrain.p1_motion.path.local_frame import LocalFrame  # site frame
    now = time.monotonic()  # fresh caches

    class _Cfg:  # the NavConfig fields _pose_view reads
        gnss_dead_ms = 1000  # nothing is stale here
        frame = LocalFrame(35.0, 135.0)  # a site frame

    class _Rt:  # the attributes _pose_view touches
        _cfg = _Cfg()  # the config fields
        _gnss = {"rx": now, "data": {"heading_valid": heading_valid, "heading_rad": 0.1,
                                     "source": source, "level": 2, "i_heading": i_heading}}
        _fix = {"rx": now, "data": {"fix_type": "rtk_fixed", "lat": 35.0, "lon": 135.0}}
        _track = _track_east()  # the robot drove east
        _climb = Climb(30.0)  # the climb under test
        _bridge = HeadingBridge(30.0, 20.0, 1.0)  # 99 U152: fresh, nothing to bridge from
        _host = type("H", (), {"route_state": route})()

        def _odom_yaw(self, t0):  # the chassis odom yaw of this case
            return odom  # the case's odometry
    return _Rt(), now  # stub and its clock


def test_pose_view_hands_the_climb_heading_to_the_gate():
    """MUTATION: drop the climb from _pose_view -> heading invalid, the gate
    vetoes (the outdoor deadlock) -> red."""
    from xbrain.p1_motion.runtime.nav_wiring import NavRuntime  # owner of _pose_view
    rt, now = _runtime(False, 1.0)  # L2-blind during a route
    view = NavRuntime._pose_view(rt, now)  # one pose view
    assert view.heading_valid and view.heading_source == "climb" and not view.yaw_capable


# the valid case is an L1 heading: a lone first COG is held one tick by the
# U152 check (test_heading_bridge.py) and would climb here, as designed
@pytest.mark.parametrize("valid,ih,route,src", [(True, 1.0, "running", "dual_antenna"),  # normal
                                                (False, 0.0, "running", "cog"),  # L3: no climb
                                                (False, 1.0, "idle", "cog")])  # no route task
def test_pose_view_climbs_only_in_the_deadlock(valid, ih, route, src):
    """MUTATION: climb on an L3 (i_heading 0) -> drives with no fix -> red."""
    from xbrain.p1_motion.runtime.nav_wiring import NavRuntime  # owner of _pose_view
    rt, now = _runtime(valid, ih, route, source=src)  # the stub of this case
    view = NavRuntime._pose_view(rt, now)  # one pose view
    assert view.heading_source != "climb"
