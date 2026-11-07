"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_heading_bridge.py
Brief: 99 U152 IMU yaw bridge and COG consistency check -- anchor, bridge, window, COG accept / reject, pose view

Description:
The user's ruling of 2026-11-06 after patrol t-20261106-007 hit a wall: GNSS
heading gaps are bridged with the chassis odom yaw change from the last
trusted heading (30 s), and a COG is used only when it agrees with that yaw
(20 deg). The measured case (replayed in test_the_wall_case): at a corner the
first "valid" COG after a slow turn read 135 deg while the body faced -106
deg; RNS turned at full rate on it into the wall. Each case names the mutant
it kills.
"""
from __future__ import annotations  # postponed annotations

import math  # angles

import pytest  # parametrize, approx

from xbrain.p1_motion.nav.nav_tick import quality_caps  # the speed table
from xbrain.p1_motion.path.heading_bridge import HeadingBridge  # under test

pytestmark = pytest.mark.no_device  # pure logic

R = math.radians  # degrees in, radians to the module


def _deg(h):  # a result heading in degrees (None stays None)
    return None if h.heading_rad is None else math.degrees(h.heading_rad)


def _b():  # the configured bridge: 30 s window, 20 deg tolerance, 1 s COG pull
    return HeadingBridge(30.0, 20.0, 1.0)


# -- dual antenna and the bridge -------------------------------------------------------
def test_dual_antenna_passes_and_a_gap_is_bridged_with_the_yaw_change():
    """MUTATION: no bridge -> the gap is invalid (the gate stops, the U151
    deadlock) -> red; odom yaw used as the heading -> 1.3 rad leaks in -> red."""
    b = _b()  # fresh
    h = b.step(0.0, True, R(-60), "dual_antenna", 1.0)  # L1, odom yaw 1.0 (relative)
    assert h.valid and h.source == "dual_antenna" and _deg(h) == pytest.approx(-60)
    h = b.step(0.5, False, None, None, 1.0 + R(-30))  # baseline lost, body turned 30 deg right
    assert h.valid and h.source == "bridge" and _deg(h) == pytest.approx(-90)


def test_the_anchor_follows_every_trusted_tick():
    """MUTATION: anchor only once -> the bridge starts from a heading seconds
    old (here: -60 instead of -10) -> red."""
    b = _b()  # fresh
    b.step(0.0, True, R(-60), "dual_antenna", 0.0)  # first anchor
    # a later trusted tick: the dual antenna moved 50 deg, the odom yaw only 30
    # (IMU drift) -- the fresh pair must replace the old one
    b.step(5.0, True, R(-10), "dual_antenna", R(30))
    h = b.step(5.5, False, None, None, R(30))  # gap, no turn since
    assert _deg(h) == pytest.approx(-10)  # an old anchor would give -30


def test_the_window_counts_from_the_last_trusted_heading():
    """MUTATION: no window -> bridges on a drifting yaw for ever -> red;
    window from the gap start instead of the anchor -> the rejected-COG
    stretch below extends it past 30 s -> red."""
    b = _b()  # fresh
    b.step(0.0, True, R(0), "dual_antenna", 0.0)  # anchor at t=0
    assert b.step(29.0, False, None, None, 0.0).valid  # inside 30 s
    b.step(29.5, True, R(90), "cog", 0.0)  # a COG 90 deg off: rejected, no re-anchor
    assert not b.step(30.5, False, None, None, 0.0).valid  # 30.5 s after the anchor: out


# -- the COG check ---------------------------------------------------------------------
def test_a_consistent_cog_is_used_and_re_anchors():
    """MUTATION: reject every COG -> straight-line COG never re-anchors, the
    bridge expires on a long COG stretch -> red."""
    b = _b()  # fresh
    b.step(0.0, True, R(-60), "dual_antenna", 0.0)  # anchor
    h = b.step(1.0, True, R(-55), "cog", 0.0)  # COG 5 deg off: fine
    assert h.valid and h.source == "cog" and _deg(h) == pytest.approx(-55)
    h = b.step(40.0, False, None, None, 0.0)  # 39 s after the COG anchor... out
    assert not h.valid
    h = b.step(40.5, True, R(-55), "cog", 0.0)  # COG again (no anchor: needs a pair)
    assert not h.valid  # first unanchored COG waits one tick
    h = b.step(40.55, True, R(-56), "cog", 0.0)  # consistent with the previous COG tick
    assert h.valid and h.source == "cog"


def test_the_wall_case():
    """The measured corner (patrol t-20261106-007): COG -122 trusted, the body
    then turned 16 deg left while COG jumped to 135. MUTATION: drop the check
    -> 135 reaches RNS (119 deg off) -> red; re-anchor on the rejected COG ->
    the next bridged tick carries 135 -> red."""
    b = _b()  # fresh
    y0 = 2.0  # odom yaw at the anchor (relative)
    assert b.step(0.0, True, R(-122), "cog", y0).valid is False  # unanchored first COG
    assert b.step(0.05, True, R(-122), "cog", y0).source == "cog"  # paired: trusted
    h = b.step(0.55, True, R(135), "cog", y0 + R(16))  # the bad COG
    assert h.source == "bridge" and _deg(h) == pytest.approx(-106)  # the body, not the track
    h = b.step(1.05, False, None, None, y0 + R(36))  # blind again, 20 deg more left
    assert _deg(h) == pytest.approx(-86)  # still from the -122 anchor


def test_an_unanchored_cog_that_swings_without_the_body_is_rejected():
    """MUTATION: accept any unanchored COG -> a track swinging 60 deg while
    the body holds still is taken -> red."""
    b = _b()  # fresh, no anchor
    b.step(0.0, True, R(10), "cog", 0.0)  # first COG
    h = b.step(0.05, True, R(70), "cog", 0.0)  # COG +60, body 0
    assert not h.valid


def test_a_cog_after_a_gap_does_not_pair_with_a_stale_cog():
    """MUTATION: pair COG ticks across a gap -> a COG seconds old vouches
    for the new one -> red."""
    b = _b()  # fresh, no anchor
    b.step(0.0, True, R(10), "cog", 0.0)  # a COG
    b.step(0.05, False, None, None, 0.0)  # blind
    assert not b.step(0.1, True, R(12), "cog", 0.0).valid  # the pair was broken


# -- no odometry -----------------------------------------------------------------------
def test_no_odometry_passes_gnss_through_and_bridges_nothing():
    """MUTATION: keep the anchor while odometry is absent -> after it returns
    a gap is bridged from a pairing that no longer holds -> red."""
    b = _b()  # fresh
    b.step(0.0, True, R(-60), "dual_antenna", 0.0)  # anchored
    h = b.step(0.5, True, R(135), "cog", None)  # no odometry: COG passes unchecked
    assert h.valid and h.source == "cog"
    assert not b.step(1.0, False, None, None, None).valid  # nothing to bridge with
    assert not b.step(1.5, False, None, None, 0.0).valid  # odometry back: no old anchor


def test_a_creeping_cog_cannot_walk_the_estimate():
    """The replay of t-20261106-007: COG lagging a slow turn crept 8 deg per
    tick, each step under the 20 deg tolerance, while the body held still.
    MUTATION: replace the estimate with each accepted COG (the first U152
    draft) -> the estimate walks to 40 deg -> red."""
    b = _b()  # fresh
    b.step(0.0, True, R(0), "dual_antenna", 0.0)  # trusted 0 deg
    h = None
    for i in range(1, 6):  # 8, 16, 24, 32, 40 deg at 20 Hz, body still
        h = b.step(0.05 * i, True, R(8 * i), "cog", 0.0)
    assert abs(_deg(h)) < 3.0  # pulled a little, then rejected


def test_a_steady_cog_is_followed_within_its_time_constant():
    """MUTATION: never pull (accept but ignore COG) -> a steady 10 deg COG
    is never followed -> red."""
    b = _b()  # fresh
    b.step(0.0, True, R(0), "dual_antenna", 0.0)  # trusted 0 deg
    h = None
    for i in range(1, 61):  # 3 s of a steady 10 deg COG at 20 Hz, body still
        h = b.step(0.05 * i, True, R(10), "cog", 0.0)
    assert _deg(h) == pytest.approx(10.0, abs=0.6)  # 3 tau: within 5 percent


@pytest.mark.parametrize("bad", [0.0, -1.0, None, True])
def test_the_keys_have_no_default(bad):
    """CLAUDE.md 3.1: missing / null / non-positive refuses. MUTATION: default
    30.0 / 20.0 / 1.0 -> no raise -> red."""
    with pytest.raises(ValueError):  # refused, never defaulted
        HeadingBridge(bad, 20.0, 1.0)  # bad window
    with pytest.raises(ValueError):  # refused, never defaulted
        HeadingBridge(30.0, bad, 1.0)  # bad tolerance
    with pytest.raises(ValueError):  # refused, never defaulted
        HeadingBridge(30.0, 20.0, bad)  # bad time constant


@pytest.mark.parametrize("fix,want", [("rtk_fixed", (None, 1.0)), ("rtk_float", (None, 0.6))])
def test_the_bridge_runs_like_the_climb(fix, want):
    """99 U153: held at the COG fixed speed 1.0 on rtk_fixed (only the dual
    antenna runs at 2.0). MUTATION: no term on rtk_fixed -> the bridge runs
    at the 2.0 top speed -> red; like a plain heading on float -> the 0.5 cap
    holds it at the COG threshold -> red."""
    assert quality_caps(fix, "bridge", 0.5, 1.0, 0.6) == want


# -- the pose view ---------------------------------------------------------------------
def _runtime(gnss, odom):  # a _pose_view stub; gnss = (valid, heading_rad, source)
    import time  # monotonic caches
    from xbrain.p1_motion.path.climb import Climb, TrackHeading  # stub fields
    from xbrain.p1_motion.path.local_frame import LocalFrame  # site frame
    now = time.monotonic()  # fresh caches

    class _Cfg:  # the NavConfig fields _pose_view reads
        gnss_dead_ms = 1000  # nothing is stale here
        frame = LocalFrame(35.0, 135.0)  # a site frame

    class _Rt:  # the attributes _pose_view touches
        _cfg = _Cfg()  # the config fields
        _fix = {"rx": now, "data": {"fix_type": "rtk_fixed", "lat": 35.0, "lon": 135.0}}
        _track = TrackHeading(1.0, 20.0)  # no movement: the climb cannot start
        _climb = Climb(30.0)  # never climbs here
        _bridge = HeadingBridge(30.0, 20.0, 1.0)  # under test
        _host = type("H", (), {"route_state": "running"})()
        _odom = odom  # the case's odometry

        def set_gnss(self, valid, heading, source):  # the next tick's L1 / L2 sample
            self._gnss = {"rx": time.monotonic(), "data": {
                "heading_valid": valid, "heading_rad": heading, "source": source,
                "level": 1 if source == "dual_antenna" else 2, "i_heading": 1.0}}

        def _odom_yaw(self, t0):  # the chassis odom yaw of this case
            return self._odom
    rt = _Rt()
    rt.set_gnss(*gnss)
    return rt  # the stub


def test_pose_view_hands_the_bridged_heading_to_the_gate():
    """MUTATION: drop the bridge from _pose_view -> the gap is a heading veto
    -> red; mark it yaw_capable -> face-heading aligns on an inertial guess
    -> red."""
    import time  # tick clock
    from xbrain.p1_motion.runtime.nav_wiring import NavRuntime  # owner of _pose_view
    rt = _runtime((True, R(-60), "dual_antenna"), 0.0)  # anchored on L1
    NavRuntime._pose_view(rt, time.monotonic())
    rt.set_gnss(False, None, "cog")  # L2-blind
    rt._odom = R(-30)  # the body turned 30 deg right
    view = NavRuntime._pose_view(rt, time.monotonic())
    assert view.heading_valid and view.heading_source == "bridge" and not view.yaw_capable
    assert math.degrees(view.yaw) == pytest.approx(-90)


def test_pose_view_drops_a_rejected_cog_with_nothing_to_bridge():
    """MUTATION: let an unchecked COG through when the bridge has nothing ->
    the first COG after a gap (possibly a sideways track) drives -> red."""
    import time  # tick clock
    from xbrain.p1_motion.runtime.nav_wiring import NavRuntime  # owner of _pose_view
    rt = _runtime((True, R(135), "cog"), 0.0)  # a COG with no anchor, no previous COG
    view = NavRuntime._pose_view(rt, time.monotonic())
    assert not view.heading_valid and view.heading_source is None and view.yaw is None
