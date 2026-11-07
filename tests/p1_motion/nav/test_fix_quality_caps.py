"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fix_quality_caps.py
Brief: 99 U142 -- float / dgps drive like rtk_fixed under a 0.5 m/s cap; an L2 (COG) heading drives at a fixed 1.0 (fixed) / 0.6 (float, dgps) m/s

Description:
The user's ruling of 2026-11-06 replaced the 11 S3.2.1 fix table: rtk_float
and dgps take tasks and drive like rtk_fixed, only slower (0.5 m/s); single
and no_fix stay "no autonomous motion". The same day fixed the L2 (COG)
speed at 1.0 m/s on rtk_fixed and 0.6 m/s on float / dgps (the slowest that
keeps COG alive): COG is only valid at >= 0.5 m/s (rtk_driver
cog_speed_thresh_mps), so the old 0.4 heading factor (2.0 x 0.4 = 0.8, and
less wherever RNS slows) let the robot fall into L2-blind -- a heading veto
it cannot drive out of.

Each case names the mutant it kills:
  * quality_caps decision table (cog beats the fix cap, float / dgps capped,
    fixed + L1 untouched);
  * compute_gate adds the named term and attributes the cut to it;
  * NavTick.run passes the caps through (a float fix moves at <= 0.5, a COG
    heading at <= 0.6, limiter named);
  * NavRuntime._pose_view hands the gate the source of a VALID heading only.
"""
from __future__ import annotations  # postponed annotations

import dataclasses  # replace() on the frozen NavInputs

import pytest  # parametrize, approx

from tests.p1_motion.nav.test_nav_tick import OK, _drive_until_moving, _goto, _inp, _stack  # the tick spine
from xbrain.p1_motion.nav.host_gate import compute_gate  # the gate
from xbrain.p1_motion.nav.nav_tick import quality_caps  # the decision table
from xbrain.p1_motion.path import gnss_pose  # i_fix table

pytestmark = pytest.mark.no_device  # pure logic

CAP, COG, COG_LOW = 0.5, 1.0, 0.6  # degraded_fix_cap / cog_drive_fixed / cog_drive_degraded


# -- the decision table ------------------------------------------------------------
@pytest.mark.parametrize("fix,source,want", [
    ("rtk_fixed", "dual_antenna", (None, None)),  # full speed: nothing added
    ("rtk_float", "dual_antenna", (CAP, None)),   # float: capped
    ("dgps", "dual_antenna", (CAP, None)),        # dgps: capped, NOT stopped (was i_fix 0)
    ("rtk_fixed", "cog", (None, COG)),            # L2 on a good fix: 1.0
    ("rtk_float", "cog", (None, COG_LOW)),        # L2 on float: 0.6, not 1.0
    ("dgps", "cog", (None, COG_LOW)),             # COG beats the cap: 0.5 would starve COG
    ("rtk_fixed", None, (None, None)),            # no valid heading: the gate vetoes anyway
    ("rtk_fixed", "bridge", (None, COG)),         # 99 U153: only L1 + fixed runs at 2.0
    ("rtk_fixed", "climb", (None, COG)),          # 99 U153: likewise
])
def test_quality_caps(fix, source, want):
    """MUTATION: check the fix before the source -> dgps + cog gets the 0.5
    cap and COG drops to blind -> red; drop dgps from the capped set -> red;
    one COG speed for every fix -> float + cog runs at 1.0 -> red."""
    assert quality_caps(fix, source, CAP, COG, COG_LOW) == want  # (fix_cap, cog_drive)


def test_float_and_dgps_factor_is_one_single_still_vetoes():
    """MUTATION: keep dgps at 0.0 -> the gate vetoes dgps on rtk -> red."""
    for fix, want in (("rtk_float", 1.0), ("dgps", 1.0), ("single", 0.0), ("no_fix", 0.0)):
        assert gnss_pose.assemble_pose(None, {"fix_type": fix})["i_fix"] == want  # 99 U142 table


# -- the gate ----------------------------------------------------------------------
def _gate(**kw):
    """compute_gate on an open, healthy, fixed tick with a 2.0 profile."""
    base = dict(v_nom_mps=2.0, spec_max_vx_mps=2.0, free_space_mps=None, health=OK,
                i_fix=1.0, i_heading=1.0, heading_valid=True, estop=False,
                perception_dead=False)  # nothing else limits
    base.update(kw)  # the case under test
    return compute_gate(**base)


def test_the_fix_cap_is_a_named_term():
    """MUTATION: ignore fix_cap_mps -> 2.0 -> red; name it profile -> red."""
    g = _gate(fix_cap_mps=CAP)  # a float / dgps tick
    assert g.v_max_fwd == pytest.approx(CAP) and g.cuts[0][0] == "rtk"  # capped and attributed


def test_the_fixed_fix_cog_speed_is_one_metre_per_second():
    """MUTATION: one COG speed for all fixes (0.6) -> red."""
    g = _gate(cog_drive_mps=COG)  # rtk_fixed + COG, profile 2.0
    assert g.v_max_fwd == pytest.approx(COG) and g.cuts[0][0] == "heading"  # 1.0, named


def test_the_cog_speed_replaces_the_fix_cap():
    """MUTATION: apply both -> min(0.5, 0.6) = 0.5 -> red."""
    g = _gate(fix_cap_mps=CAP, cog_drive_mps=COG_LOW)  # both offered: cog wins by contract
    assert g.v_max_fwd == pytest.approx(COG_LOW) and g.cuts[0][0] == "heading"  # the COG term


# -- through NavTick ---------------------------------------------------------------
@pytest.mark.parametrize("fix,source,ceiling,limiter", [
    ("rtk_float", "dual_antenna", CAP, "rtk"),  # float: <= 0.5
    ("rtk_fixed", "cog", 1.0, None),            # COG on fixed: 1.0 ties the spine's v_nom (gate test names it)
    ("dgps", "cog", COG_LOW, "heading"),        # COG on dgps: <= 0.6
])
def test_navtick_applies_the_caps(fix, source, ceiling, limiter):
    """MUTATION: NavTick does not pass the caps to compute_gate -> v_max 1.0
    (the spine's v_nom) -> red."""
    src, tick = _stack()  # v_nom 1.0, caps 0.5 / 1.0 / 0.6
    _goto(src, 5000)  # a goto in open ground

    class _Tick:  # tick.run with the fix / source of this case
        def run(self, inp):
            return tick.run(dataclasses.replace(inp, fix_type=fix, heading_source=source))
    out = _drive_until_moving(_Tick(), 5000)  # first moving output
    assert 0.0 < out.vx <= ceiling + 1e-9  # moving, never faster than the cap
    assert out.v_max == pytest.approx(ceiling)  # the ceiling of this case
    if limiter is not None:
        assert out.limiter == limiter  # named


# -- through the pose view ---------------------------------------------------------
@pytest.mark.parametrize("valid,want", [(True, "cog"), (False, None)])
def test_pose_view_passes_only_a_valid_source(valid, want):
    """L2-blind (cog, heading_valid false) must not reach the gate as "cog".
    MUTATION: pass the raw source -> blind ticks look like COG -> red (the
    False case); drop the field -> None for the valid case -> red."""
    import time  # the caches age on the monotonic clock
    from xbrain.p1_motion.path.local_frame import LocalFrame  # site frame
    from xbrain.p1_motion.path.climb import Climb, TrackHeading  # 99 U151 stub fields
    from xbrain.p1_motion.path.heading_bridge import HeadingBridge  # 99 U152 stub field
    from xbrain.p1_motion.runtime.nav_wiring import NavRuntime  # owner of _pose_view
    now = time.monotonic()  # fresh caches

    class _Cfg:  # the two NavConfig fields _pose_view reads
        gnss_dead_ms = 1000
        frame = LocalFrame(35.0, 135.0)

    class _Rt:  # just the attributes _pose_view touches
        _cfg = _Cfg()
        _gnss = {"rx": now, "data": {"heading_valid": valid, "heading_rad": 0.1 if valid else None,
                                     "source": "cog", "level": 2, "i_heading": 1.0}}
        _fix = {"rx": now, "data": {"fix_type": "rtk_fixed", "lat": 35.0, "lon": 135.0}}
        # 99 U151 climb inputs: no route running, no odometry -> never climbs here
        _track = TrackHeading(1.0, 20.0)
        _climb = Climb(30.0)
        _bridge = HeadingBridge(30.0, 20.0, 1.0)  # 99 U152: no odometry -> GNSS passes through
        _host = type("H", (), {"route_state": "idle"})()

        def _odom_yaw(self, t0):  # no chassis state in this stub
            return None
    view = NavRuntime._pose_view(_Rt(), now)  # unbound call on the stub
    assert view.heading_source == want  # source only with a valid heading
