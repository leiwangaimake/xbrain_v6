"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_clip_eval.py
Brief: fence/clip.evaluate -- soft band, hard clip, retreat, heading rotation, predicted belt, degradations

Description:
The per-tick contract of evaluate() on an allow square 0..10 m with the
rtk_fixed inset 0.3 m: far inside nothing happens; in the soft band the
approach component is capped at v_fence (times h*i); at / past the
effective boundary the outward component is removed and the tangential /
inward remainder survives (12 S7.2, the 11 S9A.9 breach example); the
body->ENU rotation follows the 11 S3.3 heading convention; the 12 S7.3
predicted crossing only engages when the brake constants leave a gap; a
soft-only polygon never hard-clips; and the three 11 S9A.7 degradations
report themselves without judging geometry.
"""
from __future__ import annotations

import math

import pytest

from xbrain.p1_motion.fence.clip import (CompiledFence, CompiledPolygon, FenceConstants,
                                         evaluate, v_fence_mps)

pytestmark = pytest.mark.no_device

CFG = FenceConstants(brake_k=1.5, brake_a_mps2=2.5, t_lat_s=0.4, soft_margin_min_m=2.0,
                     predict_dt_s=0.45,
                     margin_by_fix={"rtk_fixed": 0.3, "rtk_float": 1.0, "dgps": 1.5, "single": 3.0},
                     projection_iters=3, v_profile_max_mps=2.0, teleop_cap_degraded_mps=0.5)
SQ = CompiledPolygon("p-outer", "allow", "outer", True, True,
                     ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)), None)
FENCE = CompiledFence("fs", 1, (SQ,))


def _ev(xy, vx, vy, yaw=0.0, fence=FENCE, cfg=CFG, fix="rtk_fixed", heading=True, hi=1.0):
    return evaluate(fence, cfg, xy=xy, yaw_rad=yaw, vx=vx, vy=vy, fix_type=fix,
                    heading_valid=heading, hi_factor=hi)


def test_far_inside_passes_through():
    ev = _ev((5.0, 5.0), 1.0, 0.0)
    assert ev.enforcement == "full" and ev.state == "inside" and not ev.clipped
    assert (ev.vx, ev.vy) == (1.0, 0.0) and ev.cut_mps == 0.0
    assert ev.allow_autonomous and ev.allow_accept_task and ev.teleop_max_mps == 2.0


def test_soft_band_caps_the_approach_at_v_fence():
    ev = _ev((8.5, 5.0), 2.0, 0.0)            # d_nom 1.5, d_eff 1.2 < margin 2.40
    assert ev.state == "soft" and ev.outward and ev.clipped
    assert ev.d_eff_m == pytest.approx(1.2) and ev.inset_m == 0.3
    assert ev.vx == pytest.approx(v_fence_mps(1.2, CFG)) and ev.vy == pytest.approx(0.0)
    assert ev.cut_mps == pytest.approx(2.0 - v_fence_mps(1.2, CFG))
    assert ev.hard_ids == ()


def test_soft_cap_is_scaled_by_h_times_i():
    """mutant: ignore hi_factor in the cap -> red."""
    ev = _ev((8.5, 5.0), 2.0, 0.0, hi=0.5)
    assert ev.vx == pytest.approx(0.5 * v_fence_mps(1.2, CFG))


def test_hard_clip_keeps_the_tangential_component():
    ev = _ev((9.8, 5.0), 1.0, 1.0)            # d_eff -0.1: zero cap on (1, 0)
    assert ev.clipped and ev.hard_ids == ("p-outer",)
    assert ev.vx == pytest.approx(0.0, abs=1e-9) and ev.vy == pytest.approx(1.0)
    assert ev.state == "soft"                  # d_nom 0.2 > 0: not yet outside


def test_breached_robot_may_retreat_but_not_go_on():
    back = _ev((11.0, 5.0), -1.0, 0.0)
    assert back.state == "outside" and not back.clipped and back.vx == -1.0
    on = _ev((11.0, 5.0), 1.0, 0.3)
    assert on.clipped and on.vx == pytest.approx(0.0, abs=1e-9) and on.vy == pytest.approx(0.3)


def test_heading_rotates_body_axes_into_enu():
    """Facing north (yaw pi/2) near the top edge: body +vx is outward, body
    +vy is west (tangential). A bearing convention would swap these."""
    fwd = _ev((5.0, 9.8), 1.0, 0.0, yaw=math.pi / 2.0)
    assert fwd.clipped and fwd.vx == pytest.approx(0.0, abs=1e-9)
    side = _ev((5.0, 9.8), 0.0, 1.0, yaw=math.pi / 2.0)
    assert not side.clipped and side.vy == pytest.approx(1.0) and abs(side.vx) < 1e-9


def test_predicted_crossing_is_the_belt_for_a_small_k():
    """mutant: disable the predicted pass -> red (k = 1.0 case). With k = 1.5
    the soft cap alone keeps v * dt < d_eff, so the belt stays idle."""
    weak = FenceConstants(brake_k=1.0, brake_a_mps2=2.5, t_lat_s=0.4, soft_margin_min_m=2.0,
                          predict_dt_s=0.45,
                          margin_by_fix={"rtk_fixed": 0.3, "rtk_float": 1.0},
                          projection_iters=3, v_profile_max_mps=2.0, teleop_cap_degraded_mps=0.5)
    ev = _ev((9.65, 5.0), 2.0, 0.0, cfg=weak)   # d_eff 0.05; v_fence 0.118 * 0.45 > 0.05
    assert ev.hard_ids == ("p-outer",) and ev.vx == pytest.approx(0.0, abs=1e-9)
    strong = _ev((9.65, 5.0), 2.0, 0.0)          # k = 1.5: 0.080 * 0.45 < 0.05
    assert strong.hard_ids == () and strong.vx == pytest.approx(v_fence_mps(0.05, CFG))


def test_soft_only_polygon_never_hard_clips():
    big = CompiledPolygon("p-big", "allow", "big", True, True,
                          ((0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)), None)
    soft = CompiledPolygon("p-soft", "forbid", "soft", False, False,
                           ((20.0, 0.0), (30.0, 0.0), (30.0, 10.0), (20.0, 10.0)), None)
    fence = CompiledFence("fs", 2, (big, soft))
    inside = _ev((25.0, 3.0), 0.0, 1.0, fence=fence)   # inside the soft forbid, going deeper
    assert inside.state == "outside" and not inside.clipped and inside.hard_ids == ()
    assert inside.poly_id == "p-soft"
    approach = _ev((18.0, 5.0), 2.0, 0.0, fence=fence)  # d_eff 1.7 to the soft forbid
    assert approach.clipped and approach.hard_ids == () and approach.poly_id == "p-soft"


def test_degradations_do_not_judge_geometry():
    """mutant: skip the heading check -> red."""
    none = _ev((5.0, 5.0), 1.0, 0.0, fence=None)
    assert (none.enforcement, none.degrade_reason) == ("disabled", "no_fence")
    assert not none.allow_autonomous and none.teleop_max_mps == 0.0 and none.vx == 1.0
    dgps = _ev((5.0, 5.0), 1.0, 0.0, fix="dgps")
    assert (dgps.enforcement, dgps.degrade_reason, dgps.state) == ("warn_only", "rtk_degraded", "unknown")
    assert dgps.teleop_max_mps == 0.5 and not dgps.allow_accept_task and dgps.d_nom_m is None
    nopos = _ev(None, 1.0, 0.0)
    assert nopos.degrade_reason == "rtk_degraded"
    blind = _ev((9.8, 5.0), 1.0, 0.0, heading=False)
    assert blind.degrade_reason == "heading_lost" and not blind.clipped and blind.vx == 1.0
    empty = _ev((5.0, 5.0), 1.0, 0.0, fence=CompiledFence("fs", 1, ()))
    assert empty.degrade_reason == "no_fence"


def test_rtk_float_uses_its_own_inset():
    ev = _ev((9.5, 5.0), 1.0, 0.0, fix="rtk_float")    # inset 1.0: d_eff -0.5 -> hard
    assert ev.inset_m == 1.0 and ev.hard_ids == ("p-outer",) and ev.vx == pytest.approx(0.0, abs=1e-9)
