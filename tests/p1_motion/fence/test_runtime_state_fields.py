"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_runtime_state_fields.py
Brief: state/fence (11 S9A.5) is filled from the clip evaluation -- enforcement, geo, allow; honest before the first tick

Description:
build_fence_runtime_state with the clip fields from runtime_state_fields:
a full-enforcement tick prints its geometry (d_nom / inset / d_eff /
v_fence / normal / episode_id) and grants allow{}; a degraded tick prints
state unknown and refuses; no evaluation yet keeps the pre-first-tick
warn_only / clip_deferred shape the earlier tests pinned.
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.fence.clip import (CompiledFence, CompiledPolygon, FenceConstants,
                                         evaluate, runtime_state_fields)
from xbrain.p1_motion.fence.fence_set import (HeldFenceSet, HeldPolygon,
                                              build_fence_runtime_state)

pytestmark = pytest.mark.no_device

CFG = FenceConstants(brake_k=1.5, brake_a_mps2=2.5, t_lat_s=0.4, soft_margin_min_m=2.0,
                     predict_dt_s=0.45, margin_by_fix={"rtk_fixed": 0.3, "rtk_float": 1.0},
                     projection_iters=3, v_profile_max_mps=2.0, teleop_cap_degraded_mps=0.5)
SQ = CompiledPolygon("p-outer", "allow", "outer", True, True,
                     ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)), None)
HELD = HeldFenceSet("fs", 7, "8f3a21bc",
                    (HeldPolygon("p-outer", "allow", "camp", True,
                                 ((0.0, 0.0), (0.001, 0.0), (0.001, 0.001))),))


def test_full_tick_fills_geo_and_allow():
    """mutant: drop the clip override in the builder -> red."""
    ev = evaluate(CompiledFence("fs", 7, (SQ,)), CFG, xy=(8.5, 5.0), yaw_rad=0.0, vx=2.0, vy=0.0,
                  fix_type="rtk_fixed", heading_valid=True)
    st = build_fence_runtime_state(HELD, now_mono_s=10.0, applied_mono_s=9.0,
                                   clip=runtime_state_fields(ev, episode_id=2))
    assert st["active"]["rev"] == 7 and st["enforcement"] == "full" and st["degrade_reason"] == "none"
    geo = st["geo"]
    assert geo["state"] == "soft" and geo["poly_id"] == "p-outer" and geo["episode_id"] == 2
    assert geo["d_nom_m"] == pytest.approx(1.5) and geo["inset_m"] == 0.3 and geo["d_eff_m"] == pytest.approx(1.2)
    assert geo["margin_soft_eff_m"] == pytest.approx(2.4) and geo["clipped"] is True
    assert geo["outward_normal"] == [1.0, 0.0] and 0.0 < geo["v_fence_mps"] < 2.0
    assert st["allow"] == {"autonomous": True, "accept_task": True, "teleop_max_mps": 2.0}


def test_degraded_tick_prints_unknown_and_refuses():
    ev = evaluate(CompiledFence("fs", 7, (SQ,)), CFG, xy=(8.5, 5.0), yaw_rad=0.0, vx=2.0, vy=0.0,
                  fix_type="dgps", heading_valid=True)
    st = build_fence_runtime_state(HELD, now_mono_s=10.0, applied_mono_s=9.0,
                                   clip=runtime_state_fields(ev))
    assert st["enforcement"] == "warn_only" and st["degrade_reason"] == "rtk_degraded"
    assert st["geo"] == {"state": "unknown", "episode_id": 0}
    assert st["allow"] == {"autonomous": False, "accept_task": False, "teleop_max_mps": 0.5}


def test_before_the_first_tick_stays_clip_deferred():
    assert runtime_state_fields(None) is None
    st = build_fence_runtime_state(HELD, now_mono_s=10.0, applied_mono_s=9.0, clip=None)
    assert st["enforcement"] == "warn_only" and st["degrade_reason"] == "clip_deferred"
