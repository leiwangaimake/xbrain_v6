"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_nav_tick_fence.py
Brief: NavTick step 7 -- the fence clips the gated RNS candidate, names the limiter, reports its geometry

Description:
End-to-end through NavTick with a real RnsSource driving east toward a
goto at (12, 0) inside an allow square whose east edge is at x = 8: far
from the edge the output is the plain gate output; in the soft band vx is
capped at v_fence and gate.limiter says fence; at the effective boundary
the forward component is removed while the RNS candidate is still
positive; without an active fence the evaluation reports disabled /
no_fence and touches nothing; a NavTick built without fence constants
(the older unit stacks) has no fence field at all. Plus the attribution
rule itself (fence_attribution) on hand-made GateResults.
"""
from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from xbrain.p1_motion.fence.clip import (CompiledFence, CompiledPolygon, FenceConstants,
                                         v_fence_mps)
from xbrain.p1_motion.nav.health_factor import HealthView
from xbrain.p1_motion.nav.host_gate import GateResult, fence_attribution
from xbrain.p1_motion.nav.nav_tick import NavInputs, NavTick
from xbrain.p1_motion.nav.relmove_intake import RelMoveLimits, translate_relative_move
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.sources.arbiter_p1 import P1Arbiter
from xbrain.p1_motion.sources.rns_avoid import RnsAvoidSource
from tests.p1_motion.rns.scenes import healthy_status, snapshot, uniform_free

pytestmark = pytest.mark.no_device

_ROOT = Path(__file__).resolve().parents[3]
_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))

OK = HealthView(1.0, True, "patrol", "ok", 100)
LIM = RelMoveLimits(20.0, 6.2832, 0.02, 20.0, True)
FC = FenceConstants(brake_k=1.5, brake_a_mps2=2.5, t_lat_s=0.4, soft_margin_min_m=2.0,
                    predict_dt_s=0.45, margin_by_fix={"rtk_fixed": 0.3, "rtk_float": 1.0},
                    projection_iters=3, v_profile_max_mps=1.0, teleop_cap_degraded_mps=0.5)
SQ = CompiledPolygon("p-outer", "allow", "outer", True, True,
                     ((-5.0, -5.0), (8.0, -5.0), (8.0, 8.0), (-5.0, 8.0)), None)
FENCE = CompiledFence("fs", 1, (SQ,))


def _stack(with_fence=True):
    cfg = copy.deepcopy(_CFG)
    rns = RnsSource(cfg=cfg, r_eff_m=0.5)
    src = RnsAvoidSource(rns, cfg["rns"])
    tick = NavTick(src, P1Arbiter(dwell_ms=200), v_nom_mps=1.0, wz_max_rps=1.2,
                   spec_max_vx_mps=2.0, holonomic=True,
                   fence_consts=FC if with_fence else None)
    goal = translate_relative_move(
        {"cmd_id": "rm-1", "dx_m": 12.0, "dy_m": 0.0, "dyaw_rad": 0.0},
        pose_xy=(0.0, 0.0), yaw_rad=0.0, heading_valid=True, holonomic=True, limits=LIM)
    src.load_goto(goal, 1000)
    return tick


def _inp(now, pose, fence=FENCE):
    return NavInputs(now_mono_ms=now, pose_xy=pose, yaw_rad=0.0, heading_valid=True,
                     i_fix=1.0, i_heading=1.0,
                     perception=snapshot(uniform_free(6.0, t_capture_mono_ms=now,
                                                      t_seg_mono_ms=now - 10),
                                         None, healthy_status(t_publish_mono_ms=now)),
                     health=OK, estop=False, teleop_active=False,
                     fix_type="rtk_fixed", fence=fence)


def _run(tick, pose, fence=FENCE, ticks=12):
    out = None
    for k in range(ticks):
        out = tick.run(_inp(1000 + 50 * k, pose, fence))
        if out.raw_vx > 0.0:
            return out
    return out


def test_far_from_the_edge_is_the_plain_gate_output():
    out = _run(_stack(), (0.0, 0.0))
    assert out.raw_vx > 0.0 and out.vx == pytest.approx(out.raw_vx) and out.fence is not None
    assert out.fence.state == "inside" and not out.fence.clipped and out.limiter == "none"


def test_soft_band_caps_vx_and_names_the_fence_limiter():
    """mutant: never let fence win the attribution -> red."""
    out = _run(_stack(), (7.0, 0.0))               # d_nom 1.0, d_eff 0.7
    cap = v_fence_mps(0.7, FC)
    assert out.raw_vx > cap and out.vx == pytest.approx(cap, abs=1e-6)
    assert out.fence.state == "soft" and out.fence.clipped and out.limiter == "fence"
    assert "fence" in out.limiter_all and out.v_max == pytest.approx(cap)


def test_effective_boundary_removes_the_forward_component():
    """mutant: keep the gated vx instead of the clipped one -> red."""
    out = _run(_stack(), (7.8, 0.0))               # d_eff -0.1: hard, normal (1, 0)
    assert out.raw_vx > 0.0 and out.vx == pytest.approx(0.0, abs=1e-9)
    assert out.fence.hard_ids == ("p-outer",) and out.limiter == "fence"


def test_no_active_fence_reports_disabled_and_touches_nothing():
    out = _run(_stack(), (7.8, 0.0), fence=None)
    assert out.fence.enforcement == "disabled" and out.fence.degrade_reason == "no_fence"
    assert out.vx == pytest.approx(out.raw_vx) and out.limiter == "none"


def test_tick_without_fence_constants_has_no_fence_field():
    out = _run(_stack(with_fence=False), (7.8, 0.0))
    assert out.fence is None and out.vx == pytest.approx(out.raw_vx)


def test_fence_attribution_rule():
    free = GateResult(0.5, 1.0, False, None, 1.0, 1.0, (("free_space", 0.5),), ("free_space",))
    assert fence_attribution("free_space", ("free_space",), free, 0.2) == \
        ("free_space", ("free_space", "fence"))
    assert fence_attribution("free_space", ("free_space",), free, 0.7) == \
        ("fence", ("fence", "free_space"))
    none = GateResult(1.0, 1.0, False, None, 1.0, 1.0, (), ("profile",))
    assert fence_attribution("none", (), none, 0.1) == ("fence", ("fence",))
    veto = GateResult(0.0, 0.0, True, "estop", 0.0, 0.0, (), ())
    assert fence_attribution("estop", ("estop",), veto, 0.3) == ("estop", ("estop",))


def test_no_fence_stage_is_a_pass_through_over_a_whole_drive():
    """Zero-regression proof for the no-fence case: two stacks fed the SAME
    input sequence (a goto, open ground, no active FenceSet), one with the
    fence stage wired and one without, must publish identical vx / vy / wz /
    limiter / v_max on every tick. RNS is deterministic given its inputs
    (the batteries rely on that), so any difference here would be the fence
    stage leaking into the output. mutant: touch vx in the disabled branch
    -> red."""
    a, b = _stack(with_fence=True), _stack(with_fence=False)
    x = 0.0
    for k in range(80):
        now = 1000 + 50 * k
        pose = (x, 0.0)
        oa = a.run(_inp(now, pose, fence=None))
        ob = b.run(_inp(now, pose, fence=None))
        assert (oa.vx, oa.vy, oa.wz, oa.limiter, oa.limiter_all, oa.v_max, oa.source,
                oa.nav_state) == (ob.vx, ob.vy, ob.wz, ob.limiter, ob.limiter_all, ob.v_max,
                                  ob.source, ob.nav_state), k
        assert oa.fence is not None and oa.fence.enforcement == "disabled" and ob.fence is None
        x += oa.vx * 0.05                          # integrate the shared command
    assert x > 1.0                                 # it actually drove
