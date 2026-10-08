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

from tests.p1_motion.nav.rot_fixture import rot_limits
from tests.p1_motion.rns.scenes import healthy_status, snapshot, uniform_free
from xbrain.p1_motion.fence.clip import (
    CompiledFence,
    CompiledPolygon,
    FenceConstants,
    v_fence_mps,
)
from xbrain.p1_motion.nav.health_factor import HealthView
from xbrain.p1_motion.nav.host_gate import GateResult, fence_attribution
from xbrain.p1_motion.nav.nav_tick import NavInputs, NavTick
from xbrain.p1_motion.nav.relmove_intake import RelMoveLimits, translate_relative_move
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.sources.arbiter_p1 import P1Arbiter
from xbrain.p1_motion.sources.rns_avoid import RnsAvoidSource

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
    tick = NavTick(src, P1Arbiter(dwell_ms=200), v_nom_mps=1.0, speed_up_hold_ms=3000, d_up_margin_m=0.5, wz_max_rps=1.2,
                   spec_max_vx_mps=2.0, holonomic=True,
                   fence_consts=FC if with_fence else None,
                   rot_limits=rot_limits(), r_robot_m=None)
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


# ---------------------------------------------------------------- speed zone --
# 11 S9A.2 块 B-2 逐字: speed_limit_mps "该区内的速度上限, 直接进 S9.6.1 的
# min(); 重叠时取最小值". S9A.1A 把[施加]派给 RNS("具体速度上限由 RNS 走速度门
# 施加"). 下面四条分别钉: 区内施加 / 区外不施加 / 重叠取最小 / 无围栏零回归.


def _zone(poly_id, pts, limit):
    """一个 role=speed_limit 的 HeldPolygon. hard_enforce 恒 False --
    S9A.2 对两个 advisory role 都这么定, 而 compile_fence 会把它放进
    speed_zones 而不是 polygons(所以它永远进不了半空间投影)."""
    from xbrain.p1_motion.fence.fence_set import HeldPolygon
    return HeldPolygon(poly_id, "speed_limit", poly_id, False, tuple(pts),
                       speed_limit_mps=limit)


def _held_with_zone(*zones):
    """一个只含限速区的持有集. 刻意不放 allow/forbid: 这样编译出的 .polygons
    为空, evaluate 走 disabled/no_fence 分支 -- 于是本组用例测到的任何限速
    效果都只能来自速度门那一项, 不可能是裁剪层顺带做的."""
    from xbrain.p1_motion.fence.fence_set import HeldFenceSet
    return HeldFenceSet("fs-z", 1, "00000000", tuple(zones))


def _run_zone(held, pose, limit_pts_scale=1.0):
    """编译并跑一拍, 返回 NavOutput."""
    from xbrain.p1_motion.fence.clip import compile_fence
    from xbrain.p1_motion.path.local_frame import LocalFrame
    cf = compile_fence(held, LocalFrame(0.0, 0.0))
    return _run(_stack(), pose, fence=cf)


def test_a_speed_zone_caps_v_max_and_names_fence_as_the_limiter():
    """区内: 限速值进 min(), 且 gate.limiter 必须说是 fence.

    *** limiter 这一半和数值一样重要(11 S9.6.5): 若把限速写成事后 clamp,
    v_max 会对而 limiter 仍指向 profile -- 现场看到的是"机器人在爬, 而系统
    说是档位限的", 排障会走到完全错误的方向.
    MUTATION: 把 terms["fence"] 那一行删掉 -> v_max 不降, 第一条断言红;
    改成在 compute_gate 之后 clamp -> v_max 对而 limiter 仍是 profile,
    第二条断言红.
    """
    # 以原点为心, 半径约 50 m 的方形限速区(ENU 米, LocalFrame(0,0) 下
    # 1 deg lat ~ 111320 m, 所以 0.00045 deg ~ 50 m).
    d = 0.00045
    z = _zone("f-slow", [(-d, -d), (-d, d), (d, d), (d, -d)], 0.4)
    out = _run_zone(_held_with_zone(z), (0.0, 0.0))        # 机器人在区心
    assert out.v_max == pytest.approx(0.4, abs=1e-6)
    assert out.limiter == "fence"
    # 裁剪层没有参与: 集里没有 allow/forbid, 所以它报 disabled/no_fence.
    assert out.fence.enforcement == "disabled"


def test_outside_the_zone_the_cap_does_not_apply():
    """区外: 不施加. 否则一个画在角落的慢行区会把整个场地限速.

    MUTATION: speed_cap_mps 把 `d_nom > -inset` 写成 `d_nom < inset`(符号反)
    -> 区外反而被限速 -> 本条红.
    """
    d = 0.00045
    z = _zone("f-slow", [(-d, -d), (-d, d), (d, d), (d, -d)], 0.4)
    # 区心在原点, 机器人放到 500 m 外 -- 远超任何 inset.
    out = _run_zone(_held_with_zone(z), (500.0, 0.0))
    assert out.v_max > 0.4
    assert out.limiter != "fence"


def test_overlapping_zones_take_the_minimum():
    """重叠时取最小值(S9A.2 块 B-2 逐字).

    MUTATION: 把 min(cap, v) 换成 max 或"第一个命中即返回" -> 本条红.
    两个区同心, 一个 1.2 一个 0.4, 机器人同时在两个区内.
    """
    d = 0.00045
    # *** 两个限速值都取得比本栈的其余 min() 项低(实测该 min 是 1.0).
    # 第一版给大区 1.2, 于是 v_max 取到 1.0 而不是 1.2 -- 断言红在"取最小"
    # 上, 而真正的原因是"限速项根本没赢". 判据要测的是哪一项赢, 就必须让它
    # 有机会赢, 否则红了也不知道红在哪.
    big = _zone("f-a", [(-d, -d), (-d, d), (d, d), (d, -d)], 0.9)
    small = _zone("f-b", [(-d / 2, -d / 2), (-d / 2, d / 2),
                          (d / 2, d / 2), (d / 2, -d / 2)], 0.3)
    out = _run_zone(_held_with_zone(big, small), (0.0, 0.0))
    assert out.v_max == pytest.approx(0.3, abs=1e-6)
    # 反向: 只在大区内(小区之外)时取 0.9, 否则上面的 0.3 可能只是"恒取最后
    # 一个"或"恒取遇到的第一个", 而不是真的取最小.
    out2 = _run_zone(_held_with_zone(big, small), (0.0, 40.0))
    assert out2.v_max == pytest.approx(0.9, abs=1e-6)
    # *** NO 这里不能断言 limiter == "fence". 11 S9.6.5 末行: 没有实际裁到
    # 且 h == i == 1.0 时 limiter 就是 "none" -- 而这一拍的候选速度本来就低于
    # 0.9, 天花板摆在那里但没咬到任何东西. 第一条用例(区心, 上限 0.3)才是
    # 真咬到的那一拍, limiter 在那里断言.
    assert out2.limiter in ("none", "fence")


def test_a_zone_also_caps_reverse_and_lateral():
    """限速是[速度]上限, 各向同性 -- 倒车与横移同样受限.

    *** v_lin_free(倒车/横移的上限)故意丢掉 f 项(前向净空), 但 NO 不能连
    限速区一起丢: 在一个 1 m/s 的区里以 2 m/s 倒车, 违反限速的程度与前进
    完全一样.
    MUTATION: v_lin_free 回 min(terms["profile"], terms["spec"]) ->
    v_max_free 不受限速约束 -> 本条红.
    """
    from xbrain.p1_motion.nav.host_gate import compute_gate
    g = compute_gate(v_nom_mps=2.0, spec_max_vx_mps=2.0, free_space_mps=2.0,
                     health=OK, i_fix=1.0,
                     i_heading=1.0, heading_valid=True, estop=False,
                     perception_dead=False, fence_zone_mps=0.4)
    assert g.v_max_fwd == pytest.approx(0.4, abs=1e-9)
    assert g.v_max_free == pytest.approx(0.4, abs=1e-9)


def test_no_zone_leaves_the_gate_byte_identical():
    """零回归(NEXT I-1): 没有限速区时, 速度门算出的与加这一项之前完全一样.

    *** 这是用户 2026-10-08 明令的那条不变量在速度门这一层的落点:
    系统没有围栏数据时不对运动做任何限制.
    MUTATION: 给 fence_zone_mps 一个兜底默认值(例如 2.0)而不是 None ->
    terms 里凭空多一项 -> 一旦该默认值小于其它项就会无声限速 -> 本条红.
    """
    from xbrain.p1_motion.nav.host_gate import compute_gate
    args = dict(v_nom_mps=2.0, spec_max_vx_mps=2.0, free_space_mps=1.5,
                health=OK, i_fix=1.0, i_heading=1.0,
                heading_valid=True, estop=False, perception_dead=False)
    a = compute_gate(**args)                         # 不传该参
    b = compute_gate(**args, fence_zone_mps=None)    # 显式 None
    assert (a.v_max_fwd, a.v_max_free, a.cuts, a.tied_min) == \
           (b.v_max_fwd, b.v_max_free, b.cuts, b.tied_min)
    assert "fence" not in a.tied_min and "fence" not in [k for k, _ in a.cuts]
