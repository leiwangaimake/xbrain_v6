"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_nav_tick.py
Brief: NavTick + RnsAvoidSource over the REAL RnsSource -- P7.2 loop semantics

Description:
Drives the tick with the production RnsSource (configs/rns.yaml, the same
startup assertions the SIL runs) so what is asserted is the WIRED behaviour:
no mission -> hold and zero; a goto in open ground -> rns_avoid drives; the
perception-never safety gate (raw candidate > 0, published 0, limiter
free_space); estop / teleop suspension edges (SUSPENDED under, FOLLOW after,
never a candidate while suspended); the health and rtk vetoes; and the
adapter's entry mapping (origin, kind, supersede, cancel audit).
"""
from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from tests.p1_motion.rns.scenes import healthy_status, snapshot, uniform_free
from xbrain.p1_motion.nav.health_factor import HealthView
from xbrain.p1_motion.nav.nav_tick import NavInputs, NavTick, NavTickConfigError
from xbrain.p1_motion.nav.relmove_intake import RelMoveLimits, translate_relative_move
from xbrain.p1_motion.nav.route_intake import RouteSet
from xbrain.p1_motion.rns.inputs import PerceptionSnapshot
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind, NavState, Origin
from xbrain.p1_motion.sources.arbiter_p1 import P1Arbiter
from xbrain.p1_motion.sources.rns_avoid import RnsAvoidSource

pytestmark = pytest.mark.no_device

_ROOT = Path(__file__).resolve().parents[3]
_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))
OK = HealthView(1.0, True, "patrol", "ok", 100)
NEVER = HealthView(0.0, False, None, "never", None)
LIM = RelMoveLimits(20.0, 6.2832, 0.02, 20.0, True)


def _stack():
    cfg = copy.deepcopy(_CFG)
    rns = RnsSource(cfg=cfg, r_eff_m=0.5)
    src = RnsAvoidSource(rns, cfg["rns"])
    tick = NavTick(src, P1Arbiter(dwell_ms=200), v_nom_mps=1.0, wz_max_rps=1.2,
                   spec_max_vx_mps=2.0, holonomic=True)
    return src, tick


def _open(now):
    return snapshot(uniform_free(6.0, t_capture_mono_ms=now, t_seg_mono_ms=now - 10),
                    None, healthy_status(t_publish_mono_ms=now))


def _inp(now, *, pose=(0.0, 0.0), yaw=0.0, perception=None, health=OK,
         estop=False, teleop=False, i_fix=1.0, heading_valid=True):
    return NavInputs(now_mono_ms=now, pose_xy=pose, yaw_rad=yaw,
                     heading_valid=heading_valid, i_fix=i_fix, i_heading=1.0,
                     perception=perception if perception is not None else _open(now),
                     health=health, estop=estop, teleop_active=teleop)


def _route(points, rev=1):
    return RouteSet(cmd_id="rg-%d" % rev, route_id="r-1", route_rev=rev,
                    loop_mode="oneway", total_len_m=0.0,
                    points_xy=tuple(points), arrive_radius_m=(0.8,) * len(points))


def _goto(src, now, endpoint=(12.0, 0.0)):
    goal = translate_relative_move(
        {"cmd_id": "rm-1", "dx_m": endpoint[0], "dy_m": endpoint[1], "dyaw_rad": 0.0},
        pose_xy=(0.0, 0.0), yaw_rad=0.0, heading_valid=True, holonomic=True, limits=LIM)
    src.load_goto(goal, now)


def _drive_until_moving(tick, now, ticks=10, **kw):
    """RNS may spend a tick or two aligning; return the first moving output."""
    out = None
    for k in range(ticks):
        out = tick.run(_inp(now + 50 * k, **kw))
        if out.vx > 0.0:
            return out
    return out


def test_no_mission_is_hold_and_zero():
    src, tick = _stack()
    out = tick.run(_inp(5000))
    assert out.source == "hold" and (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)
    assert out.nav_state == "idle"


def test_goto_in_open_ground_drives_from_rns_avoid():
    src, tick = _stack()
    _goto(src, 5000)
    out = _drive_until_moving(tick, 5000)
    assert out.source == "rns_avoid" and out.vx > 0.0
    assert out.limiter in ("none", "profile", "spec", "free_space")
    assert out.freshness == "ok"


def test_perception_never_received_is_vetoed_at_the_host():
    """The P7.2 safety gate: RNS's bare follow spine still produces a
    candidate (raw_vx > 0) but the host publishes zero, limiter free_space.
    mutant: fabricate open space when f_free is None -> red."""
    src, tick = _stack()
    _goto(src, 5000)
    outs = [tick.run(_inp(5000 + 50 * k, perception=PerceptionSnapshot())) for k in range(5)]
    assert any(o.raw_vx > 0.0 for o in outs)
    assert all(o.vx == 0.0 and o.wz == 0.0 and o.limiter == "free_space" for o in outs)
    assert all(o.freshness == "failed" for o in outs)


def test_stale_profile_is_dead_at_the_host():
    src, tick = _stack()
    _goto(src, 5000)
    out = tick.run(_inp(9000, perception=_open(5000)))    # 4 s old frame
    assert out.vx == 0.0 and out.limiter == "free_space" and out.freshness == "failed"


def test_estop_suspends_and_release_reevaluates():
    """mutant: never call on_preempted -> nav_state stays follow under estop
    -> red."""
    src, tick = _stack()
    _goto(src, 5000)
    assert _drive_until_moving(tick, 5000).vx > 0.0
    out = tick.run(_inp(6000, estop=True))
    assert out.suspended and out.nav_state == NavState.SUSPENDED.value
    assert (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0) and out.limiter == "estop"
    assert out.source == "hold"
    out2 = tick.run(_inp(6050, estop=True))
    assert out2.raw_vx == 0.0                       # never asked while suspended
    out3 = _drive_until_moving(tick, 6100)
    assert not tick.suspended and out3.nav_state == NavState.FOLLOW.value and out3.vx > 0.0


def test_teleop_preempts_rns_tr_rns_1():
    src, tick = _stack()
    _goto(src, 5000)
    _drive_until_moving(tick, 5000)
    out = tick.run(_inp(6000, teleop=True))
    assert out.suspended and out.source == "hold" and out.vx == 0.0
    out = _drive_until_moving(tick, 6100)
    assert out.source == "rns_avoid" and out.vx > 0.0


def test_health_never_and_no_fix_are_vetoes():
    src, tick = _stack()
    _goto(src, 5000)
    out = tick.run(_inp(5000, health=NEVER))
    assert out.vx == 0.0 and out.limiter == "health"
    out = tick.run(_inp(5050, i_fix=None, pose=None))
    assert out.vx == 0.0 and out.limiter == "rtk"


def test_ctor_refuses_unset_limits():
    src, _ = _stack()
    with pytest.raises(NavTickConfigError):
        NavTick(src, P1Arbiter(), v_nom_mps=1.0, wz_max_rps=None,
                spec_max_vx_mps=2.0, holonomic=True)


# ---- adapter entries ---------------------------------------------------------

def test_route_origin_and_kind_mapping():
    """mutant: pass Origin.RELMOVE in load_route -> red."""
    src, _ = _stack()
    src.load_route(_route([(5.0, 0.0)]), 5000)
    assert src.origin is Origin.ROUTE and src.rns._mission.kind is MissionKind.GOTO
    assert src.rns._mission.origin is Origin.ROUTE
    src.load_route(_route([(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)], rev=2), 5100)
    assert src.rns._mission.kind is MissionKind.PATH
    assert src.route is not None and src.route.route_rev == 2


def test_route_replacement_is_audited_as_superseded_not_failed():
    src, _ = _stack()
    src.load_route(_route([(5.0, 0.0)], rev=1), 5000)
    src.load_route(_route([(6.0, 0.0)], rev=2), 5100)
    kinds = [r.kind for r in src.rns.audit.drain()]
    assert "superseded" in kinds
    assert src.take_failure() is None


def test_cancel_audits_and_returns_to_idle_without_failure():
    src, _ = _stack()
    assert src.cancel(5000) is False
    _goto(src, 5000)
    assert src.mission_loaded()
    assert src.cancel(5100) is True
    assert src.nav_state() is NavState.IDLE
    assert "cancelled" in [r.kind for r in src.rns.audit.drain()]
    assert src.take_failure() is None


def test_goto_uses_relmove_origin_and_config_radius():
    src, _ = _stack()
    _goto(src, 5000)
    assert src.origin is Origin.RELMOVE and src.rns._mission.origin is Origin.RELMOVE
    assert src.goal is not None and src.route is None


def test_max_profile_obstacle_avoid_caps_the_nominal():
    """11 S3.6 max_profile: with the tier table the nominal drops to 0.5 and
    the gate names health. mutant: ignore max_profile -> vx above 0.5 -> red."""
    src, _ = _stack()
    tick = NavTick(src, P1Arbiter(dwell_ms=200), v_nom_mps=1.0, wz_max_rps=1.2,
                   spec_max_vx_mps=2.0, holonomic=True, v_obstacle_avoid_mps=0.5)
    _goto(src, 5000)
    oa = HealthView(1.0, True, "obstacle_avoid", "ok", 100)
    outs = [tick.run(_inp(5000 + 50 * k, health=oa)) for k in range(10)]
    moving = [o for o in outs if o.vx > 0.0]
    assert moving and all(o.vx <= 0.5 + 1e-9 for o in moving)
    assert all(o.profile == "obstacle_avoid" and o.v_max == pytest.approx(0.5) for o in outs)


def test_ctrl_state_words_follow_12_s11():
    """WAIT_GRANT before the first grant, WAIT_INPUT before inputs, READY on
    hold, ACTIVE with rns_avoid driving, SAFE_STOP once a granted/ready loop
    loses the grant or an input. mutant: ACTIVE under the health veto -> red."""
    from xbrain.p1_motion.ctrl_loop import CtrlState
    from xbrain.p1_motion.nav.nav_tick import ctrl_state_for
    src, tick = _stack()
    inp = _inp(5000, health=NEVER)
    out = tick.run(inp)
    assert ctrl_state_for(inp, out, health_ever_ok=False, inputs_ever_ready=False) is CtrlState.WAIT_GRANT
    assert ctrl_state_for(inp, out, health_ever_ok=True, inputs_ever_ready=True) is CtrlState.SAFE_STOP
    inp = _inp(5050, perception=PerceptionSnapshot())
    out = tick.run(inp)
    assert ctrl_state_for(inp, out, health_ever_ok=True, inputs_ever_ready=False) is CtrlState.WAIT_INPUT
    assert ctrl_state_for(inp, out, health_ever_ok=True, inputs_ever_ready=True) is CtrlState.SAFE_STOP
    inp = _inp(5100)
    out = tick.run(inp)
    assert out.source == "hold"
    assert ctrl_state_for(inp, out, health_ever_ok=True, inputs_ever_ready=True) is CtrlState.READY
    _goto(src, 5100)
    out = _drive_until_moving(tick, 5150)
    inp = _inp(5150 + 50 * 9)
    assert ctrl_state_for(inp, out, health_ever_ok=True, inputs_ever_ready=True) is CtrlState.ACTIVE


def test_wz_never_exceeds_wz_max_on_the_wire():
    src, tick = _stack()
    _goto(src, 5000, endpoint=(0.0, 8.0))           # goal at +90 deg: a hard turn
    outs = [tick.run(_inp(5000 + 50 * k)) for k in range(20)]
    assert all(abs(o.wz) <= 1.2 + 1e-9 for o in outs)


def test_estop_without_mission_is_not_loaded():
    """RNS keeps a SUSPENDED word after on_preempted with no mission; the
    adapter's own flag must still say 'nothing loaded'. mutant: derive
    mission_loaded from nav_state -> red."""
    src, tick = _stack()
    tick.run(_inp(5000, estop=True))
    tick.run(_inp(5050, estop=False))
    assert not src.mission_loaded()
    assert src.cancel(5100) is False
    _goto(src, 5100)
    assert src.mission_loaded()
    outs = [tick.run(_inp(5100 + 50 * k)) for k in range(400)]
    assert any(o.vx > 0.0 for o in outs)
    # arrival is consumed through the adapter latch -> not loaded any more
    if src.take_arrival():
        assert not src.mission_loaded()
