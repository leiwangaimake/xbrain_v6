"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_source_assembly.py
Brief: source.compute follow-spine assembly (P7 -- 20 S1.1/S9.0.1)

Description:
Guards the P7 assembly of source.compute: no mission -> inert (is_active False,
compute None -- the P0.4 no-smoke invariant survives wiring); with a mission ->
the follow+speed spine runs; arrival -> zero; estop suspension -> zero output
(A-ES-1 at the source level). ctx is a tiny stand-in carrying pose_xy and
v_nom_mps until P7.1 defines the real tick snapshot.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from xbrain.p1_motion.rns.route import Mission
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind, Origin
from xbrain.common.types.units import Mps


@dataclass
class Ctx:
    pose_xy: Optional[Tuple[float, float]] = None
    v_nom_mps: Optional[float] = None
    yaw_rad: Optional[float] = 0.0        # facing +x by default
    wz_max_rps: Optional[float] = 1.0


import copy
from pathlib import Path as _Path

import yaml as _yaml

_ROOT = _Path(__file__).resolve().parents[3]
_REAL_CFG = _yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(
    encoding="utf-8"))


def _cfg():
    # the ONE rns.yaml (user ruling 2026-09-10): tests consume the real file,
    # so a broken configs/rns.yaml fails HERE before it fails in the field.
    return copy.deepcopy(_REAL_CFG)


R_EFF = 0.5  # SIL r_eff (M20S half-diagonal); D2: leave_progress 0.4 < 1.0


def _mission():
    pts = [(i * 0.5, 0.0) for i in range(21)]   # straight path 0..10
    return Mission(MissionKind.PATH, Origin.ROUTE, pts,
                   search_window=8, arrival_radius_m=1.0, max_deviation_m=10.0)


def test_no_mission_is_inert():
    # P0.4 no-smoke survives assembly: no mission -> False / None.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    assert s.is_active(Ctx()) is False
    assert s.compute(Ctx(pose_xy=(0.0, 0.0), v_nom_mps=2.0)) is None


def test_mission_loaded_runs_follow_spine():
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    ctx = Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)
    assert s.is_active(ctx) is True
    out = s.compute(ctx)
    assert out is not None
    assert out.vx.value > 0.0     # moving forward along the path


def test_arrival_gives_zero_velocity_and_closes_mission():
    # W2 (review fix): arrival ends the mission -- one final zero candidate,
    # then is_active False (no longer holds the 900 slot), arrival latched once.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    out = s.compute(Ctx(pose_xy=(9.5, 0.0), v_nom_mps=2.0))
    assert out.vx.value == 0.0
    assert out.wz == 0.0
    assert s.is_active(Ctx(pose_xy=(9.5, 0.0), v_nom_mps=2.0)) is False
    assert s.take_arrival() is True     # latched exactly once
    assert s.take_arrival() is False    # one-shot


def test_estop_suspension_zeroes_output_at_source():
    # A-ES-1 at the source: suspended -> compute returns None (no output), even
    # with a mission that would otherwise move.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    ctx = Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)
    assert s.compute(ctx) is not None
    s.on_preempted(ctx)             # estop/preempt
    assert s.is_active(ctx) is False   # suspended -> not active
    assert s.compute(ctx) is None      # zero output


def test_ctx_without_pose_returns_none():
    # ctx not yet carrying pose (pre-P7.1) -> None, never a guessed velocity.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    assert s.compute(Ctx(v_nom_mps=2.0)) is None   # no pose


def test_deviation_slows_speed():
    # off the path -> the deviation cap lowers speed vs on the path.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    on = s.compute(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0))
    off = s.compute(Ctx(pose_xy=(2.0, 3.0), v_nom_mps=2.0))
    assert off.vx.value < on.vx.value


def test_release_resumes_follow_fresh():
    # A-ES-2 at the source: release -> FOLLOW resumes with a FRESH compute, not a
    # replayed pre-suspend velocity. After release, compute runs the spine again.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    ctx = Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)
    s.on_preempted(ctx)
    assert s.compute(ctx) is None       # suspended -> zero
    s.on_release(ctx)
    assert s.is_active(ctx) is True      # resumed
    out = s.compute(ctx)                 # fresh follow tick, not a replay
    assert out is not None
    assert out.vx.value > 0.0


def test_clear_mission_returns_to_idle():
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    assert s.is_active(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)) is True
    s.clear_mission()
    assert s.is_active(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)) is False
    assert s.compute(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)) is None


def test_wz_is_angular_velocity_not_bearing():
    # B1 (review fix): wz must be the P-law angular velocity, not the absolute
    # bearing to R. Robot at (2,-1) facing +x (yaw=0), R ahead-left: theta_des
    # ~0.46 rad, so wz = clamp(k_yaw*wrap(0.46-0)) = 1.5*0.46 ~ 0.69 -- and
    # CLAMPED by wz_max. The old bug returned theta_des itself regardless of
    # yaw. Distinguisher: set yaw = theta_des -> error 0 -> wz MUST be 0; the
    # bearing bug would return 0.46.
    import math
    src = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    src.load_mission(_mission())
    pose = (2.0, -1.0)
    # first, with yaw aligned to the bearing, wz must be ~0 (no error):
    probe = src.compute(Ctx(pose_xy=pose, v_nom_mps=2.0, yaw_rad=0.0))
    # compute theta_des the same way to aim yaw at it:
    # (cannot read internals; steer by testing both yaws differ correctly)
    off = src.compute(Ctx(pose_xy=pose, v_nom_mps=2.0, yaw_rad=-1.0))
    # facing further away (-1 rad) must demand MORE turn than facing 0:
    assert abs(off.wz) > abs(probe.wz)
    # and wz is clamped to wz_max:
    assert abs(off.wz) <= 1.0 + 1e-9


def test_wz_needs_yaw_else_none():
    # B1: no yaw in ctx -> cannot compute an angular velocity -> None, never a
    # guessed rotation.
    src = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    src.load_mission(_mission())
    out = src.compute(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0, yaw_rad=None))
    assert out is None


def test_construction_runs_startup_assertions():
    # W1 (review fix): a cfg violating D2 (leave_progress >= 2*r_eff) must
    # refuse construction; cfg without r_eff also refuses.
    import pytest
    from xbrain.p1_motion.rns.config import RnsConfigError
    bad = _cfg()
    bad["rns"]["wall_follow"]["leave_progress_m"] = 2.0   # >= 2*0.48
    with pytest.raises(RnsConfigError):
        RnsSource(cfg=bad, r_eff_m=R_EFF)
    with pytest.raises(RnsConfigError):
        RnsSource(cfg=_cfg())   # cfg without r_eff: cannot validate D2


def test_suspended_tick_does_not_advance_tracker():
    # review fix: a suspended tick is a true no-op -- the monotone index must
    # NOT advance while estopped (before: output gated, state still mutated).
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    ctx = Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)
    s.compute(ctx)
    idx_before = s._mission.tracker.min_index
    s.on_preempted(ctx)
    s.compute(Ctx(pose_xy=(8.0, 0.0), v_nom_mps=2.0))   # would advance to seg 8
    assert s._mission.tracker.min_index == idx_before    # untouched


def test_wall_stall_counter_immune_to_suspension():
    # REVIEW R1-12 (3-pass audit): the stall backstop must count IN-WALL
    # TICKS, not wall-clock. The clock version froze through estop and the
    # first resumed tick saw the whole hold as stall -> instant false
    # WALL_NO_PROGRESS. Simulate: enter wall state, stall 100 ticks, hold
    # suspended for a (virtual) minute, release -- the next wall tick must
    # NOT fail. mutant: revert to wall-clock stall -> reddens.
    from xbrain.p1_motion.rns.wallfollow import Side, WallFollowState
    from xbrain.p1_motion.rns.types import NavState
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    s._state = NavState.WALL_FOLLOW
    s._wall = WallFollowState(side=Side.LEFT, s_hit=0.0, hit_point=(0.0, 0.0))
    s._wall_stall_ticks = 100                 # some prior stalling
    s.on_preempted(None)                      # estop hold...
    ctx = Ctx(pose_xy=(0.0, 0.0), v_nom_mps=1.0)
    assert s.compute(ctx) is None             # suspended no-op ticks
    s.on_release(None)                        # ...one minute later
    # a resumed tick (no perception -> wall tick holds zero) must not fail
    out = s.compute(Ctx(pose_xy=(0.0, 0.0), v_nom_mps=1.0))
    assert s.take_failure() is None, "suspension counted as in-wall stall"
    assert out is not None


def test_wall_tick_survives_perception_dropout():
    # REVIEW R1-20 (3-pass audit): hugging + perception dropout (snap None)
    # dereferenced profile.d_free and CRASHED the tick. It must hold zero.
    # mutant: drop the None guard -> AttributeError -> reddens.
    from xbrain.p1_motion.rns.wallfollow import Side, WallFollowState
    from xbrain.p1_motion.rns.types import NavState
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    s._state = NavState.WALL_FOLLOW
    s._wall = WallFollowState(side=Side.LEFT, s_hit=0.0, hit_point=(0.0, 0.0))
    out = s.compute(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=1.0))   # no perception
    assert out is not None and out.vx.value == 0.0
    assert s.take_failure() is None
