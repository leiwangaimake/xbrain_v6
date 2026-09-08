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


def _cfg():
    # a minimal valid rns.yaml snapshot for the follow spine.
    return {"rns": {
        "route": {"lookahead_k": 1.0, "lookahead_min_m": 1.0,
                  "lookahead_max_m": 4.0, "max_deviation_m": 10.0},
        "speed": {"dev_e0_m": 0.5, "dev_g_min": 0.1},
    }}


def _mission():
    pts = [(i * 0.5, 0.0) for i in range(21)]   # straight path 0..10
    return Mission(MissionKind.PATH, Origin.ROUTE, pts,
                   search_window=8, arrival_radius_m=1.0, max_deviation_m=10.0)


def test_no_mission_is_inert():
    # P0.4 no-smoke survives assembly: no mission -> False / None.
    s = RnsSource(cfg=_cfg())
    assert s.is_active(Ctx()) is False
    assert s.compute(Ctx(pose_xy=(0.0, 0.0), v_nom_mps=2.0)) is None


def test_mission_loaded_runs_follow_spine():
    s = RnsSource(cfg=_cfg())
    s.load_mission(_mission())
    ctx = Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)
    assert s.is_active(ctx) is True
    out = s.compute(ctx)
    assert out is not None
    assert out.vx.value > 0.0     # moving forward along the path


def test_arrival_gives_zero_velocity():
    s = RnsSource(cfg=_cfg())
    s.load_mission(_mission())
    # near the endpoint (10.0): dist 0.5 < arrival_radius 1.0 -> arrived -> zero
    out = s.compute(Ctx(pose_xy=(9.5, 0.0), v_nom_mps=2.0))
    assert out.vx.value == 0.0
    assert out.wz == 0.0


def test_estop_suspension_zeroes_output_at_source():
    # A-ES-1 at the source: suspended -> compute returns None (no output), even
    # with a mission that would otherwise move.
    s = RnsSource(cfg=_cfg())
    s.load_mission(_mission())
    ctx = Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)
    assert s.compute(ctx) is not None
    s.on_preempted(ctx)             # estop/preempt
    assert s.is_active(ctx) is False   # suspended -> not active
    assert s.compute(ctx) is None      # zero output


def test_ctx_without_pose_returns_none():
    # ctx not yet carrying pose (pre-P7.1) -> None, never a guessed velocity.
    s = RnsSource(cfg=_cfg())
    s.load_mission(_mission())
    assert s.compute(Ctx(v_nom_mps=2.0)) is None   # no pose


def test_deviation_slows_speed():
    # off the path -> the deviation cap lowers speed vs on the path.
    s = RnsSource(cfg=_cfg())
    s.load_mission(_mission())
    on = s.compute(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0))
    off = s.compute(Ctx(pose_xy=(2.0, 3.0), v_nom_mps=2.0))
    assert off.vx.value < on.vx.value


def test_release_resumes_follow_fresh():
    # A-ES-2 at the source: release -> FOLLOW resumes with a FRESH compute, not a
    # replayed pre-suspend velocity. After release, compute runs the spine again.
    s = RnsSource(cfg=_cfg())
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
    s = RnsSource(cfg=_cfg())
    s.load_mission(_mission())
    assert s.is_active(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)) is True
    s.clear_mission()
    assert s.is_active(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)) is False
    assert s.compute(Ctx(pose_xy=(2.0, 0.0), v_nom_mps=2.0)) is None
