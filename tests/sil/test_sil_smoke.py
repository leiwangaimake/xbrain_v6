"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_sil_smoke.py
Brief: SIL closed-loop smoke -- REAL RnsSource driving the sim world

Description:
The in-process version of the SIL loop (no http): SilWorld synthesizes
perception, the REAL RnsSource (configs/rns.yaml, startup assertions on) is
ticked at 20 Hz, kinematics integrate its VelocityCandidate. Pins:
  - goto converges: the robot reaches the target within a bounded tick count
    and take_arrival() latches (B1 P-law + W2 closure working end to end);
  - path missions walk the polyline to the far end;
  - the raycaster produces PROF-1-consistent bins (d_free < d_block).
This is the harness every S2 avoidance scenario builds on.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "sil"))

from sil_world import SilWorld
from xbrain.p1_motion.rns.route import Mission
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind, Origin

CFG = yaml.safe_load((ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))
DT = 0.05


class Ctx:
    def __init__(self, world, snap):
        self.pose_xy = (world.rx, world.ry)
        self.yaw_rad = world.ryaw
        self.v_nom_mps = 1.0
        self.wz_max_rps = 1.2
        self.perception = snap


def _mk_source():
    return RnsSource(cfg=CFG, r_eff_m=0.5)


def _mission(points, kind=MissionKind.GOTO):
    rc = CFG["rns"]["route"]
    return Mission(kind, Origin.ROUTE, points,
                   search_window=rc["search_window"],
                   arrival_radius_m=rc["arrival_radius_m"],
                   max_deviation_m=rc["max_deviation_m"])


def _run(world, rns, max_ticks):
    """Tick until arrival or budget; returns ticks used (None = no arrival)."""
    for t in range(max_ticks):
        snap = world.synth_snapshot(now_ms=t * 50)
        cand = rns.compute(Ctx(world, snap))
        if cand is not None:
            world.step_robot(cand.vx.value, cand.vy.value, cand.wz, DT)
        world.step_obstacles(DT)
        if rns.take_arrival():
            return t
    return None


def test_goto_converges_and_arrives():
    world = SilWorld()
    rns = _mk_source()
    rns.load_mission(_mission([(5.0, 3.0)]))
    ticks = _run(world, rns, max_ticks=400)   # 20 s budget for a ~5.8 m goto
    assert ticks is not None, "goto never arrived"
    d = math.hypot(world.rx - 5.0, world.ry - 3.0)
    assert d < CFG["rns"]["route"]["arrival_radius_m"] + 0.1


def test_path_walks_to_far_end():
    world = SilWorld()
    rns = _mk_source()
    pts = [(0.0, 3.0), (3.0, 5.0), (6.0, 5.0), (8.0, 3.0)]
    rns.load_mission(_mission(pts, kind=MissionKind.PATH))
    ticks = _run(world, rns, max_ticks=800)
    assert ticks is not None, "path never finished"
    d = math.hypot(world.rx - 8.0, world.ry - 3.0)
    assert d < 1.0


def test_raycast_profile_is_prof1_consistent():
    # PROF-1 on the synthesized profile: where both set, d_free < d_block.
    world = SilWorld()
    world.add_obstacle("rock", 0.0, 3.0)
    world.add_obstacle("wall", -2.0, 5.0, 2.0, 5.0)
    snap = world.synth_snapshot(1000)
    p = snap.profile
    assert p.n_bins == 181
    hit = 0
    for df, db in zip(p.d_free, p.d_block):
        if db is not None:
            hit += 1
            assert df is not None and df < db, "PROF-1 violated by the synth"
    assert hit > 0, "raycast saw nothing (scene has two obstacles)"


def test_dynamic_obstacle_moves_and_reports_velocity():
    world = SilWorld()
    oid = world.add_obstacle("person", 3.0, 3.0)
    world.toggle_dynamic(oid)
    x0 = world.obstacles[oid].x
    for _ in range(20):
        world.step_obstacles(DT)
    moved = math.hypot(world.obstacles[oid].x - x0, world.obstacles[oid].y - 3.0)
    assert 0.5 < moved < 1.5          # ~1 m in 1 s (person speed 1 m/s)
    snap = world.synth_snapshot(1000)
    persons = [o for o in snap.objects.objects if o.class_name == "person"]
    assert persons and persons[0].velocity_status == "moving"
    assert abs(math.hypot(*persons[0].velocity_xy) - 1.0) < 0.01
