"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_sil_avoidance.py
Brief: S2 closed-loop avoidance scenarios -- REAL RNS + synthesized perception

Description:
The first golden scenes (RNS_TODO M-3 lineage), run in-process: the assembled
source.compute consumes SilWorld's synthesized perception and must
  - detour around a rock blocking the straight goto line and still arrive;
  - STOP for a dynamic person in the corridor (WAIT_DYNAMIC), resume when the
    person leaves, and arrive;
  - fail blocked_by_dynamic when the person never leaves (RNS-N-16 budget);
  - never collide (min distance to any obstacle stays above a margin) while
    detouring.
Each scenario pins state-machine transitions through nav_state(), so a shell
that "arrives" by driving through the rock reddens on the collision check.
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
from xbrain.p1_motion.rns.types import MissionKind, NavFailReason, NavState, Origin

CFG = yaml.safe_load((ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))
DT = 0.05


class Ctx:
    def __init__(self, world, snap, now_ms):
        self.pose_xy = (world.rx, world.ry)
        self.yaw_rad = world.ryaw
        self.v_nom_mps = 1.0
        self.wz_max_rps = 1.2
        self.perception = snap
        self.now_mono_ms = now_ms


def _mission(points, kind=MissionKind.GOTO):
    rc = CFG["rns"]["route"]
    return Mission(kind, Origin.ROUTE, points,
                   search_window=rc["search_window"],
                   arrival_radius_m=rc["arrival_radius_m"],
                   max_deviation_m=rc["max_deviation_m"])


def _tick_until(world, rns, max_ticks, on_tick=None):
    """Run the loop; returns (arrived_tick, states_seen, min_clearance)."""
    states = set()
    min_clear = math.inf
    for t in range(max_ticks):
        now = t * 50
        snap = world.synth_snapshot(now)
        cand = rns.compute(Ctx(world, snap, now))
        states.add(rns.nav_state())
        if cand is not None:
            world.step_robot(cand.vx.value, cand.vy.value, cand.wz, DT)
        world.step_obstacles(DT)
        # collision monitor: distance to every obstacle centre minus its radius
        for o in world.obstacles.values():
            if o.kind == "wall":
                continue
            r = {"person": 0.3, "car": 1.0, "rock": 0.5, "cone": 0.25}[o.kind]
            d = math.hypot(world.rx - o.x, world.ry - o.y) - r
            min_clear = min(min_clear, d)
        if on_tick:
            on_tick(t, world, rns)
        if rns.take_arrival():
            return t, states, min_clear
    return None, states, min_clear


def test_detour_around_rock_and_arrive():
    # a rock dead on the straight line to the goal: must DETOUR, not plough
    # through, and still arrive.
    world = SilWorld()
    world.ryaw = 0.0
    world.add_obstacle("rock", 4.0, 0.0)          # on the line to (8, 0)
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(8.0, 0.0)]))
    tick, states, min_clear = _tick_until(world, rns, 1200)
    assert tick is not None, "never arrived (stuck at the rock?)"
    assert NavState.DETOUR in states, "no DETOUR state -- did it drive through?"
    assert min_clear > 0.1, "collided with the rock (clearance %.2f)" % min_clear


def test_thread_wall_gap_and_arrive():
    # two wall segments with a ~2.4 m gap on the way: thread it and arrive.
    world = SilWorld()
    world.ryaw = 0.0
    world.add_obstacle("wall", 4.0, -6.0, 4.0, -1.2)
    world.add_obstacle("wall", 4.0, 1.2, 4.0, 6.0)
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(8.0, 0.0)]))
    tick, states, min_clear = _tick_until(world, rns, 1200)
    assert tick is not None, "never arrived through the gap"
    assert min_clear > -0.1   # walls not in the circle monitor; sanity only


def test_dynamic_person_stops_then_resumes():
    # a person in the corridor -> STOP (WAIT_DYNAMIC); teleport the person away
    # after 2 s -> must resume and arrive.
    world = SilWorld()
    world.ryaw = 0.0
    oid = world.add_obstacle("person", 3.0, 0.0)
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(8.0, 0.0)]))

    def mover(t, w, r):
        if t == 40:                                # after 2 s, person leaves
            w.obstacles[oid].x, w.obstacles[oid].y = 3.0, 8.0
    tick, states, min_clear = _tick_until(world, rns, 1200, on_tick=mover)
    assert NavState.WAIT_DYNAMIC in states, "never stopped for the person"
    assert tick is not None, "did not resume/arrive after the person left"
    assert min_clear > 0.2, "got too close to the person"


def test_person_never_leaves_blows_wait_budget():
    # person parked in the corridor forever -> blocked_by_dynamic after the
    # 20 s budget (RNS-N-16), mission cleared, failure latched with the track.
    world = SilWorld()
    world.ryaw = 0.0
    world.add_obstacle("person", 3.0, 0.0)
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(8.0, 0.0)]))
    tick, states, _ = _tick_until(world, rns, 600)   # 30 s > 20 s budget
    assert tick is None, "arrived through a standing person?!"
    f = rns.take_failure()
    assert f is not None, "wait budget never fired"
    assert f.reason is NavFailReason.BLOCKED_BY_DYNAMIC
    assert f.detail["class_name"] == "person"
    assert rns.nav_state() is NavState.IDLE          # mission cleared (S9.0.3)


def test_u_trap_escape_via_wall_follow():
    # THE S2b acceptance (RNS-T-1 in the flesh): robot deep inside a U-shaped
    # dead end, goal beyond the back wall. Candidates are all gated (three
    # walls, no visible edge), so it must WALL_FOLLOW: hug a wall, round the
    # arm's outer side (convex corners), come back to the line beyond the back
    # wall, leave by the 2' arc condition, and arrive. A pure-reactive shell
    # oscillates in the U forever -- the tick budget kills it.
    world = SilWorld()
    world.rx, world.ry, world.ryaw = 2.5, 0.0, 0.0     # inside the U
    world.add_obstacle("wall", 4.0, -3.0, 4.0, 3.0)    # back wall
    world.add_obstacle("wall", 1.0, 3.0, 4.0, 3.0)     # upper arm
    world.add_obstacle("wall", 1.0, -3.0, 4.0, -3.0)   # lower arm
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(10.0, 0.0)]))
    tick, states, _ = _tick_until(world, rns, 6000)     # 5 sim-min budget
    assert NavState.WALL_FOLLOW in states, \
        "never entered wall-follow inside the U"
    assert tick is not None, \
        "never escaped the U (stuck: state history %s)" % sorted(
            s.value for s in states)
    d = math.hypot(world.rx - 10.0, world.ry - 0.0)
    assert d < 1.0


def test_wall_follow_audit_trail():
    # S9.3: the ring audit must hold the wall enter/exit records after a U run.
    world = SilWorld()
    world.rx, world.ry, world.ryaw = 2.5, 0.0, 0.0
    world.add_obstacle("wall", 4.0, -3.0, 4.0, 3.0)
    world.add_obstacle("wall", 1.0, 3.0, 4.0, 3.0)
    world.add_obstacle("wall", 1.0, -3.0, 4.0, -3.0)
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(10.0, 0.0)]))
    _tick_until(world, rns, 6000)
    kinds = [r.kind for r in rns.audit.drain()]
    assert "wall_enter" in kinds
    assert "wall_exit" in kinds or "wall_fail" in kinds


def test_goto_behind_robot_does_not_false_fire_watchdog():
    # FIELD BUG regression (user, 2026-09-10): a goto BEHIND the robot spends
    # its first seconds turning (big heading error, cos-tapered crawl, flat arc
    # projection). That heading-limited phase must be EXEMPT from the watchdog
    # timing domain (20 S7.3A: argmax limiter heading/rtk does not count as
    # lost). The first assembly passed limiter=None always and false-fired.
    #
    # To make the exemption the ONLY thing that saves this scene (a bigger
    # window would mask its absence -- the first version of this test did, and
    # its mutant survived), inject a SHORT window (6 s) and a SLOW turn rate
    # (wz_max 0.15 -> turning pi takes ~21 s >> window). With the exemption the
    # whole turn is heading-limited and untimed; without it (mutant: limiter =
    # None) the watchdog fires ~6 s in -> reddens.
    import copy
    cfg = copy.deepcopy(CFG)
    cfg["rns"]["watchdog"]["window_s"] = 6.0
    world = SilWorld()
    world.rx, world.ry, world.ryaw = 0.0, 0.0, 0.0   # facing +x
    rns = RnsSource(cfg=cfg, r_eff_m=0.5)
    rc = cfg["rns"]["route"]
    rns.load_mission(Mission(MissionKind.GOTO, Origin.ROUTE, [(-8.0, 0.0)],
                     search_window=rc["search_window"],
                     arrival_radius_m=rc["arrival_radius_m"],
                     max_deviation_m=rc["max_deviation_m"]))

    class SlowTurnCtx:
        def __init__(self, w, snap, now):
            self.pose_xy = (w.rx, w.ry)
            self.yaw_rad = w.ryaw
            self.v_nom_mps = 1.0
            self.wz_max_rps = 0.15          # crawl turn: pi takes ~21 s
            self.perception = snap
            self.now_mono_ms = now

    fired = None
    for t in range(700):                     # 35 s: covers turn + fire window
        now = t * 50
        snap = world.synth_snapshot(now)
        cand = rns.compute(SlowTurnCtx(world, snap, now))
        if cand is not None:
            world.step_robot(cand.vx.value, cand.vy.value, cand.wz, DT)
        f = rns.take_failure()
        if f is not None:
            fired = f
            break
    assert fired is None, (
        "watchdog false-fired %s during the heading-limited turn phase"
        % fired.reason.value)
