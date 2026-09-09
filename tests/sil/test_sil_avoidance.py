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
        self.holonomic = True


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
        # collision monitor -- EXACT body clearance (upgraded after the user's
        # SIL audit caught a real drive-through): cars are OBBs (2.0 x 1.2),
        # circles for the rest; subtract the 0.46 m body half-diagonal so
        # min_clear < 0 means TRUE overlap.
        for o in world.obstacles.values():
            if o.kind == "wall":
                continue
            if o.kind == "car":
                ca, sa = math.cos(-o.heading), math.sin(-o.heading)
                lx = ca * (world.rx - o.x) - sa * (world.ry - o.y)
                ly = sa * (world.rx - o.x) + ca * (world.ry - o.y)
                dx = max(abs(lx) - 1.0, 0.0)
                dy = max(abs(ly) - 0.6, 0.0)
                d = math.hypot(dx, dy) - 0.46
            else:
                r = {"person": 0.3, "rock": 0.5, "cone": 0.25}[o.kind]
                d = math.hypot(world.rx - o.x, world.ry - o.y) - r - 0.46
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


def test_car_wall_detour_does_not_oscillate():
    # FIELD BUG regression #2 (user 2026-09-10, flight-recorder trace): a row of
    # 7 parked cars forms a wall; detouring around its south end, the robot spun
    # toward the subgoal, which swung the lookahead R OUT of the 90-deg FOV --
    # and _ahead_blocked treated out-of-view as "clear" (empty scan window ->
    # False), so DETOUR exited, FOLLOW re-blocked, DETOUR re-entered... 254
    # detour_enters at one spot, moving_ratio 0.117. Unseen is NOT clear
    # (RNS-I-1 spirit): the DETOUR exit must require CONFIRMED-clear toward R.
    # mutant: revert unseen_is_blocked at the exit call -> stuck again -> red.
    world = SilWorld()
    for y, x in ((0.00, -14.2), (-1.63, -14.1), (-3.27, -14.2), (-5.23, -14.5),
                 (-7.17, -14.7), (-9.17, -14.5), (-11.27, -14.7)):
        world.add_obstacle("car", x, y)
    world.rx, world.ry, world.ryaw = -12.6, -8.0, math.pi   # the stuck pose
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(-20.07, -15.1)]))
    tick, states, _ = _tick_until(world, rns, 3600)          # 3 min budget
    enters = sum(1 for r in rns.audit.drain() if r.kind == "detour_enter")
    assert tick is not None, \
        "stuck at the car wall (detour_enters=%d)" % enters
    assert enters < 20, \
        "detour oscillation: %d enters (healthy runs use a handful)" % enters


def test_long_car_wall_hand_on_wall_and_no_pin_stall():
    # FIELD BUG regressions #3+#4 (user 2026-09-10, second stall): an 11-car
    # 17 m wall, goal south-west behind it.
    #  #3 pin-stall: host speed gate zeroes vx below 1.25 m while the concave-
    #     corner line sat at 0.8 -- in [0.8,1.25) RNS kept PD-walking in place
    #     forever (v=0, wz wiggling, distance-metered failure criteria blind).
    #     Fixed: front_stop 1.35 > host zero-line + wall-clock stall backstop.
    #  #4 hand-on-wall inversion: "goal left -> hug left" walked AWAY from the
    #     goal along the wall (three replay starts all went north). Fixed: goal
    #     side and wall hand are OPPOSITE (goal left = wall on right hand).
    # mutant: revert either -> this reddens (stuck forever or walks north).
    world = SilWorld()
    for x, y in ((-7.33, 1.33), (-7.53, -0.77), (-7.47, -2.83), (-7.90, -4.80),
                 (-7.70, -6.07), (-7.83, -7.57), (-8.07, -9.07), (-8.00, -10.83),
                 (-8.20, -12.50), (-8.43, -13.83), (-7.97, -15.97)):
        world.add_obstacle("car", x, y)
    world.rx, world.ry, world.ryaw = -6.2, -12.0, math.pi   # the user's stall
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(-15.07, -19.8)]))
    tick, states, _ = _tick_until(world, rns, 2400)          # 2 min budget
    assert tick is not None, "stuck/failed at the long car wall again"
    d = math.hypot(world.rx + 15.07, world.ry + 19.8)
    assert d < 1.2
    # field bug #5 (tangent side pick): entering 1.6 m from the SOUTH end, the
    # wall lap must be short. The body-axis pick walked the wall 34.5 m the
    # WRONG way (north) and only survived on the deviation exemption. mutant:
    # revert to the body-axis pick -> lap > 20 m -> reddens.
    laps = [r.detail.get("followed_m", 0.0) for r in rns.audit.drain()
            if r.kind == "wall_exit"]
    assert all(l < 20.0 for l in laps), "wall lap too long: %s" % laps


def test_wall_entry_side_pick_survives_swung_yaw():
    # field bug #5: entering wall-follow with the yaw already swung (post-DETOUR)
    # toward NORTH while the goal is SOUTH used the unstable body-y sign and
    # hugged the wrong way, then died on max_deviation. The robust pick compares
    # wall-walk bearings against the goal bearing -- yaw-swing must not flip it.
    # mutant: revert to the body-y sign pick -> hugs north -> reddens.
    world = SilWorld()
    for x, y in ((-7.33, 1.33), (-7.53, -0.77), (-7.47, -2.83), (-7.90, -4.80),
                 (-7.70, -6.07), (-7.83, -7.57), (-8.07, -9.07), (-8.00, -10.83),
                 (-8.20, -12.50), (-8.43, -13.83), (-7.97, -15.97)):
        world.add_obstacle("car", x, y)
    # yaw swung NORTH-WEST (2.6 rad) as if mid-DETOUR; goal is SOUTH-WEST.
    world.rx, world.ry, world.ryaw = -6.0, -11.0, 2.6
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(-15.07, -19.8)]))
    tick, states, _ = _tick_until(world, rns, 2400)
    assert tick is not None, "swung-yaw entry hugged the wrong way again"
    d = math.hypot(world.rx + 15.07, world.ry + 19.8)
    assert d < 1.2


def test_wall_follow_exempt_from_deviation_failure():
    # design-seam ruling (20 S2.7 v1.17): while WALL_FOLLOW, the 10 m deviation
    # bound must NOT fire (S7.6 owns the excursion budget); it resumes on
    # FOLLOW. Direct check: force the wall state with a far-off-line pose and
    # tick once -- no failure latched. mutant: drop the state check -> red.
    from xbrain.p1_motion.rns.source import RnsSource as _R
    from xbrain.p1_motion.rns.wallfollow import Side, WallFollowState
    from xbrain.p1_motion.rns.types import NavState
    world = SilWorld()
    world.add_obstacle("car", -8.0, -12.0)
    rns = _R(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(0.0, 0.0)]))
    # anchor the goto line at (0,-1) then teleport 12 m off it in wall state.
    snap = world.synth_snapshot(0)
    rns.compute(Ctx(world, snap, 0).__class__(world, snap, 0)) if False else None
    class C:
        pose_xy = (0.0, -1.0); yaw_rad = 0.0
        v_nom_mps = 2.0; wz_max_rps = 1.2
        perception = world.synth_snapshot(0); now_mono_ms = 0
    rns.compute(C())                          # anchors the line
    rns._state = NavState.WALL_FOLLOW
    rns._wall = WallFollowState(side=Side.RIGHT, s_hit=0.0, hit_point=(0.0, -1.0))
    rns._wall_last_move_ms = None
    class C2:
        pose_xy = (12.5, -1.0); yaw_rad = 0.0   # 12.5 m off the goto line
        v_nom_mps = 2.0; wz_max_rps = 1.2
        perception = world.synth_snapshot(50); now_mono_ms = 50
    rns.compute(C2())
    assert rns.take_failure() is None, \
        "deviation fired inside WALL_FOLLOW despite the S2.7 v1.17 exemption"


def test_user_full_run_never_touches_a_car():
    # COLLISION regression (user SIL audit 2026-09-10): the exact run that
    # drove THROUGH a 0.13 m car gap (min centre distance 0.267 m = 0.19 m body
    # overlap; 195 ticks inside the U54 1 m keep-out). Root causes: the
    # clearance gate probed the SUBGOAL POINT (past the gap) instead of
    # sweeping the X->S corridor, and the class-blind margin used structure 0.3.
    # mutant: revert corridor_clearance to the point probe -> min_clear goes
    # negative -> reddens.
    world = SilWorld()
    for x, y in ((-7.33, 1.33), (-7.53, -0.77), (-7.47, -2.83), (-7.90, -4.80),
                 (-7.70, -6.07), (-7.83, -7.57), (-8.07, -9.07), (-8.00, -10.83),
                 (-8.20, -12.50), (-8.43, -13.83), (-7.97, -15.97)):
        world.add_obstacle("car", x, y)
    world.rx, world.ry, world.ryaw = 0.0, 0.0, 1.57
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(-14.47, -21.03)]))
    tick, states, min_clear = _tick_until(world, rns, 2400)
    assert tick is not None, "run failed outright"
    assert min_clear > 0.0,         "BODY OVERLAP with a car: min clearance %.3f m" % min_clear


def test_user_full_run_side_pick_at_oblique_entry():
    # the user's EXACT run (2026-09-10 trace): start (0,0), goal behind the
    # 17 m car wall. The wall entry happens at ~45 deg to the wall (post-DETOUR
    # pose) -- the case where the body-axis pick and the tangent pick DIVERGE:
    # body-right projected onto the wall pointed NORTH (18 m reverse lap, then
    # max_deviation "one breath short"); the tangent pick walks SOUTH (~14 m
    # lap). mutant: body-axis pick -> lap > 20 -> reddens; square-on entries
    # cannot kill that mutant (the frames coincide there), only this one can.
    world = SilWorld()
    for x, y in ((-7.33, 1.33), (-7.53, -0.77), (-7.47, -2.83), (-7.90, -4.80),
                 (-7.70, -6.07), (-7.83, -7.57), (-8.07, -9.07), (-8.00, -10.83),
                 (-8.20, -12.50), (-8.43, -13.83), (-7.97, -15.97)):
        world.add_obstacle("car", x, y)
    world.rx, world.ry, world.ryaw = 0.0, 0.0, 1.57
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(-15.07, -19.8)]))
    tick, states, _ = _tick_until(world, rns, 2400)
    assert tick is not None, "user full run failed again"
    laps = [r.detail.get("followed_m", 0.0) for r in rns.audit.drain()
            if r.kind == "wall_exit"]
    assert all(l < 20.0 for l in laps), \
        "oblique-entry side pick walked the long way: laps %s" % laps


def test_narrow_gap_rejected_by_corridor_sweep():
    # the mutant-killer for corridor_clearance (the margin fix alone masks the
    # 0.13 m case): a 1.0 m slot between two cars. The POINT probe sees the
    # extrapolated subgoal in open space (~1.5 m clear) and threads the slot --
    # squeezing through with ~0.04 m of side clearance; the CORRIDOR sweep sees
    # the 0.5 m throat < gate 1.1 and detours around instead (>=0.6 m). The
    # margin assertion below separates the two. mutant: point probe -> reddens.
    world = SilWorld()
    world.add_obstacle("car", 4.0, 1.1)    # slot centre y=0, width 2.2-1.2=1.0
    world.add_obstacle("car", 4.0, -1.1)
    world.rx, world.ry, world.ryaw = 0.0, 0.0, 0.0
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(9.0, 0.0)]))
    tick, states, min_clear = _tick_until(world, rns, 2400)
    assert tick is not None, "narrow-gap scene failed outright"
    # vy side-shield era (user-ordered special, 2026-09-10): the residual
    # behind-FOV corner graze went 0.073 -> 0.186 via the memory ring-scan vy
    # side-step (in FOLLOW and in-wall) plus the goto leave-point cleanliness
    # check. mutant: pass holonomic=False in Ctx (shield off) -> back to
    # ~0.07 -> reddens.
    assert min_clear > 0.15, \
        "corner graze regressed (min clearance %.3f m; shield should hold " \
        ">= 0.186)" % min_clear


def test_memory_appeal_layer_rehabilitates_walked_ground():
    # hole #6 LAYERED rescue, unit-level: a candidate rejected only as
    # out-of-FOV must be rehabilitated when memory_appeal=True AND the grid has
    # walked its corridor FREE; with appeal off (the normal contest) it stays
    # rejected -- that separation is exactly why the car-wall regressions
    # survived this feature while the first (mixed-in) cut blew them all up.
    # mutant: drop the appeal block -> appeal pick returns None too -> reddens.
    from xbrain.p1_motion.rns.types import Cell, NavState
    world = SilWorld()
    world.add_obstacle("car", 4.0, 1.1)
    world.add_obstacle("car", 4.0, -1.1)
    world.ryaw = 0.0
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    rns.load_mission(_mission([(9.0, 0.0)]))
    snap = world.synth_snapshot(0)

    class C:
        pose_xy = (0.0, 0.0); yaw_rad = 0.0
        v_nom_mps = 1.0; wz_max_rps = 1.2
        perception = snap; now_mono_ms = 0
    rns.compute(C())                      # anchors mission, sets _last_now
    # paint the DIAGONAL X->S corridor FREE in memory (as a wall lap would
    # have): the appeal walks the straight line to the subgoal (~(2.3, 2.9)),
    # so the painted band must cover that line, not a horizontal strip.
    for k in range(41):
        t = k / 40.0
        cx, cy = 3.0 * t, 3.6 * t
        for off in (-0.2, 0.0, 0.2):
            rns._grid.write(cx + off, cy + off, Cell.FREE, now_ms=0)
            rns._grid.write(cx - off, cy + off, Cell.FREE, now_ms=0)
    margin = max(CFG["rns"]["clearance"]["margin_by_class"].values())

    class FS:                              # minimal FollowState stand-in
        lookahead_point = (2.0, 0.0)
    normal = rns._pick_candidate(snap.profile, (0.0, 0.0), 0.0, FS(),
                                 CFG["rns"], margin)
    appealed = rns._pick_candidate(snap.profile, (0.0, 0.0), 0.0, FS(),
                                   CFG["rns"], margin, memory_appeal=True)
    assert normal is None, "normal contest must NOT see out-of-FOV candidates"
    assert appealed is not None, \
        "memory appeal failed to rehabilitate the walked side corridor"
