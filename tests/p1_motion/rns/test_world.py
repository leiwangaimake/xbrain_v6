"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_world.py
Brief: A-WLD-1..6 -- the object world profile (20 S3.0 RNS-N-18, 99 U113)

Description:
Guards rns/world.py and its two consumers' contract: RNS and the P1 host gate
read ONLY the world profile built from confident objects, never the raw
perception profile arrays (WP-1..WP-6 of 20 S3.0). Each test names the mutant
that reddens it (CLAUDE.md 3.3).

A-WLD-5 is the field regression: on 2026-11-04 camera-fixed phantoms in the
raw profile (a body part at 0.6 m, a blind-edge "pit" at 1.3 m), painted into
the world-anchored memory while the robot turned in place, sealed the robot
in and the guidance planner proved no_path_in_domain within 10 s. The scene
here replays that raw profile and carries its own positive control -- the
same scene fed the raw profile directly (the pre-U113 path) must trap the
robot -- so the negative assertion cannot pass vacuously. Measured while
writing it: with the phantom OBJECT kept in the world profile no ring forms
either, because every other ray of the world profile is FREE to range_max
and erases the cells; the ring is a property of the raw profile's
blind-edge d_free.

What this file does NOT cover: the host gate's four-band f() itself (its own
tests in tests/p1_motion/nav/), and the SIL bench (20 #20-28: it simulates
profiles, not objects).
"""

from __future__ import annotations  # postponed annotations

import copy  # each test mutates its own copy of the shipped config
import math  # bearings and speeds
from dataclasses import dataclass, replace  # the duck-typed tick context; DTO copies
from pathlib import Path  # repo root, to read the SHIPPED rns.yaml and nav_wiring.py
from typing import Optional, Tuple  # plain values

import pytest  # approx / raises
import yaml  # the shipped config, read like the SIL server does

from tests.p1_motion.rns.scenes import (  # the module's shared DTO builders
    healthy_status,  # a fresh, healthy status message
    one_object,  # one tracked object (we take its single element)
    uniform_free,  # a raw profile: the view header plus clean arrays
    with_block,  # add a raw d_block hit (the field phantoms)
    with_unknown,  # null one raw bin
)
from xbrain.common.types.units import Mps  # candidate speeds are a unit type
from xbrain.p1_motion.rns.inputs import ObjectsMsg, PerceptionSnapshot  # 11 S3.1B DTOs
from xbrain.p1_motion.rns.route import Mission  # a goto mission
from xbrain.p1_motion.rns.source import RnsSource  # the consumer under test
from xbrain.p1_motion.rns.types import (  # closed sets
    Cell,  # grid cell states (A-WLD-7)
    MissionKind,  # GOTO: the guidance planner only serves gotos
    NavFailReason,  # failure verdicts
    NavState,  # run states
    Origin,  # report channel of the mission
    SrcBit,  # SEMANTIC marks an object hit
    VelocityCandidate,  # RNS output type
)
from xbrain.p1_motion.rns.world import WorldBuilder, keep_confident, world_profile  # under test

pytestmark = pytest.mark.no_device  # pure host-side logic, no robot hardware

_ROOT = Path(__file__).resolve().parents[3]  # tests/p1_motion/rns -> repo root
_REAL_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))  # shipped values
R_EFF = 0.5  # 12 S6A r_eff of the M20S, the D2 assertion's baseline
CM = {"car": "vehicle_dynamic", "bird": "ignore", "pit": "hazard"}  # a small class_map


def _cfg():
    return copy.deepcopy(_REAL_CFG)  # never mutate the shared parse


def _box(x0: float, x1: float, y0: float, y1: float):
    """A CCW rectangle footprint in base_link."""
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))  # CCW, as 11 S3.1B.2 requires


def _objs(*specs, t: int = 1000) -> ObjectsMsg:
    """Several objects in one message: (class, confidence, footprint)."""
    out = []  # the tracked objects
    for k, (cls, conf, fp) in enumerate(specs):  # one per spec
        o = one_object(track_id=k + 1, class_name=cls, confidence=conf,  # distinct tracks
                       footprint=fp, t_capture_mono_ms=t).objects[0]  # its single object
        out.append(o)  # keep it
    return ObjectsMsg(t_capture_mono_ms=t, t_publish_mono_ms=t + 10,  # publish 10 ms after capture
                      extrinsic_calibrated=True, objects=tuple(out))  # calibrated, so A-CAL-1 stays quiet


def _bin_of(prof, deg: float) -> int:
    return int(round((math.radians(deg) - prof.angle_min_rad) / prof.angle_step_rad))  # bearing -> bin


# ── A-WLD-1: the confidence gate (WP-1) ──────────────────────────────────────
def test_below_threshold_vanishes_from_objects_and_geometry():
    # A-WLD-1: a 0.1 car straight ahead is gone from BOTH the objects RNS sees
    # and the geometry; a 0.4 car (exactly at the non-person floor, 99 U114)
    # stays in both.
    # mutant: no filter -> the 0.1 car blocks the forward ray -> reddens;
    # mutant: '>' instead of '>=' -> the boundary car is dropped -> reddens.
    raw = PerceptionSnapshot(
        profile=uniform_free(),  # a raw header to borrow the view from
        objects=_objs(("car", 0.1, _box(2.0, 2.5, -0.2, 0.2)),  # below: straight ahead
                      ("car", 0.4, _box(4.0, 4.5, 1.0, 1.4))),  # exactly at the floor, off-axis
        status=healthy_status())  # health passes through
    snap = WorldBuilder({"object_min_confidence": 0.4, "person_min_confidence": 0.1, "v_max_mps": 1.0}, CM).build(raw)  # the production path
    assert [o.confidence for o in snap.objects.objects] == [0.4]  # only the boundary car reaches RNS
    p = snap.profile  # the world profile
    assert p.d_block[_bin_of(p, 0.0)] is None  # the 0.1 car casts no geometry
    hit = p.d_block[_bin_of(p, math.degrees(math.atan2(1.2, 4.0)))]  # the ray through the 0.4 car
    assert hit is not None and 4.0 <= hit <= 4.5  # it blocks at its near face


def test_person_has_its_own_lower_floor():
    # A-WLD-1 (99 U114): person floor 0.1, every other class 0.4. A 0.15
    # person stays while a 0.15 car goes; a 0.09 person goes too (person is
    # not immune, it has its own floor). mutants: one floor for all -> the
    # 0.15 person is dropped -> reddens; no floor for person -> the 0.09
    # person stays -> reddens.
    raw = _objs(("person", 0.15, _box(1.5, 1.8, -0.2, 0.2)),  # above the person floor
                ("car", 0.15, _box(2.5, 3.0, -0.2, 0.2)),  # same score, other class
                ("person", 0.09, _box(3.5, 3.8, -0.2, 0.2)))  # under the person floor
    kept = keep_confident(raw, 0.4, 0.1).objects  # shipped floors
    assert [(o.class_name, o.confidence) for o in kept] == [("person", 0.15)]


# ── A-WLD-2: ray casting (WP-3) ──────────────────────────────────────────────
def test_nearest_hit_range_clip_and_free_to_range_max():
    # A-WLD-2: two boxes on the same ray -> the NEAR face; a box past
    # range_max -> no hit; an empty ray -> d_free = range_max, d_block None,
    # src 0; a hit ray -> d_free == d_block, src SEMANTIC, h_block = z_max.
    # mutant: keep the far hit / skip the range clip / leave empty bins None
    # -> each reddens.
    hdr = uniform_free(range_max_m=6.0)  # the view: 181 bins over +-45 deg, 6 m
    objs = _objs(("car", 0.9, _box(3.0, 3.5, -0.3, 0.3)),  # far box on the 0-deg ray
                 ("car", 0.9, _box(2.0, 2.4, -0.3, 0.3)),  # near box on the same ray
                 ("car", 0.9, _box(7.0, 7.5, 1.9, 2.4)))  # beyond range_max
    p = world_profile(hdr, objs, CM)  # cast
    i0 = _bin_of(p, 0.0)  # straight ahead
    assert p.d_block[i0] == pytest.approx(2.0, abs=1e-9)  # the NEAR face of the near box
    assert p.d_free[i0] == p.d_block[i0] and p.src[i0] == SrcBit.SEMANTIC  # free up to it, S produced it
    assert p.h_block[i0] == pytest.approx(1.75)  # the hit object's z_max (scenes default)
    i_far = _bin_of(p, math.degrees(math.atan2(2.1, 7.2)))  # the ray through the far box
    assert p.d_block[i_far] is None and p.d_free[i_far] == 6.0  # beyond range_max: no hit
    i_empty = _bin_of(p, -40.0)  # a ray through nothing
    assert p.d_block[i_empty] is None and p.d_free[i_empty] == 6.0 and p.src[i_empty] == 0  # free to range
    assert all(d is not None for d in p.d_free)  # no UNKNOWN bin, ever


def test_hull_containing_the_origin_blocks_every_ray_at_zero():
    # A-WLD-2: base_link inside a hull -> every bin blocked at 0.
    # mutant: skip the inside test -> the rays start inside and exit through
    # the far edge -> nonzero distances -> reddens.
    p = world_profile(uniform_free(), _objs(("car", 0.9, _box(-1.0, 1.0, -1.0, 1.0))), CM)  # robot inside
    assert all(d == 0.0 for d in p.d_block)  # nothing free in any direction


def test_ignore_t_class_and_unlocalized_contribute_no_geometry():
    # A-WLD-2 / WP-2: an airborne ignore class, the T class and an unlocalized
    # object never block a ray. mutant: drop the WP-2 filter -> reddens.
    unloc = one_object(class_name="car", confidence=0.9, unlocalized=True).objects[0]  # no hull
    objs = _objs(("bird", 0.9, _box(2.0, 2.5, -0.2, 0.2)),  # ignore class
                 ("traversable_area", 0.9, _box(1.0, 5.0, -2.0, 2.0)))  # the T class
    objs = replace(objs, objects=objs.objects + (unloc,))  # plus the unlocalized car
    p = world_profile(uniform_free(), objs, CM)  # cast
    assert all(d is None for d in p.d_block)  # none of the three blocks anything


# ── A-WLD-3: no raw array reaches the world (WP-5 / WP-6) ─────────────────────
def test_world_profile_ignores_every_raw_array():
    # A-WLD-3: the 2026-11-04 phantoms were raw d_block hits (body part at
    # 0.6 m, blind-edge "pit" at 1.3 m). Two raw profiles that differ in every
    # data array give the SAME world profile. mutant: pass any raw d_block /
    # d_free / src through -> the two differ -> reddens.
    clean = uniform_free(d_free_m=6.0)  # a clean raw profile
    dirty = with_block(clean, 90, 0.6, h_block_m=0.7)  # the body-part phantom
    dirty = with_block(dirty, 80, 1.3, h_block_m=-0.1)  # the blind-edge "pit"
    dirty = with_unknown(dirty, 100)  # and an unobserved bin
    objs = _objs(("car", 0.9, _box(3.0, 3.5, -0.3, 0.3)))  # one real object
    a = world_profile(clean, objs, CM)  # from the clean header
    b = world_profile(dirty, objs, CM)  # from the dirty one
    assert (a.d_free, a.d_block, a.h_block, a.src) == (b.d_free, b.d_block, b.h_block, b.src)  # identical
    assert b.d_block[90] == pytest.approx(3.0, abs=1e-9)  # the car, not the 0.6 phantom


def test_no_header_or_no_objects_means_no_world_profile():
    # A-WLD-3 / WP-5: never a default view. mutant: hand the raw profile on
    # (or synthesize a header) -> profile not None -> reddens.
    wb = WorldBuilder({"object_min_confidence": 0.4, "person_min_confidence": 0.1, "v_max_mps": 1.0}, CM)  # production builder
    objs = _objs(("car", 0.9, _box(3.0, 3.5, -0.3, 0.3)))  # objects present
    assert wb.build(PerceptionSnapshot(profile=None, objects=objs)).profile is None  # no header
    assert wb.build(PerceptionSnapshot(profile=uniform_free(), objects=None)).profile is None  # no objects
    assert wb.build(None).profile is None  # no intake at all


def test_nav_wiring_feeds_every_consumer_the_world_snapshot():
    # A-WLD-3 wiring half (WP-6): NavRuntime cannot be built in a test (it
    # needs two zenohd routers), so this reads its source: the ONE
    # perception.latest() call in NavRuntime is the argument of the ONE
    # world.build() call. mutant: "snap = self._perception.latest(now_ms)"
    # -> the raw profile reaches RNS, the host gate and the ring -> reddens.
    import ast  # local: only this test parses source
    src = (_ROOT / "xbrain" / "p1_motion" / "runtime" / "nav_wiring.py").read_text(
        encoding="utf-8")  # the production wiring
    cls = next(n for n in ast.parse(src).body  # the NavRuntime class body
               if isinstance(n, ast.ClassDef) and n.name == "NavRuntime")
    latest, builds = [], []  # call sites found
    for node in ast.walk(cls):  # every node in the class
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):  # obj.method(...)
            owner = ast.unparse(node.func.value)  # the receiver as written
            if node.func.attr == "latest" and owner == "self._perception":  # the intake read
                latest.append(node)
            if node.func.attr == "build" and owner == "self._world":  # the world build
                builds.append(node)
    assert len(latest) == 1 and len(builds) == 1  # one read, one build
    assert builds[0].args and builds[0].args[0] is latest[0]  # the read goes straight into the build


# ── A-WLD-4: the objects' clock (WP-4) ───────────────────────────────────────
def test_world_profile_takes_the_objects_timestamps():
    # A-WLD-4: a stale objects frame under a fresh depth frame must look
    # stale. mutant: copy t_capture from the raw profile -> reddens.
    hdr = uniform_free(t_capture_mono_ms=9000)  # fresh depth frame
    objs = _objs(("car", 0.9, _box(3.0, 3.5, -0.3, 0.3)), t=4000)  # old objects frame
    p = world_profile(hdr, objs, CM)  # cast
    assert (p.t_capture_mono_ms, p.t_publish_mono_ms) == (4000, 4010)  # the objects' clock
    assert p.t_seg_mono_ms is None  # no T evidence is ever claimed


# ── A-WLD-5: the 2026-11-04 seal does not happen ─────────────────────────────
@dataclass
class _Ctx:
    """The duck-typed tick context RnsSource.compute reads."""
    pose_xy: Optional[Tuple[float, float]] = (0.0, 0.0)  # the robot stays put
    yaw_rad: Optional[float] = 0.0  # set per tick: turning in place
    v_nom_mps: Optional[float] = 1.0  # mission nominal speed
    wz_max_rps: Optional[float] = 1.0  # turn-rate ceiling
    perception: Optional[PerceptionSnapshot] = None  # the snapshot under test
    now_mono_ms: Optional[int] = 0  # tick time
    holonomic: bool = False  # M20S lateral motion not assumed here


def _field_raw_profile(now: int):
    """The raw profile the robot saw on 2026-11-04 (99 U113): d_free stuck at
    the blind edge, the body part at 0.6 m on both sides and the blind-edge
    "pit" straight ahead."""
    p = uniform_free(d_free_m=1.3, t_capture_mono_ms=now - 20, t_seg_mono_ms=None)  # stuck at the edge
    for i in list(range(56, 72)) + list(range(110, 126)):  # body part, +-11..19 deg
        p = with_block(p, i, 0.6, h_block_m=0.7, d_free_m=0.59)  # 0.6 m, 0.7 m high
    for i in range(80, 101):  # "pit" ahead, +-5 deg
        p = with_block(p, i, 1.3, h_block_m=-0.1, d_free_m=1.29)  # negative height
    return p  # what the camera reported


def _spin_in_place(use_world: bool, seconds: float = 60.0):
    """The field shape: a goto 20 m ahead, the robot turning in place at
    1 rad/s (a full turn inside the memory TTL), the field raw profile, and a
    0.15-confidence phantom fixed 1.3-1.45 m in front of the camera.
    use_world=True runs the production path (world builder); False feeds the
    raw snapshot straight to RNS -- the pre-U113 path. The pose never moves,
    so a bench this long would eventually trip the progress watchdog; 60 s
    stays inside it. Returns (failure reason or None, states entered)."""
    cfg = _cfg()  # shipped values
    rc = cfg["rns"]["route"]  # mission parameters
    rns = RnsSource(cfg=cfg, r_eff_m=R_EFF)  # startup assertions run here
    rns.load_mission(Mission(MissionKind.GOTO, Origin.ROUTE, [(20.0, 0.0)],  # 20 m straight ahead
                             search_window=rc["search_window"],
                             arrival_radius_m=rc["arrival_radius_m"],
                             max_deviation_m=rc["max_deviation_m"]))
    wb = WorldBuilder(cfg["rns"]["world"], cfg["rns"]["class_map"])  # production builder
    states = set()  # every run state entered
    for k in range(int(seconds * 20)):  # 20 Hz ticks
        now = 10_000 + 50 * k  # tick time
        raw = PerceptionSnapshot(
            profile=_field_raw_profile(now),  # the field's raw phantoms
            objects=_objs(("toilet", 0.15, _box(1.3, 1.45, -0.1, 0.05)), t=now - 20),  # camera-fixed
            status=healthy_status(t_publish_mono_ms=now - 50))  # healthy status
        snap = wb.build(raw) if use_world else raw  # production vs pre-U113 path
        rns.compute(_Ctx(yaw_rad=math.remainder(0.05 * k, 2 * math.pi),  # 1 rad/s turn
                         perception=snap, now_mono_ms=now))
        states.add(rns.nav_state())  # what RNS decided
        f = rns.take_failure()  # one-shot failure latch
        if f is not None:  # trapped / proven
            return f.reason, states
    return None, states  # survived the whole turn


def test_field_phantoms_do_not_trap_the_robot():
    # A-WLD-5: on the production path the field's raw phantoms never reach
    # the memory and the 0.15 phantom is dropped, so 60 s of turning in place
    # stays in plain FOLLOW with no failure. mutant: the world builder passes
    # the raw d_block through (or drops WP-1) -> the turn paints a ring ->
    # DETOUR / WALL_FOLLOW and a trapped-robot failure -> reddens.
    reason, states = _spin_in_place(use_world=True)  # production path
    assert reason is None  # never trapped
    assert NavState.DETOUR not in states and NavState.WALL_FOLLOW not in states  # never saw a wall


def test_even_the_raw_phantoms_cannot_seal_a_memoryless_rns():
    # A-WLD-5 (99 U114): before U113 the SAME scene fed the raw profile
    # directly painted the phantoms into a ring and trapped the robot (the
    # 2026-11-04 shape; U113's control measured it). With no memory and no
    # wall-follow (RNS-N-19) even the raw phantoms cannot seal it: no
    # WALL_FOLLOW, no no_path_in_domain. The memory half on its own is
    # pinned by test_grid_holds_only_this_frame below.
    reason, states = _spin_in_place(use_world=False)  # the pre-U113 consumption
    assert reason is not NavFailReason.NO_PATH_IN_DOMAIN  # no proof from a ring
    assert NavState.WALL_FOLLOW not in states  # nothing to hug


# ── A-WLD-7: no memory -- the grid holds this frame only (RNS-N-19) ─────────
def test_grid_holds_only_this_frame():
    # A-WLD-7 (99 U114): a car seen ahead on one tick, then the robot turns
    # 90 deg and sees nothing -> the car's cell is no longer BLOCKED. Before
    # U114 it stayed for ttl_static_s. mutant: drop the per-tick clear_all ->
    # the old cell is still BLOCKED -> reddens.
    cfg = _cfg()  # shipped values
    rns = _goto_source(cfg)  # a goto so the tick runs
    wb = WorldBuilder(cfg["rns"]["world"], cfg["rns"]["class_map"])  # production builder
    car = _objs(("car", 0.9, _box(2.0, 2.6, -0.3, 0.3)), t=9_980)  # 2 m straight ahead
    rns.compute(_Ctx(perception=wb.build(PerceptionSnapshot(
        profile=uniform_free(t_capture_mono_ms=9_980, t_seg_mono_ms=None), objects=car,
        status=healthy_status(t_publish_mono_ms=9_950))), now_mono_ms=10_000))
    assert rns._grid.read(2.0, 0.0, 10_000) == Cell.BLOCKED  # seen this tick
    rns.compute(_Ctx(yaw_rad=math.pi / 2, perception=wb.build(PerceptionSnapshot(
        profile=uniform_free(t_capture_mono_ms=10_030, t_seg_mono_ms=None),
        objects=_objs(t=10_030),
        status=healthy_status(t_publish_mono_ms=10_000))), now_mono_ms=10_050))
    assert rns._grid.read(2.0, 0.0, 10_050) != Cell.BLOCKED  # forgotten the next tick


# ── A-WLD-8: the avoidance speed floor (20 S3.0A / S8.1A) ───────────────────
def test_avoidance_shaping_caps_never_push_below_the_floor():
    # A-WLD-8 (99 U114): far off a PATH line the deviation cap is
    # dev_g_min x v_nom = 0.15 m/s; the floor lifts it to
    # speed.avoid_floor_mps (0.2). The robot faces its lookahead point, so the
    # heading taper is ~1 and the output IS the cap. mutant: drop the floor
    # on the deviation cap -> 0.15 -> reddens.
    cfg = _cfg()  # shipped values
    rc = cfg["rns"]["route"]  # mission parameters
    rns = RnsSource(cfg=cfg, r_eff_m=R_EFF)  # startup assertions run here
    rns.load_mission(Mission(MissionKind.PATH, Origin.ROUTE,  # a straight line along +x
                             [(i * 0.5, 0.0) for i in range(81)],
                             search_window=rc["search_window"],
                             arrival_radius_m=rc["arrival_radius_m"],
                             max_deviation_m=rc["max_deviation_m"]))
    e = rc["max_deviation_m"]  # lateral offset at the cap's floor
    wb = WorldBuilder(cfg["rns"]["world"], cfg["rns"]["class_map"])  # production builder
    out = None  # last candidate
    for k in range(10):  # a few ticks to settle the projection
        now = 10_000 + 50 * k  # tick time
        snap = wb.build(PerceptionSnapshot(
            profile=uniform_free(t_capture_mono_ms=now - 20, t_seg_mono_ms=None),  # view header
            objects=_objs(t=now - 20), status=healthy_status(t_publish_mono_ms=now - 50)))
        r = rns._last_R or (10.0, 0.0)  # the lookahead point once known
        yaw = math.atan2(r[1] - e, r[0] - 10.0)  # face it: taper ~1
        out = rns.compute(_Ctx(pose_xy=(10.0, e), yaw_rad=yaw, perception=snap, now_mono_ms=now))
    floor = cfg["rns"]["speed"]["avoid_floor_mps"]  # 0.2 shipped
    assert out is not None and out.vx.value == pytest.approx(floor, abs=0.01)


# ── A-WLD-9: no wall-follow, no guidance (RNS-N-19) ─────────────────────────
def test_no_way_through_holds_instead_of_wall_following():
    # A-WLD-9 (99 U114): a car row across the whole view, 2 m ahead, leaves
    # no thread / detour in this frame -> RNS holds (zero), never enters
    # WALL_FOLLOW, has no planner, and does not fail on the spot. mutants:
    # restore the wall-follow entry -> WALL_FOLLOW -> reddens; construct the
    # planner again -> _planner not None -> reddens.
    cfg = _cfg()  # shipped values
    rns = _goto_source(cfg)  # goto 20 m ahead
    assert rns._planner is None  # the guidance layer is retired
    wb = WorldBuilder(cfg["rns"]["world"], cfg["rns"]["class_map"])  # production builder
    row = _objs(("car", 0.9, _box(2.0, 2.5, -4.0, 4.0)), t=0)  # wider than the view
    states, outs = set(), []  # what RNS did
    for k in range(60):  # 3 s
        now = 10_000 + 50 * k  # tick time
        snap = wb.build(PerceptionSnapshot(
            profile=uniform_free(t_capture_mono_ms=now - 20, t_seg_mono_ms=None),  # view header
            objects=replace(row, t_capture_mono_ms=now - 20, t_publish_mono_ms=now - 10),
            status=healthy_status(t_publish_mono_ms=now - 50)))
        cand = rns.compute(_Ctx(perception=snap, now_mono_ms=now))  # standing still
        states.add(rns.nav_state())  # record the state
        outs.append(cand)  # and the output
    assert NavState.WALL_FOLLOW not in states  # nothing to hug any more
    assert outs[-1] is not None and outs[-1].vx.value == 0.0  # held
    assert rns.take_failure() is None  # a hold is not a failure (the watchdog bounds it)


def _goto_source(cfg) -> RnsSource:
    """An RnsSource with a goto 20 m straight ahead."""
    rc = cfg["rns"]["route"]  # mission parameters
    rns = RnsSource(cfg=cfg, r_eff_m=R_EFF)  # startup assertions run here
    rns.load_mission(Mission(MissionKind.GOTO, Origin.ROUTE, [(20.0, 0.0)],  # 20 m ahead
                             search_window=rc["search_window"],
                             arrival_radius_m=rc["arrival_radius_m"],
                             max_deviation_m=rc["max_deviation_m"]))
    return rns  # ready to tick


# ── A-WLD-6: the ceiling (runtime half; start half in test_config.py) ─────────
def test_follow_candidates_and_exit_clamp_respect_world_v_max():
    # A-WLD-6: open ground, a 0.3 m/s ceiling -> vx <= 0.3 on the follow
    # path, and the exit clamp's own scaling. The branch that bypasses the
    # caps (wall-follow) is pinned on a moving robot in tests/sil/
    # test_sil_avoidance.py::test_wall_follow_respects_world_v_max (this
    # stationary bench never reaches wall-follow speed). Dropping
    # caps["world"] ALONE is an equivalent mutant for the speed (the exit
    # clamp still enforces it; caps["world"] only names the limiter for the
    # watchdog / HMI attribution).
    cfg = _cfg()  # shipped values
    cfg["rns"]["world"]["v_max_mps"] = 0.3  # a ceiling well under v_nom
    rc = cfg["rns"]["route"]  # mission parameters
    rns = RnsSource(cfg=cfg, r_eff_m=R_EFF)  # startup assertions run here
    rns.load_mission(Mission(MissionKind.GOTO, Origin.ROUTE, [(20.0, 0.0)],  # open goto
                             search_window=rc["search_window"],
                             arrival_radius_m=rc["arrival_radius_m"],
                             max_deviation_m=rc["max_deviation_m"]))
    wb = WorldBuilder(cfg["rns"]["world"], cfg["rns"]["class_map"])  # production builder
    vmax = 0.0  # fastest candidate seen
    for k in range(40):  # 2 s of open running
        now = 10_000 + 50 * k  # tick time
        snap = wb.build(PerceptionSnapshot(
            profile=uniform_free(t_capture_mono_ms=now - 20, t_seg_mono_ms=None),  # view header
            objects=_objs(t=now - 20),  # no objects: open ground
            status=healthy_status(t_publish_mono_ms=now - 50)))  # healthy
        cand = rns.compute(_Ctx(v_nom_mps=2.0, perception=snap, now_mono_ms=now))  # v_nom well above
        if cand is not None:  # RNS spoke
            vmax = max(vmax, math.hypot(cand.vx.value, cand.vy.value))  # planar speed
    assert 0.0 < vmax <= 0.3 + 1e-9  # it moved, and never above the ceiling
    # the exit clamp on its own: the planar speed is scaled, direction kept
    out = rns._world_clamp(VelocityCandidate(vx=Mps(0.4), vy=Mps(0.3), wz=0.2))  # 0.5 m/s in
    assert math.hypot(out.vx.value, out.vy.value) == pytest.approx(0.3)  # scaled to the ceiling
    assert out.vx.value / out.vy.value == pytest.approx(0.4 / 0.3) and out.wz == 0.2  # same direction, wz kept
