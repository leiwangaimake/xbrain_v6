"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: world.py
Brief: Object-driven world profile -- the ONE geometry P1 motion consumes (20 S3.0 RNS-N-18)

Description:
What problem this solves. Until 99 U113 every RNS geometry consumer (memory
grid, gap edges, wall-follow, guidance planner) and the P1 host speed gate
f(d_free) read the perception PROFILE: per-bin d_free / d_block built from the
ground plane and depth voxels. The 2026-11-04 outdoor runs showed how that
fails in the field: about 57% of ground pixels had no depth, so d_free stuck
at the blind edge (f() -> 0 or 0.2 m/s); a body part of the robot read as a
0.6 m obstacle on both sides and the blind edge read as a "pit"; the memory
grid kept every such hit for 300 s in world frame, and turning in place
painted them into a ring around the robot until the guidance planner proved
no_path_in_domain. The user ruled (20 S3.0 RNS-N-18): RNS does not consume the
traversable / geometric profile at all, only perception OBJECTS whose
confidence is at least world.object_min_confidence, and threads or detours
around them.

What this file does (rules WP-1..WP-6 of 20 S3.0):
  WP-1 drop every object below the confidence threshold -- ALL classes,
       person included -- from the objects RNS sees AND from the geometry;
  WP-2 take the footprint hull of every kept, localized object whose
       behavior is neither the T class (dropped) nor ignore (airborne);
  WP-3 cast each profile bin's ray from the base_link origin against those
       hulls: d_block = nearest hit (0 when the origin is inside a hull,
       None past range_max), d_free = the hit or range_max, h_block = the
       hit object's z_max, src = SEMANTIC on a hit else 0, t_seg None;
  WP-4 time stamps and the extrinsic flag come from the OBJECTS message, so
       every age gate (T-50/T-51, 12 S3.3 freshness) follows the objects;
  WP-5 only the view HEADER of the raw profile is borrowed (angles, bin
       count, range, z_pass; blind_near_m carried, read by nobody). No raw
       profile ever, or no objects ever -> no world profile (None): the
       existing "never received -> free_space veto" applies, never a
       default field of view;
  WP-6 built ONCE per tick in runtime/nav_wiring; RNS, the host gate and the
       rotation ring sample all read that one snapshot.

What it does NOT do: no fusion with the raw profile arrays (that is the
point), no memory, no thread, no clock read (time stamps are copied, never
taken). The per-class behavior (person stop, unlocalized cap) stays in
source.py; this file only decides which objects exist and where they block.

Traps (do not re-learn):
  - Reading ANY raw d_free / d_block / src here brings the body-part and
    blind-edge phantoms straight back (A-WLD-3 holds the mutant).
  - Taking t_capture from the raw profile keeps a dead objects channel
    looking fresh while the depth stream still runs (A-WLD-4).
  - Casting all 181 rays against every hull edge every tick is ~40k segment
    tests in Python at 20 Hz; each hull is limited to the bins inside its
    angular span instead (a hull that wraps behind the robot or contains the
    origin falls back to all bins -- rare and still correct).
"""

from __future__ import annotations

import math  # atan2 / cos / sin for the per-bin rays
from dataclasses import replace  # frozen DTOs are copied, never mutated
from typing import List, Optional, Sequence, Tuple  # plain values only

from .classify import IGNORE_BEHAVIOR, effective_behavior  # WP-2 class filter, same rules as source.py
from .inputs import ObjectsMsg, PerceptionSnapshot, ProfileMsg, TrackedObject  # the 11 S3.1B DTOs
from .types import SrcBit  # the src bitmask, SEMANTIC marks an object hit

#: Parallel-ray guard for the ray/edge solve (metres squared scale).
_EPS = 1e-12  # below this the ray and the edge are treated as parallel


def keep_confident(objects: ObjectsMsg, min_confidence: float,
                   person_min_confidence: float) -> ObjectsMsg:
    """WP-1: the objects message without every object below its floor.

    Two floors (99 U114): person_min_confidence for class "person", which is
    deliberately LOW (0.1) -- with no geometric backstop a missed person is
    the costliest miss, a phantom person only costs a wait -- and
    min_confidence (0.4) for every other class. An object exactly at its
    floor is kept: the keys read "at least". mutants: strict '>' drops the
    boundary object; one floor for all classes drops a 0.2 person (A-WLD-1
    reddens either way)."""
    kept = tuple(o for o in objects.objects  # ">=": the keys are floors
                 if o.confidence >= (person_min_confidence if o.class_name == "person"
                                     else min_confidence))
    if len(kept) == len(objects.objects):  # nothing below the floor this frame
        return objects  # same message object: keeps the builder cache identity stable
    return replace(objects, objects=kept)  # timestamps / extrinsic flag carried unchanged


def _origin_inside(poly: Sequence[Tuple[float, float]]) -> bool:
    """True when base_link's origin lies inside the hull (ray casting)."""
    inside = False  # parity of crossings of the +x ray from the origin
    j = len(poly) - 1  # previous vertex index, starts at the closing edge
    for i in range(len(poly)):  # every edge (poly[j], poly[i])
        xi, yi = poly[i]  # edge end
        xj, yj = poly[j]  # edge start
        # edge straddles y = 0 and crosses it at x > 0 -> one crossing
        if (yi > 0.0) != (yj > 0.0) and 0.0 < (xj - xi) * (0.0 - yi) / (yj - yi) + xi:
            inside = not inside  # odd crossings = inside
        j = i  # advance the edge
    return inside  # ray casting result


def _ray_hit(c: float, s: float, poly: Sequence[Tuple[float, float]]) -> Optional[float]:
    """Nearest distance along the unit ray (c, s) from the origin to the hull
    boundary, or None when the ray misses it.

    Ray p(t) = t*(c, s), t >= 0; edge q(u) = A + u*(B - A), u in [0, 1].
    With 2D cross products: denom = d x e, t = (A x e) / denom,
    u = (A x d) / denom. Parallel edges (denom ~ 0) cannot be crossed
    transversally and are skipped; a grazing touch is caught by the
    neighbouring edge's endpoint."""
    best: Optional[float] = None  # nearest crossing so far (the NEAR face wins)
    n = len(poly)  # vertex count, 3..12 by contract
    for k in range(n):  # every edge A -> B, closing edge included
        ax, ay = poly[k]  # A
        bx, by = poly[(k + 1) % n]  # B, wraps to vertex 0
        ex, ey = bx - ax, by - ay  # edge vector e
        denom = c * ey - s * ex  # d x e
        if abs(denom) < _EPS:  # ray parallel to this edge
            continue  # no transversal crossing; neighbours catch a touch
        t = (ax * ey - ay * ex) / denom  # distance along the ray
        u = (ax * s - ay * c) / denom  # position along the edge
        if t >= 0.0 and 0.0 <= u <= 1.0 and (best is None or t < best):  # ahead, on the edge, nearer
            best = t  # keep the nearest (mutant: farthest -> A-WLD-2 red)
    return best  # None = this ray misses the hull


def _bin_span(poly: Sequence[Tuple[float, float]], angle_min: float,
              step: float, n_bins: int) -> range:
    """Bins whose ray can meet the hull: those inside its angular span.

    A hull containing the origin, or one whose vertices span more than pi
    (it wraps behind the robot across the +-pi seam), returns every bin --
    the cheap pruning is only valid for a hull seen within a half plane."""
    angs = [math.atan2(y, x) for x, y in poly]  # bearing of each vertex
    lo, hi = min(angs), max(angs)  # angular extent of the hull
    if hi - lo > math.pi:  # wraps across +-pi: the span is not lo..hi
        return range(n_bins)  # fall back to every bin (rare, still correct)
    # one bin of slack each side: a hull thinner than a bin still gets the
    # nearest rays tested; the exact solve decides whether they hit.
    i_lo = max(0, int(math.floor((lo - angle_min) / step)) - 1)  # first candidate bin
    i_hi = min(n_bins - 1, int(math.ceil((hi - angle_min) / step)) + 1)  # last candidate bin
    if i_lo > i_hi:  # the hull lies entirely outside the field of view
        return range(0)  # no ray can reach it
    return range(i_lo, i_hi + 1)  # inclusive span


def _geometry_objects(objects: ObjectsMsg, class_map: dict) -> List[TrackedObject]:
    """WP-2: the kept objects that contribute geometry -- localized, and
    neither the T class (the traversable-segmentation class is never an
    object, A-CLS-5) nor ignore (airborne, A-CLS-6)."""
    out = []  # objects that block rays
    for o in objects.objects:  # already confidence-filtered (WP-1)
        if not o.footprint_xy:  # 11 v2.3 row 4: detected but not localizable
            continue  # no geometry to cast; source.py caps the speed (S5.1A)
        beh, _ = effective_behavior(o.class_name, o.semantic_status, class_map)  # the same rule source.py uses
        if beh is None or beh == IGNORE_BEHAVIOR:  # T class / airborne
            continue  # never an obstacle on the ground
        out.append(o)  # a ground obstacle with a hull
    return out  # the geometry set of this frame


def world_profile(header: ProfileMsg, objects: ObjectsMsg,
                  class_map: dict) -> ProfileMsg:
    """WP-3/WP-4/WP-5: the world profile of one objects frame.

    header lends ONLY its view geometry; none of its data arrays is read
    (A-WLD-3). objects must already be confidence-filtered (WP-1)."""
    n = header.n_bins  # view: bin count
    a0 = header.angle_min_rad  # view: first bin bearing
    step = header.angle_step_rad  # view: bin width
    rmax = header.range_max_m  # view: farthest the world speaks for
    d_block: List[Optional[float]] = [None] * n  # nearest hull hit per bin
    h_block: List[Optional[float]] = [None] * n  # the hit object's top
    for o in _geometry_objects(objects, class_map):  # every ground obstacle
        poly = o.footprint_xy  # base_link hull, CCW
        if _origin_inside(poly):  # the hull covers base_link itself
            for i in range(n):  # then nothing is free in any direction
                d_block[i] = 0.0  # blocked at the robot
                h_block[i] = o.z_max  # its height
            continue  # no ray casting needed for this hull
        for i in _bin_span(poly, a0, step, n):  # only the rays that can meet it
            th = a0 + i * step  # bearing of bin i
            t = _ray_hit(math.cos(th), math.sin(th), poly)  # nearest crossing
            if t is None or t > rmax:  # misses, or beyond the view range
                continue  # mutant: drop the range clip -> A-WLD-2 red
            if d_block[i] is None or t < d_block[i]:  # nearer than any other hull
                d_block[i] = t  # the obstacle boundary on this ray
                h_block[i] = o.z_max  # and what it is
    d_free = tuple(rmax if d is None else d for d in d_block)  # FREE up to the hull or range_max
    src = tuple(0 if d is None else SrcBit.SEMANTIC for d in d_block)  # only S produced a block
    return ProfileMsg(
        t_capture_mono_ms=objects.t_capture_mono_ms,  # WP-4: ages follow the objects
        t_publish_mono_ms=objects.t_publish_mono_ms,  # WP-4: same message
        extrinsic_calibrated=objects.extrinsic_calibrated,  # WP-4: the objects' own flag
        angle_min_rad=a0,  # WP-5: view only
        angle_step_rad=step,  # WP-5: view only
        n_bins=n,  # WP-5: view only
        range_max_m=rmax,  # WP-5: view only
        blind_near_m=header.blind_near_m,  # carried for DTO completeness, read by nobody
        z_pass_m=header.z_pass_m,  # carried, the height window is perception's
        t_seg_mono_ms=None,  # no T channel (20 S3.1.11 retired)
        d_free=d_free,  # never None: no UNKNOWN band in the world profile
        d_block=tuple(d_block),  # None = nothing in range on this ray
        h_block=tuple(h_block),  # the hit object's z_max
        src=src,  # SEMANTIC on a hit, else 0
        conf=(),  # per-bin depth confidence is a raw-profile notion
    )


class WorldBuilder:
    """WP-6: builds the tick's world snapshot once, in runtime/nav_wiring.

    Holds the two config values (validated at RNS startup, never defaulted
    here) and a one-entry cache: latest() returns the same objects message
    object until a new frame lands, so an unchanged frame is not re-cast
    every 50 ms."""

    def __init__(self, world_cfg: dict, class_map: dict) -> None:
        # KeyError on a missing key is intended -- run_startup_assertions
        # already refused a null / missing world.* before this is built.
        self._min_conf = float(world_cfg["object_min_confidence"])  # WP-1 floor, non-person
        self._person_min_conf = float(world_cfg["person_min_confidence"])  # WP-1 floor, person (U114)
        self._class_map = class_map  # WP-2 classification, the rns.yaml map
        # cache identity: the raw objects message itself (held, so its id can
        # never be reused by a later message) plus the header's view geometry
        # (the raw profile object changes every depth frame; its view does not)
        self._last_raw: Optional[ObjectsMsg] = None  # the message last built from
        self._last_view: Optional[Tuple[float, float, int, float]] = None  # its view
        self._last_out: Optional[Tuple[ObjectsMsg, ProfileMsg]] = None  # (kept, world)

    def build(self, raw: Optional[PerceptionSnapshot]) -> PerceptionSnapshot:
        """The snapshot every P1 motion consumer reads this tick.

        objects: confidence-filtered (WP-1). profile: the world profile, or
        None when either the raw profile header or the objects never arrived
        (WP-5). status passes through untouched (RNS-I-2 health cap)."""
        if raw is None:  # no intake at all
            return PerceptionSnapshot()  # every key absent -> consumers time out
        if raw.objects is None:  # objects never arrived
            return PerceptionSnapshot(profile=None, objects=None, status=raw.status)  # no world, never a default
        if raw.profile is None:  # no view header yet
            # objects still go to the person rule, but there is no world
            # geometry -> the host gate sees "never received" (free_space veto)
            return PerceptionSnapshot(profile=None,
                                      objects=keep_confident(raw.objects, self._min_conf,
                                                             self._person_min_conf),  # WP-1 still applies
                                      status=raw.status)  # health passes through
        view = (raw.profile.angle_min_rad, raw.profile.angle_step_rad,
                raw.profile.n_bins, raw.profile.range_max_m)  # the only raw-profile fields read
        if raw.objects is not self._last_raw or view != self._last_view \
                or self._last_out is None:  # a new objects frame (or a new view)
            kept = keep_confident(raw.objects, self._min_conf, self._person_min_conf)  # WP-1
            self._last_out = (kept, world_profile(raw.profile, kept, self._class_map))  # WP-2..WP-5
            self._last_raw = raw.objects  # hold it: identity is the cache key
            self._last_view = view  # and the view it was cast in
        kept, prof = self._last_out  # this frame's world
        return PerceptionSnapshot(profile=prof, objects=kept, status=raw.status)  # what every consumer reads
