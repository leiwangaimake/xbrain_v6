"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: sil_world.py
Brief: SIL world model -- obstacles, raycast perception synth, kinematics

Description:
The software-in-the-loop world for RNS testing. Three jobs:
  1. obstacle store: walls (two-point segment + thickness, any angle, chainable
     into U-traps), persons/cars (static until double-clicked dynamic: person
     1 m/s, car 3 m/s, random heading), rocks/cones (pure geometry).
  2. perception synthesis: raycast from the robot pose over the 338Le sector
     (horizontal FOV 90 deg = +/-45 deg, 181 bins, range 6 m, near blind
     0.59 m -- docs/RNS-REF measured params, same as 11 S3.1B contract example)
     producing REAL ProfileMsg/ObjectsMsg/StatusMsg DTOs -- the same types the
     production module consumes. RNS is NOT mocked anywhere.
  3. kinematics: integrate the RNS VelocityCandidate at the 20 Hz tick
     (body-frame vx/vy + wz -> world pose), standing in for the quadruped.

What this file does NOT do: no RNS logic (that stays in xbrain/p1_motion/rns,
untouched), no networking (sil_server.py), no drawing (static/index.html).

Trap: obstacle raycast reports hits INSIDE the 0.59 m blind zone too. The real
camera cannot see there (the memory grid covers it); until the grid is wired
into compute (S2/S3 of the SIL plan) hiding near hits would just crash the sim
robot into things it once saw. Revisit when memory-grid consume lands.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from xbrain.p1_motion.rns.inputs import (
    ObjectsMsg, PerceptionSnapshot, ProfileMsg, StatusMsg, TrackedObject,
)
from xbrain.p1_motion.rns.types import SrcBit

# 338Le sector (docs/RNS-REF/RNS_V2_DESIGN.md; matches 11 S3.1B example values)
FOV_HALF_RAD = math.radians(45.0)
N_BINS = 181
ANGLE_STEP = (2 * FOV_HALF_RAD) / (N_BINS - 1)
RANGE_MAX_M = 6.0
BLIND_NEAR_M = 0.59

# obstacle geometry radii (m) for the circle-shaped types
RADIUS = {"person": 0.3, "car": 0.0, "rock": 0.5, "cone": 0.25}
CAR_HALF_L, CAR_HALF_W = 1.0, 0.6   # car as an oriented box
DEFAULT_WALL_THICK_M = 0.5
DYNAMIC_SPEED = {"person": 1.0, "car": 3.0}   # user spec 2026-09-10


@dataclass
class Obstacle:
    """One placed obstacle. Circles: person/rock/cone (radius by type). Boxes:
    car (fixed size, axis from heading) and wall (two-point segment + thick)."""
    oid: int
    kind: str                       # person | car | wall | rock | cone
    x: float
    y: float
    x2: float = 0.0                 # wall only: segment end
    y2: float = 0.0
    thick_m: float = DEFAULT_WALL_THICK_M
    dynamic: bool = False           # person/car only (double-click toggles)
    heading: float = 0.0            # motion direction when dynamic
    vx: float = 0.0
    vy: float = 0.0

    def set_dynamic(self, on: bool) -> None:
        self.dynamic = on and self.kind in DYNAMIC_SPEED
        if self.dynamic:
            self.heading = random.uniform(-math.pi, math.pi)
            spd = DYNAMIC_SPEED[self.kind]
            self.vx = spd * math.cos(self.heading)
            self.vy = spd * math.sin(self.heading)
        else:
            self.vx = self.vy = 0.0


def _ray_circle(ox, oy, dx, dy, cx, cy, r) -> Optional[float]:
    """Distance along ray (o + t*d, t>0) to a circle, or None."""
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0.0:
        return None
    t = -b - math.sqrt(disc)
    return t if t > 0.0 else None


def _ray_obb(ox, oy, dx, dy, cx, cy, ax, hl, hw) -> Optional[float]:
    """Ray vs oriented box: center (cx,cy), axis angle ax, half-len hl, half-
    width hw. Transform the ray into box frame and slab-test."""
    ca, sa = math.cos(-ax), math.sin(-ax)
    rx = ca * (ox - cx) - sa * (oy - cy)
    ry = sa * (ox - cx) + ca * (oy - cy)
    rdx = ca * dx - sa * dy
    rdy = sa * dx + ca * dy
    tmin, tmax = 0.0, math.inf
    for p, d, h in ((rx, rdx, hl), (ry, rdy, hw)):
        if abs(d) < 1e-12:
            if abs(p) > h:
                return None
        else:
            t1, t2 = (-h - p) / d, (h - p) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
            if tmin > tmax:
                return None
    return tmin if tmin > 1e-9 else None


class SilWorld:
    """The SIL world: obstacle store + path/waypoints + robot pose + the 20 Hz
    perception/kinematics services the server tick calls."""

    def __init__(self) -> None:
        self._next_id = 1
        self.obstacles: Dict[int, Obstacle] = {}
        self.path: List[Tuple[float, float]] = []
        self.waypoints: List[Tuple[float, float]] = []
        # robot starts at origin facing +x (matches the mock: green dog)
        self.rx, self.ry, self.ryaw = 0.0, 0.0, math.pi / 2

    # ── placement API (server REST calls these) ──────────────────────────────
    def add_obstacle(self, kind: str, x: float, y: float,
                     x2: float = 0.0, y2: float = 0.0,
                     thick_m: float = DEFAULT_WALL_THICK_M) -> int:
        oid = self._next_id
        self._next_id += 1
        self.obstacles[oid] = Obstacle(oid=oid, kind=kind, x=x, y=y,
                                       x2=x2, y2=y2, thick_m=thick_m)
        return oid

    def update_wall(self, oid: int, x: float, y: float, x2: float, y2: float,
                    thick_m: float) -> bool:
        o = self.obstacles.get(oid)
        if o is None or o.kind != "wall":
            return False
        o.x, o.y, o.x2, o.y2, o.thick_m = x, y, x2, y2, thick_m
        return True

    def toggle_dynamic(self, oid: int) -> bool:
        o = self.obstacles.get(oid)
        if o is None:
            return False
        o.set_dynamic(not o.dynamic)
        return True

    def remove_obstacle(self, oid: int) -> bool:
        return self.obstacles.pop(oid, None) is not None

    def reset(self) -> None:
        self.obstacles.clear()
        self.path.clear()
        self.waypoints.clear()
        self.rx, self.ry, self.ryaw = 0.0, 0.0, math.pi / 2

    # ── dynamic obstacle motion (server tick) ────────────────────────────────
    def step_obstacles(self, dt: float) -> None:
        for o in self.obstacles.values():
            if o.dynamic:
                o.x += o.vx * dt
                o.y += o.vy * dt
                if o.kind == "car":
                    o.heading = math.atan2(o.vy, o.vx)

    # ── raycast one obstacle ─────────────────────────────────────────────────
    def _hit(self, o: Obstacle, ox, oy, dx, dy) -> Optional[float]:
        if o.kind == "wall":
            cx, cy = (o.x + o.x2) / 2.0, (o.y + o.y2) / 2.0
            ax = math.atan2(o.y2 - o.y, o.x2 - o.x)
            hl = math.hypot(o.x2 - o.x, o.y2 - o.y) / 2.0
            return _ray_obb(ox, oy, dx, dy, cx, cy, ax, hl, o.thick_m / 2.0)
        if o.kind == "car":
            return _ray_obb(ox, oy, dx, dy, o.x, o.y, o.heading,
                            CAR_HALF_L, CAR_HALF_W)
        return _ray_circle(ox, oy, dx, dy, o.x, o.y, RADIUS[o.kind])

    # ── perception synthesis: REAL DTOs, per-bin raycast ─────────────────────
    def synth_snapshot(self, now_ms: int) -> PerceptionSnapshot:
        d_free: List[Optional[float]] = []
        d_block: List[Optional[float]] = []
        h_block: List[Optional[float]] = []
        src: List[int] = []
        conf: List[int] = []
        for i in range(N_BINS):
            ang = self.ryaw - FOV_HALF_RAD + i * ANGLE_STEP
            dx, dy = math.cos(ang), math.sin(ang)
            best: Optional[float] = None
            for o in self.obstacles.values():
                t = self._hit(o, self.rx, self.ry, dx, dy)
                if t is not None and t <= RANGE_MAX_M and (best is None or t < best):
                    best = t
            if best is None:
                d_free.append(RANGE_MAX_M)
                d_block.append(None)
                h_block.append(None)
            else:
                d_free.append(max(0.0, best - 0.05))
                d_block.append(best)
                h_block.append(0.8)
            src.append(SrcBit.SEG | SrcBit.GEOM)   # sim ground is T+G backed
            conf.append(220)
        profile = ProfileMsg(
            t_capture_mono_ms=now_ms, t_publish_mono_ms=now_ms + 5,
            extrinsic_calibrated=True,
            angle_min_rad=-FOV_HALF_RAD, angle_step_rad=ANGLE_STEP,
            n_bins=N_BINS, range_max_m=RANGE_MAX_M, blind_near_m=BLIND_NEAR_M,
            z_pass_m=0.75, t_seg_mono_ms=now_ms - 5,
            d_free=tuple(d_free), d_block=tuple(d_block),
            h_block=tuple(h_block), src=tuple(src), conf=tuple(conf))
        objs: List[TrackedObject] = []
        for o in self.obstacles.values():
            if o.kind not in ("person", "car"):
                continue
            r_near = math.hypot(o.x - self.rx, o.y - self.ry)
            if r_near > RANGE_MAX_M * 1.5:
                continue
            rad = RADIUS["person"] if o.kind == "person" else CAR_HALF_L
            fp = tuple((o.x + rad * math.cos(a), o.y + rad * math.sin(a))
                       for a in (0.0, 2.1, 4.2))
            objs.append(TrackedObject(
                track_id=o.oid, class_name=o.kind, class_id=0, confidence=0.92,
                semantic_status="confirmed", footprint_xy=fp,
                z_min=0.02, z_max=1.7, r_near=max(0.0, r_near - rad),
                velocity_xy=(o.vx, o.vy), velocity_frame="ego_removed",
                velocity_valid=True,
                velocity_status="moving" if o.dynamic else "static",
                stable_frames=50))
        objects = ObjectsMsg(t_capture_mono_ms=now_ms, extrinsic_calibrated=True,
                             objects=tuple(objs))
        status = StatusMsg(
            t_publish_mono_ms=now_ms, fps_depth=30.0, fps_infer=20.0,
            invalid_pixel_ratio=0.03, extrinsic_calibrated=True,
            traversable_seg_available=True)
        return PerceptionSnapshot(profile=profile, objects=objects, status=status)

    # ── kinematics (the sim quadruped) ───────────────────────────────────────
    def step_robot(self, vx: float, vy: float, wz: float, dt: float) -> None:
        c, s = math.cos(self.ryaw), math.sin(self.ryaw)
        self.rx += (vx * c - vy * s) * dt
        self.ry += (vx * s + vy * c) * dt
        self.ryaw = math.atan2(math.sin(self.ryaw + wz * dt),
                               math.cos(self.ryaw + wz * dt))
