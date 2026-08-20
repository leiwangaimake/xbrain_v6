"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_commit.py
Brief: BIZ-P3-43 commit a recorded route / fence into geo.db / fence.db

Description:
The COMMIT step of teach recording (11 S12A / 15 S9.6). teach.dedup_run turns
a dense pose stream into a sparse point list; this writes that list to disk as
a real geo object, atomically:

  commit_route(conn, ...)  -> geo.db: ONE routes row with INLINE geometry
      (path_points [[lat,lon]] mode B, or waypoint_ids [geo_id] mode A, XOR),
      loop_mode / direction / computed total_len_m. No waypoints and no assoc
      rows -- route vertices are no longer keypoints (PLAN A, 15 S9.3).
  commit_waypoint(conn, ...) -> geo.db: ONE named keypoint (WGS84 rtk_lat/lon).
  commit_fence(conn, ...)  -> fence.db: one fences row with its 11 S9A.2 role
      (allow/forbid/speed_limit/warning), geom_json = validated WGS84 vertex ring.

Before this, teach.py had only the dedup functions -- the '录完了 -> save to
geo' path that 11 S12A promises had no writer, so a recorded route/fence was
lost. This module is that writer.

It does NOT read a clock (now_ms injected), open the connection (the caller
passes a live geo/fence conn from the persistence layer), or accumulate points
(that is the pose-driven recording session, the execution-wiring batch). It
just persists an already-collected, already-validated geometry.

geo ids are '<prefix>-<slug>' (15 S9.3 ID-2): r- route, w- waypoint, f- fence.
The route_id / fence_id is supplied by the caller (the save handler mints it);
waypoint ids are derived from the route id so they are unique without a second
allocator.
"""
from __future__ import annotations

import json
import math
from typing import Optional, Sequence

from xbrain.p3_task.fence.geom import validate_polygon
from xbrain.p3_task.state.geo_rev import content_hash


# A route needs at least two points to be a path. The upper bound is the 11 S7.8.3
# RouteGeometry cap (<= 5000 vertices); the old 16 was the retired assoc trigger.
_MIN_ROUTE_POINTS = 2
_MAX_PATH_POINTS = 5000

_EARTH_R_M = 6371000.0     # mean earth radius; total_len_m is a display/replay len,
#                            not a survey figure, so a spherical model is enough.


def _haversine_m(a, b) -> float:
    """Great-circle distance in metres between two (lat, lon) degree pairs."""
    la1, lo1, la2, lo2 = (math.radians(a[0]), math.radians(a[1]),
                          math.radians(b[0]), math.radians(b[1]))
    h = (math.sin((la2 - la1) / 2.0) ** 2
         + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2.0) ** 2)
    return 2.0 * _EARTH_R_M * math.asin(min(1.0, math.sqrt(h)))


def _polyline_len_m(pts: Sequence) -> float:
    """Sum of segment lengths over an ordered (lat, lon) list (11 S7.8.3)."""
    return sum(_haversine_m(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


class GeoCommitError(RuntimeError):
    """A recorded geometry cannot be committed (too few/many points, etc)."""


async def commit_route(conn, *, route_id: str, name: str,
                       path_points: Optional[Sequence] = None,
                       waypoint_ids: Optional[Sequence] = None,
                       loop_mode: str = "oneway", direction: str = "forward",
                       max_speed: Optional[float] = None,
                       description: Optional[str] = None, now_ms: int) -> str:
    """Write a route to geo.db as ONE routes row with INLINE geometry (15 S9.3).
    Geometry is XOR:
      * path_points  = [(lat, lon), ...]  mode B (voice recording, 11 S7.8.3)
      * waypoint_ids = [geo_id, ...]      mode A (named anchors, config import)
    total_len_m is computed here (haversine): over path_points for mode B, or over
    the resolved anchors' rtk_lat/lon for mode A. NO waypoints rows and NO assoc
    rows are written -- route vertices are no longer keypoints (that was the old
    assoc-geometry model, retired by PLAN A). Returns route_id."""
    has_pp, has_wi = path_points is not None, waypoint_ids is not None
    if has_pp == has_wi:                          # XOR (mirror the routes CHECK, clearer error)
        raise GeoCommitError(
            "route needs exactly one of path_points / waypoint_ids")
    if loop_mode not in ("oneway", "pingpong", "closed"):
        raise GeoCommitError(f"bad loop_mode {loop_mode!r}")
    if direction not in ("forward", "reverse"):
        raise GeoCommitError(f"bad direction {direction!r}")
    await conn.execute("BEGIN IMMEDIATE")
    try:
        if has_pp:
            pts = [(float(la), float(lo)) for (la, lo) in path_points]
            n = len(pts)
            if n < _MIN_ROUTE_POINTS:
                raise GeoCommitError(
                    f"route needs >= {_MIN_ROUTE_POINTS} points, got {n}")
            if n > _MAX_PATH_POINTS:
                raise GeoCommitError(
                    f"route has {n} points, exceeds the cap {_MAX_PATH_POINTS}")
            total = _polyline_len_m(pts)
            if total <= 0.0:                      # keep the routes CHECK from aborting cryptically
                raise GeoCommitError("route has zero length (all points coincide)")
            geom_col, geom_json = "path_points", json.dumps(
                [[la, lo] for (la, lo) in pts], separators=(",", ":"))
            ch = content_hash({"pp": [[la, lo] for (la, lo) in pts], "name": name})
        else:
            wids = [str(w) for w in waypoint_ids]
            if len(wids) < _MIN_ROUTE_POINTS:
                raise GeoCommitError(
                    f"route needs >= {_MIN_ROUTE_POINTS} anchors, got {len(wids)}")
            coords = []
            for w in wids:                        # resolve anchors -> coords for total_len_m
                cur = await conn.execute(
                    "SELECT rtk_lat, rtk_lon FROM waypoints WHERE geo_id=?", (w,))
                row = await cur.fetchone()
                if row is None:
                    raise GeoCommitError(f"anchor waypoint {w!r} not found")
                coords.append((row[0], row[1]))
            total = _polyline_len_m(coords)
            if total <= 0.0:
                raise GeoCommitError("route has zero length (anchors coincide)")
            geom_col, geom_json = "waypoint_ids", json.dumps(
                wids, separators=(",", ":"))
            ch = content_hash({"wi": wids, "name": name})
        await conn.execute(
            f"INSERT INTO routes (geo_id, name, {geom_col}, max_speed, loop_mode, "
            " direction, total_len_m, description, content_hash, updated_ms) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (route_id, name, geom_json, max_speed, loop_mode, direction,
             total, description, ch, now_ms))
        await conn.commit()
    except Exception:
        await conn.rollback()
        raise
    return route_id


async def commit_waypoint(conn, *, geo_id: str, name: str, wtype: str,
                          rtk_lat: float, rtk_lon: float,
                          rtk_alt: Optional[float] = None,
                          yaw_deg: Optional[float] = None,
                          arrival_radius: float = 1.0,
                          description: Optional[str] = None,
                          now_ms: int) -> str:
    """Write ONE named keypoint to geo.db -- the F06 record_waypoint path
    (ba zhe li ji wei X, 18-C). name is the operator-given display name
    (11 S7.8.2, NOT NULL UNIQUE so voice "go to <name>" resolves to one target);
    rtk_lat/rtk_lon come from the PS-1 pose snapshot at record time (rtk_fixed
    required by the caller, 18 F06); yaw_deg is the capture heading. Idempotent
    per geo_id via INSERT OR REPLACE -- a redelivered record command overwrites
    rather than duplicating. Caveat: OR REPLACE resolves ANY unique conflict, so
    re-committing a geo_id with a name that already belongs to a DIFFERENT
    keypoint would delete that other row; callers mint one geo_id per name."""
    ch = content_hash({"name": name, "lat": rtk_lat, "lon": rtk_lon,
                       "yaw": yaw_deg})
    await conn.execute("BEGIN IMMEDIATE")
    try:
        await conn.execute(
            "INSERT OR REPLACE INTO waypoints (geo_id, name, type, rtk_lat, "
            " rtk_lon, rtk_alt, yaw_deg, arrival_radius, description, "
            " content_hash, updated_ms) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (geo_id, name, wtype, rtk_lat, rtk_lon, rtk_alt, yaw_deg,
             arrival_radius, description, ch, now_ms))
        await conn.commit()
    except Exception:
        await conn.rollback()
        raise
    return geo_id


async def commit_fence(conn, *, fence_id: str, role: str, points: Sequence,
                       now_ms: int, name: Optional[str] = None,
                       soft_margin_m: Optional[float] = None) -> str:
    """Write a polygon fence to fence.db with its 11 S9A.2 role (allow | forbid |
    speed_limit | warning) and display name. `points` is the WGS84 vertex ring
    (lat, lon) WITHOUT a duplicated closing vertex (closes implicitly), validated
    (FS-4: >= 3 unique points, non-zero area) before insert. hard_enforce is
    derived: warning fences never hard-enforce (S9A.2), all others do. The S9A.1A
    count invariants (<= 5 active, at most 1 allow) are enforced by the fence.db
    triggers; the "exactly 1 allow" existence half is a set check at broadcast
    time (validate_active_fence_set)."""
    if role not in ("allow", "forbid", "speed_limit", "warning"):
        raise GeoCommitError(f"bad fence role {role!r}")
    validate_polygon(points)                   # raises InvalidPolygon on a bad ring
    verts = [[float(a), float(b)] for a, b in points]
    geom_json = json.dumps({"points": verts}, separators=(",", ":"))
    ch = content_hash({"points": verts, "role": role, "name": name})
    hard_enforce = 0 if role == "warning" else 1
    await conn.execute("BEGIN IMMEDIATE")
    try:
        await conn.execute(
            "INSERT INTO fences (fence_id, name, role, kind, geom_json, "
            " hard_enforce, soft_margin_m, content_hash, updated_ms) "
            "VALUES (?, ?, ?, 'polygon', ?, ?, ?, ?, ?)",
            (fence_id, name, role, geom_json, hard_enforce, soft_margin_m, ch, now_ms))
        await conn.commit()
    except Exception:
        await conn.rollback()
        raise
    return fence_id
