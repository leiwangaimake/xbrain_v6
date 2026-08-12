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

  commit_route(conn, ...)  -> geo.db: one waypoints row per point, one routes
      row, and the route_waypoint_assoc order. All under one BEGIN IMMEDIATE so
      a crash mid-write never leaves a route pointing at half its waypoints.
  commit_fence(conn, ...)  -> fence.db: one fences row (kind='polygon') whose
      geom_json is the validated vertex ring.

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
from typing import Optional, Sequence

from xbrain.p3_task.fence.geom import validate_polygon
from xbrain.p3_task.state.geo_rev import content_hash


# A route needs at least two points to be a path; the per-route waypoint cap is
# 16 (schema_geo trigger). Reject early with the count, not only at the trigger.
_MIN_ROUTE_POINTS = 2
_MAX_ROUTE_POINTS = 16


class GeoCommitError(RuntimeError):
    """A recorded geometry cannot be committed (too few/many points, etc)."""


async def commit_route(conn, *, route_id: str, name: str,
                       samples: Sequence, now_ms: int) -> str:
    """Write a recorded route to geo.db (waypoints + routes + assoc) under one
    transaction. `samples` are objects with .x_m/.y_m/.heading_rad (teach
    TeachSample, already deduped). Returns route_id."""
    n = len(samples)
    if n < _MIN_ROUTE_POINTS:
        raise GeoCommitError(f"route needs >= {_MIN_ROUTE_POINTS} points, got {n}")
    if n > _MAX_ROUTE_POINTS:
        raise GeoCommitError(
            f"route has {n} points, exceeds the per-route cap {_MAX_ROUTE_POINTS}")
    await conn.execute("BEGIN IMMEDIATE")
    try:
        wp_ids = []
        for i, s in enumerate(samples):
            wid = f"{route_id}-w{i:03d}"       # unique via the unique route_id
            ch = content_hash({"x": s.x_m, "y": s.y_m, "h": s.heading_rad})
            await conn.execute(
                "INSERT INTO waypoints (waypoint_id, x_m, y_m, heading_rad, "
                " content_hash, updated_ms) VALUES (?, ?, ?, ?, ?, ?)",
                (wid, s.x_m, s.y_m, s.heading_rad, ch, now_ms))
            wp_ids.append(wid)
        await conn.execute(
            "INSERT INTO routes (route_id, name, content_hash, updated_ms) "
            "VALUES (?, ?, ?, ?)",
            (route_id, name, content_hash({"name": name, "wps": wp_ids}), now_ms))
        for seq, wid in enumerate(wp_ids):
            await conn.execute(
                "INSERT INTO route_waypoint_assoc (route_id, seq, waypoint_id) "
                "VALUES (?, ?, ?)", (route_id, seq, wid))
        await conn.commit()
    except Exception:
        await conn.rollback()
        raise
    return route_id


async def commit_fence(conn, *, fence_id: str, points: Sequence,
                       now_ms: int, zone_label: Optional[str] = None) -> str:
    """Write a recorded polygon fence to fence.db. `points` is the vertex ring
    WITHOUT a duplicated closing vertex (the geometry closes implicitly). It is
    validated (FS-4: >= 3 unique points, non-zero area) before the insert."""
    validate_polygon(points)                   # raises InvalidPolygon on a bad ring
    geom_json = json.dumps(
        {"points": [[float(x), float(y)] for x, y in points]},
        separators=(",", ":"))
    ch = content_hash({"points": [[float(x), float(y)] for x, y in points]})
    await conn.execute("BEGIN IMMEDIATE")
    try:
        await conn.execute(
            "INSERT INTO fences (fence_id, kind, geom_json, zone_label, "
            " content_hash, updated_ms) VALUES (?, 'polygon', ?, ?, ?, ?)",
            (fence_id, geom_json, zone_label, ch, now_ms))
        await conn.commit()
    except Exception:
        await conn.rollback()
        raise
    return fence_id
