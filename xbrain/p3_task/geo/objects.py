"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: objects.py
Brief: Read geo.db into the state/geo/objects broadcast payload (11 S7.10A)

Description:
The HMI is a real-time visualisation relay (17 S6.10.4): every displayable datum
reaches P5 by a live Zenoh path, and P5 never reads P3's DBs (11 S7843 single
writer). For the map's routes / keypoints / docks, P3 reads geo.db HERE and
broadcasts the full geometry on state/geo/objects (11 S7.10A); P5 caches it and
relays to the browser. (Fences keep their own cmd/fence path, 11 S9A.2.)

This module is the read + shape half (pure async against a live aiosqlite conn on
P3's single db thread, 15 S2.1). Geometry is emitted in ENU metres (e_m/n_m) --
the runtime waypoints/docks tables store x_m/y_m, and the frontend's toXY places
an [e_m, n_m] point with NO enu_origin (11 S7.10A GO-1). A route's geometry is its
ordered waypoints' coordinates (route_waypoint_assoc -> waypoints), not a separate
column. Tombstoned rows are excluded. A waypoint/dock with no name is emitted with
name=None; the frontend draws an unlabelled point rather than a fabricated name
(GO-4).

catalog_rev is max(updated_ms) across the live geo rows: it changes on any edit,
so P5 can cheaply tell "did the geo set change" without diffing the whole payload
(GO-2). It is 0 for an empty/absent geo set.
"""

from __future__ import annotations

from typing import Any, Dict, List

GEO_OBJECTS_SCHEMA = "geo_objects_v1"


async def read_geo_objects(conn) -> Dict[str, Any]:
    """Read the live (non-tombstoned) geo objects into a GeoObjects payload
    (11 S7.10A). `conn` is a live aiosqlite connection to geo.db."""
    waypoints = await _read_waypoints(conn)
    docks = await _read_docks(conn)
    routes = await _read_routes(conn)
    catalog_rev = await _catalog_rev(conn)
    return {
        "schema": GEO_OBJECTS_SCHEMA,
        "catalog_rev": catalog_rev,
        "waypoints": waypoints,
        "routes": routes,
        "docks": docks,
    }


async def _read_waypoints(conn) -> List[Dict[str, Any]]:
    # The "waypoints" layer is the KEYPOINT layer -- named standalone RTK points
    # the operator deliberately saved (18 F06), shown as labelled dots. Route
    # vertices are ALSO stored in the waypoints table (the runtime routes table
    # has no geometry column, 15 S9.3 assoc model), but they are a route's
    # polyline geometry, NOT keypoints -- they must not each draw their own dot.
    # Exclude any waypoint referenced by route_waypoint_assoc so route vertices
    # render only as the line, and only true keypoints appear as dots. A keypoint
    # that merely lies near a route is a SEPARATE (un-assoc'd) waypoint, so it is
    # kept.
    cur = await conn.execute(
        "SELECT waypoint_id, name, x_m, y_m, heading_rad, rev "
        "FROM waypoints WHERE tombstone=0 "
        "AND waypoint_id NOT IN (SELECT waypoint_id FROM route_waypoint_assoc) "
        "ORDER BY waypoint_id")
    rows = await cur.fetchall()
    return [{"geo_id": r[0], "name": r[1], "e_m": r[2], "n_m": r[3],
             "heading_rad": r[4], "rev": r[5]} for r in rows]


async def _read_docks(conn) -> List[Dict[str, Any]]:
    cur = await conn.execute(
        "SELECT dock_id, name, x_m, y_m, heading_rad, rev "
        "FROM docks WHERE tombstone=0 ORDER BY dock_id")
    rows = await cur.fetchall()
    return [{"geo_id": r[0], "name": r[1], "e_m": r[2], "n_m": r[3],
             "heading_rad": r[4], "rev": r[5]} for r in rows]


async def _read_routes(conn) -> List[Dict[str, Any]]:
    cur = await conn.execute(
        "SELECT route_id, name, rev FROM routes WHERE tombstone=0 "
        "ORDER BY route_id")
    routes = await cur.fetchall()
    out: List[Dict[str, Any]] = []
    for route_id, name, rev in routes:
        # A route's geometry = its ordered waypoints' coordinates (there is no
        # geometry column in the runtime routes table -- 15 S9.3 assoc model).
        pc = await conn.execute(
            "SELECT w.x_m, w.y_m FROM route_waypoint_assoc a "
            "JOIN waypoints w ON a.waypoint_id = w.waypoint_id "
            "WHERE a.route_id = ? ORDER BY a.seq", (route_id,))
        points = [[x, y] for (x, y) in await pc.fetchall()]
        out.append({"geo_id": route_id, "name": name, "rev": rev,
                    "points": points})
    return out


async def _catalog_rev(conn) -> int:
    """max(updated_ms) across the live geo tables -- a monotonic change token
    (GO-2). 0 when the geo set is empty (never None, so P5's compare is simple)."""
    cur = await conn.execute(
        "SELECT COALESCE(MAX(m), 0) FROM ("
        "  SELECT MAX(updated_ms) m FROM waypoints WHERE tombstone=0 "
        "  UNION ALL SELECT MAX(updated_ms) FROM routes WHERE tombstone=0 "
        "  UNION ALL SELECT MAX(updated_ms) FROM docks WHERE tombstone=0)")
    row = await cur.fetchone()
    return int(row[0]) if row and row[0] is not None else 0
