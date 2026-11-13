"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: objects.py
Brief: Read geo.db + fence.db into the state/geo/objects broadcast payload (11 S7.10A)

Description:
The HMI is a real-time visualisation relay (17 S6.10.4): every displayable datum
reaches P5 by a live Zenoh path, and P5 never reads P3's DBs (11 S7843 single
writer). For the map's routes / keypoints / docks, P3 reads geo.db HERE and
broadcasts the full geometry on state/geo/objects (11 S7.10A); P5 caches it and
relays to the browser. ~~(Fences keep their own cmd/fence path, 11 S9A.2.)~~

99 U183 (user 2026-11-13): fences ride here too, EVERY live fence with its state
and role. cmd/fence (11 S9A.2) still carries the ENFORCED set to P1, and it holds
only active fences -- so a fence just recorded, or switched off, appeared nowhere
the HMI or the cloud could see it: the operator could not see a recording's
result on the map, and the cloud could not even find the activity area it is
meant to switch on. This payload is the catalogue (what exists), cmd/fence is
what is in force; a consumer that wants "in force" reads state == 'active'.

This module is the read + shape half (pure async against a live aiosqlite conn on
P3's single db thread, 15 S2.1). v1.5 PLAN A: geometry is emitted in WGS84 as
{lat, lon} (waypoints/routes are the 15 S9.3 model now); the frontend's toXY
projects {lat,lon} through the HMI display origin -- the robot's GPS fix (99 U192;
~~the shared enu_origin~~ is not on any broadcast since 99 U193) (11 S7.10A GO-1).
Route points are emitted as {lat,lon} OBJECTS, never [a,b] arrays -- an array would
be read as ENU metres by toXY and land the route in the wrong frame. A route's
geometry is INLINE (routes.path_points mode B, or routes.waypoint_ids resolved to
their keypoints mode A), NOT the old assoc vertex list. Tombstoned rows are
excluded. A waypoint with no name never happens now (name is NOT NULL). docks are
still ENU (e_m/n_m) -- out of scope for this migration and not rendered on the map.

Every object carries its lifecycle `state` (11 S7.8.2). The map shows draft and
disabled objects as well as active ones -- an operator who just recorded a route
must SEE it, and a route that exists but is switched off is exactly what the
operator needs to be told about. Only tombstones (state='deleted') are withheld.
Distinguishing the three visible states is the frontend's job; withholding the
field would make that impossible, and withholding drafts would make a freshly
recorded route look like a recording that failed.

catalog_rev is max(updated_ms) across the live geo rows: it changes on any edit,
so P5 can cheaply tell "did the geo set change" without diffing the whole payload
(GO-2). It is 0 for an empty/absent geo set.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List

GEO_OBJECTS_SCHEMA = "geo_objects_v1"


async def read_geo_objects(conn, fence_conn=None) -> Dict[str, Any]:
    """Read the live (non-tombstoned) geo objects into a GeoObjects payload
    (11 S7.10A). `conn` is a live aiosqlite connection to geo.db, `fence_conn`
    one to fence.db (99 U183); without it the payload has no fences key, which
    is what a geo-only caller (tests, the delete sweep) gets."""
    waypoints = await _read_waypoints(conn)
    docks = await _read_docks(conn)
    routes = await _read_routes(conn)
    catalog_rev = await _catalog_rev(conn)
    out: Dict[str, Any] = {
        "schema": GEO_OBJECTS_SCHEMA,
        "catalog_rev": catalog_rev,
        "waypoints": waypoints,
        "routes": routes,
        "docks": docks,
    }
    if fence_conn is not None:
        fences, fence_rev = await _read_fences(fence_conn)
        out["fences"] = fences
        # GO-2: one change token over everything this payload carries, so a
        # fence switched on or off changes it as a route edit does
        out["catalog_rev"] = max(catalog_rev, fence_rev)
    return out


async def _read_fences(conn):
    """([fence], max updated_ms) of every live fence (99 U183). vertices are
    {lat,lon} objects like route points (GO-1); speed_limit_mps rides only on a
    speed_limit fence, as on cmd/fence (11 S9A.2 FV-13)."""
    cur = await conn.execute(
        "SELECT fence_id, name, role, state, geom_json, rev, speed_limit_mps, "
        "updated_ms FROM fences WHERE state<>'deleted' AND tombstone=0 "
        "ORDER BY fence_id")
    out: List[Dict[str, Any]] = []
    for fid, name, role, state, geom, frev, speed, _updated in await cur.fetchall():
        try:
            pts = json.loads(geom)["points"]
            vertices = [{"lat": float(a), "lon": float(b)} for a, b in pts]
        except (ValueError, KeyError, TypeError):
            continue  # an unreadable row is not drawn (cmd/fence drops it too)
        item: Dict[str, Any] = {"geo_id": fid, "name": name, "role": role,
                                "state": state, "rev": frev, "vertices": vertices}
        if role == "speed_limit" and speed is not None:
            item["speed_limit_mps"] = float(speed)
        out.append(item)
    # The token is taken over EVERY row, tombstones included: deleting the
    # newest fence must still move it, or a consumer comparing tokens keeps
    # drawing the deleted fence.
    cur = await conn.execute("SELECT COALESCE(MAX(updated_ms), 0) FROM fences")
    row = await cur.fetchone()
    return out, int(row[0] or 0)


async def _read_waypoints(conn) -> List[Dict[str, Any]]:
    # The "waypoints" layer is the KEYPOINT layer -- named WGS84 points the
    # operator deliberately saved (18 F06), shown as labelled dots. v1.5 PLAN A:
    # route vertices are INLINE on routes now, so EVERY waypoint row is a keypoint
    # (no assoc exclusion). lat/lon are emitted for the frontend to project.
    # type (poi / dock / home, set from the name at record time, 99 U119) rides
    # along since 2026-11-12: the HMI draws a charging point as a numbered
    # yellow disc and the return-home point as a green one (17 S6.10.2 U6).
    cur = await conn.execute(
        "SELECT geo_id, name, rtk_lat, rtk_lon, yaw_deg, rev, state, type "
        "FROM waypoints WHERE state<>'deleted' AND tombstone=0 ORDER BY geo_id")
    rows = await cur.fetchall()
    return [{"geo_id": r[0], "name": r[1], "lat": r[2], "lon": r[3],
             "yaw_deg": r[4], "rev": r[5], "state": r[6], "type": r[7]}
            for r in rows]


async def _read_docks(conn) -> List[Dict[str, Any]]:
    # v1.5: docks are the full 15 S9.3 WGS84 model now; emit lat/lon like
    # waypoints (the frontend does not render docks, but the payload stays
    # coordinate-consistent). heading is the dock body approach heading.
    cur = await conn.execute(
        "SELECT geo_id, name, rtk_lat, rtk_lon, dock_heading_rad, rev, state "
        "FROM docks WHERE state<>'deleted' AND tombstone=0 ORDER BY geo_id")
    rows = await cur.fetchall()
    return [{"geo_id": r[0], "name": r[1], "lat": r[2], "lon": r[3],
             "heading_rad": r[4], "rev": r[5], "state": r[6]} for r in rows]


async def _read_routes(conn) -> List[Dict[str, Any]]:
    cur = await conn.execute(
        "SELECT geo_id, name, waypoint_ids, path_points, loop_mode, rev, state "
        "FROM routes WHERE state<>'deleted' AND tombstone=0 ORDER BY geo_id")
    routes = await cur.fetchall()
    out: List[Dict[str, Any]] = []
    for (geo_id, name, waypoint_ids, path_points, loop_mode, rev,
         state) in routes:
        # Geometry is INLINE (15 S9.3): mode B = path_points [[lat,lon]] verbatim;
        # mode A = waypoint_ids resolved to each anchor keypoint's rtk_lat/lon. A
        # deleted anchor simply drops from the line (never a fabricated point).
        if path_points is not None:
            points = [{"lat": la, "lon": lo}
                      for (la, lo) in json.loads(path_points)]
        else:
            points = []
            for wid in json.loads(waypoint_ids or "[]"):
                wc = await conn.execute(
                    "SELECT rtk_lat, rtk_lon FROM waypoints WHERE geo_id=?", (wid,))
                wr = await wc.fetchone()
                if wr is not None:
                    points.append({"lat": wr[0], "lon": wr[1]})
        out.append({"geo_id": geo_id, "name": name, "rev": rev,
                    "loop_mode": loop_mode, "points": points, "state": state})
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
