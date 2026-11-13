"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: reach.py
Brief: 99 U184 -- which waypoints / routes a fence makes unreachable, and where the robot stands

Description:
User ruling 2026-11-13: "所有航点, 路径, 充电站都不能在禁止区域内, 否则无意义";
a fence outside the activity area "系统不报错, 但是永远到不了". So nothing is
refused for these; they are REPORTED (an event to the cloud, a sentence when a
voice command caused it):
  * an activity area (allow) leaves a waypoint / route outside it -> the robot
    can never get there (P1 clips at the boundary, 11 S9A.6);
  * a forbid zone covers a waypoint, or part of a route -> same;
  * the robot itself is outside an activity area, or inside a forbid zone, at
    the moment that fence is switched on -> it can only drive back in / out.

When the checks run (the callers):
  * a fence is switched on (cmd/geo set_state active: F15 and the cloud), and
    a fence is saved by recording -- against that fence's own ring;
  * a waypoint or route is saved while fences are in force -- against the
    ACTIVE activity area and forbid zones.

What this module is NOT: it refuses nothing, and it does not decide whether a
fence overlaps another (teach/validate.ring_overlap, refused at save). A dock
or home point is just a waypoint here; its name is what gets reported.

Geometry is the lat/lon-plane test of teach/validate (point_in_ring), the same
camp-scale argument: inside / outside is affine-invariant. A route counts as
unreachable when ANY of its points is (a route the robot can only half drive
is not the route the operator recorded).
"""

from __future__ import annotations  # postponed annotations

import json  # fence.db geom_json
from typing import Any, Dict, List, Optional, Sequence, Tuple  # annotations

from xbrain.p3_task.geo.objects import read_geo_objects  # the same reader the map uses
from xbrain.p3_task.teach.validate import point_in_ring  # the recording's own inside test

Ring = Sequence[Tuple[float, float]]  # [(lat, lon)], implicitly closed

#: report reasons (event detail.reason, the P4 sentence picks its wording by it)
OUTSIDE_ACTIVITY_AREA = "outside_activity_area"  # left out by an activity area
INSIDE_FORBID = "inside_forbid"  # covered by a forbid zone


def _blocked(role: str, ring: Ring, lat: float, lon: float) -> bool:
    """Is (lat, lon) unreachable because of this one fence?"""
    inside = point_in_ring((lat, lon), ring)  # lat/lon-plane ray cast
    return (role == "allow" and not inside) or (role == "forbid" and inside)  # the two boundaries


def unreachable_objects(role: str, ring: Ring, waypoints: Sequence[Dict[str, Any]],
                        routes: Sequence[Dict[str, Any]]) -> List[str]:
    """Names of the waypoints / routes this fence makes unreachable, waypoints
    first, each in the given order. Only allow and forbid constrain motion; any
    other role answers []. `waypoints` / `routes` are state/geo/objects items
    ({name, lat, lon} / {name, points: [{lat, lon}]})."""
    if role not in ("allow", "forbid") or len(ring) < 3:  # warning / speed: no boundary
        return []  # nothing it can make unreachable
    out = [w.get("name") or w.get("geo_id") for w in waypoints  # a point is one position
           if w.get("lat") is not None and w.get("lon") is not None  # positionless: skipped
           and _blocked(role, ring, float(w["lat"]), float(w["lon"]))]  # blocked here
    out += [r.get("name") or r.get("geo_id") for r in routes  # a route: any point blocked
            if any(_blocked(role, ring, float(p["lat"]), float(p["lon"]))  # one is enough
                   for p in (r.get("points") or ()))]  # every recorded point
    return out  # waypoints first, then routes


def robot_note(role: str, ring: Ring, lat: Optional[float],
               lon: Optional[float]) -> Optional[str]:
    """Where the robot stands relative to a fence being switched on:
    OUTSIDE_ACTIVITY_AREA / INSIDE_FORBID, or None (fine, another role, or no
    position -- an unknown position is not reported as either)."""
    if lat is None or lon is None or len(ring) < 3:  # no fix, or not a ring
        return None  # say nothing rather than guess
    if _blocked(role, ring, float(lat), float(lon)):  # the robot itself is shut out
        return OUTSIDE_ACTIVITY_AREA if role == "allow" else INSIDE_FORBID  # which side
    return None  # fine where it is


def _ring(geom_json: str) -> Optional[List[Tuple[float, float]]]:
    """fence.db geom_json -> [(lat, lon)], or None if unreadable."""
    try:
        return [(float(a), float(b)) for a, b in json.loads(geom_json)["points"]]  # {"points": [[lat, lon]]}
    except (ValueError, KeyError, TypeError):  # a malformed row
        return None  # the caller leaves it out


async def active_constraints(fence_conn) -> List[Tuple[str, str, Ring]]:
    """The fences in force that constrain motion: [(role, name, ring)] of the
    active allow / forbid rows. An unreadable row is left out (cmd/fence drops
    it too, so it is not in force either)."""
    cur = await fence_conn.execute(  # in force = active, live, a boundary role
        "SELECT role, name, geom_json FROM fences WHERE state='active' "
        "AND tombstone=0 AND role IN ('allow','forbid') ORDER BY fence_id")
    out = []  # (role, name, ring)
    for role, name, geom in await cur.fetchall():  # each fence in force
        ring = _ring(geom)  # its vertices
        if ring is not None:  # unreadable rows are not in force (cmd/fence drops them)
            out.append((role, name or "", ring))  # kept
    return out  # in fence_id order


async def fence_row(fence_conn, fence_id: str) -> Optional[Tuple[str, str, Ring]]:
    """(role, name, ring) of one live fence, or None (gone / unreadable)."""
    cur = await fence_conn.execute(  # one row by id, tombstones excluded
        "SELECT role, name, geom_json FROM fences WHERE fence_id=? AND tombstone=0",
        (fence_id,))
    row = await cur.fetchone()  # None when gone
    ring = _ring(row[2]) if row is not None else None  # its vertices
    return None if ring is None else (row[0], row[1] or "", ring)  # role, name, ring


def unreachable_event(entry: Dict[str, Any], trigger: str) -> Dict[str, Any]:
    """The event/warn/fence body detail for one report entry (11 S6.2 fence
    category). trigger: enable (a fence switched on) | save (a fence / waypoint
    / route saved). The P3 wiring adds eid / ts / src."""
    return {"kind": "objects_unreachable", "trigger": trigger,  # what and why now
            "poly_name": entry["fence"], "role": entry["role"],  # which fence
            "reason": entry["reason"], "objects": list(entry["objects"])}  # what is cut off


def report(role: str, fence_name: str, objects: List[str]) -> List[Dict[str, Any]]:
    """[] or one report entry {fence, role, reason, objects} for the ack detail
    and the event (the shape both the event and P4 read)."""
    if not objects:  # nothing out of reach
        return []  # no entry at all
    return [{"fence": fence_name, "role": role,  # the fence that blocks
             "reason": OUTSIDE_ACTIVITY_AREA if role == "allow" else INSIDE_FORBID,  # how
             "objects": objects}]  # the names it blocks


async def fence_reports(geo_conn, role: str, fence_name: str, ring: Ring) -> List[Dict[str, Any]]:
    """The report for one fence against every live waypoint / route in geo.db
    (switching it on, or saving it)."""
    if role not in ("allow", "forbid"):  # only boundaries cut anything off
        return []  # warning / speed zones
    geo = await read_geo_objects(geo_conn)  # every live waypoint and route
    return report(role, fence_name, unreachable_objects(  # one entry or none
        role, ring, geo["waypoints"], geo["routes"]))


async def object_reports(fence_conn, *, name: str,
                         points: Sequence[Tuple[float, float]]) -> List[Dict[str, Any]]:
    """The reports for one new waypoint (one point) or route (its points)
    against the fences in force: one entry per fence that blocks it."""
    item = {"name": name, "points": [{"lat": a, "lon": b} for a, b in points]}  # route-shaped
    out: List[Dict[str, Any]] = []  # one entry per blocking fence
    for role, fname, ring in await active_constraints(fence_conn):  # only fences in force
        out += report(role, fname, unreachable_objects(role, ring, [], [item]))  # judged as a route
    return out  # [] when nothing blocks it


__all__ = ["INSIDE_FORBID", "OUTSIDE_ACTIVITY_AREA", "active_constraints",
           "fence_reports", "fence_row", "object_reports", "report", "robot_note",
           "unreachable_event", "unreachable_objects"]
