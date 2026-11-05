"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_queries.py
Brief: G50 / G51 / G52 answers -- nearest waypoint, how far to X, how long is route X (99 U136)

Description:
The candidate queries listed with 99 U130 and picked by the user on
2026-11-05 ("最近航点, 去 X 多远, 路径多长 ... 现在可以做就做掉"):

  G50 nearest   "离我最近的航点 / 最近的航点是哪个"
                -> the live waypoint nearest the robot, straight line
  G51 distance  "去X多远 / 到X有多远 / 离X多远 / 去X要多久"
                -> the length of the path a goto to X would TAKE (the 99
                U127 plan from xbrain.common.geo.goto_plan, so the spoken
                distance is the one the robot drives), plus the time at
                the voice-set move speed when it is known (state/mode
                move_speed.mps, 99 U106)
  G52 length    "X路径多长 / X路径有多长"
                -> the route polyline's length (a closed route includes
                its closing segment)

Sources are what P4 already caches: state/geo/objects (routes and waypoints
with WGS84 geometry), state/pose (the robot) and state/mode (the speed).
Everything is local geometry over a handful of objects, so every answer is
given at once (the "正在查询, 请稍等" fallback of the U130 ruling is for an
answer that cannot be). make_geo_query_fn(membership_m) takes the U127 rule's
distance as a REQUIRED argument -- common.geo.route_membership_m, the same
key P3 navigates by -- and nothing here defaults it (CLAUDE.md 3.1).

What it does NOT do: no catalogue writes, no fuzzy names beyond the
canonical-name rule (一号 == 1号), no ETA promise -- the time is distance /
set speed, said as "约" (obstacles and the speed gate slow the robot).

Traps:
  * Answering G51 with the straight line when a route would be followed:
    the robot then drives farther than it said.
  * Using a stale pose: state/pose is the robot's position NOW (18 PS-5:
    queries use the latest value, not a first-word snapshot); no pose at all
    is said plainly, never replaced by the last known one.
"""
from __future__ import annotations  # postponed annotations

import math  # hypot / ceil
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple  # plain value types

from xbrain.common.geo.goto_plan import RouteGeom, plan_goto  # the U127 plan P3 drives
from xbrain.common.geo.polyline import Plane, Polyline  # shared geometry
from xbrain.p4_agent.slots.name_slots import canonical_name  # 一号 == 1号

#: (template key, format fields) -- the orchestrator owns the spoken strings.
Answer = Tuple[str, Dict[str, Any]]


def _live(items: Any, need: Callable[[Mapping], bool]) -> List[Mapping]:
    """The active, well-formed entries of one objects layer."""
    if not isinstance(items, list):  # layer missing
        return []  # nothing usable
    return [o for o in items if isinstance(o, Mapping)
            and o.get("state", "active") == "active" and need(o)]  # live ones


def _point_ok(w: Mapping) -> bool:
    """A waypoint with a position."""
    return all(isinstance(w.get(k), (int, float)) and not isinstance(w.get(k), bool)
               for k in ("lat", "lon"))  # numeric lat / lon


def _route_ok(r: Mapping) -> bool:
    """A route with a polyline."""
    return isinstance(r.get("points"), list) and bool(r["points"])  # something to measure


def _ll(route: Mapping) -> Tuple[Tuple[float, float], ...]:
    """The route polyline as (lat, lon) tuples."""
    return tuple((float(p["lat"]), float(p["lon"])) for p in route["points"])  # WGS84


def _find(items: List[Mapping], name: str) -> Optional[Mapping]:
    """The object whose name equals name under the canonical-name rule."""
    want = canonical_name(name)  # 一号路径 == 1号路径
    return next((o for o in items if canonical_name(str(o.get("name", ""))) == want), None)  # first match


def _robot(pose: Any) -> Optional[Tuple[float, float]]:
    """(lat, lon) from a state/pose body, or None (no usable position)."""
    if not isinstance(pose, Mapping) or not _point_ok(pose):  # absent / malformed
        return None  # never a guessed point
    return float(pose["lat"]), float(pose["lon"])  # now


def _speed(mode: Any) -> Optional[float]:
    """move_speed.mps from a state/mode body, or None (11 S4.3)."""
    ms = mode.get("move_speed") if isinstance(mode, Mapping) else None  # the setting
    v = ms.get("mps") if isinstance(ms, Mapping) else None  # current value
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0.0:
        return None  # not known
    return float(v)  # m/s


def _straight_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Straight-line distance in metres (local plane around a)."""
    plane = Plane(a)  # metres around a
    x, y = plane.xy(b)  # b relative to a
    return math.hypot(x, y)  # metres


def _spoken_time(seconds: float) -> str:
    """"约N秒" under a minute (to 10 s), else "约N分钟" (rounded up)."""
    if seconds < 60.0:  # under a minute
        return "约%d秒" % max(10, int(math.ceil(seconds / 10.0)) * 10)  # to 10 s
    return "约%d分钟" % int(math.ceil(seconds / 60.0))  # whole minutes, up


def make_geo_query_fn(membership_m: float) -> Callable[..., Answer]:
    """fn(kind, name, objects, pose, mode) -> (template key, fields).

    kind is 'nearest_point' (G50, name ignored), 'distance_to' (G51) or
    'route_length' (G52); objects / pose / mode are the state bodies (None
    when never received)."""
    limit = float(membership_m)  # common.geo.route_membership_m (U127 / U130)

    def answer(kind: str, name: str, objects: Any, pose: Any, mode: Any) -> Answer:
        if not isinstance(objects, Mapping):  # broadcast never received
            return "geo_unloaded", {}  # honest, not a guess
        points = _live(objects.get("waypoints"), _point_ok)  # keypoints with a position
        routes = _live(objects.get("routes"), _route_ok)  # routes with geometry
        if kind == "route_length":  # G52: no robot position needed
            route = _find(routes, name)  # the route asked about
            if route is None:  # unknown name
                return "route_not_found", {"name": name}  # say what was looked for
            line = _ll(route)  # its polyline
            plane = Plane(line[0])  # local metres
            poly = Polyline([plane.xy(p) for p in line], route.get("loop_mode") == "closed")  # metres
            return "route_length", {"route": route.get("name"), "length": int(round(poly.length))}  # metres
        here = _robot(pose)  # the robot now
        if here is None:  # no position
            return "no_pose", {}  # cannot measure from nowhere
        if kind == "nearest_point":  # G50
            if not points:  # nothing recorded
                return "no_points", {}  # say so
            best = min(points, key=lambda w: _straight_m(here, (float(w["lat"]), float(w["lon"]))))  # nearest
            d = _straight_m(here, (float(best["lat"]), float(best["lon"])))  # its distance
            return "nearest_point", {"name": best.get("name"), "dist": int(round(d))}  # straight line
        target = _find(points, name)  # G51: the waypoint asked about
        if target is None:  # unknown name
            return "not_found", {"name": name}  # say what was looked for
        geoms = [RouteGeom(geo_id=str(r.get("geo_id", "")), name=str(r.get("name", "")),
                           points=_ll(r), closed=r.get("loop_mode") == "closed") for r in routes]  # as P3 loads them
        plan = plan_goto(here, (float(target["lat"]), float(target["lon"])), geoms, limit)  # the path a goto takes
        fields: Dict[str, Any] = {"name": target.get("name"), "dist": int(round(plan.cost_m))}  # metres
        speed = _speed(mode)  # voice-set move speed
        fields["time"] = ("" if speed is None
                          else ", 按当前速度" + _spoken_time(plan.cost_m / speed))  # optional tail
        if plan.route_id is None:  # straight
            return "distance_straight", fields  # straight-line answer
        fields["route"] = next((g.name for g in geoms if g.geo_id == plan.route_id), "")  # the route followed
        return "distance_route", fields  # along a route

    return answer  # injected into the orchestrator
