"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: route_membership.py
Brief: G48 / G49 answers -- which waypoints are on route X, which route is waypoint X on (99 U130)

Description:
The user asked (2026-11-05) for "XX路径上有哪些航点" and "XX航点在哪条路径上".
A waypoint belongs to a route when its distance to the route POLYLINE is at
most common.geo.route_membership_m -- the very rule P3 follows a route by on a
goto (99 U127). The geometry is the shared xbrain.common.geo.polyline, so a
query and a navigation can never disagree about the same point.

make_membership_fn(limit_m) builds the function the orchestrator calls with
the parsed question and the state/geo/objects broadcast; the distance is a
required argument there and nowhere else (CLAUDE.md 3.1: no default in code).
It is local geometry over a handful of routes -- milliseconds -- so the
answer is given at once; the user's "正在查询, 请稍等" fallback is for an
answer that cannot be, and this one can.

What it does NOT do: no catalogue writes, no fuzzy name matching beyond the
canonical-name rule (numbers spelt 一号 / 1号 are equal), no network.

Traps:
  * Testing against recorded vertices only -- a point between two sparse
    anchors would read "not on the route". Distance is to the segments.
  * Listing members in catalogue order: G48 names them in the order they lie
    ALONG the route, which is what an operator walking it expects.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple  # plain value types

from xbrain.common.geo.polyline import Plane, Polyline, distance_to_route_m  # shared with P3 navigation
from xbrain.p4_agent.slots.name_slots import canonical_name  # 一号 == 1号

#: (template key, format fields) -- the orchestrator owns the spoken strings.
Answer = Tuple[str, Dict[str, Any]]


def _live(items: Any, need: Callable[[Mapping], bool]) -> List[Mapping]:
    """The active, well-formed entries of one objects layer."""
    if not isinstance(items, list):  # layer missing
        return []  # nothing usable
    return [o for o in items if isinstance(o, Mapping) and o.get("state", "active") == "active" and need(o)]  # live ones


def _route_ok(r: Mapping) -> bool:
    return isinstance(r.get("points"), list) and bool(r["points"])  # a polyline to measure against


def _point_ok(w: Mapping) -> bool:
    return isinstance(w.get("lat"), (int, float)) and isinstance(w.get("lon"), (int, float))  # a position


def _ll(route: Mapping) -> List[Tuple[float, float]]:
    return [(float(p["lat"]), float(p["lon"])) for p in route["points"]]  # [(lat, lon)]


def _find(items: List[Mapping], name: str) -> Optional[Mapping]:
    want = canonical_name(name)  # 一号路径 == 1号路径
    return next((o for o in items if canonical_name(str(o.get("name", ""))) == want), None)  # first match


def make_membership_fn(limit_m: float) -> Callable[[str, str, Any], Answer]:
    """fn(kind, name, objects) -> (template key, fields) for G48 / G49.

    kind 'route_points' (G48) or 'point_routes' (G49); objects is the
    state/geo/objects body (None when never received)."""
    limit = float(limit_m)  # common.geo.route_membership_m

    def answer(kind: str, name: str, objects: Any) -> Answer:
        if not isinstance(objects, Mapping):  # broadcast never received
            return "geo_unloaded", {}  # honest, not a guess
        routes = _live(objects.get("routes"), _route_ok)  # routes with geometry
        points = _live(objects.get("waypoints"), _point_ok)  # keypoints with a position
        if kind == "route_points":  # G48
            route = _find(routes, name)  # the route asked about
            if route is None:  # unknown name
                return "route_not_found", {"name": name}  # say what was looked for
            line = _ll(route)  # its polyline
            plane = Plane(line[0])  # local metres
            poly = Polyline([plane.xy(p) for p in line], route.get("loop_mode") == "closed")  # in metres
            hits = []  # (arc position, name)
            for w in points:  # every keypoint
                d, s = poly.project(plane.xy((float(w["lat"]), float(w["lon"]))))  # distance, arc position
                if d <= limit:  # the U127 rule, inclusive
                    hits.append((s, str(w.get("name"))))  # a member
            if not hits:  # none on it
                return "route_no_points", {"route": route.get("name")}  # say so
            names = [n for _s, n in sorted(hits)]  # along-route order
            return "route_points", {"route": route.get("name"), "count": len(names), "names": ",".join(names)}  # the list
        point = _find(points, name)  # G49: the point asked about
        if point is None:  # unknown name
            return "not_found", {"name": name}  # say what was looked for
        here = (float(point["lat"]), float(point["lon"]))  # its position
        on = [str(r.get("name")) for r in routes
              if distance_to_route_m(here, _ll(r), r.get("loop_mode") == "closed") <= limit]  # member routes
        if not on:  # on none
            return "point_no_routes", {"name": point.get("name")}  # say so
        return "point_routes", {"name": point.get("name"), "routes": "和".join(on)}  # the routes

    return answer  # injected into the orchestrator
