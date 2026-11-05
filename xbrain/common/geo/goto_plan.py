"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: goto_plan.py
Brief: the route-aware goto plan (99 U127), shared by P3 navigation and P4 "去X多远" (99 U136)

Description:
What problem this solves. A goto follows a recorded route when its target
belongs to one (99 U127): enter at the route point nearest the robot, walk
the route the shorter way to the point nearest the target, then go straight;
among several member routes the cheapest TOTAL length wins; and the robot
goes straight when it is no farther from the target than from that route.
P3 pushes that polyline. Since 99 U136 the P4 query "去X多远" answers with
the same plan's length -- the distance the robot would actually travel. Two
copies of the rule would give a spoken distance for a path the robot does
not take, so the function lives here and both import it (the same reasoning
as xbrain/common/geo/polyline, 99 U130).

Moved here unchanged from p3_task/route/route_plan.py on 2026-11-05 (99
U136); route_plan.py re-exports RouteGeom / GotoPlan / plan_goto so every P3
caller and test keeps its import.

How. All geometry is done in a local east-north plane around the robot
(Plane: equirectangular, plans span at most a few km where the error is far
below GNSS noise) and converted back to WGS84 at the end. Pure: no db, no
clock, no logging.

What it does NOT do: chain several routes into one network path (the ruling
picks ONE route per goto), avoid obstacles (RNS does that along the line it
is given), or plan a patrol (route_plan.plan_patrol, P3 only).

Traps:
  * Measuring membership against recorded VERTICES only: on a sparse route a
    waypoint right on the line would be judged off it.
  * "direct <= cost" as the straight-line test: by the triangle inequality it
    holds for every route, so no route would ever be used (U127 2B re-ruled).
  * Forgetting the robot's own position as point 0: P1 then measures its
    deviation from the entry point's segment, which can exceed 10 m.
"""
from __future__ import annotations  # postponed annotations

import math  # hypot for lengths
from dataclasses import dataclass  # frozen result types
from typing import List, Optional, Sequence, Tuple  # plain value types

from xbrain.common.geo.polyline import DEDUP_M as _DEDUP_M  # zero-length guard
from xbrain.common.geo.polyline import XY, LatLon  # value types
from xbrain.common.geo.polyline import Plane as _Plane  # local metric plane
from xbrain.common.geo.polyline import Polyline as _Poly  # projection / walk


@dataclass(frozen=True)
class RouteGeom:
    """One route as a polyline. closed: the last vertex joins the first
    (routes.loop_mode == 'closed')."""
    geo_id: str  # r-*
    name: str  # display / spoken name
    points: Tuple[LatLon, ...]  # recorded vertices in recorded order
    closed: bool  # loop_mode closed


@dataclass(frozen=True)
class GotoPlan:
    """What a goto pushes. route_id None = straight to the target."""
    points: Tuple[LatLon, ...]  # robot position first, target last
    route_id: Optional[str]  # the route followed, or None
    cost_m: float  # the planned length


def _dedup(pts: Sequence[XY]) -> List[XY]:
    """Drop consecutive points closer than _DEDUP_M (zero-length segments)."""
    out: List[XY] = []  # kept points
    for p in pts:  # in order
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > _DEDUP_M:
            out.append(p)  # a real step
    return out  # cleaned


def plan_goto(start: LatLon, target: LatLon, routes: Sequence[RouteGeom],
              membership_m: float) -> GotoPlan:
    """The goto polyline under ruling U127 (see the header): cheapest member
    route by total length; straight when the robot is no farther from the
    target than from that route (2B, re-ruled)."""
    plane = _Plane(start)  # local metres around the robot
    s_xy, t_xy = plane.xy(start), plane.xy(target)  # robot and target
    direct = math.hypot(t_xy[0] - s_xy[0], t_xy[1] - s_xy[1])  # straight-line cost
    best: Optional[Tuple[float, str, List[XY], float]] = None  # (cost, route id, points, entry distance)
    for r in routes:  # every live route
        if not r.points:  # nothing to follow
            continue  # skip it
        poly = _Poly([plane.xy(p) for p in r.points], r.closed)  # in metres
        d_x, s_x = poly.project(t_xy)  # target to route
        if d_x > membership_m:  # the target does not belong to this route
            continue  # rule: <= membership_m
        d_e, s_e = poly.project(s_xy)  # robot to route (entry)
        along, direction = poly.along(s_e, s_x)  # shorter way along it (ruling 3)
        cost = d_e + along + d_x  # ruling 1B: total length
        if best is None or cost < best[0]:  # cheapest so far
            best = (cost, r.geo_id, [s_xy] + poly.walk(s_e, s_x, direction) + [t_xy], d_e)  # robot, route leg, target
    # U127 2B (re-ruled): straight when the robot is no farther from the
    # target than from the chosen route. NOT "direct <= cost" -- that holds
    # for every route by the triangle inequality and would never use one.
    if best is None or direct <= best[3]:  # no member route, or closer to the target than to the route
        return GotoPlan(points=(start, target), route_id=None, cost_m=direct)  # plain point goto
    pts = _dedup(best[2])  # no zero-length segments
    return GotoPlan(points=tuple(plane.ll(q) for q in pts), route_id=best[1], cost_m=best[0])  # back to WGS84
