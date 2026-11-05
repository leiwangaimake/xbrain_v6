"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: route_plan.py
Brief: Route-aware geometry for goto (B01) and patrol start (B02) -- which polyline P3 pushes (99 U127)

Description:
What problem this solves. Until 2026-11-05 a goto (B01, and the B08 return)
was pushed as ONE straight point, ignoring the recorded routes even when the
target sat on one, and a patrol (B02) pushed the recorded route as-is -- so a
robot standing beside the route, more than rns.route.max_deviation_m (10 m)
from its first point, failed max_deviation on the first tick (20 S2.7). The
user ruled the behaviour on 2026-11-05 (99 U127):

  goto (B01 / B08 / B09 share it)
    * a waypoint belongs to a route when its distance to the route POLYLINE
      (segments, not just recorded vertices) is <= route.membership_m (5 m);
      membership is computed live, never stored;
    * for every route it belongs to, the robot enters at the route point
      nearest to it, follows the route (EITHER direction -- the shorter one;
      on a closed route either way round) to the point nearest the target,
      then goes straight to the target. Cost = entry leg + along-route leg +
      exit leg; the cheapest route wins (ruling 1B);
    * if the robot is no farther from the target than from the route it
      would enter (straight-line distance to the target <= distance to that
      route), it drives straight; with no member route it is the plain point
      goto. (Ruling 2B was first worded "compare the straight length with the
      via-route length, take the shorter" -- by the triangle inequality the
      straight line is never longer, so the route would never be used; the
      user re-ruled the same evening: "closer to the target than to the
      route -> straight", 99 U127.)
  patrol start (B02 / B03)
    * closed route: enter at the nearest point and go once round in the
      RECORDED direction, back to the entry point;
    * open route: enter at the nearest point, go first to the FARTHER end,
      then turn and go to the other end (full coverage). When the entry is
      already within the endpoint's arrival radius of the near end, the
      return leg is dropped -- going back would cover the route twice.

Every plan starts with the robot's own position as its first point, so the
P1 tracker's deviation is measured from the line the robot is actually on
(the same reason the single-point goto is anchored at the pose, 20 RNS-N-1).

How. All geometry is done in a local east-north plane around the robot
(equirectangular; plans span at most a few km, where the error is far below
GNSS noise) and converted back to WGS84 at the end. Pure functions: no db,
no clock, no logging -- runtime/route_push_runtime.py loads the rows and
builds the snapshot.

What it does NOT do: it never chains several routes into one network path
(the ruling picks ONE route per goto), never avoids obstacles (RNS does that
along whatever line it is given), and never decides lap counts (B04).

Traps:
  * Measuring membership against the recorded VERTICES only: on a sparse
    mode A route (anchors 50 m apart) a waypoint right on the line would be
    judged off it.
  * Taking the along-route length as |s_target - s_entry| on a CLOSED route:
    the way round through the seam can be the short one.
  * Forgetting the robot's own position as point 0: P1 then measures the
    deviation from the entry point's segment, which can exceed 10 m.
"""

from __future__ import annotations

import math  # trig for the local plane, hypot for lengths
from dataclasses import dataclass  # frozen result types
from typing import List, Optional, Sequence, Tuple  # plain value types

LatLon = Tuple[float, float]  # (lat, lon) in degrees, WGS84
XY = Tuple[float, float]  # (east, north) in metres, local plane

_EARTH_R_M = 6371000.0  # mean Earth radius, same as the snapshot builder
_DEDUP_M = 0.05  # consecutive points closer than this are one point (no zero-length segment)


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


class _Plane:
    """Local east-north plane around a reference point (equirectangular)."""

    def __init__(self, ref: LatLon) -> None:
        self._lat0 = math.radians(ref[0])  # reference latitude, radians
        self._lon0 = math.radians(ref[1])  # reference longitude, radians
        self._kx = _EARTH_R_M * math.cos(self._lat0)  # metres per radian east here
        self._ky = _EARTH_R_M  # metres per radian north

    def xy(self, p: LatLon) -> XY:
        """WGS84 -> local metres."""
        return ((math.radians(p[1]) - self._lon0) * self._kx,  # east
                (math.radians(p[0]) - self._lat0) * self._ky)  # north

    def ll(self, q: XY) -> LatLon:
        """local metres -> WGS84."""
        return (math.degrees(self._lat0 + q[1] / self._ky),  # lat
                math.degrees(self._lon0 + q[0] / self._kx))  # lon


class _Poly:
    """A polyline in the local plane with cumulative arc length. A closed
    polyline gets its closing segment as an explicit last vertex (a copy of
    vertex 0 at arc length L), unless the recording already ends on it."""

    def __init__(self, pts: Sequence[XY], closed: bool) -> None:
        v = [pts[0]]  # deduplicated vertices
        for p in pts[1:]:  # drop zero-length steps
            if math.hypot(p[0] - v[-1][0], p[1] - v[-1][1]) > _DEDUP_M:
                v.append(p)  # a real step
        if closed and len(v) > 2 and math.hypot(v[-1][0] - v[0][0], v[-1][1] - v[0][1]) > _DEDUP_M:
            v.append(v[0])  # close the ring explicitly
        self.v = v  # vertices
        self.closed = closed  # ring or not
        cum = [0.0]  # arc length at each vertex
        for a, b in zip(v, v[1:]):  # every segment
            cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))  # accumulate
        self.cum = cum  # cumulative arc length
        self.length = cum[-1]  # total length L

    def project(self, p: XY) -> Tuple[float, float]:
        """(distance to the polyline, arc length of the foot). A one-vertex
        polyline projects onto that vertex at arc length 0."""
        if len(self.v) == 1:  # degenerate route
            q = self.v[0]  # the only vertex
            return math.hypot(p[0] - q[0], p[1] - q[1]), 0.0  # distance, s = 0
        best = (math.inf, 0.0)  # (distance, s)
        for i in range(len(self.v) - 1):  # every segment, closing one included
            a, b = self.v[i], self.v[i + 1]  # segment ends
            dx, dy = b[0] - a[0], b[1] - a[1]  # segment vector
            seg2 = dx * dx + dy * dy  # squared length, > 0 after dedup
            t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / seg2))  # clamped foot parameter
            fx, fy = a[0] + t * dx, a[1] + t * dy  # foot point
            d = math.hypot(p[0] - fx, p[1] - fy)  # distance to this segment
            if d < best[0]:  # nearer than any segment so far
                best = (d, self.cum[i] + t * math.sqrt(seg2))  # its arc length
        return best  # nearest foot

    def at(self, s: float) -> XY:
        """The point at arc length s (clamped to [0, L])."""
        s = max(0.0, min(self.length, s))  # clamp
        for i in range(len(self.v) - 1):  # find the segment holding s
            if s <= self.cum[i + 1] or i == len(self.v) - 2:  # inside, or the last one
                seg = self.cum[i + 1] - self.cum[i]  # its length
                t = 0.0 if seg <= 0.0 else (s - self.cum[i]) / seg  # fraction along it
                a, b = self.v[i], self.v[i + 1]  # its ends
                return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))  # interpolate
        return self.v[0]  # one-vertex polyline

    def along(self, s_from: float, s_to: float) -> Tuple[float, int]:
        """(length, direction) of the shorter way from s_from to s_to.
        direction +1 = recorded direction, -1 = against it. On a closed
        route the way through the seam is considered too (trap 2)."""
        fwd = s_to - s_from  # signed difference along the recording
        if not self.closed:  # open: only one way between two points
            return abs(fwd), (1 if fwd >= 0.0 else -1)  # length, direction
        f = fwd % self.length  # forward, wrapping through the seam
        b = (-fwd) % self.length  # backward, wrapping
        return (f, 1) if f <= b else (b, -1)  # the shorter way

    def walk(self, s_from: float, s_to: float, direction: int) -> List[XY]:
        """Points from the foot at s_from to the foot at s_to travelling in
        `direction`, with every vertex strictly between them, wrapping
        through the seam on a closed route when needed."""
        out = [self.at(s_from)]  # start at the entry foot
        n = len(self.v)  # vertex count (closing copy included)
        if direction > 0:  # recorded direction
            if s_to >= s_from:  # no wrap
                out += [self.v[k] for k in range(n) if s_from < self.cum[k] < s_to]  # vertices between
            else:  # through the seam (closed only)
                out += [self.v[k] for k in range(n) if self.cum[k] > s_from]  # to the end (= vertex 0)
                out += [self.v[k] for k in range(1, n) if self.cum[k] < s_to]  # from vertex 0 on
        else:  # against the recording
            if s_to <= s_from:  # no wrap
                out += [self.v[k] for k in reversed(range(n)) if s_to < self.cum[k] < s_from]  # vertices between, reversed
            else:  # through the seam (closed only)
                out += [self.v[k] for k in reversed(range(n)) if self.cum[k] < s_from]  # back to vertex 0
                out += [self.v[k] for k in reversed(range(n - 1)) if self.cum[k] > s_to]  # from the end back
        out.append(self.at(s_to))  # end at the exit foot
        return out  # in travel order


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


def plan_patrol(start: LatLon, route: Sequence[LatLon], closed: bool,
                near_skip_m: float) -> Tuple[LatLon, ...]:
    """The patrol polyline under ruling U127 (see the header): closed ->
    once round in the recorded direction from the nearest point; open ->
    nearest point, farther end first, then back to the other end unless the
    near side is within near_skip_m (the endpoint's arrival radius)."""
    plane = _Plane(start)  # local metres around the robot
    s_xy = plane.xy(start)  # robot
    poly = _Poly([plane.xy(p) for p in route], closed)  # route in metres
    if poly.length <= 0.0:  # a one-point route
        return (start, plane.ll(poly.v[0]))  # just go there
    _d, s_e = poly.project(s_xy)  # entry foot
    if closed:  # once round, recorded direction, back to the entry
        if s_e <= _DEDUP_M or s_e >= poly.length - _DEDUP_M:  # entry on the seam
            loop = [poly.at(0.0)] + poly.v[1:]  # vertex 0 round to vertex 0 (closing copy)
        else:  # entry mid-ring
            loop = poly.walk(s_e, s_e - _DEDUP_M, 1)  # forward through the seam to just short of the entry
            loop.append(poly.at(s_e))  # and the entry itself
        pts = [s_xy] + loop  # robot first
    else:  # open: farther end first
        far_is_end = (poly.length - s_e) >= s_e  # which end is farther
        far_s, near_s = (poly.length, 0.0) if far_is_end else (0.0, poly.length)  # their arc lengths
        leg1 = poly.walk(s_e, far_s, 1 if far_is_end else -1)  # entry to the far end
        pts = [s_xy] + leg1  # robot, then leg 1
        if min(s_e, poly.length - s_e) > near_skip_m:  # the near side is a real stretch
            pts += poly.walk(far_s, near_s, -1 if far_is_end else 1)[1:]  # turn and cover it
    return tuple(plane.ll(q) for q in _dedup(pts))  # back to WGS84
