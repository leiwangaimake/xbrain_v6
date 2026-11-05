"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: polyline.py
Brief: Shared polyline geometry -- local metric plane, projection, walk -- and route membership (99 U127 / U130)

Description:
A waypoint belongs to a route when its distance to the route POLYLINE is at
most common.geo.route_membership_m (99 U127). P3 uses that to plan a goto
along the route, P4 uses it to answer "which waypoints are on route X" and
"which route is waypoint X on" (99 U130). The two answers must agree to the
centimetre -- a navigation that follows a route and a query that says the
point is not on it would contradict each other -- so the geometry lives here
once, moved out of p3_task/route/route_plan.py on 2026-11-05.

All geometry runs in a local east-north plane around a reference point
(equirectangular; distances of a few km, error far below GNSS noise).
Pure: no db, no clock, no logging.

Traps:
  * Measuring membership against the recorded VERTICES only: on a sparse
    mode A route (anchors 50 m apart) a point right on the line would be
    judged off it. Distance is to the SEGMENTS.
  * A closed route's closing segment counts (last vertex back to the first).
"""

from __future__ import annotations

import math  # trig for the plane, hypot for lengths
from typing import List, Sequence, Tuple  # plain value types

LatLon = Tuple[float, float]  # (lat, lon) in degrees, WGS84
XY = Tuple[float, float]  # (east, north) in metres, local plane

_EARTH_R_M = 6371000.0  # mean Earth radius, same as the snapshot builder
DEDUP_M = 0.05  # consecutive points closer than this are one point (no zero-length segment)
_DEDUP_M = DEDUP_M  # the name the moved code below uses


class Plane:
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


class Polyline:
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



def distance_to_route_m(point: LatLon, route: Sequence[LatLon], closed: bool) -> float:
    """Distance in metres from point to the route polyline (segments, the
    closing one included on a closed route). The plane is centred on the
    point itself, so the result does not depend on where the robot is."""
    plane = Plane(point)  # local metres around the point
    return Polyline([plane.xy(p) for p in route], closed).project(plane.xy(point))[0]  # nearest foot distance
