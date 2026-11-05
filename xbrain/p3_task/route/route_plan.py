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

import math  # hypot for lengths
from dataclasses import dataclass  # frozen result types
from typing import List, Optional, Sequence, Tuple  # plain value types

# The plane / polyline geometry moved to the shared layer on 2026-11-05 (99
# U130) so P4's membership queries use exactly the same math as navigation.
from xbrain.common.geo.polyline import DEDUP_M as _DEDUP_M  # zero-length guard
from xbrain.common.geo.polyline import XY, LatLon  # value types
from xbrain.common.geo.polyline import Plane as _Plane  # local metric plane
from xbrain.common.geo.polyline import Polyline as _Poly  # projection / walk


# 99 U136 (2026-11-05): RouteGeom / GotoPlan / plan_goto moved, unchanged, to
# xbrain/common/geo/goto_plan.py -- the P4 query "去X多远" answers with the
# same plan, and two copies of the rule would speak a distance for a path the
# robot does not drive. Re-exported here so every P3 caller keeps its import.
from xbrain.common.geo.goto_plan import GotoPlan, RouteGeom, plan_goto  # noqa: F401  re-export
from xbrain.common.geo.goto_plan import _dedup  # plan_patrol's zero-length guard


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
