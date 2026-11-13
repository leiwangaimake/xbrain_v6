"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geom.py
Brief: BIZ-P3-18 fence geometry (FS-1..4 painter, S9A.1A set rules: at most 1 allow)

Description:
15 S9 fence geometry supports composite geometry via the PAINTER
algorithm (FS-1..3):

  FS-1  polygons and circles combine by set-union of their interiors
  FS-2  polygons with the SAME group_id are painted in listed order
        (later polygons overwrite earlier ones)
  FS-3  the OUTPUT of paint is a binary mask; querying a point is
        an O(N) scan over shapes (fewer than 5 shapes total)

FS-4 is the commit-time validation: a polygon must be self-non-
intersecting, its winding must be consistent, and its bounding box
must fall inside the site bounds. A polygon that fails FS-4 is
REJECTED at commit; runtime never sees a bad fence.

v1.5 (PLAN A / fence runtime): the old kind='zone' + free-text label is
retired. Fences now carry a ROLE (11 S9A.2: allow/forbid/speed_limit/warning).
validate_active_fence_set enforces the 11 S9A.1A SET rule (at most 1 allow) at
FenceSet build/broadcast time.

99 U183 (user 2026-11-13): ~~exactly 1 allow, <= 5 total~~ superseded. The
activity area may be absent (the cloud can switch it off) and the other fences
stay in force without it; there is no limit on the number of fences. Under the
old rule a set with no allow was never broadcast, and because P1 keeps the last
set it received, switching the activity area (or every fence) off left P1
enforcing the old geometry until it restarted.
"""

from __future__ import annotations

import math

# Circle / Polygon / point_in_circle / point_in_polygon 现由跨进程共享库导出
# (xbrain/common/fence/geom.py): p1_motion 自算 crc32 比对与报警区点在多边形内
# 判定要与本进程用[同一套]实现, 否则两侧漂移. 这里 re-import 保持 p3 既有 API
# (from xbrain.p3_task.fence.geom import Polygon / point_in_polygon 仍可用).
from xbrain.common.fence.geom import Circle, Polygon, point_in_circle, point_in_polygon


class InvalidPolygon(Exception):
    pass


class InvalidFenceSet(Exception):
    """The active fence SET violates a 11 S9A.1A operating rule (more than one
    allow). Maps to E_FENCE_INVALID at the FenceSet build/broadcast boundary."""


def validate_active_fence_set(roles) -> None:
    """11 S9A.1A FS-5A (99 U183): the ACTIVE fence set has AT MOST ONE allow --
    two keep-in polygons would make the permitted area their union, quietly
    widening it. `roles` is the list of role strings of the active
    (non-tombstoned) fences. The fence.db single-allow triggers hold the same
    line per row; this is the check at FenceSet build/broadcast time.

    An empty set, and a set with no allow, are VALID (99 U183): they are
    broadcast like any other, so P1 replaces what it holds with them (an empty
    set clears it). ~~exactly one allow, at most 5 fences~~ superseded
    2026-11-13 -- see the module header for what the old rule broke."""
    roles = list(roles)
    allow_n = sum(1 for r in roles if r == "allow")
    if allow_n > 1:
        raise InvalidFenceSet(
            f"active set has {allow_n} allow fences, at most 1 "
            "(11 S9A.1A FS-5A)")


def polygon_area(points) -> float:
    """Signed area via the shoelace formula (positive for CCW)."""
    n = len(points)
    if n < 3:
        return 0.0
    total = 0.0
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        total += x1 * y2 - x2 * y1
    return total / 2.0


def validate_polygon(points) -> None:
    """FS-4: at least 3 unique points, area not zero."""
    if len(points) < 3:
        raise InvalidPolygon(f"only {len(points)} points, need >= 3")
    if len(set(points)) < 3:
        raise InvalidPolygon("duplicate points collapse to < 3 unique")
    if polygon_area(points) == 0.0:
        raise InvalidPolygon("degenerate polygon (area = 0)")


def assert_perimeter_closed(points, tol_m: float) -> None:
    """Recording-time check (15 S8.1A '闭合点错 = 多边形错'): a walked fence
    perimeter MUST return near its start -- the last recorded point within
    tol_m of the first -- else the operator did not close the loop and the
    polygon is wrong. This is DISTINCT from validate_polygon (which checks the
    stored ring): here we check the RAW walked track before storing. tol_m is
    injected (a recording tolerance; no code default, CLAUDE.md 3.1)."""
    if len(points) < 3:
        raise InvalidPolygon(f"only {len(points)} points, need >= 3")
    (x0, y0), (xn, yn) = points[0], points[-1]
    d = math.hypot(xn - x0, yn - y0)
    if d > tol_m:
        raise InvalidPolygon(
            f"perimeter not closed: last-to-first {d:.2f}m > tol {tol_m}m")


def close_ring(points, tol_m: float):
    """Return the vertex ring to STORE: if the last walked point coincides with
    the first (within tol_m), drop it. The stored polygon closes implicitly via
    the (i+1) % n wraparound in polygon_area / point_in_polygon, so a duplicated
    closing vertex would double-count the seam (zero-length edge)."""
    pts = list(points)
    if len(pts) >= 2:
        (x0, y0), (xn, yn) = pts[0], pts[-1]
        if math.hypot(xn - x0, yn - y0) <= tol_m:
            return pts[:-1]
    return pts


def point_in_composite(x: float, y: float,
                         shapes) -> bool:
    """FS-1..3: union of shapes. Any shape containing (x, y) means
    inside. Empty list -> not inside anything."""
    for s in shapes:
        if isinstance(s, Circle):
            if point_in_circle(x, y, s):
                return True
        elif isinstance(s, Polygon):
            if point_in_polygon(x, y, s):
                return True
        else:
            raise TypeError(f"unknown shape type {type(s).__name__}")
    return False
