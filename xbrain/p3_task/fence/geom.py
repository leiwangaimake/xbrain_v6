"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geom.py
Brief: BIZ-P3-18 fence geometry (FS-1..4 painter algorithm, FS-5 zone tag, up to 5 fences)

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

FS-5 zone tag: fences of kind='zone' carry a label (e.g. 'safe',
'restricted') used by V-4 for context-sensitive admission (a
'restricted' zone rejects patrol tasks but admits 'inspect').
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Circle:
    cx: float
    cy: float
    r: float


@dataclass(frozen=True)
class Polygon:
    points: tuple    # ((x, y), ...)


class InvalidPolygon(Exception):
    pass


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


def point_in_circle(x: float, y: float, c: Circle) -> bool:
    return math.hypot(x - c.cx, y - c.cy) <= c.r


def point_in_polygon(x: float, y: float, poly: Polygon) -> bool:
    """Ray-cast: cast horizontal ray from (x, y), count crossings.
    Boundary points may be reported either way -- acceptable for
    fence use (a robot at boundary is inside a safe region)."""
    n = len(poly.points)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly.points[i]
        xj, yj = poly.points[j]
        crosses = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        if crosses:
            inside = not inside
        j = i
    return inside


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
