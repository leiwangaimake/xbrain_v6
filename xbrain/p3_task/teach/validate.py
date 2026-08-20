"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: validate.py
Brief: 11 S12A.7 geometry validation of a recorded route / fence

Description:
Run at finish, and again at save (S12A.7 says so explicitly: the object library
can be edited between the two, and a name that was free at finish may not be at
save). Produces the TeachState.validation block: an ok flag plus an issues list
whose codes are a closed set.

The severities are what matter here. Four codes BLOCK the save
(E_TEACH_GEOMETRY) and the rest are advisory:

  blocking : too_few_points, too_few_vertices, self_intersect, area_too_small
  warn     : degenerate_edge, close_gap_large, robot_outside,
             outside_active_fence, low_quality_ratio
  info     : auto_closed

The one rule that is a safety rule rather than a geometry rule is the third of
S12A.7's fence constraints: robot_outside is a WARNING for a plain save, and a
REFUSAL when the same save also asks to activate. The reason is given in the
contract and is worth repeating: activating a keep-in fence the robot is
already outside of hands control to fence_guard (priority 1000) in the same
instant, from a position it considers a violation. So the pair (robot_outside,
activate=true) is the one combination that turns a warning into E_TEACH_GEOMETRY.

Boundaries: pure. No connection, no clock, no session. The caller supplies the
point list, the current robot position and the thresholds; this decides.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from xbrain.p3_task.teach.sampling import haversine_m

#: S12A.7 issue codes, split by what they do to a save. Kept as two frozensets
#: rather than a severity field per code so the question the caller actually
#: asks -- "may this be saved" -- is one membership test.
BLOCKING_ISSUES = frozenset({"too_few_points", "too_few_vertices",
                             "self_intersect", "area_too_small"})
ADVISORY_ISSUES = frozenset({"degenerate_edge", "auto_closed",
                             "close_gap_large", "robot_outside",
                             "outside_active_fence", "low_quality_ratio"})

#: S12A.7 defaults, both marked as awaiting field calibration in the contract.
#: They are recording-quality thresholds, not common.safety.* parameters: the
#: failure direction of a wrong value is "the operator is asked to re-record",
#: never motion, so a default here is not the CLAUDE.md 3.1 case.
DEFAULT_MIN_AREA_M2 = 100.0
DEFAULT_CLOSE_TOL_M = 5.0
#: S12A.7 degenerate_edge: neighbouring points closer than this are merged.
#: Same figure as the sampling dedup, deliberately -- an edge shorter than the
#: dedup threshold can only come from a mark point or an undo.
DEGENERATE_EDGE_M = 0.5
#: low_quality_ratio fires above this share of dropped samples.
LOW_QUALITY_RATIO = 0.2

_MIN_ROUTE_POINTS = 2
_MIN_RING_VERTICES = 3


@dataclass
class Validation:
    ok: bool = True
    issues: List[Dict[str, object]] = field(default_factory=list)

    def add(self, code: str, **extra) -> None:
        issue: Dict[str, object] = {"code": code}
        issue.update(extra)
        self.issues.append(issue)
        if code in BLOCKING_ISSUES:
            self.ok = False

    def codes(self) -> List[str]:
        return [str(i["code"]) for i in self.issues]

    def blocking(self) -> List[str]:
        return [c for c in self.codes() if c in BLOCKING_ISSUES]


def _segments_intersect(p1, p2, p3, p4) -> bool:
    """Do segments p1p2 and p3p4 cross? Planar test on lat/lon degrees.

    Treating degrees as a plane is fine for this ONE question at camp scale:
    self-intersection is topological, and the lat/lon plane preserves the
    ordering of crossings over a couple of kilometres. It would not be fine for
    area or distance, which is why those use the metric helpers instead.
    """
    def orient(a, b, c) -> float:
        return ((b[0] - a[0]) * (c[1] - a[1])
                - (b[1] - a[1]) * (c[0] - a[0]))

    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    # Strict sign change on both segments. Touching endpoints (a zero) are NOT
    # counted: adjacent edges of a ring always share one, and counting those
    # would make every polygon self-intersecting.
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def ring_self_intersects(ring: Sequence[Tuple[float, float]]) -> Optional[
        Tuple[int, int]]:
    """The first crossing edge pair (i, j), or None. S12A.7 wants the pair in
    the ack detail so the HMI can highlight the offending edges."""
    n = len(ring)
    if n < 4:
        return None
    for i in range(n):
        a1, a2 = ring[i], ring[(i + 1) % n]
        for j in range(i + 1, n):
            # Skip adjacent edges and the wrap-around pair: they legitimately
            # share an endpoint.
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue
            if _segments_intersect(a1, a2, ring[j], ring[(j + 1) % n]):
                return (i, j)
    return None


def ring_area_m2(ring: Sequence[Tuple[float, float]]) -> float:
    """Absolute area of a WGS84 ring, via an equirectangular projection about
    its own centroid. Accurate to well under a percent at camp scale, which is
    what a 100 m2 minimum needs -- it is a sanity floor, not a survey."""
    if len(ring) < 3:
        return 0.0
    import math
    lat0 = sum(p[0] for p in ring) / len(ring)
    k = math.cos(math.radians(lat0))
    m_per_deg = 111320.0
    xy = [((p[1] * k) * m_per_deg, p[0] * m_per_deg) for p in ring]
    total = 0.0
    for i in range(len(xy)):
        x1, y1 = xy[i]
        x2, y2 = xy[(i + 1) % len(xy)]
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


def point_in_ring(point: Tuple[float, float],
                  ring: Sequence[Tuple[float, float]]) -> bool:
    """Ray casting in the lat/lon plane. Used for robot_outside only."""
    lat, lon = point
    inside = False
    n = len(ring)
    for i in range(n):
        la1, lo1 = ring[i]
        la2, lo2 = ring[(i + 1) % n]
        if (la1 > lat) != (la2 > lat):
            x = lo1 + (lat - la1) * (lo2 - lo1) / (la2 - la1)
            if lon < x:
                inside = not inside
    return inside


def validate_route(points: Sequence[Tuple[float, float]], *,
                   dropped_by_quality: int = 0) -> Validation:
    """S12A.7 for a recorded route."""
    v = Validation()
    if len(points) < _MIN_ROUTE_POINTS:
        v.add("too_few_points", point_count=len(points))
        return v                     # nothing else is meaningful
    _add_degenerate(v, points)
    _add_low_quality(v, len(points), dropped_by_quality)
    return v


def validate_fence(points: Sequence[Tuple[float, float]], *,
                   dropped_by_quality: int = 0,
                   robot_at: Optional[Tuple[float, float]] = None,
                   activate: bool = False,
                   min_area_m2: float = DEFAULT_MIN_AREA_M2,
                   close_tol_m: float = DEFAULT_CLOSE_TOL_M) -> Validation:
    """S12A.7 for a recorded fence, including the auto-closure rules.

    `points` is the recorded ring WITHOUT a duplicated closing vertex; closure
    is implicit. A recording rarely ends exactly where it began, so the gap
    between first and last is reported: within close_tol_m it is an info
    (auto_closed), beyond it a warn (close_gap_large) that the TTS/HMI must
    speak out loud -- "closed with an N metre gap" -- because the operator is
    about to trust that boundary.
    """
    v = Validation()
    if len(points) < _MIN_RING_VERTICES:
        v.add("too_few_vertices", point_count=len(points))
        return v
    gap = haversine_m(points[-1], points[0])
    if gap <= close_tol_m:
        v.add("auto_closed", gap_m=round(gap, 2))
    else:
        v.add("close_gap_large", gap_m=round(gap, 2))
    crossing = ring_self_intersects(points)
    if crossing is not None:
        v.add("self_intersect", edges=list(crossing))
    area = ring_area_m2(points)
    if area < min_area_m2:
        v.add("area_too_small", area_m2=round(area, 1),
              min_area_m2=min_area_m2)
    _add_degenerate(v, points)
    _add_low_quality(v, len(points), dropped_by_quality)
    if robot_at is not None and not point_in_ring(robot_at, points):
        # *** The one issue whose severity depends on the request: a warning on
        # its own, blocking when the same save activates the fence. See the
        # module docstring.
        v.add("robot_outside", activate=activate)
        if activate:
            v.ok = False
    return v


def _add_degenerate(v: Validation,
                    points: Sequence[Tuple[float, float]]) -> None:
    """Neighbouring points closer than DEGENERATE_EDGE_M: warn, auto-merged at
    commit. Reported per pair index so the HMI can show where."""
    for i in range(len(points) - 1):
        if haversine_m(points[i], points[i + 1]) < DEGENERATE_EDGE_M:
            v.add("degenerate_edge", at=i)
            return          # one report is enough; the merge handles them all


def _add_low_quality(v: Validation, kept: int, dropped: int) -> None:
    total = kept + dropped
    if total and (dropped / total) > LOW_QUALITY_RATIO:
        v.add("low_quality_ratio", dropped=dropped, kept=kept)


def merge_degenerate(points: Sequence[Tuple[float, float]],
                     min_edge_m: float = DEGENERATE_EDGE_M) -> List[
                         Tuple[float, float]]:
    """Drop points closer than min_edge_m to their predecessor (the automatic
    merge S12A.7 promises for degenerate_edge). The FIRST and LAST points are
    always kept: they are the ends of the route the operator drove."""
    if len(points) < 3:
        return list(points)
    out = [points[0]]
    for p in points[1:-1]:
        if haversine_m(out[-1], p) >= min_edge_m:
            out.append(p)
    out.append(points[-1])
    return out
