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
                             "self_intersect", "area_too_small",
                             # 99 U176: the name must declare one type
                             "fence_type_missing",
                             # 99 U184: a forbid zone overlapping a warning,
                             # speed-limit or other forbid zone
                             "forbid_overlap"})
# ~~U176: outside_activity_area / activity_area_missing / activity_area_excludes
# / activity_area_inactive blocking~~ -- 99 U184 (user 2026-11-13): saved and
# told, never refused; activity_area_missing is gone (no activity area is legal).
ADVISORY_ISSUES = frozenset({"degenerate_edge", "auto_closed",
                             "close_gap_large", "robot_outside",
                             "outside_active_fence", "low_quality_ratio",
                             "outside_activity_area", "activity_area_excludes",
                             "activity_area_inactive"})

#: S12A.7 default, marked as awaiting field calibration in the contract. A
#: recording-quality threshold, not a common.safety.* parameter: the failure
#: direction of a wrong value is "the operator is asked to re-record", never
#: motion, so a default here is not the CLAUDE.md 3.1 case.
#: ~~DEFAULT_MIN_AREA_M2 = 100.0~~ removed 2026-11-13 (99 U191, user ruling
#: "1 平方米"): the minimum fence area is p3_task.yaml fence.min_area_m2, read
#: by the recording AND the cmd/geo path, injected, never written here.
DEFAULT_CLOSE_TOL_M = 5.0
#: 11 S9A.2 FV-2 "无零长边 (< 0.05 m)": two vertices this close (neighbours, or
#: the closing pair) are one vertex written twice. A contract figure, not a
#: tunable -- it only has to be far below any real edge.
DUPLICATE_VERTEX_M = 0.05
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


def check_fence_layout(v: Validation, points: Sequence[Tuple[float, float]], *,
                       role: Optional[str],
                       activity_area: Optional[Tuple[str, Sequence[Tuple[float, float]]]],
                       others: Sequence[Tuple[str, Sequence[Tuple[float, float]], str]] = ()
                       ) -> None:
    """A fence's place among the others, added to its validation at save time.
    `activity_area` is the activity area's (name, ring) or None, `others` the
    other live fences as (name, ring, role).

    99 U184 (user 2026-11-13), replacing 99 U176's refusals:
      * a forbid zone may not overlap a warning zone, a speed-limit zone or
        another forbid zone (the overlapped part of the first two would be
        unreachable, and two forbid zones overlapping should be one) ->
        forbid_overlap, BLOCKING, checked from either side;
      * a fence outside the activity area is saved, and told: it can never be
        reached -> outside_activity_area, advisory; an activity area that
        leaves fences outside -> activity_area_excludes, advisory;
      * no activity area at all is not an issue: the other fences are in
        force without one (99 U183).
    ~~U176: activity_area_missing / outside_activity_area /
    activity_area_excludes all blocking~~ superseded 2026-11-13 -- the user
    ruled "如果在外系统不报错, 但是永远到不了".
    A name declaring no type (role None) stays blocking (fence_type_missing):
    such a fence has no role to be saved under."""
    if role is None:
        v.add("fence_type_missing")
        return
    clash = [name for name, ring, orole in others
             if (role == "forbid" and orole in _FORBID_CLASH_ROLES
                 or role in _FORBID_CLASH_ROLES and role != "forbid" and orole == "forbid")
             and ring_overlap(points, ring)]
    if clash:
        v.add("forbid_overlap", fences=clash)
    if role == "allow":
        outside = [name for name, ring, orole in others
                   if orole != "allow" and not ring_within(ring, points)]
        if outside:
            v.add("activity_area_excludes", fences=outside)
    elif activity_area is not None and not ring_within(points, activity_area[1]):
        v.add("outside_activity_area", activity_area=activity_area[0])


#: 99 U184: the roles a forbid zone may not overlap (forbid itself included)
_FORBID_CLASH_ROLES = frozenset({"forbid", "warning", "speed_limit"})


def ring_overlap(a: Sequence[Tuple[float, float]],
                 b: Sequence[Tuple[float, float]]) -> bool:
    """99 U184: do rings a and b share any area? An edge crossing, or one ring
    holding a vertex of the other (which covers full containment both ways).
    Rings that only TOUCH along a shared edge share no area and do not count:
    _segments_intersect reports proper crossings only, and the rule is about
    an area that becomes unreachable, which a shared edge does not create."""
    if len(a) < 3 or len(b) < 3:
        return False
    n, m = len(a), len(b)
    for i in range(n):
        a1, a2 = a[i], a[(i + 1) % n]
        for j in range(m):
            if _segments_intersect(a1, a2, b[j], b[(j + 1) % m]):
                return True
    return point_in_ring(a[0], b) or point_in_ring(b[0], a)


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


def ring_within(inner: Sequence[Tuple[float, float]],
                outer: Sequence[Tuple[float, float]]) -> bool:
    """99 U176: does ring `inner` lie entirely inside ring `outer`? Every inner
    vertex inside (ray casting) AND no inner edge crossing an outer edge -- a
    vertex test alone passes a ring whose edge cuts across a concave notch of
    the outer one. Lat/lon plane, the same camp-scale argument as
    _segments_intersect (inside / crossing are affine-invariant)."""
    if len(inner) < 3 or len(outer) < 3:
        return False  # not rings: nothing can be said to be inside
    if not all(point_in_ring(p, outer) for p in inner):
        return False  # a vertex outside
    n, m = len(inner), len(outer)
    for i in range(n):
        a1, a2 = inner[i], inner[(i + 1) % n]
        for j in range(m):
            if _segments_intersect(a1, a2, outer[j], outer[(j + 1) % m]):
                return False  # an edge leaves and comes back
    return True


def validate_fence(points: Sequence[Tuple[float, float]], *,
                   dropped_by_quality: int = 0,
                   robot_at: Optional[Tuple[float, float]] = None,
                   activate: bool = False,
                   min_area_m2: Optional[float],
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
    # 99 U191: min_area_m2 is p3_task.yaml fence.min_area_m2 (required at P3
    # start); None only in tests of the teach path alone -> no area rule
    if min_area_m2 is not None and area < min_area_m2:
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


def ring_geometry_refusal(points: Sequence[Tuple[float, float]],
                          min_area_m2: Optional[float]) -> Optional[Tuple[str, Dict[str, object]]]:
    """11 S9A.2 FV-1 / FV-2 / FV-5 for a fence ring WRITTEN through cmd/geo
    (cloud SET_FENCE_CONFIG / SET_ALARM_CONFIG regions, voice / HMI edits):
    (reason, detail) of the first rule broken, or None.

    99 U191 (2026-11-13): until now cmd/geo checked only the vertex count --
    a figure-of-eight, a zero-area or a 1 cm ring, and duplicated vertices were
    stored, could be switched on and reached P1, which does not re-check them
    (its ray cast then decides "inside" for a shape that has none), while the
    cloud contract said such a fence is refused. The recording path has always
    run these rules (validate_fence); this is the same rules for a ring that
    arrives finished, so no closure gap and no auto-merge here: a vertex sent
    twice is the sender's mistake and is refused, not silently repaired.
      too_few_vertices   fewer than 3 (FV-1)
      duplicate_vertex   two vertices < DUPLICATE_VERTEX_M apart, neighbours
                         or not (FV-2 "无重复顶点, 无零长边")
      self_intersect     two non-adjacent edges cross (FV-2), with the pair
      area_too_small     area < min_area_m2 (FV-5, user ruling 1 m^2, every
                         role); min_area_m2 None = not configured (tests of the
                         geo path alone), the area rule is then skipped
    Order: the cheapest and most basic first, so the reason names the root."""
    n = len(points)  # vertex count
    if n < _MIN_RING_VERTICES:  # FV-1
        return "too_few_vertices", {"vertex_count": n}
    for i in range(n):  # FV-2: every pair, the ring is small (P5 caps 512)
        for j in range(i + 1, n):
            if haversine_m(points[i], points[j]) < DUPLICATE_VERTEX_M:
                return "duplicate_vertex", {"index": j, "same_as": i}
    crossing = ring_self_intersects(points)  # FV-2: the first crossing pair
    if crossing is not None:
        return "self_intersect", {"edges": list(crossing)}
    area = ring_area_m2(points)  # square metres
    if min_area_m2 is not None and area < min_area_m2:  # FV-5
        return "area_too_small", {"area_m2": round(area, 2), "min_area_m2": float(min_area_m2)}
    return None


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
