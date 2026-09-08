"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: route.py
Brief: Polyline follow -- projection, lookahead, monotone index (P1 -- 20 S2)

Description:
The polyline follower: the three quantities of 20 S2.2 -- nearest point F (with
its segment index i and in-segment parameter t), lateral deviation e, and the
lookahead point R at arc-length L ahead of F. R is the SINGLE basis for every
direction decision in the module (S2.2): S6 candidate scoring and S7 wall-follow
exit all reference "toward R", never the raw start-end chord.

Two invariants this file owns:
  - RNS-N-4 (S2.6): the nearest point is searched only in an index window around
    the current index, and the passed segment index NEVER retreats. Full linear
    search each tick would blow S10.2's budget (5000 pts x 0.5 m = 2.5 km); a
    retreating index makes the robot turn back and reads as "it turned around on
    its own", nearly impossible to attribute.
  - The lookahead L = clamp(k*v, L_min, L_max) (S2.2). L is arc-length, walked
    forward from F across as many segments as needed.

This slice (P1.1/P1.2): projection + lookahead + monotone index. goto/path
unification (P1.3), progressive align (P1.4), deviation-limit failure (P1.5) land
next; the route-pointer consume with route_rev cross-check (P1.8) after that.

Frame: all math is in the local ENU/base metric frame. WGS84->ENU conversion
happens upstream (S2.2); this file never sees lat/lon.

Trap this shape guards: searching the whole polyline "because it is more
accurate" reintroduces the S10.2 budget blowout and the retreat bug. The window
is not an optimization to revisit -- it is the correctness boundary (S2.6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class Projection:
    """The result of projecting X onto the polyline this tick (20 S2.2)."""
    seg_index: int      # segment i: the point lies on segment P[i]..P[i+1]
    t: float            # in-segment parameter in [0, 1]
    foot: Point         # nearest point F on the polyline
    deviation_m: float  # e = |X - F| (S2.2 quantity 2; S2.7 limit)
    s_arc_m: float      # arc-length progress of F from polyline start


def _seg_project(x: Point, a: Point, b: Point) -> Tuple[float, Point, float]:
    """Project x onto segment a->b. Returns (t clamped to [0,1], foot point,
    squared distance). t is the fraction along a->b."""
    ax, ay = a
    bx, by = b
    px, py = x
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0.0:
        # degenerate segment (duplicate point): foot is a, t = 0.
        fx, fy = ax, ay
        t = 0.0
    else:
        t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
        fx, fy = ax + t * dx, ay + t * dy
    ddx, ddy = px - fx, py - fy
    return t, (fx, fy), ddx * ddx + ddy * ddy


def _seg_len(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


class PolylineTracker:
    """Holds a polyline and the monotone current index. Constructed once per
    mission; project() is called each tick with the current position.

    RNS-N-4: the passed index (min_index) only advances. project() searches
    [min_index, min_index + window] and, if the foot lands beyond min_index,
    advances min_index -- it never moves it back."""

    def __init__(self, points: Sequence[Point], search_window: int) -> None:
        if len(points) < 1:
            raise ValueError("polyline needs at least one point (20 RNS-N-1)")
        self._pts: List[Point] = list(points)
        if search_window < 1:
            raise ValueError("search_window must be >= 1 (20 S2.6)")
        self._window = search_window
        self._min_index = 0
        # cumulative arc-length at each vertex, for s_arc and lookahead.
        self._cum: List[float] = [0.0]
        for i in range(1, len(self._pts)):
            self._cum.append(self._cum[-1] + _seg_len(self._pts[i - 1], self._pts[i]))

    @property
    def total_length_m(self) -> float:
        return self._cum[-1]

    @property
    def min_index(self) -> int:
        return self._min_index

    def project(self, x: Point) -> Projection:
        """Nearest point within the index window, advancing min_index (S2.6).
        A single point (goto) has no segment: foot is that point, deviation is
        the straight-line distance, s_arc is 0."""
        n = len(self._pts)
        if n == 1:
            fx, fy = self._pts[0]
            e = math.hypot(x[0] - fx, x[1] - fy)
            return Projection(seg_index=0, t=0.0, foot=(fx, fy),
                              deviation_m=e, s_arc_m=0.0)

        lo = self._min_index
        hi = min(lo + self._window, n - 2)  # last segment is P[n-2]..P[n-1]
        best_i = lo
        best_t = 0.0
        best_foot = self._pts[lo]
        best_dsq = float("inf")
        for i in range(lo, hi + 1):
            t, foot, dsq = _seg_project(x, self._pts[i], self._pts[i + 1])
            if dsq < best_dsq:
                best_dsq = dsq
                best_i = i
                best_t = t
                best_foot = foot
        # RNS-N-4 monotonicity is STRUCTURAL, not a guard: the search lower bound
        # is min_index and range(lo, ...) only visits i >= min_index, so best_i
        # >= min_index always. Advancing to best_i therefore never retreats.
        # (A `if best_i > min_index` guard here would be dead code / an
        # equivalent mutant -- CLAUDE.md 7.2.1: note the real source instead of
        # asserting a branch that can never be false.) The design cost is
        # deliberate: a point nearest a segment BEHIND min_index is never found;
        # that is the S2.6 trade ("worst case: thinks it passed a segment and
        # keeps going, caught by the S2.7 deviation limit").
        self._min_index = best_i
        s_arc = self._cum[best_i] + best_t * _seg_len(
            self._pts[best_i], self._pts[best_i + 1])
        return Projection(seg_index=best_i, t=best_t, foot=best_foot,
                          deviation_m=math.sqrt(best_dsq), s_arc_m=s_arc)

    def lookahead_point(self, proj: Projection, lookahead_m: float) -> Point:
        """R: walk forward arc-length lookahead_m from the foot F (S2.2). Clamps
        at the polyline end (goto / final approach: R is the endpoint)."""
        if len(self._pts) == 1:
            return self._pts[0]
        target_s = proj.s_arc_m + max(0.0, lookahead_m)
        if target_s >= self._cum[-1]:
            return self._pts[-1]
        # find the segment containing target_s
        i = proj.seg_index
        while i < len(self._pts) - 1 and self._cum[i + 1] < target_s:
            i += 1
        seg_len = _seg_len(self._pts[i], self._pts[i + 1])
        if seg_len == 0.0:
            return self._pts[i]
        frac = (target_s - self._cum[i]) / seg_len
        ax, ay = self._pts[i]
        bx, by = self._pts[i + 1]
        return (ax + frac * (bx - ax), ay + frac * (by - ay))


def lookahead_distance(v_mps: float, k: float, l_min: float, l_max: float) -> float:
    """L = clamp(k * v, L_min, L_max) (20 S2.2). All args from rns.yaml
    (route.lookahead_*); no defaults here (CLAUDE.md 3.1 -- config supplies them
    or refuse-to-start upstream)."""
    raw = k * v_mps
    return l_min if raw < l_min else (l_max if raw > l_max else raw)
