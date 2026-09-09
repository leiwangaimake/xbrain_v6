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

This slice (P1.1/P1.2/P1.3): projection + lookahead + monotone index, and the
Mission layer that unifies goto and path (RNS-N-1: one follow code path, not two).
Progressive align (P1.4), deviation-limit failure (P1.5), and the route-pointer
consume with route_rev cross-check (P1.8) land next.

RNS-N-1 (S2.1): goto = a one-segment polyline, path = a dense polyline. Both run
the SAME PolylineTracker; there is no separate goto branch (A-RT-1). Arrival is
judged ONLY at the endpoint (A-RT-2 / S2.4): applying arrival_radius to an
intermediate 0.5 m point would jump the index past several points and destroy the
path shape.

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

from .types import NavFailReason, NavFailure

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


@dataclass(frozen=True)
class FollowState:
    """One tick's follow output: where the foot is, how far off the path, the
    lookahead point R, and whether the endpoint is reached. This is what the
    speed/heading layers (P1.4/P1.7) consume; it is NOT a velocity yet."""
    projection: Projection
    lookahead_point: Point
    endpoint: Point
    dist_to_endpoint_m: float
    arrived: bool           # endpoint reached (radius only; heading is P1.4)
    # (deviation_exceeded removed with the deviation-failure mechanism,
    # 20 S2.7 v1.20 user ruling; deviation_m in projection still feeds the
    # speed cap.)


class Mission:
    """A navigation mission: goto or path, unified as a polyline (RNS-N-1).

    Carries origin (route|relmove) so the failure/terminal report goes to the
    right carrier (12 S4.2c.1): the Mission does not choose the channel itself,
    but source.py reads mission.origin to pick it. Kind is goto|path (follow_target
    is reserved, #20-13, and never constructed here this phase).

    advance(x) runs one tick of follow: project, compute R, and judge arrival --
    ONLY at the endpoint (A-RT-2). There is no per-intermediate-point arrival."""

    def __init__(
        self,
        kind,            # MissionKind (typed in types.py; kept loose to avoid a
        origin,          # Origin        cyclic import -- source.py passes enums)
        points: Sequence[Point],
        *,
        search_window: int,
        arrival_radius_m: float,
        max_deviation_m: float,
    ) -> None:
        self.kind = kind
        self.origin = origin
        self._tracker = PolylineTracker(points, search_window)
        self._search_window = search_window
        self._arrival_radius_m = arrival_radius_m
        self._max_deviation_m = max_deviation_m
        self._endpoint: Point = tuple(points[-1])  # type: ignore[assignment]
        # B4: a single-point mission anchors its start at the first observed
        # pose (RNS-N-1: "the start vertex IS the current pose"); done lazily in
        # advance() because the pose is not known at construction.
        self._goto_anchored = len(points) > 1

    @property
    def tracker(self) -> PolylineTracker:
        return self._tracker

    @property
    def endpoint(self) -> Point:
        return self._endpoint

    def advance(self, x: Point, lookahead_m: float) -> FollowState:
        """One follow tick. Arrival is endpoint-only (S2.4): dist(X, P[n]) <
        arrival_radius. Intermediate points never trigger arrival -- they only
        shape F and R."""
        # REVIEW-FIX B4 (2026-09-09): RNS-N-1 -- a single-point goto's polyline
        # starts at the CURRENT POSE. A raw one-point tracker has no line to
        # deviate from, so project() returned deviation = straight-line distance
        # to the goal; the deviation cap then crawled a far goto at dev_g_min and
        # the deviation cap crawled at dev_g_min on the first tick.
        # Prepend the first observed pose, making it a two-point polyline with
        # correct perpendicular-deviation semantics.
        if not self._goto_anchored:
            self._tracker = PolylineTracker([x, self._endpoint],
                                            self._search_window)
            self._goto_anchored = True
        proj = self._tracker.project(x)
        r = self._tracker.lookahead_point(proj, lookahead_m)
        dist_end = math.hypot(x[0] - self._endpoint[0], x[1] - self._endpoint[1])
        # A-RT-2: arrival is judged against the ENDPOINT only, never against the
        # current segment's far vertex. On a 0.5 m path, an intermediate arrival
        # radius (~1 m) would skip several points and break the shape.
        arrived = dist_end < self._arrival_radius_m
        return FollowState(
            projection=proj,
            lookahead_point=r,
            endpoint=self._endpoint,
            dist_to_endpoint_m=dist_end,
            arrived=arrived,
        )


# ── progressive alignment (P1.4 -- 20 S2.5 / RNS-N-3) ─────────────────────────

def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]. Used for every heading difference so a 359 deg error
    reads as -1 deg, not 359 (20 S2.5 uses wrap on all heading deltas)."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def align_weight(d_remaining_m: float, align_dist_m: float) -> float:
    """w(d) = clamp(1 - d/align_dist_m, 0, 1) (20 S2.5). 0 far out (heading = path
    direction), 1 at the endpoint (heading = goal heading). Linear between: the
    heading BLENDS in over the last align_dist_m, so the robot turns while moving,
    never in place (RNS-N-3)."""
    w = 1.0 - d_remaining_m / align_dist_m
    return 0.0 if w < 0.0 else (1.0 if w > 1.0 else w)


def desired_heading(theta_path: float, psi_goal: Optional[float],
                    d_remaining_m: float, align_dist_m: float) -> float:
    """theta_des = theta_path (+) w(d)*wrap(psi_goal - theta_path) (20 S2.5). With
    no goal heading (psi_goal None), theta_des is just the path direction -- no
    alignment demand (S2.4: intermediate points have no heading requirement)."""
    if psi_goal is None:
        return theta_path
    w = align_weight(d_remaining_m, align_dist_m)
    return theta_path + w * wrap_angle(psi_goal - theta_path)


def align_omega(theta_des: float, psi_now: float, k_yaw: float,
                wz_max: float) -> float:
    """omega = clamp(k_yaw * wrap(theta_des - psi), +/- wz_max) (20 S2.5 P law)."""
    raw = k_yaw * wrap_angle(theta_des - psi_now)
    return wz_max if raw > wz_max else (-wz_max if raw < -wz_max else raw)


def arrived_with_heading(
    dist_to_endpoint_m: float, arrival_radius_m: float,
    psi_goal: Optional[float], psi_now: float, yaw_tol_rad: float,
) -> bool:
    """Arrival DOUBLE condition (20 S2.5, A-ALN-3): radius AND heading tolerance
    (when a goal heading exists). Judging radius only would report arrival with
    the heading still 30 deg off -- the exact bug this eliminates. No goal heading
    -> radius alone (S2.4)."""
    if dist_to_endpoint_m >= arrival_radius_m:
        return False
    if psi_goal is None:
        return True
    return abs(wrap_angle(psi_goal - psi_now)) <= yaw_tol_rad


# ── route pointer + route_rev cross-check (P1.8 -- 12 S3.5A / 11 S7.12.1) ─────

class RouteRevMismatch(RuntimeError):
    """A BehaviorCommand's route_rev does not match the loaded RouteGeometry's
    (11 S7.12.1). P1 must refuse to act on a command aimed at a stale route --
    executing it would follow the wrong geometry."""


def check_route_rev(command_route_rev: int, loaded_route_rev: int) -> None:
    """11 S7.12.1: cross-check the route_rev carried on a BehaviorCommand against
    the route_rev of the currently loaded RouteGeometry. Mismatch -> refuse
    (RouteRevMismatch), never silently follow the loaded one -- the command was
    aimed at a different revision of the path."""
    if command_route_rev != loaded_route_rev:
        raise RouteRevMismatch(
            "route_rev mismatch: command=%d loaded=%d (11 S7.12.1 -- refuse, do "
            "not follow the wrong geometry)"
            % (command_route_rev, loaded_route_rev))
