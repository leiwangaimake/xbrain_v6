"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: candidate.py
Brief: Thread/detour candidates -- subgoal, gates, cost, hysteresis (P4 -- 20 S6)

Description:
Threading and detouring are ONE algorithm (RNS-N-9, S6.1): edge discontinuity ->
extrapolate a safe radius -> subgoal -> score. There is no "is this a thread or a
detour" branch; the map-level difference falls out of the score.

The pipeline, and the assertions each step carries:
  1. edges: where d_block(theta) JUMPS along the profile (one side has an obstacle
     at 1.5 m, the neighbor is clear to 8 m) -- the obstacle's tangent under that
     view.
  2. subgoal: MUST extrapolate by `clear` off the edge (A-GAP-1). Aiming at the
     tangent grazes the corner; aiming at a gap midpoint stops the robot IN the
     gap.
  3. hard gates (S6.2): clearance < r_eff+margin (A-GAP-2), dynamic footprint
     crossing the P->S corridor (A-GAP-6), fence/hazard, S in unobserved space.
     ANY failed gate EXCLUDES the candidate -- clearance is a GATE, not a soft
     penalty (A-GAP-2/3 pair).
  4. cost (S6.4): saturating clearance (A-GAP-3: width saturates past the gate,
     never linear), angle to R not to the raw polyline (A-GAP-4), path length
     |X->S|+|S->R| (A-GAP-5: the side-select term), unknown ratio, hysteresis.
  5. hysteresis (S6.6): change candidate only after side_hold_ticks (A-HYS-1),
     but NEVER hold a candidate a hard gate just excluded (A-HYS-2: hysteresis
     does not cross the gate).

Trap this file's shape guards: making clearance a soft cost term (not a gate)
lets a too-narrow gap win on other merits (A-GAP-2); making width a linear cost
lets a wide gap behind-and-to-the-side beat an adequate gap dead ahead (A-GAP-3).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class Candidate:
    """One thread/detour candidate: the subgoal S and the geometry the gates and
    cost read. clearance_m is the gap half-width available at S."""
    subgoal: Point
    clearance_m: float
    dynamic_blocked: bool     # a dynamic footprint crosses the P->S corridor
    in_unobserved: bool       # S lands in unobserved-and-unremembered space
    fence_hazard: bool        # P->S crosses allow-fence-outside / hazard


@dataclass(frozen=True)
class Edge:
    """A profile discontinuity: at bearing theta, d_block jumps between the near
    side (obstacle) and the far side (clear). The tangent point is on the near
    side at d_near. clear_side records WHICH bearing side is clear (+1 = the
    higher-bearing bin, -1 = the lower) -- REVIEW-FIX B5: without it the outward
    normal was fixed CCW and extrapolated INTO the obstacle whenever the clear
    space lay clockwise of the ray."""
    theta_rad: float
    d_near_m: float           # obstacle distance on the near side
    d_far_m: float            # clear distance on the far side
    clear_side: int = 1       # +1 clear at higher bearing, -1 at lower (B5)


def find_edges(d_block: Sequence[Optional[float]], angle_min: float,
               angle_step: float, edge_jump_m: float) -> List[Edge]:
    """Scan the profile for bearings where d_block JUMPS by more than edge_jump_m
    between adjacent bins (S6.1). A None (no obstacle) neighbor next to a finite
    d_block is the strongest jump -- the edge of an obstacle against open space."""
    edges: List[Edge] = []
    for i in range(len(d_block) - 1):
        a, b = d_block[i], d_block[i + 1]
        # treat None as "far" (no obstacle in range) for jump detection.
        av = a if a is not None else math.inf
        bv = b if b is not None else math.inf
        if abs(av - bv) > edge_jump_m and (av != math.inf or bv != math.inf):
            near = min(av, bv)
            far = max(av, bv)
            # B5: which side is clear? av is bin i (lower bearing), bv is bin
            # i+1 (higher). Obstacle on the lower side (av < bv) -> clear side
            # is the higher bearing (+1); obstacle on the higher side -> -1.
            side = 1 if av < bv else -1
            # the edge bearing is the boundary between the two bins.
            theta = angle_min + (i + 0.5) * angle_step
            edges.append(Edge(theta_rad=theta, d_near_m=near, d_far_m=far,
                              clear_side=side))
    return edges


def detour_subgoal(edge: Edge, clear_m: float) -> Point:
    """Detour subgoal (S6.1): S = E + clear * n_hat, where E is the tangent point
    and n_hat is the outward normal (away from the obstacle). MUST extrapolate by
    clear (A-GAP-1) -- aiming at E grazes the corner. Here the outward direction
    is approximated as the far-side bearing offset."""
    ex = edge.d_near_m * math.cos(edge.theta_rad)
    ey = edge.d_near_m * math.sin(edge.theta_rad)
    # outward normal: perpendicular to the ray, toward the CLEAR side (B5).
    # (+90 deg is the higher-bearing direction; clear_side flips it when the
    # clear space lies clockwise. The old fixed-CCW form extrapolated into the
    # obstacle for clockwise-clear edges and got the candidate gate-rejected.)
    nx = -math.sin(edge.theta_rad) * edge.clear_side
    ny = math.cos(edge.theta_rad) * edge.clear_side
    return (ex + clear_m * nx, ey + clear_m * ny)


def thread_subgoal(e1: Point, e2: Point, robot: Point, clear_m: float) -> Point:
    """Thread subgoal (S6.1): M = (E1+E2)/2, S = M + clear * u_hat, u_hat = robot
    -> M direction. MUST extrapolate past the midpoint (A-GAP-1) -- aiming at M
    means arrival = done = stopped in the gap."""
    mx = (e1[0] + e2[0]) / 2.0
    my = (e1[1] + e2[1]) / 2.0
    dx, dy = mx - robot[0], my - robot[1]
    norm = math.hypot(dx, dy)
    if norm == 0.0:
        return (mx, my)
    ux, uy = dx / norm, dy / norm
    return (mx + clear_m * ux, my + clear_m * uy)


def clear_extrapolation(r_eff_m: float, margin_m: float,
                        subgoal_extra_m: float) -> float:
    """clear = r_eff + margin_by_class + subgoal_extra_m (S6.1, v1.11). Same first
    two terms as hard gate 1 (S6.2) -- not an independent magic number."""
    return r_eff_m + margin_m + subgoal_extra_m


# ── hard gates (S6.2) -- ANY failure excludes the candidate ────────────────────
def passes_hard_gates(c: Candidate, r_eff_m: float, margin_m: float) -> bool:
    """All four hard gates (S6.2). Clearance is a GATE (A-GAP-2): insufficient
    clearance EXCLUDES, it is never traded off in the cost. Dynamic-occupied
    (A-GAP-6), fence/hazard, and unobserved-S also exclude."""
    if c.clearance_m < r_eff_m + margin_m:   # gate 1: A-GAP-2
        return False
    if c.dynamic_blocked:                     # gate 2: A-GAP-6
        return False
    if c.fence_hazard:                        # gate 3
        return False
    if c.in_unobserved:                       # gate 4: RNS-I-6
        return False
    return True


# ── cost (S6.4) ────────────────────────────────────────────────────────────────
def clearance_penalty(clearance_m: float, r_eff_m: float, margin_m: float,
                      gate_saturate_ratio: float) -> float:
    """Saturating clearance penalty (S6.3/A-GAP-3): past the gate, extra width
    saturates -- it does NOT linearly reduce the penalty. rho = clearance /
    (r_eff+margin); penalty falls from 1 at the gate toward 0, but flattens at
    gate_saturate_ratio. A wider-than-saturation gap gets no further credit, so a
    wide gap to the side cannot beat an adequate gap dead ahead on width alone."""
    base = r_eff_m + margin_m
    if base <= 0.0:
        raise ValueError("r_eff+margin must be > 0")
    rho = clearance_m / base
    if rho <= 1.0:
        return 1.0            # at/below the gate (gate excludes below 1 anyway)
    if rho >= gate_saturate_ratio:
        return 0.0            # saturated: no further credit for extra width
    # linear from 1 (rho=1) to 0 (rho=saturate) -- bounded, not unbounded.
    return 1.0 - (rho - 1.0) / (gate_saturate_ratio - 1.0)


def _angle_to_r(subgoal: Point, robot: Point, r_point: Point) -> float:
    """Delta theta between (robot -> subgoal) and (robot -> R) (A-GAP-4: the
    reference is R, the lookahead point, NOT the raw polyline). On the line R is
    the path direction; off the line R points back, so "returns to line"
    candidates score better automatically."""
    a = math.atan2(subgoal[1] - robot[1], subgoal[0] - robot[0])
    b = math.atan2(r_point[1] - robot[1], r_point[0] - robot[0])
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d <= -math.pi:
        d += 2 * math.pi
    return abs(d)


def candidate_cost(
    c: Candidate, robot: Point, r_point: Point,
    unknown_ratio: float, is_previous: bool,
    *, r_eff_m: float, margin_m: float, gate_saturate_ratio: float,
    w_clr: float, w_ang: float, w_len: float, w_unk: float, w_hys: float,
) -> float:
    """The five-weight cost (S6.4). Length uses |X->S| + |S->R| (A-GAP-5: the
    side-select term -- without it the two ends of a wall are indistinguishable
    when clearance and angle match). Angle is to R (A-GAP-4). Hysteresis term
    rewards keeping the previous candidate (applied AFTER the gates, S6.6)."""
    clr_pen = clearance_penalty(c.clearance_m, r_eff_m, margin_m, gate_saturate_ratio)
    ang = _angle_to_r(c.subgoal, robot, r_point)
    len_xs = math.hypot(c.subgoal[0] - robot[0], c.subgoal[1] - robot[1])
    len_sr = math.hypot(r_point[0] - c.subgoal[0], r_point[1] - c.subgoal[1])
    hys = 0.0 if is_previous else 1.0
    return (w_clr * clr_pen + w_ang * ang + w_len * (len_xs + len_sr)
            + w_unk * unknown_ratio + w_hys * hys)


# ── selection with hysteresis (S6.6) ──────────────────────────────────────────
class CandidateSelector:
    """Selects among feasible candidates with change-hysteresis (S6.6). Switch
    to a new best only after it wins for side_hold_ticks consecutive ticks
    (A-HYS-1). But if the CURRENT candidate fails a hard gate, drop it IMMEDIATELY
    -- hysteresis never crosses the gate (A-HYS-2)."""

    def __init__(self, side_hold_ticks: int) -> None:
        self._hold = side_hold_ticks
        self._current: Optional[Candidate] = None
        self._challenger: Optional[Candidate] = None
        self._challenger_streak = 0

    def select(self, current_feasible: bool, best: Optional[Candidate],
               best_is_current: bool) -> Optional[Candidate]:
        """current_feasible: does the currently-held candidate still pass the
        gates this tick. best: the lowest-cost feasible candidate this tick.
        best_is_current: whether best is the same as the held one."""
        # A-HYS-2: current failed a gate -> drop NOW, no hysteresis across a gate.
        if self._current is not None and not current_feasible:
            self._current = None
            self._challenger = None
            self._challenger_streak = 0
        if self._current is None:
            self._current = best        # nothing held -> take best immediately
            self._challenger = None
            self._challenger_streak = 0
            return self._current
        if best_is_current or best is None:
            self._challenger = None     # best is what we hold, or nothing better
            self._challenger_streak = 0
            return self._current
        # a different candidate is best: it must win side_hold_ticks in a row.
        # REVIEW-FIX B3 (2026-09-09): compare by VALUE (==), not identity (is).
        # The real loop rebuilds Candidate objects from the profile every tick;
        # identical geometry is a NEW object each time, so `is` was always False,
        # the streak reset to 1 every tick, and hysteresis switching could never
        # happen (hold > 1). Candidate is a frozen dataclass -- == is well-defined.
        if self._challenger == best:
            self._challenger_streak += 1
        else:
            self._challenger = best
            self._challenger_streak = 1
        if self._challenger_streak >= self._hold:   # A-HYS-1
            self._current = best
            self._challenger = None
            self._challenger_streak = 0
        return self._current


# ── perception-side candidate construction (S2 assembly -- 20 S6.1/S6.2) ──────
def _polar_to_xy(theta: float, d: float) -> Point:
    return (d * math.cos(theta), d * math.sin(theta))


def obstacle_points(d_block: Sequence[Optional[float]], angle_min: float,
                    angle_step: float) -> List[Point]:
    """Body-frame cartesian points of every blocked bin -- the geometry the
    clearance and unknown checks measure against. Perception data only."""
    pts: List[Point] = []
    for i, db in enumerate(d_block):
        if db is not None:
            pts.append(_polar_to_xy(angle_min + i * angle_step, db))
    return pts


def clearance_at(p: Point, obs_pts: Sequence[Point], range_max_m: float) -> float:
    """Clearance of a body-frame point = distance to the nearest sensed obstacle
    point (profile-derived, NOT world truth -- RNS consumes perception only).
    No obstacle in view -> the sensing range bounds what we can claim."""
    if not obs_pts:
        return range_max_m
    return min(math.hypot(p[0] - ox, p[1] - oy) for ox, oy in obs_pts)


def corridor_clearance(s_pt: Point, obs_pts: Sequence[Point],
                       range_max_m: float) -> float:
    """COLLISION FIX (user SIL audit 2026-09-10): min clearance of the WHOLE
    X->S walk (robot at body origin), i.e. min distance from the segment
    (0,0)->S to every sensed obstacle point. The old gate tested only the
    SUBGOAL point -- a thread subgoal extrapolates BEYOND the gap, so a 0.13 m
    slot between two cars passed the gate (the endpoint sat in open space) and
    the robot drove straight through the car bodies. 20 S6.2's clearance is the
    PASSAGE clearance; the gate must sweep the corridor, not probe its end."""
    if not obs_pts:
        return range_max_m
    sx, sy = s_pt
    seg_len_sq = sx * sx + sy * sy
    best = range_max_m
    for ox, oy in obs_pts:
        if seg_len_sq < 1e-12:
            d = math.hypot(ox, oy)
        else:
            t = (ox * sx + oy * sy) / seg_len_sq
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            d = math.hypot(ox - t * sx, oy - t * sy)
        if d < best:
            best = d
    return best


def unknown_ratio_toward(p: Point, d_free: Sequence[Optional[float]],
                         angle_min: float, angle_step: float,
                         window_bins: int = 5) -> float:
    """Fraction of bins around the subgoal's bearing whose confirmed-free
    distance does NOT reach the subgoal (S6.4 w_unk input): free shorter than
    |X->S| (or unobserved None) means part of that ray is UNKNOWN."""
    theta = math.atan2(p[1], p[0])
    dist = math.hypot(p[0], p[1])
    n = len(d_free)
    center = round((theta - angle_min) / angle_step)
    lo, hi = max(0, center - window_bins), min(n - 1, center + window_bins)
    if hi < lo:
        return 1.0    # bearing outside the sector: nothing confirmed
    bad = 0
    for i in range(lo, hi + 1):
        df = d_free[i]
        if df is None or df < dist:
            bad += 1
    return bad / (hi - lo + 1)


def candidates_from_profile(
    d_block: Sequence[Optional[float]], d_free: Sequence[Optional[float]],
    angle_min: float, angle_step: float, range_max_m: float,
    edge_jump_m: float, clear_m: float,
) -> List[Candidate]:
    """Build the tick's candidate set from the profile (S6.1): edges -> detour
    subgoals off each edge, thread subgoals in each gap between an obstacle's
    right edge and the next obstacle's left edge. All body-frame; the caller
    rotates to world. Gates/cost run on the returned Candidates as usual --
    this function only CONSTRUCTS, it never filters (gates are the filter)."""
    edges = find_edges(d_block, angle_min, angle_step, edge_jump_m)
    obs = obstacle_points(d_block, angle_min, angle_step)
    cands: List[Candidate] = []

    def _mk(p: Point) -> Candidate:
        # PASSAGE clearance: sweep the X->S corridor (collision fix, see
        # corridor_clearance) -- the subgoal-point probe let sub-body gaps pass.
        clr = corridor_clearance(p, obs, range_max_m)
        dist = math.hypot(p[0], p[1])
        theta = math.atan2(p[1], p[0])
        n = len(d_free)
        bi = round((theta - angle_min) / angle_step)
        # unobserved: bearing outside the sector, or the free ray does not
        # confirm space out to the subgoal (RNS-I-6 / gate 4 input).
        if bi < 0 or bi >= n:
            unobs = True
        else:
            df = d_free[bi]
            unobs = df is None or df + clear_m < dist
        return Candidate(subgoal=p, clearance_m=clr, dynamic_blocked=False,
                         in_unobserved=unobs, fence_hazard=False)

    for e in edges:
        cands.append(_mk(detour_subgoal(e, clear_m)))
    # thread: a gap is a +1 edge (clear at higher bearing) followed by a -1
    # edge (clear at lower bearing) -- open space between two obstacles.
    for a, b in zip(edges, edges[1:]):
        if a.clear_side == 1 and b.clear_side == -1:
            e1 = _polar_to_xy(a.theta_rad, a.d_near_m)
            e2 = _polar_to_xy(b.theta_rad, b.d_near_m)
            cands.append(_mk(thread_subgoal(e1, e2, (0.0, 0.0), clear_m)))
    return cands
