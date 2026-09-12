"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: clip.py
Brief: fence clipping geometry -- ENU compile, signed edge distance, v_fence, offset projection

Description:
Why this exists: 12 S7 opens with "必须作用于最终速度指令,不是规划期检查" --
the fence is enforced on the velocity P1 is about to publish, every tick, by
geometry, not by trusting the planner to stay inside. Without this module a
FenceSet is held (fence_set.py) and reported (state/fence) but constrains
nothing, which state/fence honestly called clip_deferred.

What it computes per 20 Hz tick (11 S9A.6 (2)-(5), S9A.8, 12 S7.1-7.3):
  * d_nom     signed distance from the robot to the nearest edge of each
              constraining polygon, allowed side positive (FE-2: analytic
              edge distance over allow / forbid polygons, no SDF grid);
  * inset     the RTK-quality inset from common.fence.margin_by_fix (12 S7.4,
              12 S6.6 table (1), 11 S3.2.1 table); d_eff = d_nom - inset is
              the distance to the EFFECTIVE hard boundary;
  * v_fence   the closed-form brake inverse of 11 S9A.6 (4),
              (a/k) * (sqrt((k t)^2 + 2 k d_eff / a) - k t), zero at
              d_eff <= 0, with k / t_lat / a the SAME common.safety values the
              speed gate uses (M-01: one set of brake constants);
  * the clip  the ENU velocity projected onto every half-space
              {v . n_j <= cap_j}: cap_j = v_fence(d_eff_j) * h * i for a
              polygon still ahead (the soft band), 0 for a hard_enforce
              polygon at or beyond its effective boundary now or at the
              predicted point p + v * predict_dt_s (12 S7.2 / S7.3). Rounds of
              alternating projection (11 S9A.8 POCS, projection_iters); a
              residual above DEGENERATE_EPS_MPS afterwards is a concave corner
              with no inward direction -> zero + degenerate flag.

Why a half-space projection and not the literal scalar min() of 11 S9A.6 (5):
the scalar form is exactly what this reduces to when the robot drives
straight at the boundary (v parallel to n: v . n <= cap  <=>  |v| <= cap), so
the contract's worked numbers hold. Off-axis it is the only reading that
agrees with the rest of the contract: 12 S7.2 "置零会让机器人卡在边界上无法
退回", and the 11 S9A.9 breach example itself keeps v_after.vx = 0.35 at
d_eff = -1.56 -- a literal min() would print 0.00 there. The half-space form
also has no chatter: a scalar ceiling switched on/off by the sign of v . n
would flip between v_fence and full speed while driving parallel inside the
band. What is registered as a doc-sync item is only this wording.

What it does NOT do: publish, emit events (episodes.py turns FenceEval
transitions into 11 S9A.9 events), or evaluate warning polygons (zones.py:
point-in-polygon, no distance semantics, 12 FS-12). speed_limit polygons are
"区内限速由 RNS 施加" (11 S9A.1A, pending) and are not boundaries here.

Things that look right and are wrong:
  * treating heading_rad as a compass bearing. 11 S3.3: ENU, east = 0,
    counter-clockwise positive; R(yaw) below is that convention. A bearing
    would rotate the outward normal by ~90 deg and clip the wrong axis with
    no visible error.
  * predicting the crossing with the RAW candidate. The soft cap already
    bounds the approach speed to what can be stopped inside d_eff; predicting
    with the uncapped candidate would zero the approach ~0.9 m out at 2 m/s
    and make the soft band dead. The prediction runs on the soft-capped
    velocity and is the belt for the k -> 1 corner where k * t_lat can be
    less than predict_dt_s.
  * the 11 S9A.6 (1) inset formula (r_body + k_pos cov_h + v t_pos + ...) is
    NOT used: 12 S7.4 (the P1 design) shrinks by margin_by_fix[fix_type] and
    the configs carry only that table. The two formulations are a pending
    ruling (NEXT.md); inset_for_fix is the one function that changes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Mapping, Optional, Sequence, Tuple

from xbrain.common.fence.geom import Polygon, point_in_polygon
from xbrain.p1_motion.fence.fence_set import HeldFenceSet
from xbrain.p1_motion.path.local_frame import LocalFrame, LocalFrameError


class FenceClipError(ValueError):
    """A fence constant missing / off-range, or geometry that cannot be
    compiled (CLAUDE.md 3.1: refuse and name the key)."""


#: 11 S3.2.1: fix types whose position admits a fence evaluation (inset from
#: margin_by_fix). dgps / single / no_fix -> fence_eval unavailable (B6: no
#: inset, no clipping; the gate's rtk veto already forbids autonomous motion).
FIX_WITH_FENCE: Tuple[str, ...] = ("rtk_fixed", "rtk_float")
#: 11 S9A.8: residual outward speed after projection_iters rounds above which
#: the constraint cone is degenerate (concave corner) -> zero velocity.
DEGENERATE_EPS_MPS = 0.01
#: 11 S9A.2 roles that constrain motion (see module note for the other two).
CONSTRAINT_ROLES: Tuple[str, ...] = ("allow", "forbid")
#: 11 S9A.5 wire strings this module produces, spelled once.
ENFORCEMENT_FULL = "full"
ENFORCEMENT_WARN_ONLY = "warn_only"
ENFORCEMENT_DISABLED = "disabled"
REASON_NONE = "none"
REASON_RTK = "rtk_degraded"
REASON_HEADING = "heading_lost"
REASON_NO_FENCE = "no_fence"
STATE_INSIDE = "inside"
STATE_SOFT = "soft"
STATE_OUTSIDE = "outside"
STATE_UNKNOWN = "unknown"
#: below this distance the robot is ON the boundary: the gradient direction is
#: undefined and the edge normal is used instead (boundary_hit).
_ON_BOUNDARY_M = 1e-9
#: probe step for orienting an edge normal (which side is the interior).
_PROBE_M = 1e-3


def _positive(name: str, v: object) -> float:
    """Injected constant guard: finite and > 0, else FenceClipError naming it.
    bool is excluded explicitly (True == 1 would pass an isinstance check)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)) \
            or not math.isfinite(v) or v <= 0.0:
        raise FenceClipError("fence constant %s must be a finite positive "
                             "number, got %r" % (name, v))
    return float(v)


@dataclass(frozen=True)
class FenceConstants:
    """Everything the clip reads from the resolved snapshot, injected at
    construction, no defaults (CLAUDE.md 3.1). Key truth (11 S9A.6 M-01 /
    S9A.12 / 12 S12): brake_k = common.safety.brake.k, brake_a_mps2 =
    common.safety.brake.a_mps2, t_lat_s = common.safety.t_lat_s,
    soft_margin_min_m / predict_dt_s / margin_by_fix = common.fence.*,
    projection_iters = p1 fence.projection_iters, v_profile_max_mps = the
    patrol tier (11 S9A.6 (3) d_stop(v_profile_max)), teleop_cap_degraded_mps
    = the 0.5 m/s manual cap of 11 S3.2.1 / S9A.7 (U54: the obstacle_avoid
    tier and the manual fallback are the same row)."""
    brake_k: float
    brake_a_mps2: float
    t_lat_s: float
    soft_margin_min_m: float
    predict_dt_s: float
    margin_by_fix: Mapping[str, float]
    projection_iters: int
    v_profile_max_mps: float
    teleop_cap_degraded_mps: float

    def __post_init__(self) -> None:
        # every scalar is a safety quantity: a null leaf must refuse here, not
        # become 0 (v_fence(d) with a = 0 divides by zero; k = 0 makes the band
        # vanish -- both are the fail-silent shapes CLAUDE.md 3.1 names).
        for name in ("brake_k", "brake_a_mps2", "t_lat_s", "soft_margin_min_m",
                     "predict_dt_s", "v_profile_max_mps", "teleop_cap_degraded_mps"):
            _positive(name, getattr(self, name))
        if isinstance(self.projection_iters, bool) \
                or not isinstance(self.projection_iters, int) or self.projection_iters < 1:
            raise FenceClipError("fence.projection_iters must be a positive int, got %r"
                                 % (self.projection_iters,))
        for fix in FIX_WITH_FENCE:
            v = self.margin_by_fix.get(fix) if isinstance(self.margin_by_fix, Mapping) else None
            # 0.0 is legal here (no inset) only because 12 S12.1 S-6 guards the
            # table's monotonicity at freeze; a missing / null leaf is not.
            if isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not math.isfinite(v) or v < 0.0:
                raise FenceClipError("common.fence.margin_by_fix.%s must be a finite "
                                     "non-negative number, got %r" % (fix, v))


# --- physics (11 S9A.6 (3)/(4)) -------------------------------------------

def d_stop_m(v_mps: float, c: FenceConstants) -> float:
    """11 S9.6.1 / S9A.6 (3): k * (v t_lat + v^2 / (2 a)) -- the distance the
    robot needs to stop from v with the SAME brake constants as the gate."""
    return c.brake_k * (v_mps * c.t_lat_s + v_mps * v_mps / (2.0 * c.brake_a_mps2))


def v_fence_mps(d_eff_m: float, c: FenceConstants) -> float:
    """11 S9A.6 (4): the speed from which d_eff is exactly the stopping
    distance -- the inverse of d_stop_m, closed form. d_eff <= 0 -> 0 (the
    hard clip takes over). Identity: v_fence(d_stop(v)) == v.
    mutant: drop the '- k t' term -> the robot is allowed ~1 m/s at the
    effective boundary -> test_v_fence_is_the_brake_inverse red."""
    if d_eff_m <= 0.0:
        return 0.0
    kt = c.brake_k * c.t_lat_s
    return (c.brake_a_mps2 / c.brake_k) * (
        math.sqrt(kt * kt + 2.0 * c.brake_k * d_eff_m / c.brake_a_mps2) - kt)


def margin_soft_eff_m(c: FenceConstants, declared_min_m: Optional[float] = None) -> float:
    """11 S9A.6 (3): max(fence.soft_margin_min_m, d_stop(v_profile_max)); a
    set / polygon-level declared minimum (11 S9A.2) can only widen it. This
    is where the soft band STARTS (geo.state = soft, soft_enter event); the
    speed inside it is v_fence_mps, not a separate ramp."""
    floor = c.soft_margin_min_m
    if declared_min_m is not None:
        floor = max(floor, declared_min_m)
    return max(floor, d_stop_m(c.v_profile_max_mps, c))


def inset_for_fix(fix_type: Optional[str], c: FenceConstants) -> Optional[float]:
    """12 S7.4 / 12 S6.6 table (1): the inset by fix quality, None when the
    fix does not admit a fence evaluation (B6). See the module note for the
    pending ruling against the 11 S9A.6 (1) formula."""
    if fix_type not in FIX_WITH_FENCE:
        return None
    return float(c.margin_by_fix[fix_type])


# --- compiled geometry ------------------------------------------------------

@dataclass(frozen=True)
class CompiledPolygon:
    """One constraining polygon in ENU metres. keep_in: the allowed side is
    the interior (allow); False for forbid (allowed side is outside)."""
    poly_id: str
    role: str
    name: str
    hard_enforce: bool
    keep_in: bool
    xy: Tuple[Tuple[float, float], ...]
    soft_margin_min_m: Optional[float]


@dataclass(frozen=True)
class CompiledFence:
    """The active set's constraint polygons, compiled once per rev (12 FS-1:
    compile off the tick, swap atomically) -- the tick only reads."""
    fence_set_id: str
    rev: int
    polygons: Tuple[CompiledPolygon, ...]


def compile_fence(held: HeldFenceSet, frame: LocalFrame) -> CompiledFence:
    """HeldFenceSet (WGS84 vertices) -> CompiledFence about the site frame
    (11 S9A.2: enu_origin is common.geo.enu_origin, every process projects
    with the same origin so ENU agrees to the bit). warning / speed_limit
    polygons are dropped here (module note). A vertex the frame rejects
    (non-finite / out of range) refuses the whole set: a partly compiled
    fence would be a fence with a hole in it."""
    polys: List[CompiledPolygon] = []
    for p in held.polygons:
        if p.role not in CONSTRAINT_ROLES:
            continue
        try:
            xy = tuple(frame.to_xy(lat, lon) for lat, lon in p.vertices)
        except LocalFrameError as exc:
            raise FenceClipError("fence %s polygon %s: %s"
                                 % (held.fence_set_id, p.poly_id, exc)) from exc
        declared = p.soft_margin_min_m if p.soft_margin_min_m is not None \
            else held.soft_margin_min_m
        polys.append(CompiledPolygon(
            poly_id=p.poly_id, role=p.role, name=p.name,
            hard_enforce=p.hard_enforce, keep_in=(p.role == "allow"),
            xy=xy, soft_margin_min_m=declared))
    return CompiledFence(held.fence_set_id, held.rev, tuple(polys))


# --- signed distance (FE-2 analytic edge distance) --------------------------

@dataclass(frozen=True)
class BoundaryHit:
    """d_nom_m: allowed side positive. normal: ENU unit vector pointing the
    way d_nom DECREASES (toward the violation) -- taken from the same nearest
    feature as the distance, so the two cannot disagree (FE-2)."""
    d_nom_m: float
    normal: Tuple[float, float]


def _nearest_on_segment(px: float, py: float, ax: float, ay: float,
                        bx: float, by: float) -> Tuple[float, float]:
    """Closest point of segment AB to P (clamped parameter)."""
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 <= 0.0:
        return ax, ay
    t = ((px - ax) * dx + (py - ay) * dy) / l2
    t = max(0.0, min(1.0, t))
    return ax + t * dx, ay + t * dy


def boundary_hit(x: float, y: float, poly: CompiledPolygon) -> BoundaryHit:
    """Signed distance + outward normal of P against one polygon. The nearest
    feature is found over all edges (a vertex is the clamped end of an edge);
    inside/outside comes from the shared ray-cast; the sign is 'inside ==
    keep_in'. Normal: for d > 0 it points from P to the nearest boundary
    point (moving that way shrinks the margin); for d < 0 it points from the
    boundary point away through P (deeper into the violation). On the
    boundary itself the gradient is undefined, so the edge normal is used,
    oriented by probing which side is the interior.
    mutant: flip the normal for d < 0 -> a breached robot has its RETREAT
    removed and its escape kept -> test_breached_normal_points_further_out
    red."""
    n = len(poly.xy)
    best_d2 = math.inf
    best_q = (x, y)
    best_edge = 0
    for i in range(n):
        ax, ay = poly.xy[i]
        bx, by = poly.xy[(i + 1) % n]
        qx, qy = _nearest_on_segment(x, y, ax, ay, bx, by)
        d2 = (x - qx) * (x - qx) + (y - qy) * (y - qy)
        if d2 < best_d2:
            best_d2, best_q, best_edge = d2, (qx, qy), i
    dist = math.sqrt(best_d2)
    inside = point_in_polygon(x, y, Polygon(points=poly.xy))
    d_nom = dist if inside == poly.keep_in else -dist
    if dist > _ON_BOUNDARY_M:
        ux, uy = (x - best_q[0]) / dist, (y - best_q[1]) / dist
        normal = (-ux, -uy) if d_nom > 0.0 else (ux, uy)
    else:
        ax, ay = poly.xy[best_edge]
        bx, by = poly.xy[(best_edge + 1) % n]
        ex, ey = bx - ax, by - ay
        el = math.hypot(ex, ey) or 1.0
        n1 = (ey / el, -ex / el)
        probe_inside = point_in_polygon(best_q[0] + _PROBE_M * n1[0],
                                        best_q[1] + _PROBE_M * n1[1],
                                        Polygon(points=poly.xy))
        # n1 points to the interior iff the probe is inside; outward (toward
        # the violation) is the side that is NOT the allowed one.
        normal = n1 if probe_inside != poly.keep_in else (-n1[0], -n1[1])
    return BoundaryHit(d_nom, normal)


# --- projection (11 S9A.8) ---------------------------------------------------

def project_halfspaces(vw: Tuple[float, float],
                       constraints: Sequence[Tuple[Tuple[float, float], float]],
                       iters: int) -> Tuple[Tuple[float, float], bool]:
    """Alternating projection of the ENU velocity onto every half-space
    {v . n <= cap}. cap = 0 is 12 S7.2 verbatim (v - max(0, v.n) n); cap > 0
    is the soft band (the approach component limited to v_fence). POCS
    converges inside iters rounds for a convex cone; a residual above
    DEGENERATE_EPS_MPS afterwards means no feasible inward direction (concave
    corner, 11 S9A.8 item 2) -> zero, flagged so the caller raises
    fence_clip_degenerate instead of a silent 'stopped for no reason'.
    mutant: skip the residual check -> a dead-end corner leaks an outward
    velocity -> test_concave_corner_is_degenerate_and_zero red."""
    vx, vy = vw
    if not constraints:
        return (vx, vy), False
    for _ in range(iters):
        for (nx, ny), cap in constraints:
            excess = vx * nx + vy * ny - cap
            if excess > 0.0:
                vx -= excess * nx
                vy -= excess * ny
    residual = max(vx * nx + vy * ny - cap for (nx, ny), cap in constraints)
    if residual > DEGENERATE_EPS_MPS:
        return (0.0, 0.0), True
    return (vx, vy), False


# --- the per-tick evaluation -------------------------------------------------

@dataclass(frozen=True)
class PolyDistance:
    """One polygon's numbers this tick (events + state/fence detail)."""
    poly_id: str
    name: str
    role: str
    hard_enforce: bool
    d_nom_m: float
    d_eff_m: float
    normal: Tuple[float, float]
    hard_now: bool            # a zero-cap constraint was active (now or predicted)


@dataclass(frozen=True)
class FenceEval:
    """The 11 S9A.5 geo / enforcement / allow facts plus the clipped body
    velocity. enforcement != full -> geometry fields are None / unknown and
    (vx, vy) pass through untouched (the gate's own vetoes already hold the
    robot in those cases; here we only refuse to CLAIM a fence is enforced)."""
    enforcement: str
    degrade_reason: str
    state: str
    poly_id: Optional[str]
    role: Optional[str]
    poly_name: str
    d_nom_m: Optional[float]
    inset_m: Optional[float]
    d_eff_m: Optional[float]
    margin_soft_eff_m: Optional[float]
    v_fence_mps: Optional[float]
    outward: bool
    outward_normal: Optional[Tuple[float, float]]
    clipped: bool
    degenerate: bool
    vx: float
    vy: float
    cut_mps: float
    per_poly: Tuple[PolyDistance, ...]
    allow_autonomous: bool
    allow_accept_task: bool
    teleop_max_mps: float

    @property
    def hard_ids(self) -> Tuple[str, ...]:
        return tuple(p.poly_id for p in self.per_poly if p.hard_now)


def _unavailable(enforcement: str, reason: str, vx: float, vy: float,
                 teleop_max: float) -> FenceEval:
    """The three degraded shapes of 11 S9A.5 / S9A.7: nothing is judged,
    autonomous / task admission refused (warn_only '交还给人': teleop stays
    at the degraded cap; disabled: nothing at all)."""
    return FenceEval(
        enforcement=enforcement, degrade_reason=reason, state=STATE_UNKNOWN,
        poly_id=None, role=None, poly_name="", d_nom_m=None, inset_m=None,
        d_eff_m=None, margin_soft_eff_m=None, v_fence_mps=None, outward=False,
        outward_normal=None, clipped=False, degenerate=False, vx=vx, vy=vy,
        cut_mps=0.0, per_poly=(), allow_autonomous=False,
        allow_accept_task=False, teleop_max_mps=teleop_max)


def evaluate(fence: Optional[CompiledFence], c: FenceConstants, *,
             xy: Optional[Tuple[float, float]], yaw_rad: Optional[float],
             vx: float, vy: float, fix_type: Optional[str],
             heading_valid: bool, hi_factor: float = 1.0) -> FenceEval:
    """One tick. (vx, vy) is the GATED body-frame candidate (12 S2.2 step 7
    runs after step 6), hi_factor = h * i the gate applied (11 S9A.6 (5)
    multiplies the fence term by them too). Order of the degradations is
    11 S9A.7: no fence -> disabled; fix not rtk_* (or no position) ->
    warn_only rtk_degraded; no heading -> warn_only heading_lost (the
    body->ENU rotation is undefined without it, 'L3 下必须降级'); else full.
    mutant: evaluate with heading_valid False -> the projection uses a stale
    yaw -> test_no_heading_is_warn_only red."""
    if fence is None or not fence.polygons:
        return _unavailable(ENFORCEMENT_DISABLED, REASON_NO_FENCE, vx, vy, 0.0)
    inset = inset_for_fix(fix_type, c)
    if inset is None or xy is None:
        return _unavailable(ENFORCEMENT_WARN_ONLY, REASON_RTK, vx, vy,
                            c.teleop_cap_degraded_mps)
    if not heading_valid or yaw_rad is None:
        return _unavailable(ENFORCEMENT_WARN_ONLY, REASON_HEADING, vx, vy,
                            c.teleop_cap_degraded_mps)
    hi = max(0.0, min(1.0, float(hi_factor)))
    # body -> ENU: 11 S3.3 heading (east 0, ccw positive).
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    vwx, vwy = vx * cy - vy * sy, vx * sy + vy * cy
    px, py = xy
    # pass 1: the soft caps (and the zero caps of hard polygons already at or
    # beyond their effective boundary).
    hits = [(poly, boundary_hit(px, py, poly)) for poly in fence.polygons]
    constraints: List[Tuple[Tuple[float, float], float]] = []
    hard_now = {}
    for poly, hit in hits:
        d_eff = hit.d_nom_m - inset
        if d_eff > 0.0:
            constraints.append((hit.normal, v_fence_mps(d_eff, c) * hi))
        elif poly.hard_enforce:
            constraints.append((hit.normal, 0.0))
            hard_now[poly.poly_id] = True
        # a soft-only polygon at / past its boundary: 11 S9A.2 hard_enforce
        # false = "只产生软减速与事件,不做硬裁剪" -- no zero cap.
    (c1x, c1y), degenerate = project_halfspaces((vwx, vwy), constraints,
                                                c.projection_iters)
    # pass 2: 12 S7.3 predicted crossing on the soft-capped velocity (module
    # note), hard polygons only, zero cap with the normal at the predicted
    # point (that is where it would cross).
    if not degenerate:
        nx_, ny_ = px + c1x * c.predict_dt_s, py + c1y * c.predict_dt_s
        added = False
        for poly, _hit in hits:
            if not poly.hard_enforce or hard_now.get(poly.poly_id):
                continue
            nxt = boundary_hit(nx_, ny_, poly)
            if nxt.d_nom_m - inset <= 0.0:
                constraints.append((nxt.normal, 0.0))
                hard_now[poly.poly_id] = True
                added = True
        if added:
            (c1x, c1y), degenerate = project_halfspaces((vwx, vwy), constraints,
                                                        c.projection_iters)
    per = tuple(PolyDistance(
        poly_id=poly.poly_id, name=poly.name, role=poly.role,
        hard_enforce=poly.hard_enforce, d_nom_m=hit.d_nom_m,
        d_eff_m=hit.d_nom_m - inset, normal=hit.normal,
        hard_now=bool(hard_now.get(poly.poly_id))) for poly, hit in hits)
    nearest = min(per, key=lambda p: p.d_eff_m)
    declared = next(p.soft_margin_min_m for p in fence.polygons
                    if p.poly_id == nearest.poly_id)
    msoft = margin_soft_eff_m(c, declared)
    vf = v_fence_mps(nearest.d_eff_m, c)
    outward = (vwx * nearest.normal[0] + vwy * nearest.normal[1]) > 0.0
    clipped = degenerate or abs(c1x - vwx) > 1e-9 or abs(c1y - vwy) > 1e-9
    # ENU -> body (R(yaw) transposed).
    bvx, bvy = c1x * cy + c1y * sy, -c1x * sy + c1y * cy
    if nearest.d_nom_m < 0.0:
        state = STATE_OUTSIDE
    elif nearest.d_eff_m < msoft:
        state = STATE_SOFT
    else:
        state = STATE_INSIDE
    return FenceEval(
        enforcement=ENFORCEMENT_FULL, degrade_reason=REASON_NONE, state=state,
        poly_id=nearest.poly_id, role=nearest.role, poly_name=nearest.name,
        d_nom_m=nearest.d_nom_m, inset_m=inset, d_eff_m=nearest.d_eff_m,
        margin_soft_eff_m=msoft, v_fence_mps=vf, outward=outward,
        outward_normal=nearest.normal, clipped=clipped, degenerate=degenerate,
        vx=bvx, vy=bvy,
        cut_mps=max(0.0, math.hypot(vwx, vwy) - math.hypot(c1x, c1y)),
        per_poly=per, allow_autonomous=True, allow_accept_task=True,
        teleop_max_mps=c.v_profile_max_mps)


__all__ = ["FenceClipError", "FenceConstants", "CompiledPolygon", "CompiledFence",
           "BoundaryHit", "PolyDistance", "FenceEval", "compile_fence",
           "boundary_hit", "project_halfspaces", "evaluate", "d_stop_m",
           "v_fence_mps", "margin_soft_eff_m", "inset_for_fix",
           "FIX_WITH_FENCE", "CONSTRAINT_ROLES", "DEGENERATE_EPS_MPS",
           "ENFORCEMENT_FULL", "ENFORCEMENT_WARN_ONLY", "ENFORCEMENT_DISABLED",
           "REASON_NONE", "REASON_RTK", "REASON_HEADING", "REASON_NO_FENCE",
           "STATE_INSIDE", "STATE_SOFT", "STATE_OUTSIDE", "STATE_UNKNOWN"]
