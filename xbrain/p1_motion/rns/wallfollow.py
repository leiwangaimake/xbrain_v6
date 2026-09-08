"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: wallfollow.py
Brief: Wall-follow state machine (P5 -- 20 S7 / S7A)

Description:
Bug-style wall following (20 S7), the mechanism that gives RNS its convergence
guarantee (RNS-T-1). Structurally depends on the memory grid (RNS-I-7): the wall
being hugged is often behind the robot's view, so it lives in memory.

The four pieces this file owns:
  - ENTRY guard (S7.2, v1.13): enter only when candidates are all gated out AND a
    real BLOCKED boundary exists (perception or memory). No candidates AND no
    boundary -> do NOT enter (executing wall-follow on a non-existent wall is
    undefined); the caller escalates to the watchdog branch-two.
  - EXIT (S7.3): three conditions -- back on the line, ARC-LENGTH progress past
    the hit point (2', the key to convergence L3), and the goal direction is now
    open. s is the monotone arc-length progress s*, recorded in the LOCALIZATION
    frame (S7.3).
  - D3 grazing relaxation (S7A.1): if a full loop back to H would fire failure,
    but a boundary crossing with 0 < s_gain <= delta_s was recorded, leave at the
    max-gain crossing instead of failing -- the grazing false-unreachable guard.
  - FAILURE (S7.6): three criteria -- back to H after a full loop (checking D3
    first), no-progress over no_progress_m, and a max_follow_m backstop.

2' (arc-length, not Euclidean) is what makes L3 hold: Euclidean distance to the
goal is non-monotone on a curved path, so a leave point can be legitimate yet
farther in straight-line distance. See 20 S7.3 v1.8 for the ring-path counter-
example this replaced.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from .types import NavFailReason, NavFailure


class Side(str, Enum):
    LEFT = "left"
    RIGHT = "right"


@dataclass
class WallFollowState:
    """The wall-follow run's memory. s_hit is the arc-length progress at entry;
    best_leave_s is the supremum of leave-eligible progress seen (for failure
    criterion 2); crossings holds D3's grazing exit points."""
    side: Side
    s_hit: float
    hit_point: tuple                     # H in the localization frame
    followed_m: float = 0.0              # total wall-follow distance walked
    best_leave_s: float = float("-inf")  # sup of progress that could satisfy 2'
    best_leave_m_at: float = 0.0         # followed_m when best_leave_s was set
    # D3 record: crossings with 0 < s_gain <= delta_s (grazing), (s_gain, s_at).
    grazing: List[tuple] = field(default_factory=list)


def can_enter(candidates_all_gated: bool, boundary_exists: bool) -> bool:
    """ENTRY guard (S7.2, v1.13). Enter iff all candidates are gated out AND a
    BLOCKED boundary exists. No boundary -> do NOT enter (undefined on a
    non-existent wall); caller goes to watchdog branch-two. mutant: drop the
    boundary check -> wall-follow runs with no wall -> the fence-truncation
    stall bug (v1.13)."""
    return candidates_all_gated and boundary_exists


def can_leave(
    deviation_m: float, s_now: float, s_hit: float,
    goal_dir_open: bool, e_ok_m: float, leave_progress_m: float,
) -> bool:
    """EXIT three conditions (S7.3). ALL must hold:
      1. back on the line: |e| < e_ok_m
      2'. arc-length progress past the hit point: s_now >= s_hit + leave_progress
      3. goal direction open: a candidate passes the gates that way
    2' is arc-length (A-CVG-4): the Euclidean version fails on curved paths."""
    if deviation_m >= e_ok_m:
        return False
    if s_now < s_hit + leave_progress_m:   # 2' -- the convergence-critical one
        return False
    return goal_dir_open


def record_crossing(state: WallFollowState, s_gain: float, s_at: float) -> None:
    """Record a boundary crossing for D3 (S7A.1). A crossing with 0 < s_gain <=
    delta_s is grazing (too small to satisfy 2' but real); it goes in the D3
    record so a full loop can relax-leave there instead of false-failing."""
    if 0.0 < s_gain:
        # best_leave tracks the SUP of any leave-eligible progress (criterion 2).
        candidate_s = state.s_hit + s_gain
        if candidate_s > state.best_leave_s:
            state.best_leave_s = candidate_s
            state.best_leave_m_at = state.followed_m
        state.grazing.append((s_gain, s_at))


def d3_relax_leave(state: WallFollowState, leave_progress_m: float) -> Optional[float]:
    """D3 grazing relaxation (S7A.1): before firing failure criterion 1, if the
    record has a crossing with 0 < s_gain <= delta_s, leave at the MAX-gain
    crossing (免除 2' once). Returns the s_at of that crossing, or None if the
    record has no grazing crossing (then failure stands). mutant: skip D3 -> a
    grazing path circles and FALSE-reports unreachable (A-CVG-6)."""
    grazing = [(g, s_at) for (g, s_at) in state.grazing
               if 0.0 < g <= leave_progress_m]
    if not grazing:
        return None
    _, s_at = max(grazing, key=lambda gs: gs[0])
    return s_at


def check_failure(
    state: WallFollowState, dist_to_H_m: float,
    min_loop_m: float, no_progress_m: float, max_follow_m: float,
    leave_progress_m: float,
) -> Optional[NavFailure]:
    """FAILURE three criteria (S7.6), OR. Criterion 1 checks D3 FIRST (S7A.1):
    a full loop back to H fails only if the D3 record has no grazing exit.

      1. dist(H) < eps AND followed > min_loop -> closed loop, unless D3 relaxes.
      2. best_leave_s unchanged for no_progress_m of travel -> no progress.
      3. followed > max_follow_m -> backstop (sensor/control anomaly).
    """
    # criterion 3: backstop first (cheap, always valid).
    if state.followed_m > max_follow_m:
        return NavFailure(NavFailReason.WALL_BUDGET,
                          detail={"followed_m": state.followed_m})
    # criterion 1: back to H after a full loop -- but check D3 grazing first.
    eps = 0.5  # H-proximity epsilon (m); a fixed small radius, not a tuned param
    if dist_to_H_m < eps and state.followed_m > min_loop_m:
        if d3_relax_leave(state, leave_progress_m) is not None:
            return None    # D3 relaxes: a grazing exit exists, do not fail
        return NavFailure(NavFailReason.WALL_CLOSED_LOOP,
                          detail={"H": state.hit_point,
                                  "loop_m": state.followed_m})
    # criterion 2: no improvement in leave-eligible progress over no_progress_m.
    if (state.followed_m - state.best_leave_m_at) > no_progress_m and \
            state.best_leave_s == float("-inf"):
        return NavFailure(NavFailReason.WALL_NO_PROGRESS,
                          detail={"best_leave_s": None,
                                  "followed_m": state.followed_m})
    return None


def select_side(
    left_sees_end: bool, right_sees_end: bool,
    left_cost: float, right_cost: float,
    goal_side: Side,
    left_exits_fence: bool, right_exits_fence: bool,
) -> Optional[Side]:
    """Side selection (S7.2), by priority. A side that would leave the allow
    fence is excluded (rule 4). Returns None if both sides are excluded by fence
    (caller fails). Rules: (1) the side that sees the wall's end; (2) both see ->
    smaller detour cost; (3) neither sees -> the goal's side (heuristic)."""
    left_ok = not left_exits_fence
    right_ok = not right_exits_fence
    if not left_ok and not right_ok:
        return None
    if not left_ok:
        return Side.RIGHT
    if not right_ok:
        return Side.LEFT
    # rule 1: only one side sees the end
    if left_sees_end and not right_sees_end:
        return Side.LEFT
    if right_sees_end and not left_sees_end:
        return Side.RIGHT
    # rule 2: both see the end -> smaller detour cost
    if left_sees_end and right_sees_end:
        return Side.LEFT if left_cost <= right_cost else Side.RIGHT
    # rule 3: neither sees -> goal's side
    return goal_side


# ── keep-distance PD + corners (P5 -- 20 S7.7) ────────────────────────────────
def keep_distance_omega(
    d_side_m: float, d_wall_m: float, d_side_rate: float,
    side: Side, kp_wall: float, kd_wall: float, wz_max: float,
) -> float:
    """Keep-distance PD (S7.7-1, A-WF-2/3). d_side = min d_block over the wall-
    side sector (perception U memory). e_w = d_side - d_wall; omega = clamp(sigma
    * (kp*e_w + kd*e_w_rate), +/- wz_max). sigma is the side sign so the
    correction turns toward/away correctly. Too close -> turn away; too far ->
    turn toward (else it cuts into unknown at a convex corner)."""
    sigma = 1.0 if side == Side.LEFT else -1.0
    e_w = d_side_m - d_wall_m
    raw = sigma * (kp_wall * e_w + kd_wall * d_side_rate)
    return wz_max if raw > wz_max else (-wz_max if raw < -wz_max else raw)


def wall_vanished(side_sector_has_blocked: bool) -> bool:
    """Convex corner (S7.7-2): the wall dropped out of the side sector (no BLOCKED
    there, perception U memory). Turning toward the goal now would cut the corner;
    the caller walks corner_overshoot_m along the wall's extension first."""
    return not side_sector_has_blocked


def inner_corner_stop(front_min_d_free: float, front_stop_m: float) -> bool:
    """Concave (inner) corner (S7.7-3, A-WF-7): the wall's turn blocks the front.
    front d_free < front_stop_m -> stop and rotate away from the wall (via the
    S6A rotation permit; the return side is already observed so RNS-I-6 holds).
    mutant: drop this -> the keep-distance law drives the robot into the inner
    corner and stalls -> reddens."""
    return front_min_d_free < front_stop_m
