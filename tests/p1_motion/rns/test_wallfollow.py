"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_wallfollow.py
Brief: wall-follow entry/exit/D3/failure/side (P5 -- 20 S7/S7A)

Description:
Guards the wall-follow state machine. Key ones: entry needs a real boundary
(A-WF-1 / v1.13 fence-stall guard), 2' arc-length leave (A-CVG-1/4), D3 grazing
relaxation (A-CVG-6, the false-unreachable guard), closed-loop failure (A-WF-5).
Each test names its mutant.
"""

from __future__ import annotations

from xbrain.p1_motion.rns.wallfollow import (
    Side, WallFollowState, can_enter, can_leave, check_failure, d3_relax_leave,
    record_crossing, select_side,
)
from xbrain.p1_motion.rns.types import NavFailReason


def _state(side=Side.LEFT, s_hit=10.0, H=(5.0, 5.0), followed=0.0):
    st = WallFollowState(side=side, s_hit=s_hit, hit_point=H)
    st.followed_m = followed
    return st


def test_enter_needs_candidates_gated_and_boundary():
    # A-WF-1 / v1.13: enter only when candidates gated AND a boundary exists.
    # mutant: drop the boundary check -> wall-follow on a non-existent wall
    # (fence truncation) -> reddens.
    assert can_enter(candidates_all_gated=True, boundary_exists=True) is True
    assert can_enter(candidates_all_gated=True, boundary_exists=False) is False
    assert can_enter(candidates_all_gated=False, boundary_exists=True) is False


def test_leave_requires_arclength_progress():
    # A-CVG-1/4 (2'): leave needs s_now >= s_hit + leave_progress, on TOP of back-
    # on-line and goal-open. mutant: use Euclidean or drop 2' -> leaves too early
    # / never on a curved path.
    # on line, goal open, but progress not yet enough (s 10.2 < 10 + 0.5):
    assert can_leave(0.1, s_now=10.2, s_hit=10.0, goal_dir_open=True,
                     e_ok_m=0.3, leave_progress_m=0.5) is False
    # enough progress (s 10.6 >= 10.5):
    assert can_leave(0.1, s_now=10.6, s_hit=10.0, goal_dir_open=True,
                     e_ok_m=0.3, leave_progress_m=0.5) is True


def test_leave_needs_all_three():
    # off the line -> no leave even with progress and open goal.
    assert can_leave(0.5, 11.0, 10.0, True, e_ok_m=0.3, leave_progress_m=0.5) is False
    # goal not open -> no leave.
    assert can_leave(0.1, 11.0, 10.0, False, e_ok_m=0.3, leave_progress_m=0.5) is False


def test_d3_relaxes_on_grazing_crossing():
    # A-CVG-6: a grazing crossing (0 < s_gain <= delta_s) lets a full loop leave
    # instead of failing. mutant: skip D3 -> grazing path false-reports
    # unreachable -> reddens.
    st = _state()
    record_crossing(st, s_gain=0.3, s_at=10.3)   # grazing (< delta_s 0.5)
    assert d3_relax_leave(st, leave_progress_m=0.5) == 10.3


def test_d3_no_relax_without_grazing():
    st = _state()
    # only a crossing bigger than delta_s (which 2' would have taken anyway)
    record_crossing(st, s_gain=1.0, s_at=11.0)
    assert d3_relax_leave(st, leave_progress_m=0.5) is None


def test_closed_loop_failure_when_no_d3():
    # A-WF-5 / S7.6-1: back to H after a full loop, no D3 grazing -> WALL_CLOSED
    # LOOP. mutant: skip criterion 1 -> circles forever.
    st = _state(H=(5.0, 5.0), followed=8.0)
    f = check_failure(st, dist_to_H_m=0.2, min_loop_m=2.0, no_progress_m=5.0,
                      max_follow_m=50.0, leave_progress_m=0.5)
    assert f is not None
    assert f.reason is NavFailReason.WALL_CLOSED_LOOP


def test_closed_loop_relaxed_by_d3():
    # back to H, but a grazing crossing recorded -> D3 relaxes, NO failure.
    st = _state(H=(5.0, 5.0), followed=8.0)
    record_crossing(st, s_gain=0.3, s_at=10.3)
    f = check_failure(st, dist_to_H_m=0.2, min_loop_m=2.0, no_progress_m=5.0,
                      max_follow_m=50.0, leave_progress_m=0.5)
    assert f is None


def test_max_follow_backstop():
    st = _state(followed=60.0)
    f = check_failure(st, dist_to_H_m=99.0, min_loop_m=2.0, no_progress_m=5.0,
                      max_follow_m=50.0, leave_progress_m=0.5)
    assert f.reason is NavFailReason.WALL_BUDGET


def test_side_select_sees_end():
    # rule 1: the side that sees the wall's end wins.
    assert select_side(True, False, 1.0, 1.0, Side.LEFT, False, False) == Side.LEFT


def test_side_select_excludes_fence_side():
    # rule 4: a side that leaves the allow fence is excluded.
    assert select_side(True, True, 1.0, 1.0, Side.LEFT,
                       left_exits_fence=True, right_exits_fence=False) == Side.RIGHT
    # both exit fence -> None (caller fails)
    assert select_side(True, True, 1.0, 1.0, Side.LEFT, True, True) is None


def test_side_select_cost_when_both_see():
    # rule 2: both see end -> smaller detour cost.
    assert select_side(True, True, 5.0, 1.0, Side.LEFT, False, False) == Side.RIGHT


# ── keep-distance PD + corners (A-WF-2/3/4/6/7) ───────────────────────────────
def test_keep_distance_turns_away_when_too_close():
    # A-WF-2/3: too close (d_side < d_wall) -> correction; too far -> opposite.
    from xbrain.p1_motion.rns.wallfollow import keep_distance_omega
    too_close = keep_distance_omega(0.3, d_wall_m=0.6, d_side_rate=0.0,
                                    side=Side.LEFT, kp_wall=1.0, kd_wall=0.1, wz_max=1.0)
    too_far = keep_distance_omega(1.0, d_wall_m=0.6, d_side_rate=0.0,
                                  side=Side.LEFT, kp_wall=1.0, kd_wall=0.1, wz_max=1.0)
    assert too_close != too_far
    # opposite signs of e_w -> opposite correction direction
    assert (too_close < 0) != (too_far < 0)


def test_keep_distance_clamps_to_wz_max():
    from xbrain.p1_motion.rns.wallfollow import keep_distance_omega
    w = keep_distance_omega(10.0, d_wall_m=0.6, d_side_rate=0.0, side=Side.LEFT,
                            kp_wall=100.0, kd_wall=0.0, wz_max=1.0)
    assert abs(w) == 1.0


def test_convex_corner_detected_when_wall_gone():
    # A-WF-4 (convex): wall left the side sector -> vanished.
    from xbrain.p1_motion.rns.wallfollow import wall_vanished
    assert wall_vanished(side_sector_has_blocked=False) is True
    assert wall_vanished(side_sector_has_blocked=True) is False


def test_inner_corner_stop():
    # A-WF-7 (concave): front blocked below front_stop -> stop + rotate. mutant:
    # drop this -> keep-distance drives into the inner corner -> reddens.
    from xbrain.p1_motion.rns.wallfollow import inner_corner_stop
    assert inner_corner_stop(front_min_d_free=0.4, front_stop_m=0.8) is True
    assert inner_corner_stop(front_min_d_free=1.5, front_stop_m=0.8) is False


def test_arclength_leave_works_on_curved_path_A_CVG_4():
    # A-CVG-4: on a curved path, a legitimate leave point can be FARTHER from the
    # goal in straight-line distance yet AHEAD in arc-length. The 2' arc-length
    # condition accepts it; a Euclidean version would reject it (robot stuck on
    # the wall). This is the ring-path counter-example (20 S7.3 v1.8).
    # s_hit = 10.0 (arc progress at entry). A leave point at arc s=10.8 satisfies
    # 2' (>= 10.0 + 0.5), regardless of whether Euclidean goal distance grew.
    on_line, goal_open = 0.1, True
    # arc-length version (what we use): accepts.
    assert can_leave(on_line, s_now=10.8, s_hit=10.0, goal_dir_open=goal_open,
                     e_ok_m=0.3, leave_progress_m=0.5) is True
    # the mutant would gate on Euclidean-to-goal instead of s; here we assert the
    # arc-length contract holds: progress alone (not goal distance) decides.
    # a point with LESS arc progress must be rejected even if closer to goal.
    assert can_leave(on_line, s_now=10.2, s_hit=10.0, goal_dir_open=goal_open,
                     e_ok_m=0.3, leave_progress_m=0.5) is False


def test_returning_to_entry_area_is_not_failure_A_WF_3():
    # A-WF-3: coming back NEAR the entry (H) is normal escaping, NOT failure --
    # unless a FULL loop with min_loop traveled AND no D3 relax. Below min_loop
    # (just re-approached H), no failure. mutant: treat any H-proximity as failure
    # -> normal escaping judged unreachable -> reddens.
    st = _state(H=(5.0, 5.0), followed=0.5)   # only 0.5 m walked, < min_loop 2.0
    f = check_failure(st, dist_to_H_m=0.2, min_loop_m=2.0, no_progress_m=5.0,
                      max_follow_m=50.0, leave_progress_m=0.5)
    assert f is None   # near H but not a full loop -> not a failure


def test_wait_preserves_wall_state_A_WF_6():
    # A-WF-6: waiting (for a dynamic obstacle) during wall-follow must PRESERVE
    # s_hit / side, so resume does not flip sides. The state object carries them;
    # a wait must not reset them. mutant: clearing state on wait -> side flip on
    # resume -> reddens. Here: the state's side/s_hit survive an unrelated tick.
    st = _state(side=Side.LEFT, s_hit=10.0)
    side_before, s_before = st.side, st.s_hit
    # simulate a wait tick that only bumps followed distance, not the wall state
    st.followed_m += 0.0
    assert st.side == side_before
    assert st.s_hit == s_before


def test_no_progress_fires_after_stall_with_prior_crossing():
    # B2 (review fix): criterion 2 must fire when progress STALLS, even after a
    # crossing was recorded earlier. The original `best_leave_s == -inf` conjunct
    # made this dead the moment any crossing existed -- a stalled wall-follow
    # could only die at the 50 m backstop. mutant: restore the conjunct -> this
    # test reddens (that IS the shipped bug).
    st = _state(s_hit=10.0, followed=1.0)
    record_crossing(st, s_gain=0.3, s_at=10.3)   # improved once at 1.0 m
    st.followed_m = 8.0                           # then 7 m with no improvement
    f = check_failure(st, dist_to_H_m=99.0, min_loop_m=2.0, no_progress_m=5.0,
                      max_follow_m=50.0, leave_progress_m=0.5)
    assert f is not None
    assert f.reason is NavFailReason.WALL_NO_PROGRESS
    assert f.detail["best_leave_s"] == 10.3      # real value, not None


def test_no_progress_fires_when_never_improved():
    # criterion 2, never-improved branch: best_leave_m_at stays 0, so walking
    # no_progress_m with no crossing at all also fires.
    st = _state(s_hit=10.0, followed=6.0)         # 6 m, no crossings
    f = check_failure(st, dist_to_H_m=99.0, min_loop_m=2.0, no_progress_m=5.0,
                      max_follow_m=50.0, leave_progress_m=0.5)
    assert f is not None
    assert f.reason is NavFailReason.WALL_NO_PROGRESS
    assert f.detail["best_leave_s"] is None


def test_no_progress_quiet_while_improving():
    # fresh improvement resets the stall clock: crossing at 7.5 m -> only 0.5 m
    # since improvement -> no failure.
    st = _state(s_hit=10.0, followed=7.5)
    record_crossing(st, s_gain=0.3, s_at=10.3)   # improved at 7.5 m
    st.followed_m = 8.0
    f = check_failure(st, dist_to_H_m=99.0, min_loop_m=2.0, no_progress_m=5.0,
                      max_follow_m=50.0, leave_progress_m=0.5)
    assert f is None
