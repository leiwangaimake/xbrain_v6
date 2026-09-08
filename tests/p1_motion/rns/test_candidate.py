"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_candidate.py
Brief: subgoal/gates/cost/hysteresis (P4 -- 20 S6)

Description:
Guards candidate.py's five steps. Key kill-pairs: A-GAP-2/3 (clearance is a gate
not a soft cost; width saturates), A-HYS-1/2 (change needs N ticks, but a gate
drop is immediate). Each test names its mutant (CLAUDE.md 3.3).
"""

from __future__ import annotations

import math

import pytest

from xbrain.p1_motion.rns.candidate import (
    Candidate, CandidateSelector, candidate_cost, clear_extrapolation,
    clearance_penalty, detour_subgoal, Edge, find_edges, passes_hard_gates,
    thread_subgoal,
)


def _cand(clearance=1.0, dyn=False, unobs=False, fence=False, s=(3.0, 0.0)):
    return Candidate(subgoal=s, clearance_m=clearance, dynamic_blocked=dyn,
                     in_unobserved=unobs, fence_hazard=fence)


def test_find_edges_detects_jump():
    # a profile: clear... obstacle at 1.5 ... clear. Two edges (into and out of
    # the obstacle). edge_jump 1.0 m.
    d_block = [None, None, 1.5, 1.5, None, None]
    edges = find_edges(d_block, angle_min=-0.4, angle_step=0.1, edge_jump_m=1.0)
    assert len(edges) == 2
    assert edges[0].d_near_m == 1.5


def test_detour_subgoal_extrapolates_off_edge():
    # A-GAP-1: S must be `clear` off the tangent, not AT it. With clear=0 the
    # subgoal is the tangent itself; with clear>0 it moves off. mutant: drop the
    # extrapolation (return the tangent) -> grazes corner -> reddens.
    e = Edge(theta_rad=0.0, d_near_m=2.0, d_far_m=8.0)
    at_edge = detour_subgoal(e, clear_m=0.0)
    off_edge = detour_subgoal(e, clear_m=0.6)
    assert at_edge == pytest.approx((2.0, 0.0))
    assert off_edge != at_edge
    assert math.hypot(off_edge[0] - 2.0, off_edge[1] - 0.0) == pytest.approx(0.6)


def test_thread_subgoal_extrapolates_past_midpoint():
    # A-GAP-1 (thread): S is past the gap midpoint, not AT it (else arrival =
    # stopped in the gap).
    e1, e2 = (2.0, -0.6), (2.0, 0.6)   # gap centered at (2.0, 0.0)
    s = thread_subgoal(e1, e2, robot=(0.0, 0.0), clear_m=0.5)
    assert s[0] > 2.0   # extrapolated forward past the midpoint


def test_clearance_is_a_hard_gate():
    # A-GAP-2: clearance < r_eff+margin EXCLUDES, regardless of other merits.
    # mutant: make clearance a soft penalty (always pass gate) -> a too-narrow
    # gap survives -> reddens.
    narrow = _cand(clearance=0.4)   # < r_eff 0.48 + margin 0.1 = 0.58
    wide = _cand(clearance=1.0)
    assert passes_hard_gates(narrow, r_eff_m=0.48, margin_m=0.1) is False
    assert passes_hard_gates(wide, r_eff_m=0.48, margin_m=0.1) is True


def test_dynamic_occupied_candidate_excluded():
    # A-GAP-6: a candidate whose corridor a dynamic footprint crosses is excluded.
    # mutant: drop the gate -> robot heads for a gap a person occupies -> red.
    assert passes_hard_gates(_cand(dyn=True), r_eff_m=0.48, margin_m=0.1) is False


def test_unobserved_and_fence_candidates_excluded():
    assert passes_hard_gates(_cand(unobs=True), r_eff_m=0.48, margin_m=0.1) is False
    assert passes_hard_gates(_cand(fence=True), r_eff_m=0.48, margin_m=0.1) is False


def test_width_saturates_not_linear():
    # A-GAP-3: past the gate, extra width saturates. penalty at rho=2 and rho=5
    # (both past saturate=3) is the same (0), NOT linearly better at rho=5.
    # mutant: linear penalty -> rho=5 scores strictly better -> a wide side gap
    # beats an adequate forward gap -> reddens.
    base = 0.58   # r_eff+margin
    at_sat = clearance_penalty(base * 3.0, 0.48, 0.1, gate_saturate_ratio=3.0)
    past_sat = clearance_penalty(base * 5.0, 0.48, 0.1, gate_saturate_ratio=3.0)
    assert at_sat == 0.0
    assert past_sat == 0.0    # saturated, no extra credit
    mid = clearance_penalty(base * 2.0, 0.48, 0.1, gate_saturate_ratio=3.0)
    assert 0.0 < mid < 1.0


def test_cost_angle_reference_is_R():
    # A-GAP-4: angle is measured to R, not the raw polyline. Two subgoals equal
    # in clearance/length but different angle-to-R get different cost.
    kw = dict(r_eff_m=0.48, margin_m=0.1, gate_saturate_ratio=3.0,
              w_clr=1.0, w_ang=10.0, w_len=0.0, w_unk=0.0, w_hys=0.0)
    r = (5.0, 0.0)   # lookahead straight ahead
    aligned = candidate_cost(_cand(s=(3.0, 0.0)), (0.0, 0.0), r, 0.0, False, **kw)
    off = candidate_cost(_cand(s=(3.0, 3.0)), (0.0, 0.0), r, 0.0, False, **kw)
    assert off > aligned   # larger angle to R -> higher cost


def test_cost_includes_s_to_r_length():
    # A-GAP-5: the |S->R| term. Two candidates equal in clearance/angle but one
    # farther from R costs more. Without this term wall ends tie.
    kw = dict(r_eff_m=0.48, margin_m=0.1, gate_saturate_ratio=3.0,
              w_clr=0.0, w_ang=0.0, w_len=1.0, w_unk=0.0, w_hys=0.0)
    r = (10.0, 0.0)
    near_r = candidate_cost(_cand(s=(8.0, 0.0)), (0.0, 0.0), r, 0.0, False, **kw)
    far_r = candidate_cost(_cand(s=(8.0, 5.0)), (0.0, 0.0), r, 0.0, False, **kw)
    assert far_r > near_r


def test_hysteresis_holds_until_streak():
    # A-HYS-1: switch only after side_hold_ticks consecutive wins.
    sel = CandidateSelector(side_hold_ticks=3)
    a, b = _cand(s=(3.0, 0.0)), _cand(s=(3.0, 1.0))
    assert sel.select(True, a, best_is_current=False) is a   # nothing held -> take a
    # b becomes best but must win 3 in a row:
    assert sel.select(True, b, best_is_current=False) is a   # streak 1 -> hold a
    assert sel.select(True, b, best_is_current=False) is a   # streak 2
    assert sel.select(True, b, best_is_current=False) is b   # streak 3 -> switch


def test_hysteresis_does_not_cross_gate():
    # A-HYS-2: if the held candidate fails a gate, drop it IMMEDIATELY (no streak).
    # mutant: apply hysteresis before the gate -> follow a candidate a gate just
    # excluded -> reddens.
    sel = CandidateSelector(side_hold_ticks=3)
    a, b = _cand(s=(3.0, 0.0)), _cand(s=(3.0, 1.0))
    sel.select(True, a, best_is_current=False)               # hold a
    # a fails its gate this tick -> drop immediately, take b (no 3-tick wait)
    out = sel.select(current_feasible=False, best=b, best_is_current=False)
    assert out is b
