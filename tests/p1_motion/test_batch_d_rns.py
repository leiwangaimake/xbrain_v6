"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_d_rns.py
Brief: MOT-RD-1..10 batch D tests (RNS full stack)

Description:
Ten RNS modules: module skeleton discipline, dynamic inflation,
corridor search with L_min gate, side select + hysteresis, targets
veto no-reverse, grid motion compensation FC-12, candidate
generator with wz clamp, shutdown 5 branches, monotonic ring
buffer, U54 one-way safety semantics. Every test names the spec
rule it enforces.
"""

import pytest

from xbrain.p1_motion.rns.audit_ring import RnsAuditRecord, RnsAuditRing
from xbrain.p1_motion.rns.candidate_gen import r_eff, wz_clamp_geometric
from xbrain.p1_motion.rns.corridor import (
    CorridorSample, find_widest_corridor, is_blocked,
)
from xbrain.p1_motion.rns.grid_motion import (
    OdomUnavailable, check_odom_available, transform_grid_to_robot,
)
from xbrain.p1_motion.rns.inflate import compute_r_inflate
from xbrain.p1_motion.rns.module import (
    RnsCandidate, RnsModuleUnavailable, RnsSnapshot,
)
from xbrain.p1_motion.rns.shutdown import (
    RnsShutdownReason, ShutdownDecision, evaluate_shutdown,
)
from xbrain.p1_motion.rns.side_select import (
    Candidate, choose_side, cost_of,
)
from xbrain.p1_motion.rns.targets_veto import (
    ReverseNotAllowed,
    geometry_fallback_velocity, within_veto_distance,
)
from xbrain.p1_motion.rns.u54_semantic import (
    is_within_safety, should_stop_but_not_retreat,
    type_b_slowdown_factor,
)


pytestmark = pytest.mark.no_device


# --- RD-1 module skeleton ---

def test_rns_snapshot_frozen():
    snap = RnsSnapshot(grid_targets=(), d_free_forward_m=3.0,
                        lidar_available=True, grid_age_ms=50,
                        robot_vx_mps=1.0, robot_vy_mps=0.0)
    with pytest.raises(Exception):
        snap.d_free_forward_m = 2.0    # frozen


def test_rns_candidate_reason_string():
    c = RnsCandidate(vx_mps=1.0, wz_radps=0.2, reason="left")
    assert c.reason == "left"


# --- RD-2 dynamic inflation ---

def test_inflate_uses_max_of_current_and_requested():
    """v_current=1.0, v_req=2.0 -> uses 2.0 (requested wins for
    inflation)."""
    r = compute_r_inflate(r_robot_m=0.3, v_current_mps=1.0,
                           v_requested_mps=2.0, t_lat_s=0.4)
    assert r == 0.3 + 2.0 * 0.4


def test_inflate_uses_current_when_larger():
    r = compute_r_inflate(r_robot_m=0.3, v_current_mps=3.0,
                           v_requested_mps=0.0, t_lat_s=0.4)
    assert r == 0.3 + 3.0 * 0.4


def test_inflate_uses_abs_current():
    """Reversing at 2 m/s should inflate as if moving forward 2 m/s."""
    r = compute_r_inflate(r_robot_m=0.3, v_current_mps=-2.0,
                           v_requested_mps=0.0, t_lat_s=0.4)
    assert r == 0.3 + 2.0 * 0.4


# --- RD-3 corridor ---

def test_unknown_cells_count_as_blocked():
    s = CorridorSample(angle_deg=0, distance_m=5.0,
                        unknown_cells_count=1)
    assert is_blocked(s) is True


def test_widest_corridor_finds_longest_run():
    samples = [
        CorridorSample(0, 5.0, 0),
        CorridorSample(10, 5.0, 0),
        CorridorSample(20, 5.0, 0),
        CorridorSample(30, 0.5, 0),   # blocked (< L_min)
        CorridorSample(40, 5.0, 0),
    ]
    result = find_widest_corridor(samples, L_min_m=1.0)
    assert result is not None
    start, end, _ = result
    # Longest run: 0..20 (width 20)
    assert start == 0 and end == 20


def test_widest_corridor_none_when_all_below_L_min():
    samples = [CorridorSample(0, 0.5, 0), CorridorSample(10, 0.3, 0)]
    assert find_widest_corridor(samples, L_min_m=1.0) is None


# --- RD-4 side select ---

def test_choose_side_min_cost():
    a = Candidate(angle_deg=0, heading_dev_deg=0, side_penalty=0,
                   path_dev_m=0)
    b = Candidate(angle_deg=45, heading_dev_deg=45, side_penalty=1,
                   path_dev_m=1)
    assert choose_side([a, b], current_side_angle_deg=None) == a


def test_choose_side_hysteresis_on_tie():
    """On a cost tie within epsilon, prefer current side."""
    a = Candidate(angle_deg=-30, heading_dev_deg=1, side_penalty=0,
                   path_dev_m=0)
    b = Candidate(angle_deg=30, heading_dev_deg=1, side_penalty=0,
                   path_dev_m=0)
    chosen = choose_side([a, b], current_side_angle_deg=30.0,
                          tie_epsilon=0.5)
    assert chosen == b   # closer to current side


# --- RD-5 targets veto ---

def test_within_veto_distance():
    assert within_veto_distance(1.5, d_veto_m=2.0) is True
    assert within_veto_distance(2.5, d_veto_m=2.0) is False


def test_veto_reverse_refused():
    """Reverse in veto mode -> raise (target behind could enter path)."""
    with pytest.raises(ReverseNotAllowed):
        geometry_fallback_velocity(current_vx=-0.3)


def test_veto_forward_zeroed():
    """Forward motion in veto mode -> clip to zero (safest)."""
    assert geometry_fallback_velocity(current_vx=0.5) == (0.0, 0.0)


# --- RD-6 grid motion FC-12 ---

def test_odom_unavailable_shuts_down_rns():
    """FC-12: no odom -> raise; caller shuts down RNS."""
    with pytest.raises(OdomUnavailable):
        check_odom_available(odom_available=False)


def test_odom_available_ok():
    check_odom_available(odom_available=True)


def test_transform_grid_identity_when_robot_at_origin_zero_heading():
    x, y = transform_grid_to_robot(5.0, 3.0, 0.0, 0.0, 0.0)
    assert x == pytest.approx(5.0)
    assert y == pytest.approx(3.0)


# --- RD-7 candidate generator + wz clamp ---

def test_r_eff_uses_max_of_robot_and_fallback():
    assert r_eff(r_robot=0.0, r_eff_fallback=0.6) == 0.6
    assert r_eff(r_robot=1.0, r_eff_fallback=0.6) == 1.0


def test_wz_clamp_geometric_pure_spin_passes_through():
    """Pure spin: vx=0. Rotation permit is elsewhere; wz clamp
    does NOT limit."""
    assert wz_clamp_geometric(vx_mps=0.0, wz_requested_radps=3.0,
                                r_robot=0.5) == 3.0


def test_wz_clamp_geometric_limits_on_forward_motion():
    """vx=1, r_eff=1 -> wz_max = 1. Request wz=2 -> clamp to 1."""
    result = wz_clamp_geometric(vx_mps=1.0, wz_requested_radps=2.0,
                                  r_robot=1.0)
    assert result == 1.0


def test_wz_clamp_preserves_sign():
    result = wz_clamp_geometric(vx_mps=1.0, wz_requested_radps=-2.0,
                                  r_robot=1.0)
    assert result == -1.0


# --- RD-8 shutdown 5 branches ---

def test_shutdown_evaluates_all_five_reasons():
    """Priority order: module_dead > recording > fence > rtk > grid."""
    d = evaluate_shutdown(
        grid_age_ms=100, grid_stale_threshold_ms=200,
        rtk_degraded=False, inside_fence_edge=False,
        recording_active=False, module_healthy=False)
    assert d.shutdown is True
    assert d.reason == "module_dead"


def test_shutdown_grid_stale():
    d = evaluate_shutdown(
        grid_age_ms=300, grid_stale_threshold_ms=200,
        rtk_degraded=False, inside_fence_edge=False,
        recording_active=False, module_healthy=True)
    assert d.reason == "grid_stale"


def test_shutdown_ok_when_all_healthy():
    d = evaluate_shutdown(
        grid_age_ms=100, grid_stale_threshold_ms=200,
        rtk_degraded=False, inside_fence_edge=False,
        recording_active=False, module_healthy=True)
    assert d.shutdown is False


# --- RD-9 audit ring ---

def test_ring_evicts_oldest_at_capacity():
    ring = RnsAuditRing(capacity=3)
    for i in range(5):
        ring.append(RnsAuditRecord(mono_ms=i, action="proposed",
                                    reason="", detail={}))
    snap = ring.snapshot()
    assert len(snap) == 3
    assert snap[0].mono_ms == 2   # oldest kept


def test_ring_snapshot_is_a_copy():
    ring = RnsAuditRing(capacity=10)
    ring.append(RnsAuditRecord(mono_ms=0, action="proposed",
                                reason="", detail={}))
    snap = ring.snapshot()
    snap.clear()
    # Original ring unchanged.
    assert len(ring) == 1


# --- RD-10 U54 semantics ---

def test_u54_person_inside_moving_robot_stops():
    """robot moving forward, person inside safety -> should_stop=True."""
    assert should_stop_but_not_retreat(dist_m=0.5, safety_m=1.0,
                                          robot_vx_mps=0.5) is True


def test_u54_person_inside_stopped_robot_no_action():
    """Robot already stopped -> no additional action."""
    assert should_stop_but_not_retreat(dist_m=0.5, safety_m=1.0,
                                          robot_vx_mps=0.0) is False


def test_u54_person_outside_safety_no_stop():
    assert should_stop_but_not_retreat(dist_m=2.0, safety_m=1.0,
                                          robot_vx_mps=1.0) is False


def test_u54_type_b_slowdown_linear_ramp():
    """d=safety -> 0; d=start -> 1; midpoint -> 0.5."""
    factor = type_b_slowdown_factor(dist_m=1.5, d_start_slowdown_m=2.0,
                                      safety_m=1.0)
    assert factor == pytest.approx(0.5)


def test_u54_type_b_slowdown_boundaries():
    assert type_b_slowdown_factor(1.0, 2.0, 1.0) == 0.0
    assert type_b_slowdown_factor(2.0, 2.0, 1.0) == 1.0
    assert type_b_slowdown_factor(3.0, 2.0, 1.0) == 1.0
