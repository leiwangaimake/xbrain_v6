"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_e.py
Brief: BIZ-P3-15/16/17/10 charge triggers + docking + executor + dock arbiter tests

Description:
Batch E: CR-1..8 trigger ordering (CR-3 first because it commands
stop-in-place); three-tier dock selection with paired negative
variants (unreachable dock filtered out; empty candidates -> None);
charge stage progression rejects skips; dock arbiter reserve/release/
demand/cede/refresh with contention scenarios.
"""

import pytest

from xbrain.p3_task.charge.dock_arbiter import (
    DockArbiter, DockOp, DockResult,
)
from xbrain.p3_task.charge.dock_select import (
    Dock, cost_select, energy_reach_filter, route_filter,
)
from xbrain.p3_task.charge.executor import (
    ChargeStage, InvalidChargeStageTransition,
    dedup_key, is_forward_transition, next_stage,
)
from xbrain.p3_task.charge.trigger import (
    ChargeThresholds, evaluate,
)


pytestmark = pytest.mark.no_device


THR = ChargeThresholds(
    soc_low_pct=30.0, soc_urgent_pct=15.0, soc_critical_pct=5.0,
    soc_recover_pct=40.0, idle_charge_after_s=600.0)


# --- BIZ-P3-15 charge triggers ---

def test_cr3_critical_stop_in_place_wins_over_user_command():
    d = evaluate(soc_pct=3.0, user_requested=True,
                  patrol_block_ended=False, return_home_auto=False,
                  dock_admission_ok_for_suspended=False,
                  idle_seconds=0.0, thr=THR)
    assert d.trigger_code == "CR-3" and d.stop_in_place is True


def test_cr2_urgent():
    d = evaluate(10.0, False, False, False, False, 0.0, THR)
    assert d.trigger_code == "CR-2" and d.stop_in_place is False


def test_cr1_low():
    d = evaluate(25.0, False, False, False, False, 0.0, THR)
    assert d.trigger_code == "CR-1"


def test_cr4_user_over_normal_soc():
    d = evaluate(50.0, True, False, False, False, 0.0, THR)
    assert d.trigger_code == "CR-4"


def test_cr8_idle_timer():
    d = evaluate(50.0, False, False, False, False, 700.0, THR)
    assert d.trigger_code == "CR-8"


def test_no_trigger_when_all_conditions_false():
    d = evaluate(50.0, False, False, False, False, 0.0, THR)
    assert d.triggered is False


# --- BIZ-P3-16 dock selection ---

def test_reach_filter_excludes_out_of_reach():
    """A far dock exceeds SoC budget -> filtered."""
    docks = [Dock("near", 5.0, 0.0), Dock("far", 500.0, 0.0)]
    kept = energy_reach_filter(
        docks, robot_x=0.0, robot_y=0.0, soc_pct=20.0,
        energy_per_meter_pct=0.1, dock_reserve_pct=5.0)
    assert {c.dock.dock_id for c in kept} == {"near"}


def test_reach_filter_strict_inequality():
    """SoC == need is REJECTED (matches V-3 reserve untouched rule)."""
    docks = [Dock("a", 10.0, 0.0)]
    kept = energy_reach_filter(
        docks, robot_x=0.0, robot_y=0.0, soc_pct=10.0,
        energy_per_meter_pct=0.5, dock_reserve_pct=5.0)   # need == 10
    assert kept == []


def test_route_filter_computes_skip_len():
    docks = [Dock("d1", 5.0, 0.0)]
    cands = energy_reach_filter(
        docks, 0.0, 0.0, 50.0, 0.1, 5.0)
    route = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)]
    out = route_filter(cands, route)
    assert out[0].skip_len_m > 0


def test_route_filter_no_route_zero_skip():
    docks = [Dock("d1", 5.0, 0.0)]
    cands = energy_reach_filter(docks, 0.0, 0.0, 50.0, 0.1, 5.0)
    out = route_filter(cands, [])
    assert out[0].skip_len_m == 0.0


def test_cost_select_smallest_wins():
    from xbrain.p3_task.charge.dock_select import DockCandidate
    a = DockCandidate(Dock("a", 0, 0), d_to_handover_m=5.0,
                       skip_len_m=0.0)
    b = DockCandidate(Dock("b", 0, 0), d_to_handover_m=10.0,
                       skip_len_m=0.0)
    assert cost_select([a, b], w1=1.0, w2=0.0).dock.dock_id == "a"


def test_cost_select_empty_returns_none():
    assert cost_select([], 1.0, 0.0) is None


# --- BIZ-P3-17 charge executor ---

def test_next_stage_progression():
    assert next_stage("approach") == "arrived"
    assert next_stage("arrived") == "charging"
    assert next_stage("charging") == "detach"


def test_next_stage_beyond_detach_raises():
    with pytest.raises(InvalidChargeStageTransition, match="no next"):
        next_stage("detach")


def test_next_stage_unknown_raises():
    with pytest.raises(InvalidChargeStageTransition, match="unknown stage"):
        next_stage("halfway")


def test_is_forward_transition_rejects_skip():
    assert is_forward_transition("approach", "arrived") is True
    assert is_forward_transition("approach", "charging") is False
    assert is_forward_transition("charging", "approach") is False


def test_dedup_key_composition():
    assert dedup_key("t1", "charging") == ("t1", "charging")


# --- BIZ-P3-10 dock arbiter ---

def test_reserve_empty_grants():
    a = DockArbiter()
    assert a.apply("d1", "t1", 10, DockOp.RESERVE) == DockResult.GRANTED


def test_reserve_occupied_denied():
    a = DockArbiter()
    a.apply("d1", "t1", 10, DockOp.RESERVE)
    assert a.apply("d1", "t2", 20, DockOp.RESERVE) == DockResult.DENIED


def test_reserve_same_task_idempotent():
    """Same task reserving twice -> GRANTED both times."""
    a = DockArbiter()
    a.apply("d1", "t1", 10, DockOp.RESERVE)
    assert a.apply("d1", "t1", 10, DockOp.RESERVE) == DockResult.GRANTED


def test_release_without_holding_is_noop():
    a = DockArbiter()
    assert a.apply("d1", "t1", 10, DockOp.RELEASE) == DockResult.NO_OP


def test_demand_higher_priority_preempts():
    a = DockArbiter()
    a.apply("d1", "t1", 10, DockOp.RESERVE)
    r = a.apply("d1", "t2", 20, DockOp.DEMAND)
    assert r == DockResult.PREEMPTED
    assert a.current_holder("d1").task_id == "t2"


def test_demand_lower_priority_denied():
    a = DockArbiter()
    a.apply("d1", "t1", 20, DockOp.RESERVE)
    r = a.apply("d1", "t2", 10, DockOp.DEMAND)
    assert r == DockResult.DENIED
    assert a.current_holder("d1").task_id == "t1"


def test_cede_releases_when_holder():
    a = DockArbiter()
    a.apply("d1", "t1", 10, DockOp.RESERVE)
    r = a.apply("d1", "t1", 10, DockOp.CEDE)
    assert r == DockResult.GRANTED
    assert a.current_holder("d1") is None


def test_refresh_only_when_holder():
    a = DockArbiter()
    assert a.apply("d1", "t1", 10, DockOp.REFRESH) == DockResult.NO_OP
    a.apply("d1", "t1", 10, DockOp.RESERVE)
    assert a.apply("d1", "t1", 10, DockOp.REFRESH) == DockResult.GRANTED
