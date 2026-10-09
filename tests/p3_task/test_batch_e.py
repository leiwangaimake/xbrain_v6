"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_e.py
Brief: BIZ-P3-16/17/10 docking + executor + dock arbiter tests

Description:
Batch E: three-tier dock selection with paired negative variants
(unreachable dock filtered out; empty candidates -> None); charge stage
progression rejects skips; dock arbiter reserve/release/demand/cede/
refresh with contention scenarios.

*** The charge TRIGGER cases that used to open this file are gone
(2026-10-09, T11). They asserted an invented eight-condition spec;
15 S8.2 defines two criteria. See the block further down for why they
were deleted rather than updated, and tests/p3_task/test_charge_trigger.py
for the replacement. docs/TODO_EVIDENCE.tsv's BIZ-P3-15 row was moved to
that file in the same batch -- a criterion that moves without its
evidence mapping leaves the mapping pointing at a file that no longer
covers the item, and nothing reports that.
"""

import pytest

from xbrain.p3_task.charge.dock_arbiter import (
    DockArbiter,
    DockOp,
    DockResult,
)
from xbrain.p3_task.charge.dock_select import (
    Dock,
    cost_select,
    energy_reach_filter,
    route_filter,
)
from xbrain.p3_task.charge.executor import (
    InvalidChargeStageTransition,
    dedup_key,
    is_forward_transition,
    next_stage,
)

pytestmark = pytest.mark.no_device


# ===========================================================================
# BIZ-P3-15 charge triggers -- SIX CASES REMOVED 2026-10-09 (T11)
# ===========================================================================
#
# This file used to carry test_cr3_critical_stop_in_place_wins_over_user_command
# / test_cr2_urgent / test_cr1_low / test_cr4_user_over_normal_soc /
# test_cr8_idle_timer / test_no_trigger_when_all_conditions_false, plus a THR
# constant built from soc_low_pct / soc_urgent_pct / soc_recover_pct /
# idle_charge_after_s.
#
# WHY THEY WERE REMOVED RATHER THAN UPDATED: every one of them pinned a
# specification that does not exist. 15 S8.2 defines TWO criteria joined by OR
# ("两个判据取'或'"); charge/trigger.py had implemented eight, and these cases
# asserted the eight. Four of the conditions have no source in any volume
# (patrol block ended / return_home auto-charge / dock admission clearing a
# suspended task / idle timer) and were deleted under CLAUDE.md S9.3; the two
# SoC tiers collapsed into the contract's single soc_threshold_pct, because
# 15 S12 CD-4 says verbatim "不要再加第三个阈值". The codes CR-1..CR-8 were
# themselves a collision with 15's own CR-1..CR-8 (the S8.2A critical rules).
#
# So there was nothing in these six cases to carry forward: their inputs,
# their thresholds and their expected codes all referred to things that are
# now gone. Updating them would have meant writing new cases in old clothes.
#
# *** THE PART WORTH REMEMBERING: these six were GREEN for as long as they
# existed, and they were the only cases that ever touched trigger.py. A full
# suite of green criteria over an invented specification is indistinguishable
# from a correctly tested module -- until somebody reads the source volume.
# They also hid the real gap: criterion B (CHG-11, "is 30% even enough from
# HERE") was never implemented, and no case could notice an absence.
#
# Replacement: tests/p3_task/test_charge_trigger.py, 14 cases against 15 S8.2
# and S8.2A, eight mutants killed.


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
