"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_c.py
Brief: BIZ-P3-7/8/9/25 state machine + preconditions + scheduling + dispatcher tests

Description:
Batch C: 12-value state machine transitions (positive + negative
variants), suspend_kind orthogonality, preconditions V-1/V-2/V-3/V-5/
V-6 with paired failures, priority-FIFO scheduler + preemption, and
the 7-type dispatcher with a startup completeness gate.
"""

import pytest

from xbrain.p3_task.schedule.dispatcher import (
    Dispatcher, DispatcherIncomplete, UnknownDispatchTarget,
)
from xbrain.p3_task.schedule.loop import (
    ScheduleCandidate, decide, pick_next,
)
from xbrain.p3_task.state.machine import (
    InvalidTransition, TASK_STATES, TERMINAL_STATES, TRANSITIONS,
    apply_transition, validate_suspend_fields,
)
from xbrain.p3_task.state.preconditions import (
    check_v1_type, check_v2_priority, check_v3_energy_reach,
    check_v5_mission_parses, check_v6_step_count,
)


pytestmark = pytest.mark.no_device


# --- BIZ-P3-7 state machine (11 S4.4 12-value model) ---

def test_transition_pending_validate_ok():
    r = apply_transition("pending", "validate_ok")
    assert r.to_state == "ready" and r.idempotent is False


def test_transition_pending_defers():
    """Admission can defer a task by time or dependency (15 S3.2)."""
    assert apply_transition("pending", "validate_defer_time").to_state \
        == "scheduled"
    assert apply_transition("pending", "validate_defer_dep").to_state \
        == "blocked"
    # A rejected task lands in a terminal (failed), not left pending.
    assert apply_transition("pending", "validate_fail").to_state == "failed"


def test_transition_scheduled_and_blocked_to_ready():
    assert apply_transition("scheduled", "due").to_state == "ready"
    assert apply_transition("blocked", "unblock").to_state == "ready"


def test_transition_ready_dispatch_to_running():
    assert apply_transition("ready", "dispatch").to_state == "running"


def test_transition_running_to_done():
    """Normal completion is 'done' (11 S4.4), not the old 'completing'.
    MUTATION: reverting the target to 'completing' would fail the CHECK on
    INSERT because 'completing' is no longer a member of the closed set."""
    r = apply_transition("running", "complete")
    assert r.to_state == "done"


def test_transition_suspend_and_resume():
    assert apply_transition("running", "suspend").to_state == "suspended"
    assert apply_transition("suspended", "resume").to_state == "ready"
    assert apply_transition("suspended", "review").to_state == "needs_review"


def test_transition_unknown_from_state_rejected():
    with pytest.raises(InvalidTransition, match="unknown from_state"):
        apply_transition("halfway", "validate_ok")


def test_transition_bad_event_rejected():
    """running + 'validate_ok' is not in the table -- must reject
    (CLAUDE.md 3.5 closed set)."""
    with pytest.raises(InvalidTransition, match="no transition"):
        apply_transition("running", "validate_ok")


def test_transition_terminal_idempotent():
    """T-3: re-emitting 'complete' on a task already 'done' is a no-op (the
    idempotency check keys on the TARGET state)."""
    r = apply_transition("done", "complete")
    assert r.idempotent is True and r.to_state == "done"


def test_interrupted_has_no_producer():
    """15 S3.2 v0.4: 'interrupted' stays in the closed set but no arrow emits
    it this period. MUTATION: an arrow producing it would show up here."""
    assert "interrupted" in TASK_STATES
    assert "interrupted" not in set(TRANSITIONS.values())


def test_terminal_states_complete():
    assert TERMINAL_STATES == frozenset({
        "done", "failed", "cancelled", "needs_review", "interrupted",
        "wait_for_power_off"})


def test_suspend_fields_required_when_suspended():
    with pytest.raises(InvalidTransition, match="present iff suspended"):
        validate_suspend_fields("suspended", None, None)


def test_suspend_fields_must_be_empty_when_not_suspended():
    with pytest.raises(InvalidTransition, match="present iff suspended"):
        validate_suspend_fields("running", "passive", "operator_pause")


def test_suspend_kind_closed_set():
    with pytest.raises(InvalidTransition, match="unknown suspend_kind"):
        validate_suspend_fields("suspended", "coffee_break", "low_battery")


def test_suspend_reason_excludes_energy_unreachable():
    """CR-6: energy_unreachable is a closed-set MEMBER but has no producer, so
    the machine (matching the DDL CHECK) must reject it. MUTATION: adding it
    back to SUSPEND_REASONS would let this pass and then fail the INSERT."""
    with pytest.raises(InvalidTransition, match="unknown suspend_reason"):
        validate_suspend_fields("suspended", "passive", "energy_unreachable")


def test_suspend_valid_passive_pair():
    validate_suspend_fields("suspended", "passive", "low_battery")   # no raise


def test_suspend_valid_yielding_pair():
    validate_suspend_fields("suspended", "yielding", "preempted")    # no raise


def test_suspend_cr8_pairing_enforced():
    """CR-8: kind and reason must agree on yielding-vs-passive. The combo
    passive+preempted passes each closed-set CHECK alone but is fail-silent
    (a preempted task written passive never auto-resumes). MUTATION: dropping
    the pairing check lets this through."""
    with pytest.raises(InvalidTransition, match="does not pair"):
        validate_suspend_fields("suspended", "passive", "preempted")
    with pytest.raises(InvalidTransition, match="does not pair"):
        validate_suspend_fields("suspended", "yielding", "low_battery")


# --- BIZ-P3-8 preconditions ---

def test_v1_unknown_type_fails():
    r = check_v1_type("halfway")
    assert r is not None and r.code == "V-1"


def test_v1_valid_type_passes():
    assert check_v1_type("patrol") is None


def test_v2_priority_out_of_range_fails():
    assert check_v2_priority(-1) is not None
    assert check_v2_priority(101) is not None


def test_v2_priority_in_range_passes():
    assert check_v2_priority(0) is None
    assert check_v2_priority(50) is None
    assert check_v2_priority(100) is None


def test_v3_refuses_when_equal_to_need():
    """V-3: soc == need is REFUSED (reserve is untouched)."""
    r = check_v3_energy_reach(
        soc_pct=20.0, distance_m=100.0,
        energy_per_meter_pct=0.1, return_reserve_pct=10.0)
    assert r is not None and r.code == "V-3"


def test_v3_passes_when_over_need():
    r = check_v3_energy_reach(
        soc_pct=25.0, distance_m=100.0,
        energy_per_meter_pct=0.1, return_reserve_pct=10.0)
    assert r is None


def test_v5_empty_json_fails():
    r = check_v5_mission_parses("   ")
    assert r is not None and r.code == "V-5"


def test_v5_bad_json_fails():
    r = check_v5_mission_parses("{not_json")
    assert r is not None and r.code == "V-5"


def test_v5_null_fails():
    """V-5: JSON 'null' is not a valid mission body."""
    r = check_v5_mission_parses("null")
    assert r is not None


def test_v5_good_json_passes():
    assert check_v5_mission_parses('{"steps": []}') is None


def test_v6_step_types_require_positive_total():
    r = check_v6_step_count("patrol", 0)
    assert r is not None and r.code == "V-6"


def test_v6_stepless_types_accept_zero_total():
    """standby / follow / etc don't require step_count."""
    assert check_v6_step_count("standby", 0) is None


# --- BIZ-P3-9 scheduling ---

def test_pick_next_returns_none_on_empty():
    assert pick_next([]) is None


def test_pick_next_priority_desc_seq_asc():
    q = [
        ScheduleCandidate("a", priority=10, submit_seq=1, preemptible=True),
        ScheduleCandidate("b", priority=30, submit_seq=5, preemptible=True),
        ScheduleCandidate("c", priority=30, submit_seq=2, preemptible=True),
    ]
    assert pick_next(q).task_id == "c"


def test_decide_start_fresh_when_nothing_running():
    q = [ScheduleCandidate("a", 10, 1, True)]
    d = decide(q, running=None)
    assert d.pick_task_id == "a" and d.preempt == ""


def test_decide_preempts_lower_priority():
    running = ScheduleCandidate("r", priority=10, submit_seq=0, preemptible=True)
    q = [ScheduleCandidate("h", priority=50, submit_seq=1, preemptible=True)]
    d = decide(q, running=running)
    assert d.pick_task_id == "h" and d.preempt == "r"


def test_decide_wont_preempt_nonpreemptible():
    running = ScheduleCandidate("r", priority=10, submit_seq=0, preemptible=False)
    q = [ScheduleCandidate("h", priority=50, submit_seq=1, preemptible=True)]
    d = decide(q, running=running)
    assert d.preempt == "" and d.pick_task_id == ""


def test_decide_wont_preempt_equal_priority():
    """A same-priority newcomer waits (FIFO respected via submit_seq
    at admission time; running task keeps running)."""
    running = ScheduleCandidate("r", 30, 0, preemptible=True)
    q = [ScheduleCandidate("n", 30, 1, preemptible=True)]
    d = decide(q, running=running)
    assert d.preempt == "" and d.pick_task_id == ""


# --- BIZ-P3-25 dispatcher ---

def test_dispatcher_unknown_type_rejected_at_register():
    d = Dispatcher()
    with pytest.raises(UnknownDispatchTarget, match="unknown type"):
        d.register("halfway", lambda: None)


def test_dispatcher_missing_types_fails_startup():
    """assert_complete must catch a missing type BEFORE runtime,
    per CLAUDE.md 3.5."""
    d = Dispatcher()
    d.register("patrol", lambda: None)
    with pytest.raises(DispatcherIncomplete, match="missing handlers"):
        d.assert_complete()


def test_dispatcher_all_seven_ok():
    d = Dispatcher()
    for t in ("patrol", "goto", "charge", "return_home", "standby",
               "teach", "follow"):
        d.register(t, lambda t=t: t)
    d.assert_complete()      # no raise
    assert d.dispatch("goto")() == "goto"


def test_dispatcher_double_register_rejected():
    d = Dispatcher()
    d.register("patrol", lambda: None)
    with pytest.raises(UnknownDispatchTarget, match="already registered"):
        d.register("patrol", lambda: None)


def test_dispatcher_lookup_miss_raises():
    d = Dispatcher()
    with pytest.raises(UnknownDispatchTarget, match="no handler"):
        d.dispatch("patrol")
