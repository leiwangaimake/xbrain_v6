"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_events.py
Brief: 11 S6.2 task-event mapping -- (to_state, reason) -> kind/sev

Description:
Pins the 11 S6.2 task row (info/warn, accept/reject/start/complete/fail): a
validate-fail reason is a reject (warn); ready/running/succeeded are info;
failed/aborted/cancelled are warn; internal states (pending/suspended) produce no
event. Mutations paired per 3.3.
"""

import pytest

from xbrain.p3_task.state.task_events import task_event_for_transition


pytestmark = pytest.mark.no_device


def test_validate_fail_is_reject_warn():
    assert task_event_for_transition("pending", "E_GEO_INCOMPLETE") == \
        ("rejected", "warn")


def test_ready_is_accepted_info():
    assert task_event_for_transition("ready", "") == ("accepted", "info")


def test_running_is_started_info():
    assert task_event_for_transition("running", "") == ("started", "info")


def test_succeeded_is_completed_info():
    assert task_event_for_transition("succeeded", "") == ("completed", "info")


def test_failed_is_warn():
    # MUTATION: a failed task at info would not stand out from a normal completion.
    assert task_event_for_transition("failed", "") == ("failed", "warn")


def test_aborted_cancelled_are_warn():
    assert task_event_for_transition("aborted", "")[1] == "warn"
    assert task_event_for_transition("cancelled", "")[1] == "warn"


def test_internal_states_produce_no_event():
    # pending is the queued head; suspended/resumed are bookkeeping, not S6.2 events.
    assert task_event_for_transition("pending", "") is None
    assert task_event_for_transition("suspended", "") is None
