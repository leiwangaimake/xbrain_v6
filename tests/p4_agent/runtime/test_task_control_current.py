"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_control_current.py
Brief: B-class voice control resolves its task from 11 S4.4 TaskState.current

Description:
11 S9.5A: "P4/P2 在生成 TaskCommand 时从 state/task.current.task_id 取值填入".
P4 read `active_task` instead -- the field name of P3's placeholder broadcast.
Both sides were off-contract in the same direction, so nothing looked wrong until
P3 started publishing the real TaskState.

These cases pin the two properties the field change must not lose: the id comes
from `current`, and a queued task is NEVER substituted when there is no current
one. The second is the one worth a test of its own -- picking the head of the
queue would look helpful and would cancel a task the operator did not mean.
"""

from __future__ import annotations

import pytest

from xbrain.p4_agent.runtime.task_control_request import (
    TaskControlError, current_task_from_state, spoken_target,
    to_task_control_command,
)


# INF-TS-1: 纯单测, 不碰设备(无 zenohd / 无底盘 / 无 ORIN 专属硬件).
pytestmark = pytest.mark.no_device


def _state(body):
    return {"state/task": body}


_TASK_STATE = {"schema": "task_state_v1",
               "current": {"task_id": "t-20260902-007", "state": "running"},
               "queue": [{"task_id": "t-queued-1", "state": "ready"}],
               "suspended": []}


def test_the_task_id_comes_from_current():
    """MUTATION: read body["active_task"] -> red (the key no longer exists)."""
    assert current_task_from_state(_state(_TASK_STATE))["task_id"] == \
        "t-20260902-007"


def test_a_queued_task_is_never_substituted_for_a_missing_current():
    """"暂停当前任务" with nothing running must refuse, not pick the head of the
    queue: that task has not started, so it is not the one the operator meant.
    MUTATION: fall back to queue[0] when current is null -> red."""
    idle = dict(_TASK_STATE, current=None)
    assert current_task_from_state(_state(idle)) is None
    with pytest.raises(TaskControlError) as exc:
        to_task_control_command("cancel_task", state=_state(idle),
                                cmd_id="c-1", source="voice")
    assert "no_active_task" in str(exc.value)


def test_a_suspended_task_is_not_treated_as_current_either():
    """A paused task is not "the running task"; 11 S4.4 keeps it in its own list.
    MUTATION: fall back to suspended[0] -> red."""
    idle = dict(_TASK_STATE, current=None,
                suspended=[{"task_id": "t-susp", "state": "suspended"}])
    assert current_task_from_state(_state(idle)) is None


def test_the_command_names_the_real_task_id():
    """S7.2 forbids handing "guess which one" to P3, so the id must be on the
    wire. MUTATION: omit task_id from the payload -> red."""
    cmd = to_task_control_command("cancel_task", state=_state(_TASK_STATE),
                                  cmd_id="c-9", source="voice")
    assert cmd["task_id"] == "t-20260902-007"
    assert cmd["cmd_id"] == "c-9"


def test_the_operator_is_told_which_task_will_be_acted_on():
    """S7.2's first justification is "队列是活的" -- the operator can only stop a
    wrong pick if the pick is spoken. MUTATION: return "" always -> red."""
    assert spoken_target(_state(_TASK_STATE)) == "t-20260902-007"


def test_never_having_received_state_task_reads_as_no_task():
    """Same answer as current: null -- both mean nothing is controllable.
    MUTATION: raise on a missing key -> red."""
    assert current_task_from_state(None) is None
    assert current_task_from_state({}) is None
    assert current_task_from_state(_state({"schema": "task_state_v1"})) is None
