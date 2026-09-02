"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_state_task_mapping.py
Brief: HMI-W7 state/task -> plan-panel mapping tests (17 S6.10.4)

Description:
Guards the W7 fix: P3 publishes state/task as {schema, active_task:{task_id,
state, mono_ms}} (11 S2.2.2 / p3 main_wiring), and the plan panel reads flat task
keys (data_readers._plan). The MVP wrapped the whole envelope as one plan, so
_plan read state/targets off {schema, active_task} and got None -- a card that
showed a task_id-less, state-less blank. These tests pin the extract+project
chain end to end and pair each claim with the red mutant (CLAUDE.md 3.3).

The load-bearing case is test_extract_reads_nested_state: a mutant that returns
[payload] (the old wrap-the-envelope bug) makes the projected state None -> red.
Boundary: pure functions, no zenoh -- main_wiring's top level imports nothing
heavy (zenoh lives inside run_voice_loop_wiring), so the helper imports cleanly.
"""

from __future__ import annotations

from xbrain.p5_gateway.hmi.data_readers import plan_group
from xbrain.p5_gateway.runtime.main_wiring import _extract_active_tasks


# The real P3 envelope (p3_task main_wiring _make_publish / _record_one).
_P3_ENVELOPE = {
    "schema": "state_task_v1",
    "active_task": {"task_id": "t-42", "state": "running", "mono_ms": 1000},
}


def test_extract_reads_nested_active_task():
    # {schema, active_task:{...}} -> [the task], not [the envelope].
    # RED MUTANT: return [payload] (old wrap bug) -> the element has no top-level
    # task_id/state, so the next assertion (and _plan below) go None.
    tasks = _extract_active_tasks(_P3_ENVELOPE)
    assert len(tasks) == 1
    assert tasks[0]["task_id"] == "t-42"
    assert tasks[0]["state"] == "running"


def test_extract_reads_nested_state_end_to_end():
    # extract -> _plan projects the flat card the frontend renders.
    # RED MUTANT: [payload] wrap -> plan["state"] is None (state lives under
    # active_task in the envelope), exactly the blank card W7 fixes.
    tasks = _extract_active_tasks(_P3_ENVELOPE)
    grp = plan_group(tasks)
    assert grp["available"] is True
    plan = grp["plans"][0]
    assert plan["task_id"] == "t-42"
    assert plan["state"] == "running"
    # EX-1 gated: no route expansion yet -> no fabricated fraction (17 S6.10.4).
    assert plan["progress"] == {"done": 0, "total": None}
    assert plan["targets"] == []


def test_envelope_without_task_id_yields_no_card():
    # A malformed/empty active_task must NOT produce a blank plan card.
    # RED MUTANT: drop the at.get("task_id") guard -> [{}] -> a ghost card.
    assert _extract_active_tasks({"schema": "state_task_v1",
                                  "active_task": {}}) == []
    assert _extract_active_tasks({"schema": "state_task_v1"}) == []


def test_forward_compat_heartbeat_list():
    # A future 1 Hz heartbeat carrying a task list (progress.py HeartbeatState:
    # current_step/total_steps) must map straight through so progress shows.
    # RED MUTANT: only handle active_task -> the list is dropped, panel empty.
    payload = {"schema": "state_task_v1", "active_tasks": [
        {"task_id": "t-1", "state": "running",
         "current_step": 2, "total_steps": 5},
    ]}
    tasks = _extract_active_tasks(payload)
    plan = plan_group(tasks)["plans"][0]
    assert plan["progress"] == {"done": 2, "total": 5}


def test_non_dict_payload_is_empty():
    # A torn/garbage payload maps to no plans, never a crash.
    # RED MUTANT: remove the isinstance guard -> AttributeError on .get.
    assert _extract_active_tasks(None) == []       # type: ignore[arg-type]
    assert _extract_active_tasks([1, 2, 3]) == []  # type: ignore[arg-type]

# --- 11 S4.4 TaskState: P3 现在发的形状 -------------------------------

#: 三列表齐全的一帧. current 在前, 因为取首条的消费者要的是在跑那条.
_TASK_STATE = {
    "schema": "task_state_v1",
    "current": {"task_id": "t-run", "type": "goto", "state": "running",
                "route_id": "r-charge", "started_ts": 1788339000.0},
    "queue": [{"task_id": "t-q1", "type": "goto", "state": "ready"}],
    "suspended": [{"task_id": "t-s1", "type": "patrol",
                   "state": "suspended"}],
}


def test_task_state_three_lists_are_flattened_current_first():
    """P3 改发 11 S4.4 TaskState 之后, 适配器必须认这个形状 -- 不认的话
    HMI 计划面板与云端快照同时空掉.

    MUTATION: 去掉 TaskState 分支 -> 红.
    """
    tasks = _extract_active_tasks(_TASK_STATE)
    assert [t["task_id"] for t in tasks] == ["t-run", "t-q1", "t-s1"]


def test_the_queue_and_suspended_lists_are_not_dropped():
    """只取 current 的话 queue 与 suspended 在云端快照里恒空 -- 正是这次
    修复要解决的症状之一.

    MUTATION: 只 append current -> 红.
    """
    tasks = _extract_active_tasks(_TASK_STATE)
    assert len(tasks) == 3, tasks


def test_the_contract_field_names_pass_through_unchanged():
    """route_id / started_ts / type 要原样传到投影层, 在那里才做 v2.0 的
    改名. 适配器顺手改名的话, 两处改名各改一半是必然的.

    MUTATION: 在适配器里把 type 改成 task_type -> 红.
    """
    first = _extract_active_tasks(_TASK_STATE)[0]
    assert first["route_id"] == "r-charge"
    assert first["started_ts"] == 1788339000.0
    assert first["type"] == "goto"


def test_a_current_without_a_task_id_is_not_a_card():
    """一个有键无 id 的 current(半截报文 / 上游 bug)不能变成一张空白卡片 --
    面板上会出现一条点不开, 也说不出是哪个任务的记录. 与旧形状那条
    "active_task 为 {} 时不出卡" 是同一条规矩, 只是换了字段名.

    MUTATION: 把 append 的条件放宽成只判 isinstance(cur, dict) -> 红.
    """
    assert _extract_active_tasks({"schema": "task_state_v1", "current": {},
                                  "queue": [], "suspended": []}) == []
    # 列表里的成员同样要过 id 这一关.
    assert _extract_active_tasks({"schema": "task_state_v1", "current": None,
                                  "queue": [{}, {"state": "ready"}],
                                  "suspended": []}) == []


def test_an_idle_task_state_yields_no_cards():
    """current 为 null 且两个列表为空时不能造一张空卡片.

    MUTATION: 无条件 append current -> 红.
    """
    assert _extract_active_tasks({"schema": "task_state_v1", "current": None,
                                  "queue": [], "suspended": []}) == []
