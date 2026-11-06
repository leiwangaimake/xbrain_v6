"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_schedule_rules.py
Brief: 99 U144 -- one timed task per minute, a due timed task runs first, queue items carry their schedule, clear_queue by state

Description:
The user's rulings of 2026-11-06 on timed tasks, P3 half:
  * a timed task that comes due while the robot works waits for the current
    job, then runs BEFORE the other queued tasks (same priority), earliest
    first;
  * only one timed task per time: a submit in the minute of a scheduled task
    is refused (E_BUSY, detail.reason schedule_conflict, naming the task);
  * the 11 S4.4 queue item carries not_before_ts / waypoint_id / route_id so
    P4 can name a scheduled task for a voice cancel;
  * clear_queue may be filtered to the scheduled state ("取消所有定时任务"),
    and each cleared task reports its OWN from-state (the audit found the
    loop variable of the SELECT used for all of them).
Each case names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import json  # mission_json

import aiosqlite  # in-memory task db
import pytest  # marks
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_BUSY  # the refusal code
from xbrain.p3_task.dao.tasks_dao import TasksDAO  # seed rows
from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload  # submit / clear_queue
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task.db DDL
from xbrain.p3_task.schedule.driver import scheduler_tick  # the tick
from xbrain.p3_task.state.task_state import queue_item  # the 11 S4.4 item
from tests.p3_task.live_loop import goto_row  # a goto row

pytestmark = pytest.mark.no_device  # pure logic

T15 = 1_800_000_000.0  # a scheduled minute (epoch, minute-aligned)


def _timed(task_id, state, seq, at, wp="w-gate"):  # a timed goto row
    """A goto row with a not_before_ts in its mission."""
    row = goto_row(task_id, state, seq)  # the base row
    mission = {"params": {"waypoint_id": wp}, "not_before_ts": at}  # as the ingest writes it
    return row.__class__(**{**row.__dict__, "mission_json": json.dumps(mission)})  # same row, mission with the time


@pytest_asyncio.fixture
async def ctx():  # fresh database per case
    """(TaskContext, conn) over an in-memory task.db."""
    conn = await aiosqlite.connect(":memory:")  # DAO tests use in-memory (CLAUDE.md 7.2)
    for stmt in ALL_DDL_STATEMENTS:
        await conn.execute(stmt)  # real schema
    await conn.commit()  # durable
    yield TaskContext(task_conn=conn, dao=TasksDAO(conn)), conn  # both
    await conn.close()  # cleanup


def _submit(cmd_id, at):  # a voice timed goto submit
    """A voice timed goto submit (11 S7.2)."""
    return {"cmd_id": cmd_id, "action": "submit", "source": "voice",
            "task": {"type": "goto", "priority": 5, "not_before_ts": at,
                     "params": {"waypoint_id": "w-gate"}}}


# -- one per minute ------------------------------------------------------------------
@pytest.mark.asyncio
@pytest.mark.parametrize("at,result", [(T15, "rejected"), (T15 + 30, "rejected"),  # same minute
                                       (T15 + 60, "accepted")])  # the next minute
async def test_one_timed_task_per_minute(ctx, at, result):  # one case per submit time
    """MUTATION: compare seconds -> T15+30 accepted -> red; drop the check ->
    T15 accepted -> red."""
    c, conn = ctx  # context and connection
    await c.dao.insert(_timed("t-old", "scheduled", 1, T15))  # already scheduled
    await conn.commit()  # as on disk
    ack = await handle_task_payload(_submit("c-1", at), c, now_mono_ms=1000,  # the new timed submit
                                    created_at="", date_str="20261106")  # the new one
    assert ack["result"] == result  # refused only in the same minute
    if result == "rejected":  # the refusal carries its reason
        assert ack["code"] == E_BUSY and ack["detail"]["reason"] == "schedule_conflict"
        assert ack["detail"]["task_id"] == "t-old"  # names the task already there


@pytest.mark.asyncio
async def test_untimed_and_finished_tasks_never_clash(ctx):  # cancelled tasks free their slot
    """MUTATION: include terminal states -> a cancelled 15:00 task blocks a new
    one -> red."""
    c, conn = ctx  # context and connection
    await c.dao.insert(_timed("t-gone", "cancelled", 1, T15))  # cancelled earlier
    await conn.commit()
    ack = await handle_task_payload(_submit("c-2", T15), c, now_mono_ms=1000,
                                    created_at="", date_str="20261106")
    assert ack["result"] == "accepted"  # the slot is free again


# -- due timed task first ---------------------------------------------------------------
@pytest.mark.asyncio
async def test_a_due_timed_task_runs_before_earlier_queued_tasks(ctx):  # due timed tasks jump the queue
    """MUTATION: keep the plain (priority, submit_seq) order -> t-q1 (seq 1)
    runs first -> red; ignore the due time order -> t-late first -> red."""
    c, conn = ctx  # context and connection
    for row in (goto_row("t-q1", "ready", 1),  # queued before the timed ones
                _timed("t-late", "ready", 2, T15 + 600),  # due, later time
                _timed("t-early", "ready", 3, T15)):  # due, earliest time
        await c.dao.insert(row)
    await conn.commit()

    async def _noop(*_a):  # nothing to publish here
        return None  # no publishing in this test
    made = await scheduler_tick(conn, c.dao, now_mono_ms=1000, on_transition=_noop,
                                now_wall_s=T15 + 3600)  # all due, nothing running
    assert ("t-early", "ready", "running") in made  # the earliest due timed task
    order = []  # finish each and see who is next
    for _ in range(2):  # finish the running task twice
        cur = await conn.execute("SELECT task_id FROM tasks WHERE state='running'")  # who runs now
        tid = (await cur.fetchone())[0]
        order.append(tid)
        await conn.execute("UPDATE tasks SET state='done' WHERE task_id=?", (tid,))  # finish it
        await conn.commit()
        await scheduler_tick(conn, c.dao, now_mono_ms=2000, on_transition=_noop,
                             now_wall_s=T15 + 3600)
    assert order == ["t-early", "t-late"]  # timed first, by time; the plain task after


# -- queue item fields -------------------------------------------------------------------
def test_queue_item_carries_the_schedule():  # P4 reads these fields
    """MUTATION: drop the schedule fields -> P4 cannot name the task -> red."""
    row = {"task_id": "t-1", "task_type": "goto", "state": "scheduled", "priority": 5,
           "route_geo_id": None,
           "mission_json": json.dumps({"params": {"waypoint_id": "w-1"}, "not_before_ts": T15})}
    item = queue_item(row)  # the 11 S4.4 entry
    assert item["not_before_ts"] == T15 and item["target_waypoint_id"] == "w-1"  # named, timed
    assert "target_route_id" not in item and "route_id" not in item  # absent, never null


# -- clear_queue -------------------------------------------------------------------------
@pytest.mark.asyncio
async def test_clear_queue_by_state_and_each_from_state(ctx):  # 所有定时任务 = scheduled only
    """MUTATION: ignore the state filter -> the ready task is cancelled too ->
    red; report the leftover loop state -> the scheduled one says ready -> red."""
    c, conn = ctx  # context and connection
    await c.dao.insert(_timed("t-s", "scheduled", 1, T15))  # a scheduled task
    await c.dao.insert(goto_row("t-r", "ready", 2))  # a ready one (listed last)
    await conn.commit()
    seen = []  # (task_id, from_state) as published

    async def _rec(task_id, frm, to, reason):
        seen.append((task_id, frm, to))
    ack = await handle_task_payload(
        {"cmd_id": "c-3", "action": "clear_queue", "source": "voice",
         "filter": {"state": "scheduled"}}, c, now_mono_ms=1000,
        created_at="", date_str="20261106", on_transition=_rec)
    assert ack["detail"]["cleared_ids"] == ["t-s"]  # only the scheduled one
    assert seen == [("t-s", "scheduled", "cancelled")]  # its own from-state
    cur = await conn.execute("SELECT state FROM tasks WHERE task_id='t-r'")
    assert (await cur.fetchone())[0] == "ready"  # untouched


@pytest.mark.asyncio
async def test_clear_queue_reports_each_tasks_own_from_state(ctx):
    """Unfiltered, two tasks in DIFFERENT states. MUTATION: pass the SELECT
    loop's leftover state -> both report the last row's state -> red."""
    c, conn = ctx  # context and connection
    await c.dao.insert(_timed("t-s", "scheduled", 1, T15))  # scheduled
    await c.dao.insert(goto_row("t-r", "ready", 2))  # ready
    await conn.commit()  # as on disk
    seen = []  # (task_id, from_state)

    async def _rec(task_id, frm, to, reason):  # the publish hook
        seen.append((task_id, frm))  # what it was told
    await handle_task_payload({"cmd_id": "c-4", "action": "clear_queue", "source": "voice"},
                              c, now_mono_ms=1000, created_at="", date_str="20261106",
                              on_transition=_rec)  # clear everything queued
    assert sorted(seen) == [("t-r", "ready"), ("t-s", "scheduled")]  # each its own

