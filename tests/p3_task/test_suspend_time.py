"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_suspend_time.py
Brief: every suspend records paused_at, so state/task suspended[] carries suspended_ts

Description:
99 U111, found in the live retest: no suspend path wrote paused_at, so every
suspended[] entry had suspended_ts null and "继续" (P4 picks the latest
passive one) resumed an old task instead of the one just paused. Each of the
three suspend writers is pinned here. MUTATION: drop paused_at from any one.
"""
from __future__ import annotations

import aiosqlite  # in-memory task.db
import pytest  # marker
import pytest_asyncio  # async fixture

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO  # the table
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # DDL
from xbrain.p3_task.state.task_state import read_task_state  # 11 S4.4 body

pytestmark = pytest.mark.no_device  # no robot


@pytest_asyncio.fixture
async def dao_conn():
    async with aiosqlite.connect(":memory:") as c:  # one db per case
        for stmt in ALL_DDL_STATEMENTS:  # full DDL
            await c.execute(stmt)  # one statement
        await c.commit()  # schema ready
        yield TasksDAO(c), c  # dao + connection


async def _running(dao, task_id, seq=1):
    """Insert one running goto."""
    await dao.insert(TaskRow(
        task_id=task_id, task_type="goto", state="running", priority=40,
        submit_seq=seq, mission_json="{}", total_steps=1, current_step=0,
        step_status_json="[]", created_ms=0, updated_ms=0, source="local",
        trace_id="tr", resume_policy="continue"))  # a running task


async def _suspended_ts(conn):
    """task_id -> suspended_ts as state/task publishes it."""
    body = await read_task_state(conn)  # the 11 S4.4 body
    return {i["task_id"]: i["suspended_ts"] for i in body["suspended"]}


@pytest.mark.asyncio
async def test_estop_and_yield_suspends_record_the_time(dao_conn):
    dao, conn = dao_conn  # fixture
    await _running(dao, "t-estop")  # one task
    await dao.suspend_task("t-estop", "passive", "estop_soft", 1)  # ES-2
    await _running(dao, "t-yield", seq=2)  # another task
    await dao.preempt_task("t-yield", "preempted", 2)  # yields to a higher one
    await conn.commit()  # visible
    ts = await _suspended_ts(conn)  # as published
    assert isinstance(ts["t-estop"], float) and isinstance(ts["t-yield"], float)


@pytest.mark.asyncio
async def test_operator_pause_records_the_time(dao_conn):
    """The pause path writes through task_apply._write_state."""
    from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload
    dao, conn = dao_conn  # fixture
    await _running(dao, "t-pause")  # one task
    await conn.commit()  # visible
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "pause", "task_id": "t-pause",
         "source": "voice"}, TaskContext(conn, dao), now_mono_ms=5)  # 暂缓任务
    assert ack["result"] == "accepted"  # paused
    assert isinstance((await _suspended_ts(conn))["t-pause"], float)  # set
