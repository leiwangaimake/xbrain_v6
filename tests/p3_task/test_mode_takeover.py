"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mode_takeover.py
Brief: 15 S4.1A -- B / D takeover yields the running task (yielding / mode_takeover) and resumes it

Description:
xbrain/p3_task/lifecycle/mode_takeover.py is the task-layer half of the
2026-10-28 B / D motion hold (99 U99). These cases pin:
  * only a RUNNING patrol / goto / return_home is suspended, with the CR-8
    pairing yielding + mode_takeover (auto-resume, not "wait for a person");
  * a charge task (docking) is left alone -- the 15 S4.1A list is verbatim;
  * the suspension is COMMITTED (the estop_suspend lesson: an uncommitted
    write reads back fine on the same connection and still breaks every later
    BEGIN IMMEDIATE);
  * resume touches mode_takeover tasks only, never "preempted" ones (their
    resume condition is the preempting task ending, 15 S3.2).
Same in-memory aiosqlite fixture as test_p3_estop_resume.py (CLAUDE.md 7.2).
Mutants are named per case and were run with __pycache__ cleared.
"""

from __future__ import annotations  # postponed annotations

import aiosqlite  # CLAUDE.md 7.2: in-memory DB
import pytest  # marks
import pytest_asyncio  # async fixture

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO  # the real DAO
from xbrain.p3_task.lifecycle.mode_takeover import (  # under test
    resume_after_takeover,
    suspend_running_for_takeover,
)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # production DDL

pytestmark = pytest.mark.no_device  # pure in-process


@pytest_asyncio.fixture  # one DB per case
async def dao_conn():  # DAO + its connection
    """A fresh in-memory task DB with the production DDL."""
    async with aiosqlite.connect(":memory:") as c:  # fresh, private
        for stmt in ALL_DDL_STATEMENTS:  # the real schema, CHECKs included
            await c.execute(stmt)  # tables + CHECKs
        await c.commit()  # schema committed
        yield TasksDAO(c), c  # both, like the wiring


def _task(task_id, state, task_type="patrol", priority=50, seq=1):  # test row builder
    """One task row in `state`; the fields the CHECKs need, nothing else."""
    return TaskRow(  # a row the CHECKs accept
        task_id=task_id, task_type=task_type, state=state,
        priority=priority, submit_seq=seq, mission_json='{"a":1}',
        total_steps=1, current_step=0, step_status_json="[]", created_ms=0,
        updated_ms=0, source="cloud", trace_id="tr", resume_policy="continue")


@pytest.mark.asyncio  # async case
async def test_running_patrol_yields_to_the_mode(dao_conn):
    """MUTATION: write passive / a different reason -> this fails (and CR-8
    would make the auto-resume never happen)."""
    dao, conn = dao_conn  # fixture parts
    await dao.insert(_task("t1", "running"))  # a patrol on the move
    await conn.commit()  # visible to the next transaction
    assert await suspend_running_for_takeover(dao, conn, 100) == (
        "t1", "suspended", "mode_takeover")
    row = await dao.fetch_by_id("t1")  # read back the committed row
    assert (row.state, row.suspend_kind, row.suspend_reason) == (
        "suspended", "yielding", "mode_takeover")


@pytest.mark.asyncio  # async case
async def test_the_suspension_is_committed(dao_conn):
    """The next transaction must still open (estop_suspend's 2026-09-01
    lesson). MUTATION: drop conn.commit() -> BEGIN IMMEDIATE raises here."""
    dao, conn = dao_conn  # fixture parts
    await dao.insert(_task("t1", "running"))  # running patrol
    await conn.commit()  # visible to the next transaction
    await suspend_running_for_takeover(dao, conn, 100)  # B / D starts
    await conn.execute("BEGIN IMMEDIATE")  # fails if left in a transaction
    await conn.rollback()  # close the probe transaction


@pytest.mark.asyncio  # async case
async def test_a_charge_task_is_not_suspended(dao_conn):
    """15 S4.1A lists patrol / goto / return_home only. MUTATION: drop the
    type filter -> the charge task is suspended -> this fails."""
    dao, conn = dao_conn  # fixture parts
    await dao.insert(_task("c1", "running", task_type="charge"))  # docking
    await conn.commit()  # visible to the next transaction
    assert await suspend_running_for_takeover(dao, conn, 100) is None  # nothing suspended
    assert (await dao.fetch_by_id("c1")).state == "running"  # untouched


@pytest.mark.asyncio  # async case
async def test_nothing_running_is_a_no_op(dao_conn):
    """Called every loop pass while the takeover lasts: idempotent."""
    dao, conn = dao_conn  # fixture parts
    await dao.insert(_task("t1", "ready"))  # queued, not running
    await conn.commit()  # visible to the next transaction
    assert await suspend_running_for_takeover(dao, conn, 100) is None  # nothing suspended


@pytest.mark.asyncio  # async case
async def test_resume_brings_back_only_mode_takeover_tasks(dao_conn):
    """MUTATION: resume every yielding task -> the preempted one is resumed
    too, against its own resume rule (15 S3.2) -> this fails."""
    dao, conn = dao_conn  # fixture parts
    await dao.insert(_task("t1", "running", seq=1))  # will yield to the mode
    await dao.insert(_task("t2", "running", seq=2))  # will be "preempted"
    await conn.commit()  # visible to the next transaction
    await dao.preempt_task("t2", "preempted", 50)  # yielding / preempted
    await conn.commit()  # visible to the next transaction
    await suspend_running_for_takeover(dao, conn, 100)  # B / D starts
    assert await resume_after_takeover(dao, conn, 200) == [("t1", "ready")]  # only t1
    assert (await dao.fetch_by_id("t1")).state == "ready"  # resumed
    row2 = await dao.fetch_by_id("t2")  # left for its own rule
    assert (row2.state, row2.suspend_reason) == ("suspended", "preempted")  # untouched
    assert await resume_after_takeover(dao, conn, 300) == []  # idempotent
