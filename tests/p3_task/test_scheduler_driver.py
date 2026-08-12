"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_scheduler_driver.py
Brief: scheduler_tick drives the task machine (validate + dispatch) (BIZ-P3-42)

Description:
Tests the scheduler pass against a real in-memory task.db: a pending task
validates to 'ready', the highest-priority ready task dispatches to 'running',
only one runs at a time, and a task that fails a precondition goes to 'failed'.
Each has a mutation guard per CLAUDE.md 3.3.
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.schedule.driver import apply_motion_result, scheduler_tick


pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def dao_conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield TasksDAO(c), c


def _pending(task_id, priority=50, seq=1, task_type="goto", mission='{"a":1}'):
    return TaskRow(
        task_id=task_id, task_type=task_type, state="pending",
        priority=priority, submit_seq=seq, mission_json=mission,
        total_steps=1, current_step=0, step_status_json="[]", created_ms=0,
        updated_ms=0, source="local", trace_id="tr", resume_policy="continue")


async def _collect(dao, conn):
    """Run one tick, capturing the transitions the callback sees."""
    seen = []

    async def on_t(task_id, to_state, reason):
        seen.append((task_id, to_state, reason))

    made = await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=on_t)
    return made, seen


@pytest.mark.asyncio
async def test_pending_validates_to_ready(dao_conn):
    dao, conn = dao_conn
    await dao.insert(_pending("t1"))
    await conn.commit()
    made, seen = await _collect(dao, conn)
    # t1 goes pending->ready, then (nothing else running) ready->running.
    assert ("t1", "pending", "ready") in made
    assert (await dao.fetch_by_id("t1")).state == "running"
    assert ("t1", "running", "") in seen


@pytest.mark.asyncio
async def test_dispatch_picks_highest_priority(dao_conn):
    """MUTATION: dispatching FIFO instead of by priority would run t-lo."""
    dao, conn = dao_conn
    await dao.insert(_pending("t-lo", priority=10, seq=1))
    await dao.insert(_pending("t-hi", priority=90, seq=2))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1,
                         on_transition=_noop)
    assert (await dao.fetch_by_id("t-hi")).state == "running"
    assert (await dao.fetch_by_id("t-lo")).state == "ready"   # waits


@pytest.mark.asyncio
async def test_only_one_runs_at_a_time(dao_conn):
    """A second tick must NOT dispatch a second task while one runs.
    MUTATION: dropping the 'nothing running' guard runs both."""
    dao, conn = dao_conn
    await dao.insert(_pending("t1", priority=50, seq=1))
    await dao.insert(_pending("t2", priority=50, seq=2))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop)
    await scheduler_tick(conn, dao, now_mono_ms=2, on_transition=_noop)
    states = {tid: (await dao.fetch_by_id(tid)).state for tid in ("t1", "t2")}
    assert list(states.values()).count("running") == 1
    assert states["t1"] == "running" and states["t2"] == "ready"


@pytest.mark.asyncio
async def test_bad_type_validate_fails(dao_conn):
    """A task whose type is outside the closed set fails V-1 -> 'failed', not
    'ready'. MUTATION: skipping validation would send it to ready/running."""
    dao, conn = dao_conn
    # Insert directly with a state the CHECK allows but a bogus... type must be
    # valid for the DDL, so use a valid type but an unparseable mission (V-5).
    await dao.insert(_pending("tbad", mission="   "))    # empty mission -> V-5
    await conn.commit()
    made, seen = await _collect(dao, conn)
    assert (await dao.fetch_by_id("tbad")).state == "failed"
    assert any(s[0] == "tbad" and s[1] == "failed" and "V-5" in s[2]
               for s in seen)


async def _noop(task_id, to_state, reason):
    return None


# -- apply_motion_result: running -> done/failed (PB8) --------------------

async def _run_one(dao, conn, task_id="t1"):
    """Record a task and drive it to 'running' via a tick."""
    await dao.insert(_pending(task_id))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop)


@pytest.mark.asyncio
async def test_motion_succeeded_completes(dao_conn):
    dao, conn = dao_conn
    await _run_one(dao, conn)
    made = await apply_motion_result(conn, dao, "t1", "succeeded",
                                     now_mono_ms=2, on_transition=_noop)
    assert made is True
    assert (await dao.fetch_by_id("t1")).state == "done"


@pytest.mark.asyncio
async def test_motion_aborted_fails(dao_conn):
    """MUTATION: mapping aborted to 'complete' would mark a crashed run done."""
    dao, conn = dao_conn
    await _run_one(dao, conn)
    await apply_motion_result(conn, dao, "t1", "aborted", now_mono_ms=2,
                              on_transition=_noop)
    assert (await dao.fetch_by_id("t1")).state == "failed"


@pytest.mark.asyncio
async def test_motion_in_flight_is_noop(dao_conn):
    """'running'/'accepted' are in-flight -> no transition. MUTATION: treating
    them as terminal would end the task on the first progress frame."""
    dao, conn = dao_conn
    await _run_one(dao, conn)
    made = await apply_motion_result(conn, dao, "t1", "running", now_mono_ms=2,
                                     on_transition=_noop)
    assert made is False
    assert (await dao.fetch_by_id("t1")).state == "running"


@pytest.mark.asyncio
async def test_motion_result_for_non_running_is_noop(dao_conn):
    """A late 'succeeded' after the task already left running (e.g. cancelled)
    must not resurrect it. MUTATION: skipping the state guard would."""
    dao, conn = dao_conn
    await dao.insert(_pending("t1"))
    await conn.commit()                            # stays pending, never ran
    made = await apply_motion_result(conn, dao, "t1", "succeeded",
                                     now_mono_ms=2, on_transition=_noop)
    assert made is False
    assert (await dao.fetch_by_id("t1")).state == "pending"
