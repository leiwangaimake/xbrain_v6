"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_id_alloc.py
Brief: task_id (t-YYYYMMDD-NNN) + submit_seq allocation (BIZ-P3-40)

Description:
Tests id_alloc against a real in-memory task.db: task_id form + per-day
increment + max-from-table (restart continuation) + submit_seq monotonicity.
Each assertion has a mutation guard per CLAUDE.md 3.3: the per-day scan is
proven by inserting a lower and a higher id and checking the NEXT is max+1
(a naive count-based impl would return a colliding value).
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.ingest.id_alloc import next_submit_seq, next_task_id
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS


pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield c


def _row(task_id: str, submit_seq: int) -> TaskRow:
    return TaskRow(
        task_id=task_id, task_type="patrol", state="pending", priority=5,
        submit_seq=submit_seq, mission_json="{}", total_steps=1, current_step=0,
        step_status_json="[]", created_ms=0, updated_ms=0, source="local",
        trace_id="tr", resume_policy="continue")


@pytest.mark.asyncio
async def test_first_task_id_is_001(conn):
    assert await next_task_id(conn, "20260811") == "t-20260811-001"


@pytest.mark.asyncio
async def test_task_id_increments_per_day(conn):
    dao = TasksDAO(conn)
    await dao.insert(_row("t-20260811-001", 1))
    await dao.insert(_row("t-20260811-002", 2))
    await conn.commit()
    assert await next_task_id(conn, "20260811") == "t-20260811-003"


@pytest.mark.asyncio
async def test_task_id_is_max_plus_one_not_count(conn):
    """MUTATION: a count-based NNN (len(rows)+1) would return 002 here and
    collide with the existing 005. It must be max(NNN)+1."""
    dao = TasksDAO(conn)
    await dao.insert(_row("t-20260811-005", 1))     # a gap: only 005 exists
    await conn.commit()
    assert await next_task_id(conn, "20260811") == "t-20260811-006"


@pytest.mark.asyncio
async def test_task_id_scoped_to_the_day(conn):
    """Yesterday's ids do not raise today's sequence."""
    dao = TasksDAO(conn)
    await dao.insert(_row("t-20260810-009", 1))     # a different day
    await conn.commit()
    assert await next_task_id(conn, "20260811") == "t-20260811-001"


@pytest.mark.asyncio
async def test_task_id_rejects_bad_date(conn):
    with pytest.raises(ValueError, match="YYYYMMDD"):
        await next_task_id(conn, "2026-08-11")


@pytest.mark.asyncio
async def test_submit_seq_starts_at_one(conn):
    assert await next_submit_seq(conn) == 1


@pytest.mark.asyncio
async def test_submit_seq_is_max_plus_one(conn):
    """MUTATION: a process counter reset on restart would return 1 here and
    collide; max+1 from the table continues the sequence."""
    dao = TasksDAO(conn)
    await dao.insert(_row("t-20260811-001", 7))     # highest seq on disk = 7
    await conn.commit()
    assert await next_submit_seq(conn) == 8
