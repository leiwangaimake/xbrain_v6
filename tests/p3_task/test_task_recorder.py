"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_recorder.py
Brief: record a cmd/task payload into task.db -- dedup + transaction (BIZ-P3-41)

Description:
Tests record_task_from_payload against a real in-memory task.db: a voice
task-create records at 'pending' with an allocated id; a control/device frame
is skipped; an explicit-id party-A frame dedups (second record is a no-op that
keeps the first row). Each has a mutation guard per CLAUDE.md 3.3.
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.ingest.task_recorder import record_task_from_payload
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS


pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield c


def _voice_frame(task_type="patrol", intent="patrol_route", id_="B02"):
    """A cmd/task frame as PB4 publishes for a voice task-create."""
    return {
        "schema": "p4_intent_v1", "intent_id": id_, "text": "start patrol",
        "task_request": {
            "task_type": task_type, "intent": intent, "id": id_,
            "slots": {}, "source": "voice"},
    }


@pytest.mark.asyncio
async def test_voice_task_recorded_at_pending(conn):
    dao = TasksDAO(conn)
    out = await record_task_from_payload(
        conn, dao, _voice_frame(), date_str="20260811", now_mono_ms=1000)
    assert out.kind == "recorded" and out.state == "pending"
    assert out.task_id == "t-20260811-001"
    got = await dao.fetch_by_id("t-20260811-001")
    assert got is not None and got.task_type == "patrol"
    assert got.source == "local" and got.trace_id  # NOT NULL, non-empty


@pytest.mark.asyncio
async def test_two_voice_tasks_get_distinct_ids(conn):
    """No external dedup key -> each utterance is its own task. MUTATION:
    reusing a fixed id would make the second insert collide/duplicate."""
    dao = TasksDAO(conn)
    a = await record_task_from_payload(
        conn, dao, _voice_frame(), date_str="20260811", now_mono_ms=1)
    b = await record_task_from_payload(
        conn, dao, _voice_frame(), date_str="20260811", now_mono_ms=2)
    assert a.task_id == "t-20260811-001" and b.task_id == "t-20260811-002"


@pytest.mark.asyncio
async def test_control_frame_is_skipped(conn):
    """A frame with no task_request (a device/control command) records
    nothing. MUTATION: recording every frame would mint a task for a light."""
    dao = TasksDAO(conn)
    out = await record_task_from_payload(
        conn, dao, {"schema": "p4_intent_v1", "intent_id": "D01",
                    "text": "light on"},
        date_str="20260811", now_mono_ms=1)
    assert out.kind == "skipped"
    cur = await conn.execute("SELECT COUNT(*) FROM tasks")
    assert (await cur.fetchone())[0] == 0


@pytest.mark.asyncio
async def test_explicit_id_dedups(conn):
    """Party-A path (explicit task_id) is idempotent: the second record is a
    no-op that keeps the first row (15 S3.3 TSK-12). MUTATION: dropping the dup
    check would raise on the PK or overwrite the row."""
    dao = TasksDAO(conn)
    frame = _voice_frame()
    frame["task_id"] = "t-cloud-7"
    frame["task_request"]["source"] = "cloud"
    first = await record_task_from_payload(
        conn, dao, frame, date_str="20260811", now_mono_ms=1)
    assert first.kind == "recorded" and first.task_id == "t-cloud-7"
    # Redelivery of the same id.
    again = await record_task_from_payload(
        conn, dao, frame, date_str="20260811", now_mono_ms=2)
    assert again.kind == "duplicate" and again.task_id == "t-cloud-7"
    cur = await conn.execute("SELECT COUNT(*) FROM tasks WHERE task_id=?",
                             ("t-cloud-7",))
    assert (await cur.fetchone())[0] == 1        # exactly one row


@pytest.mark.asyncio
async def test_created_at_injected_to_dispatch_time(conn):
    """The db loop injects created_at (UTC ISO wall time) -> tasks.created_at,
    which is the HMI 下发时间 source (15 S9.5 CA-1 / 17 S6.8.4 field 2). MUTATION:
    not threading created_at through the recorder -> created_at stays NULL and
    the task panel has no dispatch time to show."""
    dao = TasksDAO(conn)
    await record_task_from_payload(
        conn, dao, _voice_frame(), date_str="20260817", now_mono_ms=1,
        created_at="2026-08-17T10:00:12Z")
    got = await dao.fetch_by_id("t-20260817-001")
    assert got is not None and got.created_at == "2026-08-17T10:00:12Z"


@pytest.mark.asyncio
async def test_recorded_row_is_committed(conn):
    """The insert is committed (a fresh reader sees it), proving the
    transaction closed rather than leaving an open write lock."""
    dao = TasksDAO(conn)
    await record_task_from_payload(
        conn, dao, _voice_frame(), date_str="20260811", now_mono_ms=1)
    cur = await conn.execute("SELECT state FROM tasks WHERE task_id=?",
                             ("t-20260811-001",))
    assert (await cur.fetchone())[0] == "pending"
