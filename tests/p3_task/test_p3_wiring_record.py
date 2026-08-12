"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p3_wiring_record.py
Brief: p3 wiring _record_one -- record + reflect into state/task (PB5/2)

Description:
The Zenoh loop of p3 main_wiring is integration-only, but its recording step
_record_one is unit-testable: given a decoded cmd/task payload and a live
task.db conn, it records a task-create and reflects the recorded task_id/state
into state/task, and it publishes NOTHING for a control frame. A fake state
publisher captures the reflected frames. Mutation guards per CLAUDE.md 3.3.
"""
from __future__ import annotations

import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.runtime.main_wiring import _record_one


pytestmark = pytest.mark.no_device


class _FakePub:
    """Captures state/task frames the wiring reflects."""

    def __init__(self):
        self.frames = []

    def put(self, data: bytes):
        self.frames.append(json.loads(data.decode("utf-8")))


@pytest_asyncio.fixture
async def conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield c


def _voice_frame():
    return {"schema": "p4_intent_v1", "intent_id": "B02", "text": "巡逻",
            "task_request": {"task_type": "patrol", "intent": "patrol_route",
                             "id": "B02", "slots": {}, "source": "voice"}}


@pytest.mark.asyncio
async def test_record_one_records_and_reflects(conn):
    dao = TasksDAO(conn)
    pub = _FakePub()
    n = await _record_one(conn, dao, _voice_frame(), pub)
    assert n == 1                                     # one new task
    # It landed in task.db at pending...
    cur = await conn.execute("SELECT task_id, state FROM tasks")
    row = await cur.fetchone()
    assert row[1] == "pending"
    # ...and was reflected into state/task with the recorded id.
    assert len(pub.frames) == 1
    active = pub.frames[0]["active_task"]
    assert active["task_id"] == row[0] and active["state"] == "pending"


@pytest.mark.asyncio
async def test_record_one_skips_control_frame(conn):
    """A device/control frame (no task_request) records nothing AND reflects
    nothing. MUTATION: reflecting or recording it would show a frame/row."""
    dao = TasksDAO(conn)
    pub = _FakePub()
    n = await _record_one(
        conn, dao, {"schema": "p4_intent_v1", "intent_id": "D01",
                    "text": "开灯"}, pub)
    assert n == 0
    assert pub.frames == []
    cur = await conn.execute("SELECT COUNT(*) FROM tasks")
    assert (await cur.fetchone())[0] == 0


@pytest.mark.asyncio
async def test_record_one_survives_a_bad_row(conn):
    """A malformed task_request must not kill the loop: _record_one logs and
    returns 0 rather than raising. MUTATION: letting the exception escape would
    crash the p3 consumer loop on one bad task."""
    dao = TasksDAO(conn)
    pub = _FakePub()
    bad = {"task_request": {"task_type": "not_a_type", "intent": "x",
                            "id": "x", "slots": {}, "source": "voice"}}
    n = await _record_one(conn, dao, bad, pub)
    assert n == 0 and pub.frames == []
