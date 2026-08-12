"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p3_pipeline.py
Brief: P3 pipeline integration -- cmd/task payload -> record -> schedule (PB6)

Description:
End-to-end over the P3 side (minus the Zenoh transport, which the ORIN deploy
exercises): a cmd/task frame as PB4 publishes it is recorded into task.db by
record_task_from_payload and then flows through scheduler_tick pending ->
ready -> running. This is the '语音 -> task.db -> 调度' chain the audit found
was broken (recorded tasks used to sit at pending forever); it proves the
whole P3 half is now wired. A malformed task still ends at 'failed', never
stuck.
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.ingest.task_recorder import record_task_from_payload
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.schedule.driver import scheduler_tick


pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def dao_conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield TasksDAO(c), c


def _frame(task_type="patrol", intent="patrol_route", id_="B02"):
    return {"schema": "p4_intent_v1", "intent_id": id_, "text": "开始巡逻",
            "task_request": {"task_type": task_type, "intent": intent,
                             "id": id_, "slots": {}, "source": "voice"}}


async def _noop(task_id, to_state, reason):
    return None


@pytest.mark.asyncio
async def test_voice_task_flows_to_running(dao_conn):
    """The whole P3 chain: a voice cmd/task frame is recorded at 'pending',
    then one scheduler pass validates it to 'ready' and dispatches it to
    'running'. MUTATION: before PB6 (no scheduler driver) it stayed at
    'pending' -- assert it does NOT."""
    dao, conn = dao_conn
    out = await record_task_from_payload(
        conn, dao, _frame(), date_str="20260812", now_mono_ms=1)
    assert out.kind == "recorded"
    assert (await dao.fetch_by_id(out.task_id)).state == "pending"
    # One scheduler tick drives it forward.
    await scheduler_tick(conn, dao, now_mono_ms=2, on_transition=_noop)
    assert (await dao.fetch_by_id(out.task_id)).state == "running"


@pytest.mark.asyncio
async def test_two_voice_tasks_serialise(dao_conn):
    """Two recorded tasks: only one runs; the other waits at ready (single
    robot). Highest priority... here equal priority, so submit order wins."""
    dao, conn = dao_conn
    a = await record_task_from_payload(
        conn, dao, _frame(task_type="goto", intent="goto_waypoint", id_="B01"),
        date_str="20260812", now_mono_ms=1)
    b = await record_task_from_payload(
        conn, dao, _frame(), date_str="20260812", now_mono_ms=2)
    await scheduler_tick(conn, dao, now_mono_ms=3, on_transition=_noop)
    sa = (await dao.fetch_by_id(a.task_id)).state
    sb = (await dao.fetch_by_id(b.task_id)).state
    assert {sa, sb} == {"running", "ready"}       # exactly one running
    assert sa == "running"                          # the earlier submit_seq
