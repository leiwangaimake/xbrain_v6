"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p3_pipeline.py
Brief: P3 pipeline integration -- cmd/task payload -> record -> schedule (PB6)

Description:
End-to-end over the P3 side (minus the Zenoh transport, which the ORIN deploy
exercises): a cmd/task frame is admitted into task.db and then flows through
scheduler_tick pending -> ready -> running. This is the '语音 -> task.db ->
调度' chain the audit found was broken (recorded tasks used to sit at pending
forever); it proves the whole P3 half is now wired.

*** Ported 2026-08-23 from the legacy ingest to the contract path.
It used to drive record_task_from_payload with p4_agent's private
`task_request` frame. Since batch 15 nothing emits that shape -- p4_agent, the
HMI and the cloud all send the 11 S7.2 TaskCommand -- so the test was
exercising an ingest no sender could reach, while the path every real frame
takes had no pipeline-level test at all. Same chain, real entry point.
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload
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


def _frame(task_type="patrol", intent="patrol_route", id_="B02", cmd_id="c-1"):
    """An 11 S7.2 TaskCommand as p4_agent / the HMI / the cloud all send it.

    task.task_id is omitted on purpose: S7.2 (corrected 2026-08-20) lets the
    sender leave it out because the t-YYYYMMDD-NNN per-day sequence is P3's
    alone, and the idempotency key is cmd_id.
    """
    return {"cmd_id": cmd_id, "action": "submit", "source": "voice",
            "task": {"type": task_type,
                     "params": {"intent": intent, "id": id_, "slots": {},
                                "text": "开始巡逻"}}}


async def _submit(conn, dao, frame, *, now_mono_ms):
    """Run one frame through the contract receiver; return its task_id."""
    ack = await handle_task_payload(
        frame, TaskContext(conn, dao), now_mono_ms=now_mono_ms,
        date_str="20260812", created_at="")
    assert ack["result"] == "accepted", ack
    return ack["detail"]["task_id"]


async def _noop(task_id, to_state, reason):
    return None


@pytest.mark.asyncio
async def test_voice_task_flows_to_running(dao_conn):
    """The whole P3 chain: a voice cmd/task frame is recorded at 'pending',
    then one scheduler pass validates it to 'ready' and dispatches it to
    'running'. MUTATION: before PB6 (no scheduler driver) it stayed at
    'pending' -- assert it does NOT."""
    dao, conn = dao_conn
    task_id = await _submit(conn, dao, _frame(), now_mono_ms=1)
    assert (await dao.fetch_by_id(task_id)).state == "pending"
    # One scheduler tick drives it forward.
    await scheduler_tick(conn, dao, now_mono_ms=2, on_transition=_noop)
    assert (await dao.fetch_by_id(task_id)).state == "running"


@pytest.mark.asyncio
async def test_two_voice_tasks_serialise(dao_conn):
    """Two recorded tasks: only one runs; the other waits at ready (single
    robot). Highest priority... here equal priority, so submit order wins."""
    dao, conn = dao_conn
    a = await _submit(conn, dao,
                      _frame(task_type="goto", intent="goto_waypoint",
                             id_="B01", cmd_id="c-a"), now_mono_ms=1)
    b = await _submit(conn, dao, _frame(cmd_id="c-b"), now_mono_ms=2)
    await scheduler_tick(conn, dao, now_mono_ms=3, on_transition=_noop)
    sa = (await dao.fetch_by_id(a)).state
    sb = (await dao.fetch_by_id(b)).state
    assert {sa, sb} == {"running", "ready"}       # exactly one running
    assert sa == "running"                          # the earlier submit_seq
