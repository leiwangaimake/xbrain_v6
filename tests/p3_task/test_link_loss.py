"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_link_loss.py
Brief: F-5 link-loss return_home -- trigger idempotency + task insert (11 S4.6.4)

Description:
Pins the F-5 realization: the trigger fires ONCE per outage (idempotent by
gw_start_mono + link_epoch), never below L3; the injected row is a return_home on
the charge lane; and maybe_inject_return_home inserts exactly one row at L3 and none
at L2, against a real in-memory task.db (same DDL + DAO as any task). Mutations
paired per 3.3.
"""

import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.lifecycle.link_loss import (
    RETURN_HOME_PRIORITY, LinkLossReturnTrigger, build_return_home_row,
    maybe_inject_return_home,
)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS


pytestmark = pytest.mark.no_device


# --- LinkLossReturnTrigger (pure) ---

def test_l3_first_time_fires():
    t = LinkLossReturnTrigger()
    assert t.should_inject(3, gw_start_mono=100.0, link_epoch=2) is True


def test_l3_same_outage_is_idempotent():
    t = LinkLossReturnTrigger()
    assert t.should_inject(3, 100.0, 2) is True
    # MUTATION: without the (gw_start_mono, link_epoch) guard, a sustained L3 (which
    # stays L3 for the whole outage) would inject a return_home every tick.
    assert t.should_inject(3, 100.0, 2) is False
    assert t.should_inject(3, 100.0, 2) is False


def test_new_outage_fires_again():
    t = LinkLossReturnTrigger()
    t.should_inject(3, 100.0, 2)
    assert t.should_inject(3, 100.0, 3) is True     # link_epoch bumped = new outage


def test_below_l3_never_fires():
    t = LinkLossReturnTrigger()
    for lvl in (0, 1, 2):
        assert t.should_inject(lvl, 100.0, 2) is False


def test_missing_fields_never_fire():
    t = LinkLossReturnTrigger()
    assert t.should_inject(None, 100.0, 2) is False
    assert t.should_inject(3, None, 2) is False
    assert t.should_inject(3, 100.0, None) is False


# --- build_return_home_row (pure) ---

def test_return_home_row_shape():
    row = build_return_home_row(
        "t-20260817-001", 5, priority=RETURN_HOME_PRIORITY, level=3,
        disconnected_s=1801.0, link_epoch=2, gw_start_mono=100.0,
        now_mono_ms=123, trace_id="tr")
    assert row.task_type == "return_home" and row.state == "pending"
    assert row.source == "charge" and row.priority == 95
    assert row.total_steps == 1                  # 15 S4.2.1: one step (go home)
    m = json.loads(row.mission_json)
    assert m["reason"] == "cloud_link_lost" and m["link_epoch"] == 2


# --- maybe_inject_return_home (real task.db) ---

@pytest_asyncio.fixture
async def conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield c


async def _tasks(conn):
    cur = await conn.execute("SELECT task_type, source, state, priority FROM tasks")
    rows = await cur.fetchall()
    return [{"task_type": r[0], "source": r[1], "state": r[2], "priority": r[3]}
            for r in rows]


@pytest.mark.asyncio
async def test_l3_injects_one_return_home(conn):
    dao = TasksDAO(conn)
    t = LinkLossReturnTrigger()
    link = {"level": 3, "gw_start_mono": 100.0, "link_epoch": 2,
            "disconnected_s": 1801.0}
    tid = await maybe_inject_return_home(
        conn, dao, t, link, priority=RETURN_HOME_PRIORITY, now_mono_ms=123)
    assert tid == "rh-100-2"                      # 15 S4.2.1 deterministic task_id
    rows = await _tasks(conn)
    assert len(rows) == 1
    assert rows[0]["task_type"] == "return_home"
    assert rows[0]["source"] == "charge" and rows[0]["priority"] == 95
    assert rows[0]["state"] == "pending"
    # PERSISTENT idempotency: a FRESH trigger (as after a P3 restart) with the same
    # outage still injects nothing, because the task_id already exists in task.db.
    # MUTATION: a per-tick / per-restart insert would flood the queue OR hit the PK.
    tid2 = await maybe_inject_return_home(
        conn, dao, LinkLossReturnTrigger(), link,
        priority=RETURN_HOME_PRIORITY, now_mono_ms=124)
    assert tid2 is None
    assert len(await _tasks(conn)) == 1


@pytest.mark.asyncio
async def test_l2_injects_nothing(conn):
    dao = TasksDAO(conn)
    t = LinkLossReturnTrigger()
    link = {"level": 2, "gw_start_mono": 100.0, "link_epoch": 2}
    tid = await maybe_inject_return_home(
        conn, dao, t, link, priority=RETURN_HOME_PRIORITY, now_mono_ms=123)
    assert tid is None
    assert await _tasks(conn) == []
