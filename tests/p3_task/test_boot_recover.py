"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_boot_recover.py
Brief: 15 S10 INIT -- a task left `running` by an unclean shutdown is suspended at start-up

Description:
Measured on the robot 2026-11-06: the stack was stopped while a goto ran,
started again, and the dead `running` row kept every new task in `ready`
for ever (the scheduler dispatches only while nothing runs). 99 U141 records
the fix. Each case names the mutant it kills:
  * every running row is suspended with passive / unclean_shutdown, nothing
    else is touched, the change is COMMITTED and the connection still takes
    a write transaction afterwards (the ES-2 open-transaction bug);
  * a clean shutdown opens no transaction at all;
  * through the real P3 loop (tests/p3_task/live_loop.py): with a dead
    running row and a ready goto in task.db, the goto is dispatched and
    pushed, and the 11 S9.14 system event is published.
"""
from __future__ import annotations  # postponed annotations

import aiosqlite  # a file task db: a second connection must see only commits
import pytest  # marks
import pytest_asyncio  # async fixture

from xbrain.p3_task.dao.tasks_dao import TasksDAO  # seed rows
from xbrain.p3_task.lifecycle.boot_recover import suspend_unclosed_running  # under test
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task.db DDL
from tests.p3_task.live_loop import (  # the real loop over fake planes
    goto_row, route_ops, start_p3, stop_p3, task_state, wait_for)

pytestmark = pytest.mark.no_device  # pure logic plus an in-process loop

#: The CR-8 pair an old operator pause carries (must survive untouched).
PAUSED = {"suspend_kind": "passive", "suspend_reason": "operator_pause"}


@pytest_asyncio.fixture
async def db(tmp_path):
    """(connection, path) of a fresh file task.db."""
    path = str(tmp_path / "task.db")  # a file, not :memory:, for the second reader
    conn = await aiosqlite.connect(path)  # the connection under test
    for stmt in ALL_DDL_STATEMENTS:  # real schema, real CHECKs
        await conn.execute(stmt)  # create
    await conn.commit()  # schema durable
    yield conn, path  # hand both out
    await conn.close()  # cleanup


@pytest.mark.asyncio
async def test_every_running_row_is_suspended_and_committed(db):
    """MUTATION: suspend only running[0] -> t-b stays running -> red; drop
    the commit -> the fresh connection still reads running -> red."""
    conn, path = db  # connection and file
    dao = TasksDAO(conn)  # writes through the DAO, as P3 does
    for row in (goto_row("t-a", "running", 1), goto_row("t-b", "running", 2),  # two dead rows
                goto_row("t-q", "ready", 3), goto_row("t-p", "suspended", 4, **PAUSED)):
        await dao.insert(row)  # the state a crash between two writes can leave
    await conn.commit()  # as on disk after the crash
    got = await suspend_unclosed_running(dao, conn, 1000)  # INIT
    assert sorted(got) == ["t-a", "t-b"]  # both dead rows reported
    for tid in ("t-a", "t-b"):  # read on a FRESH connection: committed or not
        assert await task_state(path, tid) == ("suspended", "passive", "unclean_shutdown")
    assert await task_state(path, "t-q") == ("ready", None, None)  # queued untouched
    assert await task_state(path, "t-p") == ("suspended", "passive", "operator_pause")  # untouched
    await conn.execute("BEGIN IMMEDIATE")  # the connection is still writable
    await conn.rollback()  # (nothing to keep)


@pytest.mark.asyncio
async def test_a_clean_shutdown_opens_no_transaction(db):
    """MUTATION: BEGIN before the early return -> in_transaction stays true
    and every later submit fails -> red."""
    conn, _path = db  # empty task table
    assert await suspend_unclosed_running(TasksDAO(conn), conn, 1000) == []  # nothing to do
    assert not conn.in_transaction  # and nothing left open


@pytest.mark.asyncio
async def test_the_live_loop_dispatches_after_a_dead_running_row(tmp_path, monkeypatch):
    """The 2026-11-06 robot case end to end. MUTATION: remove the INIT call
    in main_wiring -> t-new never leaves ready, no route frame -> red;
    drop detail.type from the system event -> red."""
    rows = [goto_row("t-dead", "running", 1), goto_row("t-new", "ready", 2)]  # a power cut's leftovers
    h, stop, loop, task_db = await start_p3(tmp_path, monkeypatch, rows=rows)  # P3 restarts
    try:
        assert await wait_for(lambda: route_ops(h)), "the queued goto was never pushed"  # unblocked
        assert await task_state(task_db, "t-dead") == ("suspended", "passive", "unclean_shutdown")
        assert (await task_state(task_db, "t-new"))[0] == "running"  # dispatched
        events = h.frames("event/warn/system")  # 11 S9.14 "补发给云端"
        assert {"type": "unclean_shutdown", "task_id": "t-dead"} in [e["detail"] for e in events]
    finally:
        await stop_p3(stop, loop)  # never leave the loop running
