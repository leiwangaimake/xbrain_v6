"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_scheduled_tasks.py
Brief: a task with not_before_ts waits in `scheduled` and becomes `ready` when due (15 S3.2)

Description:
99 U110. P3 used to ignore 11 S7.2 task.not_before_ts and ran a timed task
at once. Now the submit stores it (mission_json + scheduled_at) and the
scheduler defers a not-yet-due task (validate_defer_time) and releases it
on its time (due), by the injected wall clock. Each case names the mutant
it kills.
"""
from __future__ import annotations  # postponed annotations

import json  # mission_json

import aiosqlite  # in-memory task.db
import pytest  # marker, raises
import pytest_asyncio  # async fixture

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO  # the table
from xbrain.p3_task.ingest.task_command import parse_task_command  # S7.2
from xbrain.p3_task.ingest.task_row import task_row_from_command  # unit under test
from xbrain.p3_task.ingest.voice_task import VoiceTaskIngestError  # refusals
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # DDL
from xbrain.p3_task.schedule.driver import scheduler_tick  # unit under test

pytestmark = pytest.mark.no_device  # no robot

_DUE = 1_800_000_000.0  # the task's not_before_ts


@pytest_asyncio.fixture  # one db per case
async def dao_conn():  # dao + connection
    async with aiosqlite.connect(":memory:") as c:  # in memory
        for stmt in ALL_DDL_STATEMENTS:  # full DDL
            await c.execute(stmt)  # one statement
        await c.commit()  # schema ready
        yield TasksDAO(c), c  # dao + raw connection


def _row(task_id, not_before=None, seq=1):  # row builder
    """A pending goto, optionally timed."""
    mission = {"a": 1} if not_before is None else {"a": 1, "not_before_ts": not_before}  # timed or not
    return TaskRow(  # a pending goto
        task_id=task_id, task_type="goto", state="pending", priority=50,  # type, state, priority
        submit_seq=seq, mission_json=json.dumps(mission), total_steps=1,  # mission carries the time
        current_step=0, step_status_json="[]", created_ms=0, updated_ms=0,  # progress fields
        source="local", trace_id="tr", resume_policy="continue")  # provenance


async def _noop(*_a):  # transition sink
    return None  # no publish


async def _state(conn, task_id):  # state reader
    cur = await conn.execute("SELECT state FROM tasks WHERE task_id=?", (task_id,))  # one row
    return (await cur.fetchone())[0]  # current state


def test_submit_keeps_not_before_ts():  # ingest keeps the time
    """15 S9.5A.1: not_before_ts is the source of scheduled_at. MUTATION:
    drop it at ingest (the pre-U110 behaviour)."""
    cmd = parse_task_command({"cmd_id": "c-1", "action": "submit", "source": "voice",  # an S7.2 submit
                              "task": {"type": "goto", "not_before_ts": _DUE,  # with not_before_ts
                                       "params": {"waypoint_id": "w-1"}}})  # a goto
    row = task_row_from_command(cmd, submit_seq=1, now_mono_ms=0)  # mapped to a row
    assert json.loads(row.mission_json)["not_before_ts"] == _DUE  # kept
    assert row.scheduled_at.startswith("2027-01-15T")  # ISO UTC mirror


def test_a_bad_not_before_ts_is_refused():  # bad time
    cmd = parse_task_command({"cmd_id": "c-1", "action": "submit", "source": "voice",  # an S7.2 submit
                              "task": {"type": "goto", "not_before_ts": "soon",  # not a number
                                       "params": {"waypoint_id": "w-1"}}})  # a goto
    with pytest.raises(VoiceTaskIngestError):  # refused
        task_row_from_command(cmd, submit_seq=1, now_mono_ms=0)  # at ingest


@pytest.mark.asyncio
async def test_a_timed_task_waits_then_runs(dao_conn):  # waits, then runs
    """MUTATION: ignore not_before_ts and the task is 'running' at once."""
    dao, conn = dao_conn  # fixture
    await dao.insert(_row("t-1", not_before=_DUE))  # timed task
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop,  # early tick
                         now_wall_s=_DUE - 60)  # a minute early
    assert await _state(conn, "t-1") == "scheduled"  # deferred
    await scheduler_tick(conn, dao, now_mono_ms=2, on_transition=_noop,  # still early
                         now_wall_s=_DUE - 1)  # still early
    assert await _state(conn, "t-1") == "scheduled"  # still waiting
    await scheduler_tick(conn, dao, now_mono_ms=3, on_transition=_noop,  # on time
                         now_wall_s=_DUE)  # due: ready, then dispatched
    assert await _state(conn, "t-1") == "running"  # it runs


@pytest.mark.asyncio
async def test_without_a_wall_clock_a_timed_task_never_runs_early(dao_conn):  # no clock
    """now_wall_s None. MUTATION: treat 'no clock' as 'due'."""
    dao, conn = dao_conn  # fixture
    await dao.insert(_row("t-1", not_before=_DUE))  # timed task
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop)  # no now_wall_s
    assert await _state(conn, "t-1") == "scheduled"  # held


@pytest.mark.asyncio
async def test_an_untimed_task_is_unchanged(dao_conn):  # untimed
    dao, conn = dao_conn  # fixture
    await dao.insert(_row("t-1"))  # no time
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop,  # one tick
                         now_wall_s=_DUE)  # a clock that changes nothing
    assert await _state(conn, "t-1") == "running"  # straight through
