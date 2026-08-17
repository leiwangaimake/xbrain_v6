"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_queryable.py
Brief: query/tasks selector parsing + reply (11 S12.2A, the first queryable)

Description:
Pins the P3 query/tasks queryable's read half: selector parsing (scope/limit/
before with clamping and safe fallbacks) and answer_task_query (a JSON reply from
a real in-memory task.db). The load-bearing check is that parse_query_params reads
a REAL zenoh Selector -- zenoh params are ';'-separated, not '&', so a client that
writes '&' silently loses scope/limit; this test would redden if the parse ever
relied on '&'. Each check names its mutation (CLAUDE.md 3.3).
"""
from __future__ import annotations

import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.query.queryable import (
    DEFAULT_LIMIT, MAX_LIMIT, answer_task_query, parse_query_params,
)

pytestmark = pytest.mark.no_device


# -- selector parsing ----------------------------------------------------------

def test_defaults_when_absent():
    # No params -> current scope, default page, no cursor. MUTATION: a required
    # scope (no default) would reject the common "just show me current" query.
    assert parse_query_params({}) == ("current", DEFAULT_LIMIT, None)


def test_explicit_values():
    got = parse_query_params({"scope": "history", "limit": "20", "before": "42"})
    assert got == ("history", 20, 42)


def test_limit_clamped_and_bad_values_fall_back():
    # Over the cap -> MAX; below 1 -> 1; non-numeric -> default. A paging hint is
    # never worth erroring the whole query. MUTATION: dropping the clamp lets a
    # client pull the whole 30-day table in one reply.
    assert parse_query_params({"limit": "99999"})[1] == MAX_LIMIT
    assert parse_query_params({"limit": "0"})[1] == 1
    assert parse_query_params({"limit": "abc"})[1] == DEFAULT_LIMIT
    # A malformed cursor -> None (start from newest), never a crash.
    assert parse_query_params({"before": "xyz"})[2] is None


def test_parse_reads_semicolon_selector():
    # The real trap: zenoh selector params split on ';', NOT '&'. A real Selector
    # must parse to the three values. MUTATION: if the client (or this parse) used
    # '&', scope/limit/before would collapse into one param and default out.
    zenoh = pytest.importorskip("zenoh")
    sel = zenoh.Selector("query/tasks?scope=history;limit=20;before=42")
    assert parse_query_params(sel.parameters) == ("history", 20, 42)


# -- answer_task_query (reply bytes from a real db) ----------------------------

@pytest_asyncio.fixture
async def conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield c


async def _insert(conn, *, task_id, state, submit_seq, command_text=""):
    await TasksDAO(conn).insert(TaskRow(
        task_id=task_id, task_type="patrol", state=state, priority=50,
        submit_seq=submit_seq, mission_json="{}", total_steps=0, current_step=0,
        step_status_json="[]", created_ms=submit_seq, updated_ms=submit_seq,
        source="local", trace_id="tr", resume_policy="continue",
        command_text=command_text))


@pytest.mark.asyncio
async def test_answer_returns_reply_json(conn):
    await _insert(conn, task_id="t-1", state="running", submit_seq=1,
                  command_text="开始巡逻东区")
    await _insert(conn, task_id="t-2", state="done", submit_seq=2)
    raw = await answer_task_query(conn, {"scope": "current", "limit": "10"})
    reply = json.loads(raw.decode("utf-8"))
    # Shape the P5 client + frontend rely on.
    assert set(reply) == {"tasks", "has_more", "next_before"}
    ids = [t["task_id"] for t in reply["tasks"]]
    assert ids == ["t-1"]                       # only the non-terminal one
    assert reply["tasks"][0]["command_text"] == "开始巡逻东区"


@pytest.mark.asyncio
async def test_answer_bad_scope_raises(conn):
    # An unknown scope must raise (the callback logs + sends no reply) rather than
    # silently answering the wrong set. MUTATION: defaulting a bad scope to
    # 'current' would return data the caller never asked for.
    with pytest.raises(ValueError):
        await answer_task_query(conn, {"scope": "bogus", "limit": "5"})
