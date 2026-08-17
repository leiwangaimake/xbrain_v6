"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_query.py
Brief: P3 task-panel query -- current/history split + card projection (17 S6.8.4)

Description:
Pins query_task_cards against a real in-memory task.db: the current/history split
(non-terminal vs terminal), newest-first ordering, keyset paging (has_more +
next_before), and the five-field card projection with fields 1/2/3/5 (task_id /
created_at / command_text / percent) and the deferred field 4 (targets == []).
Each check names the mutation it reddens (CLAUDE.md 3.3). The load-bearing ones
are the terminal split (a wrong set would file a done task under 当前) and the
percent-None-when-total-zero rule (a fabricated 0% would look like real progress).
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.query.task_query import (
    _percent, query_task_cards, task_card_from_row,
)

pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def dao():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield TasksDAO(c)


async def _insert(dao, *, task_id, state, submit_seq, total_steps=0,
                  current_step=0, command_text="", created_at="",
                  task_type="patrol", source="local"):
    await dao.insert(TaskRow(
        task_id=task_id, task_type=task_type, state=state, priority=50,
        submit_seq=submit_seq, mission_json="{}", total_steps=total_steps,
        current_step=current_step, step_status_json="[]", created_ms=submit_seq,
        updated_ms=submit_seq, source=source, trace_id="tr-" + task_id,
        resume_policy="continue", command_text=command_text,
        created_at=created_at))


async def _seed(dao):
    """Two active + three terminal tasks, submit_seq 1..5 in creation order."""
    await _insert(dao, task_id="t-1", state="running", submit_seq=1,
                  total_steps=3, current_step=1, command_text="开始巡逻东区",
                  created_at="2026-08-17T10:00:12Z")
    await _insert(dao, task_id="t-2", state="pending", submit_seq=2)  # total 0
    await _insert(dao, task_id="t-3", state="done", submit_seq=3,
                  total_steps=2, current_step=2, command_text="回充电站")
    await _insert(dao, task_id="t-4", state="failed", submit_seq=4)
    await _insert(dao, task_id="t-5", state="cancelled", submit_seq=5)


# -- current / history split ---------------------------------------------------

@pytest.mark.asyncio
async def test_current_scope_excludes_terminal(dao):
    await _seed(dao)
    reply = await query_task_cards(dao._conn, scope="current", limit=10)
    ids = [c["task_id"] for c in reply["tasks"]]
    # Only the non-terminal ones (running, pending). MUTATION: a wrong terminal
    # set would file done/failed/cancelled here.
    assert set(ids) == {"t-1", "t-2"}


@pytest.mark.asyncio
async def test_history_scope_terminal_newest_first(dao):
    await _seed(dao)
    reply = await query_task_cards(dao._conn, scope="history", limit=10)
    ids = [c["task_id"] for c in reply["tasks"]]
    # done/failed/cancelled, ordered by submit_seq DESC (newest first). MUTATION:
    # ASC order, or leaking an active task, reddens.
    assert ids == ["t-5", "t-4", "t-3"]
    assert reply["has_more"] is False and reply["next_before"] is None


@pytest.mark.asyncio
async def test_bad_scope_raises(dao):
    with pytest.raises(ValueError):
        await query_task_cards(dao._conn, scope="all", limit=10)


# -- card projection: fields 1/2/3/5 + deferred field 4 ------------------------

@pytest.mark.asyncio
async def test_card_carries_five_fields(dao):
    await _seed(dao)
    reply = await query_task_cards(dao._conn, scope="current", limit=10)
    card = next(c for c in reply["tasks"] if c["task_id"] == "t-1")
    assert card["created_at"] == "2026-08-17T10:00:12Z"     # field 2
    assert card["command_text"] == "开始巡逻东区"            # field 3
    assert card["progress"]["percent"] == pytest.approx(33.3)  # field 5
    # field 4 deferred to the keypoint layer -> empty SHAPE, not fabricated.
    # MUTATION: dropping the key -> KeyError; fabricating points -> non-empty.
    assert card["targets"] == []
    # submit_seq is an internal cursor, NOT exposed on the card.
    assert "submit_seq" not in card


def test_percent_none_when_total_zero_else_value():
    # total 0 (route not expanded) -> None, NEVER 0.0. MUTATION: returning 0.0
    # would render a real-looking 0% for a task whose progress is simply unknown.
    assert _percent(0, 0) is None
    assert _percent(1, 3) == pytest.approx(33.3)
    assert _percent(2, 2) == pytest.approx(100.0)


@pytest.mark.asyncio
async def test_pending_task_percent_is_none(dao):
    await _seed(dao)
    reply = await query_task_cards(dao._conn, scope="current", limit=10)
    card = next(c for c in reply["tasks"] if c["task_id"] == "t-2")
    assert card["progress"]["percent"] is None


def test_empty_command_and_created_become_none():
    # '' from the DB reads back as None on the card (frontend renders blank),
    # not an empty string it must special-case.
    card = task_card_from_row({
        "task_id": "t-x", "created_at": "", "command_text": "",
        "state": "done", "source": "charge", "task_type": "return_home",
        "current_step": 0, "total_steps": 0, "submit_seq": 9})
    assert card["created_at"] is None and card["command_text"] is None


# -- keyset paging -------------------------------------------------------------

@pytest.mark.asyncio
async def test_history_paging_has_more_and_cursor(dao):
    await _seed(dao)
    # Page 1: newest 2 terminal tasks, more remain.
    p1 = await query_task_cards(dao._conn, scope="history", limit=2)
    assert [c["task_id"] for c in p1["tasks"]] == ["t-5", "t-4"]
    # has_more + a cursor at the last returned row (submit_seq 4). MUTATION: not
    # fetching limit+1 -> has_more stuck False and the client never pages.
    assert p1["has_more"] is True and p1["next_before"] == 4
    # Page 2: rows strictly older than the cursor -> the last terminal task.
    p2 = await query_task_cards(dao._conn, scope="history", limit=2,
                                before=p1["next_before"])
    assert [c["task_id"] for c in p2["tasks"]] == ["t-3"]
    assert p2["has_more"] is False and p2["next_before"] is None
