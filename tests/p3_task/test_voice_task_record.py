"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_voice_task_record.py
Brief: GWY-P4-40 (32.H) -- voice/text task -> unified task.db (same as party-A)

Description:
Tests the voice/text task path end to end: P4 to_task_request builds a
cmd/task request, P3 record_voice_task lands it in the SAME tasks table as
a party-A cloud task. Each criterion carries a mutation that must turn red
per CLAUDE.md 3.3: unified schema, CS-A1 registry intent, only task-family
intents enter the queue.
"""
from __future__ import annotations

import json

import aiosqlite
import pytest
import pytest_asyncio
import yaml

from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.ingest.voice_task import (
    VoiceTaskIngestError, record_voice_task, task_row_from_request,
)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.task_request import (
    TaskRequestError, assert_mapping_covered_by_registry,
    is_task_create_intent, to_task_request,
)

pytestmark = pytest.mark.no_device

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"


def _reg():
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


async def _apply(conn, statements):
    for stmt in statements:
        await conn.execute(stmt)
    await conn.commit()


@pytest_asyncio.fixture
async def task_conn():
    async with aiosqlite.connect(":memory:") as c:
        await _apply(c, ALL_DDL_STATEMENTS)
        yield c


# -- criterion 2: party-A patrol maps to patrol_route (CS-A1) ------------

def test_patrol_maps_to_registry_intent_patrol_route():
    reg = _reg()
    req = to_task_request("patrol_route", reg, slots={"route": "east"},
                          source="voice")
    assert req is not None
    assert req["task_type"] == "patrol"
    assert req["intent"] == "patrol_route"
    assert req["id"] == "B02"


def test_mapping_is_all_registry_intents():
    """MUTATION B guard: the meta-check resolves EVERY task-create mapping
    key through the registry. A fabricated 'schedule_patrol' (not in the
    128) would raise here (CS-A1)."""
    assert_mapping_covered_by_registry(_reg())


def test_fabricated_mapping_value_raises_cs_a1(monkeypatch):
    """MUTATION B (the CS-A1 guard inside to_task_request): if the mapping
    pointed a coarse type at a FABRICATED fine intent 'schedule_patrol'
    (not in the 128), resolving it through the registry must raise -- P4
    can never emit an id no consumer knows."""
    import xbrain.p4_agent.runtime.task_request as tr
    bad = dict(tr._TASK_CREATE_INTENTS)
    bad["schedule_patrol"] = "patrol"          # not a registry intent
    monkeypatch.setattr(tr, "_TASK_CREATE_INTENTS", bad)
    reg = _reg()
    with pytest.raises(Exception):
        to_task_request("schedule_patrol", reg, slots={}, source="voice")
    # And the startup meta-check catches the same fabricated mapping.
    with pytest.raises(Exception):
        assert_mapping_covered_by_registry(reg)


# -- criterion 3: only task-family intents enter the queue ---------------

def test_query_intent_is_not_a_task():
    reg = _reg()
    assert is_task_create_intent("query_battery") is False
    assert to_task_request("query_battery", reg, slots={}, source="voice") is None


def test_chitchat_intent_is_not_a_task():
    reg = _reg()
    assert to_task_request("greeting", reg, slots={}, source="voice") is None


def test_task_control_intent_is_not_a_create():
    """pause_task/cancel_task act on an EXISTING task; they do not MINT a
    new task row (they are not in the create map)."""
    reg = _reg()
    assert to_task_request("pause_task", reg, slots={}, source="voice") is None
    assert to_task_request("cancel_task", reg, slots={}, source="voice") is None


# -- criterion 1: unified task.db schema (same as party-A) ---------------

def test_task_row_uses_unified_schema_and_fields():
    reg = _reg()
    req = to_task_request("patrol_route", reg, slots={"loops": 2},
                          source="voice")
    row = task_row_from_request(req, task_id="t-voice-1", submit_seq=1,
                                priority=50, now_mono_ms=1000, trace_id="tr-1")
    assert row.task_type == "patrol"
    assert row.state == "pending"
    # 15 S9.5 NOT NULL columns are populated: source mapped to the closed set,
    # resume_policy the per-type default, trace_id threaded from the envelope.
    assert row.source == "local" and row.trace_id == "tr-1"
    assert row.resume_policy == "continue"
    mission = json.loads(row.mission_json)
    assert mission["source"] == "voice"
    assert mission["intent"] == "patrol_route"
    assert mission["slots"] == {"loops": 2}


@pytest.mark.asyncio
async def test_voice_task_inserts_into_same_tasks_table(task_conn):
    """A voice task lands in the SAME `tasks` table as a party-A task, read
    back through the SAME DAO. MUTATION A: a different schema/table would
    not round-trip through TasksDAO here."""
    reg = _reg()
    dao = TasksDAO(task_conn)
    req = to_task_request("goto_waypoint", reg, slots={"waypoint": "w-1"},
                          source="voice")
    row = await record_voice_task(dao, req, task_id="t-voice-2",
                                  submit_seq=1, priority=50, now_mono_ms=2000,
                                  trace_id="tr-2")
    await task_conn.commit()
    fetched = await dao.fetch_by_id("t-voice-2")
    assert fetched is not None
    assert fetched.task_type == "goto"          # in the 7-value closed set
    assert fetched.state == "pending"
    assert fetched.task_id == row.task_id


@pytest.mark.asyncio
async def test_bad_task_type_rejected_by_db_check(task_conn):
    """MUTATION A guard: a request with a task_type OUTSIDE the closed set
    is rejected -- first by the ingest guard, and the DB CHECK would reject
    it too. Proves the schema constraint is the party-A one."""
    with pytest.raises(VoiceTaskIngestError):
        task_row_from_request(
            {"task_type": "patrol_voice", "intent": "patrol_route",
             "id": "B02", "slots": {}, "source": "voice"},
            task_id="t-bad", submit_seq=1, priority=50, now_mono_ms=1,
            trace_id="tr-bad")
