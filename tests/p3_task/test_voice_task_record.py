"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_voice_task_record.py
Brief: GWY-P4-40 (32.H) -- voice/text task -> unified task.db (same as party-A)

Description:
Tests the voice/text task path end to end: P4 to_task_command builds an
11 S7.2 TaskCommand, P3 lands it in the SAME tasks table as a party-A cloud
task. Each criterion carries a mutation that must turn red per CLAUDE.md 3.3:
unified schema, CS-A1 registry intent, only task-family intents enter the
queue.

*** Migrated 2026-08-20 from p4_agent's private task_request shape.

The two senders had DIFFERENT shapes for the same key: p4 emitted
{task_type, intent, id, slots, source} while the HMI and the cloud send the
contract's {action, task:{...}}, and P3 understood only the former. This file
used to exercise the private one end to end, which is why it stayed green
while the contract path was unreachable. It now drives the contract path, so
"voice task" and "HMI task" are the same test subject rather than two.
"""
from __future__ import annotations

import json
from dataclasses import replace

import aiosqlite
import pytest
import pytest_asyncio
import yaml

from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.ingest.task_command import parse_task_command
from xbrain.p3_task.ingest.task_row import task_row_from_command
from xbrain.p3_task.ingest.voice_task import VoiceTaskIngestError
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.task_request import (
    assert_mapping_covered_by_registry, is_task_create_intent, to_task_command,
)

pytestmark = pytest.mark.no_device

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"


def _reg():
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


def _row(cmd, *, task_id, submit_seq=1, now_mono_ms=1000, created_at=""):
    """The TaskRow P3 builds from a parsed command, with the id P3 allocated.

    Mirrors task_apply._submit: voice omits task.task_id, so the id is minted
    inside the transaction and stamped onto the command before the row is
    built (S7.2 corrected 2026-08-20).
    """
    return task_row_from_command(replace(cmd, task_id=task_id),
                                 submit_seq=submit_seq,
                                 now_mono_ms=now_mono_ms,
                                 created_at=created_at)


def _cmd(intent, reg, *, slots, source="voice", text="", route_id=None):
    """Build P4's frame and parse it as P3 does -- both halves, every time.

    Calling only the builder is what let the shapes drift apart, so no test in
    this file is allowed to look at the builder's output alone.
    """
    frame = to_task_command(intent, reg, slots=slots, source=source,
                            cmd_id="c-" + intent, text=text, route_id=route_id)
    return None if frame is None else parse_task_command(frame)


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
    cmd = _cmd("patrol_route", reg, slots={"route": "east"})
    assert cmd is not None
    assert cmd.action == "submit"
    assert cmd.task["type"] == "patrol"
    assert cmd.task["params"]["intent"] == "patrol_route"
    assert cmd.task["params"]["id"] == "B02"


def test_mapping_is_all_registry_intents():
    """MUTATION B guard: the meta-check resolves EVERY task-create mapping
    key through the registry. A fabricated 'schedule_patrol' (not in the
    128) would raise here (CS-A1)."""
    assert_mapping_covered_by_registry(_reg())


def test_fabricated_mapping_value_raises_cs_a1(monkeypatch):
    """MUTATION B (the CS-A1 guard inside to_task_command): if the mapping
    pointed a coarse type at a FABRICATED fine intent 'schedule_patrol'
    (not in the 128), resolving it through the registry must raise -- P4
    can never emit an id no consumer knows."""
    import xbrain.p4_agent.runtime.task_request as tr
    bad = dict(tr._TASK_CREATE_INTENTS)
    bad["schedule_patrol"] = "patrol"          # not a registry intent
    monkeypatch.setattr(tr, "_TASK_CREATE_INTENTS", bad)
    reg = _reg()
    with pytest.raises(Exception):
        to_task_command("schedule_patrol", reg, slots={}, source="voice",
                        cmd_id="c-x")
    # And the startup meta-check catches the same fabricated mapping.
    with pytest.raises(Exception):
        assert_mapping_covered_by_registry(reg)


# -- criterion 3: only task-family intents enter the queue ---------------

def test_query_intent_is_not_a_task():
    reg = _reg()
    assert is_task_create_intent("query_battery") is False
    assert _cmd("query_battery", reg, slots={}) is None


def test_chitchat_intent_is_not_a_task():
    reg = _reg()
    assert _cmd("greeting", reg, slots={}) is None


def test_task_control_intent_is_not_a_create():
    """pause_task/cancel_task act on an EXISTING task; they do not MINT a
    new task row (they are not in the create map)."""
    reg = _reg()
    assert _cmd("pause_task", reg, slots={}) is None
    assert _cmd("cancel_task", reg, slots={}) is None


# -- criterion 1: unified task.db schema (same as party-A) ---------------

def test_task_row_uses_unified_schema_and_fields():
    reg = _reg()
    cmd = _cmd("patrol_route", reg, slots={"loops": 2})
    row = _row(cmd, task_id="t-voice-1")
    assert row.task_type == "patrol"
    assert row.state == "pending"
    # 15 S9.5 NOT NULL columns are populated: source mapped through 15 S4.2's
    # channel table to the closed set, resume_policy the per-type default,
    # trace_id threaded from the envelope -- which for a command that supplied
    # none is the cmd_id, the frame an auditor would follow back to.
    assert row.source == "local" and row.trace_id == "c-patrol_route"
    assert row.resume_policy == "continue"
    # Priority comes from 15 S4.2's origin table, not a caller-supplied 50:
    # local is 40, and a made-up middle value used to outrank wecom (60).
    assert row.priority == 40
    mission = json.loads(row.mission_json)
    assert mission["source"] == "local"
    assert mission["intent"] == "patrol_route"
    assert mission["slots"] == {"loops": 2}


@pytest.mark.asyncio
async def test_voice_task_inserts_into_same_tasks_table(task_conn):
    """A voice task lands in the SAME `tasks` table as a party-A task, read
    back through the SAME DAO. MUTATION A: a different schema/table would
    not round-trip through TasksDAO here."""
    reg = _reg()
    dao = TasksDAO(task_conn)
    cmd = _cmd("goto_waypoint", reg, slots={"waypoint": "w-1"})
    row = _row(cmd, task_id="t-voice-2", now_mono_ms=2000)
    await dao.insert(row)
    await task_conn.commit()
    fetched = await dao.fetch_by_id("t-voice-2")
    assert fetched is not None
    assert fetched.task_type == "goto"          # in the 7-value closed set
    assert fetched.state == "pending"
    assert fetched.task_id == row.task_id


# -- command_text: raw command stored for party-A traceability (15 S9.5A.4) ----

def test_to_task_command_carries_command_text():
    """to_task_command threads the raw command `text` into task.params.
    MUTATION: drop the text= param (or the "text" key) -> the field is absent
    and the command never reaches tasks.command_text."""
    reg = _reg()
    cmd = _cmd("patrol_route", reg, slots={"route": "east"},
               text="开始巡逻东区")
    assert cmd.task["params"]["text"] == "开始巡逻东区"


def test_task_row_stores_command_text_and_defaults_empty():
    """task_row_from_command copies task.params.text into command_text, and
    leaves it '' when the command has none (a system task). MUTATION: not
    reading params.get('text') -> command_text stays '' even for a real
    command, losing the traceability record party-A requires."""
    reg = _reg()
    row = _row(_cmd("patrol_route", reg, slots={}, text="回充电站"),
               task_id="t-voice-3", now_mono_ms=1)
    assert row.command_text == "回充电站"
    # No text supplied -> '' (a system-minted task; DAO will store NULL).
    row2 = _row(_cmd("patrol_route", reg, slots={}),   # text defaults to ''
                task_id="t-voice-4", submit_seq=2, now_mono_ms=1)
    assert row2.command_text == ""


@pytest.mark.asyncio
async def test_command_text_round_trips_through_dao(task_conn):
    """The raw command actually PERSISTS: insert a voice task carrying text,
    read it back through the SAME DAO, and the text survives. This is the
    load-bearing party-A traceability guarantee. MUTATION: dropping
    command_text from _COLUMNS makes the insert/select skip the column, so the
    fetched command_text comes back '' -- the stored command is silently lost."""
    reg = _reg()
    dao = TasksDAO(task_conn)
    cmd = _cmd("goto_waypoint", reg, slots={"waypoint": "w-1"},
               text="去西门入口")
    await dao.insert(_row(cmd, task_id="t-voice-5", now_mono_ms=2000))
    await task_conn.commit()
    fetched = await dao.fetch_by_id("t-voice-5")
    assert fetched is not None
    assert fetched.command_text == "去西门入口"


@pytest.mark.asyncio
async def test_absent_command_text_round_trips_as_empty(task_conn):
    """A task with no command text stores NULL and reads back '' (never None on
    the str field). MUTATION: dropping command_text from _NULLABLE_TEXT makes
    the '' persist as '' (not NULL) or the NULL read back as None, breaking the
    '' <-> NULL contract the other nullable TEXT columns rely on."""
    reg = _reg()
    dao = TasksDAO(task_conn)
    cmd = _cmd("goto_waypoint", reg, slots={"waypoint": "w-2"})   # no text
    await dao.insert(_row(cmd, task_id="t-voice-6", now_mono_ms=2000))
    await task_conn.commit()
    fetched = await dao.fetch_by_id("t-voice-6")
    assert fetched is not None
    assert fetched.command_text == ""


@pytest.mark.asyncio
async def test_bad_task_type_rejected_by_db_check(task_conn):
    """MUTATION A guard: a command whose task.type is OUTSIDE the closed set
    is rejected -- first by the ingest guard, and the DB CHECK would reject
    it too. Proves the schema constraint is the party-A one."""
    with pytest.raises(VoiceTaskIngestError):
        task_row_from_command(
            parse_task_command(
                {"cmd_id": "c-bad", "action": "submit", "source": "voice",
                 "task": {"task_id": "t-bad", "type": "patrol_voice",
                          "params": {}}}),
            submit_seq=1, now_mono_ms=1)
