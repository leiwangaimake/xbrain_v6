"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_a.py
Brief: GWY-P5-25/01/02 skeleton + pipeline + events schema tests

Description:
Batch A: nine-task registry with G-1 startup gate; seven-step
pipeline stages with schema / dedupe / level branches (each with a
paired negative test); events DDL applies against in-memory
aiosqlite; UNIQUE (source, event_id) enforced; SEQ-3 cursor advance
gates against gaps and rewinds.
"""

import pytest
import pytest_asyncio
import aiosqlite

from xbrain.p5_gateway.event.pipeline import (
    PIPELINE_STAGES, PipelineOrderViolation, VALID_CATEGORIES,
    VALID_LEVELS, assert_stage_order,
    stage_dedupe, stage_level, stage_schema,
)
from xbrain.p5_gateway.event.schema import (
    ALL_EVENT_STATEMENTS, CONSUMERS, SeqOrderViolation,
    UnknownConsumer, advance_cursor, validate_consumer,
)
from xbrain.p5_gateway.lifecycle.skeleton import (
    DuplicateTaskRegistration, IncompleteTaskRegistration,
    NINE_TASKS, P1_P2_WRITERS, TaskRegistry,
)


pytestmark = pytest.mark.no_device


# --- GWY-P5-25 skeleton ---

def test_registry_starts_empty():
    r = TaskRegistry.empty()
    assert r.tasks == {}


def test_registry_rejects_unknown_task_name():
    r = TaskRegistry.empty()
    with pytest.raises(DuplicateTaskRegistration, match="unknown"):
        r.register("halfway", object())


def test_registry_rejects_duplicate_register():
    r = TaskRegistry.empty()
    r.register("watchdog", object())
    with pytest.raises(DuplicateTaskRegistration, match="already"):
        r.register("watchdog", object())


def test_registry_assert_complete_fails_when_missing():
    r = TaskRegistry.empty()
    r.register("watchdog", object())
    with pytest.raises(IncompleteTaskRegistration, match="missing"):
        r.assert_complete()


def test_registry_all_nine_pass():
    r = TaskRegistry.empty()
    for n in NINE_TASKS:
        r.register(n, object())
    r.assert_complete()


def test_p1_p2_are_the_two_writers():
    """P-1 event_pipeline, P-2 delivery_ledger. No others."""
    assert P1_P2_WRITERS == frozenset(
        {"event_pipeline", "delivery_ledger"})


def test_writer_check():
    r = TaskRegistry.empty()
    assert r.can_write_record_db("event_pipeline") is True
    assert r.can_write_record_db("hmi_ws") is False


# --- GWY-P5-01 pipeline ---

def test_stage_order_matches_expected():
    assert_stage_order(PIPELINE_STAGES)


def test_stage_order_bad_raises():
    with pytest.raises(PipelineOrderViolation):
        assert_stage_order(("schema", "level", "dedupe"))


def test_schema_rejects_unknown_category():
    r = stage_schema({"event_id": "e1", "source": "p1",
                        "category": "halfway"})
    assert r.dropped and "unknown_category" in r.reason


def test_schema_rejects_missing_source():
    r = stage_schema({"event_id": "e1", "category": "safety"})
    assert r.dropped and "missing_id_or_source" in r.reason


def test_schema_valid_event_passes():
    r = stage_schema({"event_id": "e1", "source": "p1",
                        "category": "safety"})
    assert r.dropped is False


def test_dedupe_drops_repeat():
    seen = set()
    e = {"event_id": "e1", "source": "p1", "category": "safety"}
    r1 = stage_dedupe(e, seen)
    r2 = stage_dedupe(e, seen)
    assert r1.dropped is False
    assert r2.dropped is True


def test_level_bad_value_dropped():
    r = stage_level({"level": "critical"})
    assert r.dropped and "bad_level" in r.reason


def test_level_default_info_applied():
    e = {}
    r = stage_level(e)
    assert r.dropped is False and e["level"] == "info"


def test_valid_categories_and_levels_are_closed_sets():
    assert VALID_CATEGORIES == frozenset({
        "safety", "task", "sensor", "network", "audit", "diagnostic",
    })
    assert VALID_LEVELS == frozenset({"info", "warn", "error"})


# --- GWY-P5-02 events schema ---

@pytest_asyncio.fixture
async def event_conn():
    async with aiosqlite.connect(":memory:") as c:
        for s in ALL_EVENT_STATEMENTS:
            await c.execute(s)
        await c.commit()
        yield c


@pytest.mark.asyncio
async def test_events_table_created(event_conn):
    cur = await event_conn.execute(
        "SELECT name FROM sqlite_master WHERE name='events'")
    assert await cur.fetchone() is not None


@pytest.mark.asyncio
async def test_events_unique_source_event_id(event_conn):
    await event_conn.execute(
        "INSERT INTO events (event_id, source, category, level, "
        " payload_json, received_ms) VALUES ('e1', 'p1', 'safety', "
        " 'info', '{}', 0)")
    with pytest.raises(Exception):
        await event_conn.execute(
            "INSERT INTO events (event_id, source, category, level, "
            " payload_json, received_ms) VALUES ('e1', 'p1', 'safety', "
            " 'info', '{}', 0)")


@pytest.mark.asyncio
async def test_events_check_rejects_bad_level(event_conn):
    with pytest.raises(Exception):
        await event_conn.execute(
            "INSERT INTO events (event_id, source, category, level, "
            " payload_json, received_ms) VALUES ('e2', 'p1', 'safety', "
            " 'critical', '{}', 0)")


def test_consumer_names_closed_set():
    assert CONSUMERS == frozenset({"cloud", "hmi"})


def test_validate_consumer_rejects_unknown():
    with pytest.raises(UnknownConsumer):
        validate_consumer("smtp")


def test_advance_cursor_by_one_ok():
    assert advance_cursor(current=5, next_seq=6) == 6


def test_advance_cursor_same_seq_ok():
    """SEQ-3: cursor can stay put (idempotent delivery)."""
    assert advance_cursor(current=5, next_seq=5) == 5


def test_advance_cursor_rewind_rejected():
    with pytest.raises(SeqOrderViolation, match="rewind"):
        advance_cursor(current=5, next_seq=3)


def test_advance_cursor_gap_rejected():
    with pytest.raises(SeqOrderViolation, match="gap"):
        advance_cursor(current=5, next_seq=8)
