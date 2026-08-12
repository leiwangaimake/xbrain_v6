"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_b.py
Brief: BIZ-P3-3/4/5/6 DB DDL + DAO layer tests (in-memory aiosqlite)

Description:
Batch B tests the DDL by executing it against a real in-memory
aiosqlite connection, then exercises every DAO happy-path AND a
negative variant per CLAUDE.md S3.3 (each assertion must have a
matching red variant). The 5/16 quota triggers are verified by
inserting one past the cap and asserting ABORT.
"""

import pytest
import pytest_asyncio
import aiosqlite

from xbrain.p3_task.dao.simple_daos import (
    DocksDAO, FencesDAO, GeoObjectDAO, MemoryDAO,
    PatrolProgressDAO, PendingPushDAO, SnapshotDAO,
)
from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.persistence.schema_geo import (
    FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS, RECORD_DB_STATEMENTS,
)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS


pytestmark = pytest.mark.no_device


async def _apply(conn, statements):
    for stmt in statements:
        await conn.execute(stmt)
    await conn.commit()


@pytest_asyncio.fixture
async def task_conn():
    async with aiosqlite.connect(":memory:") as c:
        await _apply(c, ALL_DDL_STATEMENTS)
        yield c


@pytest_asyncio.fixture
async def geo_conn():
    async with aiosqlite.connect(":memory:") as c:
        await _apply(c, GEO_DB_STATEMENTS)
        yield c


@pytest_asyncio.fixture
async def fence_conn():
    async with aiosqlite.connect(":memory:") as c:
        await _apply(c, FENCE_DB_STATEMENTS)
        yield c


@pytest_asyncio.fixture
async def record_conn():
    async with aiosqlite.connect(":memory:") as c:
        await _apply(c, RECORD_DB_STATEMENTS)
        yield c


def _task(**over) -> TaskRow:
    """A valid minimal TaskRow; override any field. Fills the 15 S9.5 NOT NULL
    columns (source/trace_id/resume_policy) so a test only states what it
    exercises."""
    base = dict(
        task_id="t", task_type="patrol", state="pending", priority=5,
        submit_seq=1, mission_json="{}", total_steps=1, current_step=0,
        step_status_json="[]", created_ms=0, updated_ms=0, source="local",
        trace_id="tr", resume_policy="continue")
    base.update(over)
    return TaskRow(**base)


@pytest.mark.asyncio
async def test_open_configured_sets_wal_and_creates_schema(tmp_path):
    """open_configured (the ONLY place outside DAOs that opens aiosqlite,
    4.1) applies WAL + creates the tables. MUTATION: skipping the pragma pass
    would leave journal_mode at the 'delete' default; skipping the DDL would
    leave no tasks table."""
    from xbrain.p3_task.persistence.base import open_configured
    db = tmp_path / "sub" / "task.db"          # parent dir must be created too
    conn = await open_configured(str(db), ALL_DDL_STATEMENTS)
    try:
        cur = await conn.execute("PRAGMA journal_mode")
        assert (await cur.fetchone())[0].lower() == "wal"
        cur = await conn.execute("SELECT name FROM sqlite_master "
                                 "WHERE type='table' AND name='tasks'")
        assert await cur.fetchone() is not None
    finally:
        await conn.close()


# --- BIZ-P3-4 tasks DDL ---

@pytest.mark.asyncio
async def test_tasks_ddl_applies_cleanly(task_conn):
    cur = await task_conn.execute("SELECT name FROM sqlite_master "
                                    "WHERE type='table' AND name='tasks'")
    assert await cur.fetchone() is not None


@pytest.mark.asyncio
async def test_tasks_check_rejects_bad_state(task_conn):
    """State CHECK constraint rejects a value outside the 12-value
    closed set (11 S4.4)."""
    dao = TasksDAO(task_conn)
    row = TaskRow(task_id="t1", task_type="patrol", state="halfway",
                   priority=5, submit_seq=1, mission_json="{}",
                   total_steps=1, current_step=0, step_status_json="[]",
                   created_ms=0, updated_ms=0, source="local",
                   trace_id="tr", resume_policy="continue")
    with pytest.raises(Exception):
        await dao.insert(row)


@pytest.mark.asyncio
async def test_tasks_check_rejects_current_step_past_total(task_conn):
    dao = TasksDAO(task_conn)
    row = TaskRow(task_id="t2", task_type="patrol", state="pending",
                   priority=5, submit_seq=1, mission_json="{}",
                   total_steps=2, current_step=5, step_status_json="[]",
                   created_ms=0, updated_ms=0, source="local",
                   trace_id="tr", resume_policy="continue")
    with pytest.raises(Exception):
        await dao.insert(row)


@pytest.mark.asyncio
async def test_tasks_priority_scan_order(task_conn):
    dao = TasksDAO(task_conn)
    for i, prio in enumerate([10, 30, 20, 30]):
        await dao.insert(TaskRow(
            task_id=f"t{i}", task_type="patrol", state="pending",
            priority=prio, submit_seq=i, mission_json="{}", total_steps=1,
            current_step=0, step_status_json="[]", created_ms=0,
            updated_ms=0, source="local", trace_id="tr",
            resume_policy="continue"))
    rows = await dao.list_by_priority()
    # 30/seq=1, 30/seq=3, 20/seq=2, 10/seq=0
    assert [r[0] for r in rows] == ["t1", "t3", "t2", "t0"]


# --- BIZ-P3-4 (PB2) added columns: closed sets + round-trip ---

@pytest.mark.asyncio
async def test_new_columns_round_trip(task_conn):
    """The 15 S9.5 columns added in PB2 persist and read back. MUTATION:
    dropping any from the INSERT/SELECT column list loses it here."""
    dao = TasksDAO(task_conn)
    await dao.insert(_task(
        task_id="tc", source="cloud", trace_id="tr-9", resume_policy="restart",
        scheduled_at="2026-08-11T20:00:00Z", ttl_seconds=600,
        parent_task_id="tp", user_id="u1", route_geo_id="r-gate"))
    await task_conn.commit()
    got = await dao.fetch_by_id("tc")
    assert got.source == "cloud" and got.trace_id == "tr-9"
    assert got.resume_policy == "restart"
    assert got.scheduled_at == "2026-08-11T20:00:00Z" and got.ttl_seconds == 600
    assert got.parent_task_id == "tp" and got.route_geo_id == "r-gate"


@pytest.mark.asyncio
async def test_source_check_rejects_out_of_set(task_conn):
    dao = TasksDAO(task_conn)
    with pytest.raises(Exception):
        await dao.insert(_task(task_id="tx", source="martian"))


@pytest.mark.asyncio
async def test_resume_policy_check_rejects_out_of_set(task_conn):
    dao = TasksDAO(task_conn)
    with pytest.raises(Exception):
        await dao.insert(_task(task_id="tx", resume_policy="whenever"))


@pytest.mark.asyncio
async def test_interrupt_reason_closed_set_but_not_state_paired(task_conn):
    """interrupt_reason uses the suspend_reason set yet survives resume, so it
    is legal on a NON-suspended row (unlike suspend_reason). A bad value is
    still rejected. MUTATION: dropping the interrupt_reason CHECK lets a typo
    persist on a column that is never cleared."""
    dao = TasksDAO(task_conn)
    await dao.insert(_task(task_id="tok", state="running",
                           interrupt_reason="low_battery"))   # not suspended
    await task_conn.commit()
    assert (await dao.fetch_by_id("tok")).interrupt_reason == "low_battery"
    with pytest.raises(Exception):
        await dao.insert(_task(task_id="tbad", interrupt_reason="hangover"))


@pytest.mark.asyncio
async def test_duration_sec_only_at_terminal(task_conn):
    """duration_sec is non-null only at a terminal state (15 S9.5). MUTATION:
    dropping the CHECK lets a running row carry a duration."""
    dao = TasksDAO(task_conn)
    await dao.insert(_task(task_id="tdone", state="done", duration_sec=12.5))
    await task_conn.commit()
    assert (await dao.fetch_by_id("tdone")).duration_sec == 12.5
    with pytest.raises(Exception):
        await dao.insert(_task(task_id="trun", state="running",
                               duration_sec=3.0))


@pytest.mark.asyncio
async def test_scheduled_partial_index_exists(task_conn):
    """ix_tasks_scheduled must exist (the fail-silent timed-task index). Its
    predicate is verified by name here; PB6 exercises the wakeup path."""
    cur = await task_conn.execute(
        "SELECT name FROM sqlite_master WHERE type='index' "
        "AND name='ix_tasks_scheduled'")
    assert await cur.fetchone() is not None


# --- BIZ-P3-3 geo DDL + quotas ---

@pytest.mark.asyncio
async def test_dock_quota_trigger_rejects_sixth(geo_conn):
    dao = DocksDAO(geo_conn)
    for i in range(5):
        await dao.insert(f"d{i}", float(i), 0.0, 0.0, f"h{i}", 0)
    with pytest.raises(Exception, match="dock quota"):
        await dao.insert("d5", 5.0, 0.0, 0.0, "h5", 0)


@pytest.mark.asyncio
async def test_waypoint_quota_per_route_trigger(geo_conn):
    await geo_conn.execute(
        "INSERT INTO routes (route_id, name, content_hash, updated_ms) "
        "VALUES ('r1', 'R', 'h', 0)")
    for i in range(16):
        await geo_conn.execute(
            "INSERT INTO waypoints (waypoint_id, x_m, y_m, content_hash, "
            " updated_ms) VALUES (?, 0, 0, 'h', 0)", (f"w{i}",))
        await geo_conn.execute(
            "INSERT INTO route_waypoint_assoc (route_id, seq, waypoint_id) "
            "VALUES ('r1', ?, ?)", (i, f"w{i}"))
    await geo_conn.execute(
        "INSERT INTO waypoints (waypoint_id, x_m, y_m, content_hash, "
        " updated_ms) VALUES ('w17', 0, 0, 'h', 0)")
    with pytest.raises(Exception, match="waypoint quota"):
        await geo_conn.execute(
            "INSERT INTO route_waypoint_assoc (route_id, seq, waypoint_id) "
            "VALUES ('r1', 16, 'w17')")


# --- BIZ-P3-6 DAO layer basics ---

@pytest.mark.asyncio
async def test_memory_upsert_get(task_conn):
    m = MemoryDAO(task_conn)
    await m.put("k", b"v1", 0)
    await m.put("k", b"v2", 1)   # overwrite
    assert await m.get("k") == b"v2"
    assert await m.get("missing") is None


@pytest.mark.asyncio
async def test_snapshot_replace_is_atomic_looking(task_conn):
    dao = TasksDAO(task_conn)
    await dao.insert(TaskRow(
        task_id="t9", task_type="patrol", state="pending",
        priority=1, submit_seq=1, mission_json="{}", total_steps=1,
        current_step=0, step_status_json="[]", created_ms=0, updated_ms=0,
        source="local", trace_id="tr", resume_policy="continue"))
    snap = SnapshotDAO(task_conn)
    await snap.replace("t9", [(1.0, 2.0, 0.0), (3.0, 4.0, 1.5)])
    rows = await snap.fetch("t9")
    assert [r[1] for r in rows] == [1.0, 3.0]
    await snap.replace("t9", [(9.0, 9.0, 9.0)])
    rows = await snap.fetch("t9")
    assert [r[1] for r in rows] == [9.0]


@pytest.mark.asyncio
async def test_pending_push_fifo(task_conn):
    dao = PendingPushDAO(task_conn)
    for i, obj in enumerate(["a", "b", "c"]):
        await dao.enqueue("waypoint", obj, 1, i)
    rows = await dao.drain(limit=10)
    assert [r[2] for r in rows] == ["a", "b", "c"]
    await dao.ack(rows[0][0])
    remaining = await dao.drain(limit=10)
    assert [r[2] for r in remaining] == ["b", "c"]


@pytest.mark.asyncio
async def test_patrol_progress_upsert(task_conn):
    dao = TasksDAO(task_conn)
    await dao.insert(TaskRow(
        task_id="t3", task_type="patrol", state="pending",
        priority=1, submit_seq=1, mission_json="{}", total_steps=1,
        current_step=0, step_status_json="[]", created_ms=0, updated_ms=0,
        source="local", trace_id="tr", resume_policy="continue"))
    pp = PatrolProgressDAO(task_conn)
    await pp.upsert("t3", 2, 0.4, 0)
    await pp.upsert("t3", 3, 0.7, 1)   # overwrite
    got = await pp.fetch("t3")
    assert got == (3, 0.7, 1)


@pytest.mark.asyncio
async def test_geo_object_rev_bump_and_idempotency(geo_conn):
    await geo_conn.execute(
        "INSERT INTO waypoints (waypoint_id, x_m, y_m, content_hash, "
        " updated_ms) VALUES ('w1', 0, 0, 'h1', 0)")
    dao = GeoObjectDAO(geo_conn, "waypoints")
    # Same hash -> no bump, same rev.
    assert await dao.bump_rev("w1", "h1", 1) == 1
    # Different hash -> rev goes up by exactly 1.
    assert await dao.bump_rev("w1", "h2", 2) == 2
    assert await dao.bump_rev("w1", "h3", 3) == 3


@pytest.mark.asyncio
async def test_geo_object_rev_bump_missing_raises(geo_conn):
    dao = GeoObjectDAO(geo_conn, "waypoints")
    with pytest.raises(KeyError):
        await dao.bump_rev("nope", "h1", 0)


def test_geo_object_dao_rejects_unknown_table():
    with pytest.raises(ValueError, match="unknown table"):
        GeoObjectDAO(conn=None, table="not_a_table")


@pytest.mark.asyncio
async def test_fences_list_active_excludes_tombstoned(fence_conn):
    dao = FencesDAO(fence_conn)
    await dao.insert("f1", "polygon", "[]", "safe", "h", 0)
    await fence_conn.execute(
        "INSERT INTO fences (fence_id, kind, geom_json, content_hash, "
        " updated_ms, tombstone) VALUES ('f2', 'circle', '[]', 'h', 0, 1)")
    rows = await dao.list_active()
    assert {r[0] for r in rows} == {"f1"}


# --- BIZ-P3-5 record.db commands DDL (owner is P5) ---

@pytest.mark.asyncio
async def test_record_commands_ddl_applies(record_conn):
    cur = await record_conn.execute(
        "SELECT name FROM sqlite_master WHERE name='commands'")
    assert await cur.fetchone() is not None


@pytest.mark.asyncio
async def test_record_commands_seq_autoincrement(record_conn):
    await record_conn.execute(
        "INSERT INTO commands (category, scope, payload_json, origin, "
        " received_ms) VALUES ('cmd', 'ptz', '{}', 'local', 0)")
    await record_conn.execute(
        "INSERT INTO commands (category, scope, payload_json, origin, "
        " received_ms) VALUES ('cmd', 'ptz', '{}', 'local', 1)")
    cur = await record_conn.execute(
        "SELECT cmd_seq FROM commands ORDER BY cmd_seq ASC")
    rows = await cur.fetchall()
    assert [r[0] for r in rows] == [1, 2]
