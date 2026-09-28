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
from xbrain.p3_task.persistence.schema_task import (
    ALL_DDL_STATEMENTS, PatrolProgressShapeConflict,
    ensure_patrol_progress_shape,
)


pytestmark = pytest.mark.no_device


# Every negative DDL test below names aiosqlite.IntegrityError AND matches
# "CHECK constraint failed", never pytest.raises(Exception).
#
# The reason is the one CLAUDE.md 3.2 form 1 describes. Each of these tests
# builds a row by hand and hands it to a DAO; raises(Exception) is satisfied by
# ANY failure on that path -- a TypeError from a TaskRow field that was renamed,
# an OperationalError from a column the INSERT no longer has, an AttributeError
# from a DAO method that moved. All of those are green here while saying
# nothing at all about the CHECK the test is named after, and every one of them
# is a change somebody will plausibly make.
#
# The match= half matters as much as the type: DELETING a CHECK from the DDL
# does not make the insert succeed if some OTHER constraint (NOT NULL, UNIQUE,
# a foreign key) also rejects the row -- it would still raise IntegrityError
# and the test would still pass. "CHECK constraint failed" is the only string
# sqlite emits for the constraint class these tests are about.
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
    with pytest.raises(aiosqlite.IntegrityError,
                       match="CHECK constraint failed"):
        await dao.insert(row)


@pytest.mark.asyncio
async def test_tasks_check_rejects_current_step_past_total(task_conn):
    dao = TasksDAO(task_conn)
    row = TaskRow(task_id="t2", task_type="patrol", state="pending",
                   priority=5, submit_seq=1, mission_json="{}",
                   total_steps=2, current_step=5, step_status_json="[]",
                   created_ms=0, updated_ms=0, source="local",
                   trace_id="tr", resume_policy="continue")
    with pytest.raises(aiosqlite.IntegrityError,
                       match="CHECK constraint failed"):
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
    with pytest.raises(aiosqlite.IntegrityError,
                       match="CHECK constraint failed"):
        await dao.insert(_task(task_id="tx", source="martian"))


@pytest.mark.asyncio
async def test_resume_policy_check_rejects_out_of_set(task_conn):
    dao = TasksDAO(task_conn)
    with pytest.raises(aiosqlite.IntegrityError,
                       match="CHECK constraint failed"):
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
    with pytest.raises(aiosqlite.IntegrityError,
                       match="CHECK constraint failed"):
        await dao.insert(_task(task_id="tbad", interrupt_reason="hangover"))


@pytest.mark.asyncio
async def test_duration_sec_only_at_terminal(task_conn):
    """duration_sec is non-null only at a terminal state (15 S9.5). MUTATION:
    dropping the CHECK lets a running row carry a duration."""
    dao = TasksDAO(task_conn)
    await dao.insert(_task(task_id="tdone", state="done", duration_sec=12.5))
    await task_conn.commit()
    assert (await dao.fetch_by_id("tdone")).duration_sec == 12.5
    with pytest.raises(aiosqlite.IntegrityError,
                       match="CHECK constraint failed"):
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
    # v1.5: full 15 S9.3 dock (WGS84 body + handover), keyed by geo_id 'd-*';
    # names are UNIQUE. MUTATION: dropping the trigger lets a 6th active dock in.
    dao = DocksDAO(geo_conn)
    for i in range(5):
        await dao.insert(f"d-{i}", f"桩{i}", 31.2, 121.5, 0.0,
                         31.2, 121.5, 0.0, f"h{i}", 0)
    with pytest.raises(Exception, match="dock quota"):
        await dao.insert("d-5", "桩5", 31.2, 121.5, 0.0,
                         31.2, 121.5, 0.0, "h5", 0)


# The old "16 waypoints per route" trigger was RETIRED by PLAN A (v1.5): route
# geometry is inline (routes.path_points), so the cap moved into commit_route
# (11 S7.8.3 <= 5000), covered by test_geo_commit::test_commit_route_rejects_over_cap.
# route_waypoint_assoc is now a proximity relation with no per-route cap.


# --- BIZ-P3-6 DAO layer basics ---

@pytest.mark.asyncio
async def test_memory_upsert_get(task_conn):
    m = MemoryDAO(task_conn)
    await m.put("k", b"v1", 0)
    await m.put("k", b"v2", 1)   # overwrite
    assert await m.get("k") == b"v2"
    assert await m.get("missing") is None


@pytest.mark.asyncio
async def test_snapshot_replace_writes_one_row_per_task(task_conn):
    """15 S9.3A: ONE row per task, WGS84 points as JSON.

    *** Rewritten 2026-09-28 with the table (docs/NEXT.md EX-2). The old body
    wrote (x_m, y_m, heading_rad) triples -- ENU metres, one row per point.
    cmd/motion/route is frame "wgs84" and P1 projects it itself, and the
    per-point shape had nowhere to keep route_id / rev / loop_mode /
    total_len_m / arclen, i.e. everything S7.3A's remap reads.

    MUTATION: split the INSERT so points_json and arclen_json are written by
    two statements -- SN-2 then no longer holds structurally, and a crash
    between them leaves a snapshot whose arc lengths belong to another route.
    That mismatch produces a WRONG L0 with no error at all, which is why
    S9.3A calls out the same-transaction requirement by name.
    """
    from xbrain.p3_task.route.snapshot_build import build_snapshot

    dao = TasksDAO(task_conn)
    await dao.insert(TaskRow(
        task_id="t9", task_type="patrol", state="pending",
        priority=1, submit_seq=1, mission_json="{}", total_steps=1,
        current_step=0, step_status_json="[]", created_ms=0, updated_ms=0,
        source="local", trace_id="tr", resume_policy="continue"))
    snap = SnapshotDAO(task_conn)
    built = build_snapshot(
        task_id="t9", route_id="r-gate", rev=4, loop_mode="closed",
        path_points=[[35.0, 135.0], [35.001, 135.0]],
        recorded_arrive_radius_m=1.0)
    await snap.replace(built, now_iso="2026-09-28T00:00:00.000Z")
    row = await snap.fetch("t9")
    assert row[0] == "r-gate" and row[1] == 4 and row[3] == 2
    # Replacing keeps it at one row -- the task_id PRIMARY KEY is what makes
    # "the snapshot" singular (11 S7.12.1 R1: the task runs the snapshot).
    built2 = build_snapshot(
        task_id="t9", route_id="r-gate", rev=5, loop_mode="closed",
        path_points=[[35.0, 135.0], [35.002, 135.0], [35.003, 135.0]],
        recorded_arrive_radius_m=1.0)
    await snap.replace(built2, now_iso="2026-09-28T00:01:00.000Z")
    cur = await task_conn.execute(
        "SELECT COUNT(*) FROM task_route_snapshot WHERE task_id='t9'")
    assert (await cur.fetchone())[0] == 1
    assert (await snap.fetch("t9"))[1] == 5


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


_LEGACY_PATROL_DDL = (
    "CREATE TABLE patrol_progress ("
    " task_id TEXT PRIMARY KEY, waypoint_ix INTEGER NOT NULL,"
    " progress REAL NOT NULL, updated_ms INTEGER NOT NULL)")


@pytest.mark.asyncio
async def test_legacy_patrol_progress_is_reshaped_when_empty():
    """An existing database keeps its old table through the DDL burst, because
    every statement is CREATE ... IF NOT EXISTS. ensure_patrol_progress_shape
    is what makes the EX-6 rebuild actually reach data/run/task.db.

    MUTATION: make ensure_patrol_progress_shape return False unconditionally
    (or run it AFTER the DDL) and this goes red on the missing route_rev --
    which, unguarded, is what the robot would hit at its first waypoint.
    """
    async with aiosqlite.connect(":memory:") as c:
        await c.execute(_LEGACY_PATROL_DDL)
        await c.commit()
        assert await ensure_patrol_progress_shape(c) is True
        await _apply(c, ALL_DDL_STATEMENTS)
        cur = await c.execute("PRAGMA table_info(patrol_progress)")
        cols = {r[1] for r in await cur.fetchall()}
        assert {"route_rev", "seg_done_m", "dist_done_m", "dir_sign"} <= cols
        assert "waypoint_ix" not in cols
        # Idempotent: a second open must not drop the table it just built.
        assert await ensure_patrol_progress_shape(c) is False


@pytest.mark.asyncio
async def test_legacy_patrol_progress_with_rows_raises_instead_of_dropping():
    """The drop is justified by "the legacy table had no writer, so it is
    empty". Where that reasoning does not hold, the code must stop rather than
    act on it -- losing a breakpoint silently is the failure U07a exists to
    prevent.

    MUTATION: drop the row count and DROP unconditionally; this goes red, and
    in the field it would delete a resume point with no trace.
    """
    async with aiosqlite.connect(":memory:") as c:
        await c.execute(_LEGACY_PATROL_DDL)
        await c.execute("INSERT INTO patrol_progress VALUES ('t1', 3, 0.5, 0)")
        await c.commit()
        with pytest.raises(PatrolProgressShapeConflict) as exc:
            await ensure_patrol_progress_shape(c)
        # The message must carry the row count and the columns found: "refusing
        # to drop it" without saying what is there is not actionable at 2am.
        assert "1 row" in str(exc.value) and "waypoint_ix" in str(exc.value)


@pytest.mark.asyncio
async def test_patrol_progress_start_then_update(task_conn):
    """15 S9.5 shape: open the active row, then flush progress onto it.

    *** Rewritten 2026-09-28 with the table (docs/NEXT.md EX-6). The old body
    called pp.upsert("t3", 2, 0.4, 0) -- waypoint_ix plus a derived 0..1
    `progress` fraction, two columns 15 S9.5 does not have and PP-3 forbids
    storing.

    MUTATION: drop the `AND status='active'` from update_progress and the
    UPDATE also rewrites completed historical runs, so a task that ran the
    same route twice reports the second run's position under the first run's
    row -- no error anywhere, just a wrong history.
    """
    dao = TasksDAO(task_conn)
    await dao.insert(TaskRow(
        task_id="t3", task_type="patrol", state="pending",
        priority=1, submit_seq=1, mission_json="{}", total_steps=1,
        current_step=0, step_status_json="[]", created_ms=0, updated_ms=0,
        source="local", trace_id="tr", resume_policy="continue"))
    pp = PatrolProgressDAO(task_conn)
    await pp.start_run(task_id="t3", route_name="east gate",
                       route_geo_id="r-east_gate", route_rev=7,
                       direction="forward", loop_mode="closed",
                       waypoint_total=64, route_total_m=1620.0, loop_total=2,
                       now_iso="2026-09-28T00:00:00.000Z")
    # A fresh row is at the breakpoint "no waypoint passed yet" (-1), not 0.
    assert (await pp.fetch_active("t3"))[4] == -1
    n = await pp.update_progress(
        "t3", waypoint_index=17, waypoint_total=64, seg_done_m=12.4,
        dist_done_m=431.8, odom_dist_m=440.0, loop_index=0, loop_total=2,
        skipped_m=0.0, dir_sign=1, now_iso="2026-09-28T00:00:05.000Z")
    assert n == 1
    got = await pp.fetch_active("t3")
    assert got == ("r-east_gate", 7, "forward", "closed", 17, 64, 12.4,
                   431.8, 440.0, 1620.0, 0, 2, 0.0, 1)


@pytest.mark.asyncio
async def test_patrol_progress_second_active_row_is_refused(task_conn):
    """idx_patrol_active (15 S9.5) is what makes "where is this task" have one
    answer. Two active rows would be resolved by whichever the query returned
    first, and U07a would resume from that one.

    MUTATION: drop the WHERE clause from idx_patrol_active (making it a plain
    unique index on task_id) and this raises on the SECOND run of any task
    instead -- the opposite failure, and equally silent to a reader who only
    runs one patrol.
    """
    dao = TasksDAO(task_conn)
    await dao.insert(TaskRow(
        task_id="t4", task_type="patrol", state="pending",
        priority=1, submit_seq=1, mission_json="{}", total_steps=1,
        current_step=0, step_status_json="[]", created_ms=0, updated_ms=0,
        source="local", trace_id="tr", resume_policy="continue"))
    pp = PatrolProgressDAO(task_conn)
    kw = dict(task_id="t4", route_name="r", route_geo_id="r-x", route_rev=1,
              direction="forward", loop_mode="oneway", waypoint_total=4,
              route_total_m=10.0, loop_total=1,
              now_iso="2026-09-28T00:00:00.000Z")
    await pp.start_run(**kw)
    # Commit before provoking the violation: the rollback below has to undo
    # the FAILED insert only. Without the commit it also undoes the first
    # row, and the test then passes for the wrong reason (there is no active
    # row left to conflict with, so close_run finds nothing).
    await task_conn.commit()
    with pytest.raises(aiosqlite.IntegrityError):
        await pp.start_run(**kw)
    await task_conn.rollback()
    # After the first run closes, a second run of the same task is legal --
    # that is why the index is partial.
    assert await pp.close_run("t4", "completed",
                              "2026-09-28T00:01:00.000Z") == 1
    await pp.start_run(**kw)


@pytest.mark.asyncio
async def test_patrol_progress_update_leaves_the_closed_run_alone(task_conn):
    """A flush must reach the ACTIVE row only. Written because the obvious
    `WHERE task_id=?` is wrong in a way nothing else here notices: with one
    run per task every assertion above still passes, and the damage only
    appears the second time a task runs the same route -- the historical row
    silently takes the new run's position.

    MUTATION: drop `AND status='active'` from update_progress and this goes
    red on the closed row's waypoint_index (5 -> 9).
    """
    dao = TasksDAO(task_conn)
    await dao.insert(TaskRow(
        task_id="t6", task_type="patrol", state="pending",
        priority=1, submit_seq=1, mission_json="{}", total_steps=1,
        current_step=0, step_status_json="[]", created_ms=0, updated_ms=0,
        source="local", trace_id="tr", resume_policy="continue"))
    pp = PatrolProgressDAO(task_conn)
    kw = dict(task_id="t6", route_name="r", route_geo_id="r-x", route_rev=1,
              direction="forward", loop_mode="oneway", waypoint_total=12,
              route_total_m=10.0, loop_total=1,
              now_iso="2026-09-28T00:00:00.000Z")
    await pp.start_run(**kw)
    await pp.update_progress(
        "t6", waypoint_index=5, waypoint_total=12, seg_done_m=1.0,
        dist_done_m=2.0, odom_dist_m=2.0, loop_index=0, loop_total=1,
        skipped_m=0.0, dir_sign=1, now_iso="2026-09-28T00:00:01.000Z")
    await pp.close_run("t6", "completed", "2026-09-28T00:00:02.000Z")
    # Second run of the same task, further along the same route.
    await pp.start_run(**kw)
    await pp.update_progress(
        "t6", waypoint_index=9, waypoint_total=12, seg_done_m=3.0,
        dist_done_m=8.0, odom_dist_m=8.0, loop_index=0, loop_total=1,
        skipped_m=0.0, dir_sign=1, now_iso="2026-09-28T00:00:03.000Z")
    cur = await task_conn.execute(
        "SELECT status, waypoint_index FROM patrol_progress"
        " WHERE task_id='t6' ORDER BY id")
    assert await cur.fetchall() == [("completed", 5), ("active", 9)]


@pytest.mark.asyncio
async def test_patrol_progress_unbounded_laps_survive_the_second_lap(task_conn):
    """loops = 0 is a standing patrol (11 S7.8.3 LM-4). The CHECK must bound
    loop_index only when loop_total != 0.

    MUTATION: write the CHECK as the naive `loop_index <= loop_total` and this
    raises when the standing patrol starts its second lap -- i.e. the robot
    stops being able to record progress about 20 minutes in, on the robot.
    """
    dao = TasksDAO(task_conn)
    await dao.insert(TaskRow(
        task_id="t5", task_type="patrol", state="pending",
        priority=1, submit_seq=1, mission_json="{}", total_steps=1,
        current_step=0, step_status_json="[]", created_ms=0, updated_ms=0,
        source="local", trace_id="tr", resume_policy="continue"))
    pp = PatrolProgressDAO(task_conn)
    await pp.start_run(task_id="t5", route_name="r", route_geo_id="r-x",
                       route_rev=1, direction="forward", loop_mode="closed",
                       waypoint_total=4, route_total_m=10.0, loop_total=0,
                       now_iso="2026-09-28T00:00:00.000Z")
    assert await pp.update_progress(
        "t5", waypoint_index=1, waypoint_total=4, seg_done_m=0.0,
        dist_done_m=12.0, odom_dist_m=12.0, loop_index=3, loop_total=0,
        skipped_m=0.0, dir_sign=1,
        now_iso="2026-09-28T00:05:00.000Z") == 1


@pytest.mark.asyncio
async def test_geo_object_rev_bump_and_idempotency(geo_conn):
    # v1.5: waypoints are WGS84 named keypoints keyed by geo_id (GeoObjectDAO
    # looks up by geo_id now, not the retired waypoint_id).
    await geo_conn.execute(
        "INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon, "
        " content_hash, updated_ms) VALUES ('w-1', '甲', 'poi', 31.2, 121.5, 'h1', 0)")
    dao = GeoObjectDAO(geo_conn, "waypoints")
    # Same hash -> no bump, same rev.
    assert await dao.bump_rev("w-1", "h1", 1) == 1
    # Different hash -> rev goes up by exactly 1.
    assert await dao.bump_rev("w-1", "h2", 2) == 2
    assert await dao.bump_rev("w-1", "h3", 3) == 3


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
    await dao.insert("f-1", "allow", "[]", "h", 0, name="活动区")
    await fence_conn.execute(
        "INSERT INTO fences (fence_id, name, role, kind, geom_json, content_hash, "
        " hard_enforce, updated_ms, tombstone) "
        "VALUES ('f-2', '禁区', 'forbid', 'polygon', '[]', 'h', 1, 0, 1)")
    rows = await dao.list_active()
    assert {r[0] for r in rows} == {"f-1"}
    # list_active row = (fence_id, name, role, kind, geom_json, hard_enforce, rev);
    # name + role carried out for the HMI/P1. MUTATION: wrong column order breaks
    # the FenceSet builder downstream.
    assert rows[0][1] == "活动区" and rows[0][2] == "allow"


@pytest.mark.asyncio
async def test_fence_total_quota_trigger_rejects_sixth(fence_conn):
    # 11 S9A.1: at most 5 active fences. MUTATION: dropping the trigger lets a 6th
    # active fence through. Use forbid for 2..5 (single-allow rule limits allow=1).
    dao = FencesDAO(fence_conn)
    await dao.insert("f-a", "allow", "[]", "h", 0)
    for i in range(4):
        await dao.insert(f"f-{i}", "forbid", "[]", "h", 0)
    with pytest.raises(Exception, match="fence quota"):
        await dao.insert("f-6", "forbid", "[]", "h", 0)


@pytest.mark.asyncio
async def test_fence_single_allow_trigger_rejects_second(fence_conn):
    # 11 S9A.1A: at most ONE active allow. MUTATION: dropping the trigger lets a
    # second activity area through (ambiguous keep-in).
    dao = FencesDAO(fence_conn)
    await dao.insert("f-a1", "allow", "[]", "h", 0)
    with pytest.raises(Exception, match="allow fence"):
        await dao.insert("f-a2", "allow", "[]", "h", 0)


def test_validate_active_fence_set_exactly_one_allow():
    from xbrain.p3_task.fence.geom import (
        InvalidFenceSet, validate_active_fence_set,
    )
    validate_active_fence_set(["allow", "forbid", "warning"])   # ok: 1 allow, 3 total
    # 0 allow (no activity area) and >= 2 allow both reject (FS-5A existence half,
    # which the per-row triggers cannot assert). MUTATION: only checking <= 1 allow
    # would let a set with NO allow go live.
    with pytest.raises(InvalidFenceSet, match="need exactly 1"):
        validate_active_fence_set(["forbid", "warning"])
    with pytest.raises(InvalidFenceSet, match="need exactly 1"):
        validate_active_fence_set(["allow", "allow"])
    with pytest.raises(InvalidFenceSet, match="max 5"):
        validate_active_fence_set(["allow", "f", "f", "f", "f", "f"])


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
