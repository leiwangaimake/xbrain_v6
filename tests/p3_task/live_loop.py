"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: live_loop.py
Brief: run the REAL p3_task main loop in-process over fake Zenoh planes (test helper, not a test)

Description:
Why this exists. The 2026-11-06 audit found two P3 defects that every unit
test had passed over, because each lived in the WIRING, not in a pure
function: a cancel never reached P1 (no stop frame was ever published) and a
dead `running` row blocked dispatch after a restart (start-up never looked).
Only driving xbrain.p3_task.runtime.main_wiring._amain itself -- its real
subscribers, publishers, scheduler tick and push paths -- can see either.

What it provides:
  * Harness   -- the planes object _amain opens: records every publisher
                 frame per key and every subscriber callback per key, and
                 delivers a frame to a subscriber as a Zenoh sample would;
  * wait_for  -- poll a condition on the loop (the loop runs concurrently);
  * start_p3  -- seed geo.db (+ optional task rows) and start _amain, return
                 the harness, the stop flag and the loop task;
  * goto_row  -- a minimal valid goto TaskRow;
  * task_state / route_ops -- read back a task's committed state and the op
                 of every cmd/motion/route frame in order.

What it does NOT do: no real Zenoh, no P1, no clock control. Timings are the
loop's own (0.5 s pass, 3 s ack window), so a test that needs the ack window
to expire really waits for it.

Trap: the stop flag must be set and the task awaited in a finally block, or
a failing assertion leaves _amain running and the next test's planes patch
lands on a loop that is still using the previous one.
"""
from __future__ import annotations  # postponed annotations

import asyncio  # the loop runs as a task beside the test
import json  # frames on the fake wire are JSON bytes, as on the real one

import aiosqlite  # seed / read back the same files _amain opens

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO  # seed rows through the DAO
from xbrain.p3_task.persistence.schema_geo import GEO_DB_STATEMENTS  # geo.db DDL
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task.db DDL

#: The one waypoint every scenario drives to: ~111 m north of the robot, so a
#: push always has a non-zero leg (a zero-length goto is refused by design).
WAYPOINT_SQL = ("INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon,"
                " arrival_radius, rev, content_hash, updated_ms)"
                " VALUES ('w-gate', 'east gate', 'poi', 35.001, 135.0, 1.0, 2, 'h', 0)")
#: Where the robot is (state/pose body as P1 publishes it).
ROBOT_POSE = {"data": {"lat": 35.0, "lon": 135.0}}


class Harness:
    """The fake planes: one session object serves both planes, as the code
    under test only needs declare_* and put."""

    def __init__(self) -> None:
        self.subs = {}  # key -> the callback _amain declared
        self.pubs = {}  # key -> every payload put on it, in order
        harness = self  # captured by the nested classes below

        class _Pub:  # what declare_publisher returns
            def __init__(self, key):
                self.key = key  # the key this publisher puts on
                harness.pubs.setdefault(key, [])  # declared == listed, even if silent

            def put(self, payload):
                harness.pubs[self.key].append(payload)  # record, in send order

        class _Session:  # what open_planes yields
            def declare_subscriber(self, key, cb):
                harness.subs[key] = cb  # the test delivers to it via send()
                return object()  # a handle the code keeps alive (CLAUDE.md 4.3)

            def declare_publisher(self, key):
                return _Pub(key)  # recorded publisher

            def declare_queryable(self, key, cb):
                harness.subs[key] = cb  # treated like a subscriber here
                return object()  # handle

            def put(self, key, payload):
                harness.pubs.setdefault(key, []).append(payload)  # session-level put

        class _Planes:  # the context manager open_planes returns
            def __enter__(self):
                return _Session()  # both planes share it

            def __exit__(self, *_a):
                return False  # never swallow an exception from the loop

        self.planes = _Planes  # handed to the monkeypatched open_planes

    def send(self, key: str, body) -> None:
        """Deliver one JSON body to key's subscriber, shaped like a sample."""
        class _Sample:  # only .payload is read by P3's callbacks
            payload = json.dumps(body).encode("utf-8")  # bytes, as Zenoh delivers
        self.subs[key](_Sample())  # runs on the test's thread, like the Rust pool would

    def frames(self, key: str) -> list:
        """Every payload published on key so far, decoded."""
        return [json.loads(p.decode("utf-8")) for p in self.pubs.get(key, [])]  # in order


async def wait_for(cond, n: int = 400) -> bool:
    """Poll cond() every 20 ms, up to n times; True once it holds."""
    for _ in range(n):  # bounded: a hung loop fails the test, not the run
        if cond():  # checked before sleeping, so an already-true cond is instant
            return True  # condition met
        await asyncio.sleep(0.02)  # yield to the loop under test
    return False  # timed out; the caller's assert reports what it saw


def goto_row(task_id: str, state: str, seq: int, priority: int = 5, **extra) -> TaskRow:
    """A goto to w-gate in the given state (extra: suspend_kind / reason)."""
    return TaskRow(task_id=task_id, task_type="goto", state=state, priority=priority,
                   submit_seq=seq,  # queue order among equal priorities
                   mission_json=json.dumps({"params": {"waypoint_id": "w-gate"}}),  # the target
                   total_steps=1, current_step=0, step_status_json="[]",  # one-step task
                   created_ms=0, updated_ms=0, source="local", trace_id="tr-" + task_id,  # audit
                   resume_policy="continue", **extra)  # 15 S7.5 default for goto


async def start_p3(tmp_path, monkeypatch, rows=(), geo_sql=(), **amain_kw):
    """Start the real loop over the fake planes; return (harness, stop, task,
    task_db). rows are written to task.db BEFORE the loop starts (what a power
    cut leaves behind); the robot pose is delivered once subscribers exist.
    amain_kw reach _amain unchanged (e.g. the rtk_loss sustain pair);
    geo_sql are extra (sql, params) rows for geo.db (routes, docks, home)."""
    from xbrain.p3_task.runtime import main_wiring as mw  # the code under test
    import xbrain.common.runtime.session_ctx as sctx  # where _amain gets its planes
    h = Harness()  # fresh planes per scenario
    monkeypatch.setattr(sctx, "open_planes", lambda _p: h.planes())  # no real Zenoh
    task_db, geo_db = str(tmp_path / "task.db"), str(tmp_path / "geo.db")  # per-test files
    async with aiosqlite.connect(geo_db) as g:  # the catalogue: one waypoint
        for stmt in GEO_DB_STATEMENTS:  # same DDL _amain would apply
            await g.execute(stmt)  # create
        await g.execute(WAYPOINT_SQL)  # w-gate
        for sql, params in geo_sql:  # extra catalogue rows for the scenario
            await g.execute(sql, params)  # as the test gives them
        await g.commit()  # visible to _amain's own connection
    if rows:  # pre-existing tasks: create the schema first so the DAO can insert
        async with aiosqlite.connect(task_db) as t:  # same file _amain opens
            for stmt in ALL_DDL_STATEMENTS:  # idempotent DDL (_amain re-runs it)
                await t.execute(stmt)  # create
            for row in rows:  # in the given order
                await TasksDAO(t).insert(row)  # through the DAO, as P3 writes them
            await t.commit()  # durable before the loop reads
    stop = {"stop": False}  # _amain polls this every pass
    loop = asyncio.create_task(mw._amain(stop, 99.0, task_db, geo_db_path=geo_db,
                                         fence_db_path=str(tmp_path / "fence.db"),
                                         **amain_kw))  # run it
    ready = await wait_for(lambda: mw.CMD_TASK_TOPIC in h.subs and mw.STATE_POSE_TOPIC in h.subs)
    assert ready, "p3 loop never declared its subscribers"  # start-up failed
    h.send(mw.STATE_POSE_TOPIC, ROBOT_POSE)  # a push needs the robot's position
    return h, stop, loop, task_db  # the caller stops it in a finally block


async def insert_rows(task_db: str, *rows: TaskRow) -> None:
    """Add tasks while the loop runs (a new submit seen by the next tick)."""
    async with aiosqlite.connect(task_db) as t:  # a second connection, as HMI would be
        for row in rows:  # in order
            await TasksDAO(t).insert(row)  # through the DAO
        await t.commit()  # the tick reads committed rows only


async def stop_p3(stop: dict, loop) -> None:
    """Stop the loop and wait for it (always from a finally block)."""
    stop["stop"] = True  # the loop exits at its next pass
    await asyncio.wait_for(loop, timeout=10.0)  # and its exception, if any, surfaces here


async def task_state(task_db: str, task_id: str):
    """(state, suspend_kind, suspend_reason) as committed, on a fresh connection."""
    async with aiosqlite.connect(task_db) as t:  # fresh: sees only committed rows
        cur = await t.execute("SELECT state, suspend_kind, suspend_reason FROM tasks"
                              " WHERE task_id=?", (task_id,))  # one row
        return await cur.fetchone()  # None when absent


def route_ops(h: Harness) -> list:
    """The op of every cmd/motion/route frame, in publish order."""
    return [f.get("op") for f in h.frames("cmd/motion/route")]  # "set" / "clear"
