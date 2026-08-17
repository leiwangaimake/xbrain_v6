"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_event_subsystem.py
Brief: EventSubsystem sync/async bridge -- persist, deliver, degrade (batch 5)

Description:
Drives the SYNC facade end to end against a real file record.db (three connections
+ WAL need a file, not :memory:). Verifies that a submitted event is persisted on
the event-loop thread and that need_ack=0-connected is marked delivered while
need_ack=1 is not, and that a bad db path degrades to a no-op that never raises
(the voice loop must survive a broken event store). Reads are done from a separate
aiosqlite reader on the same WAL file. Mutations paired per 3.3.
"""

import asyncio

import aiosqlite
import pytest

from xbrain.p5_gateway.runtime.event_subsystem import EventSubsystem


pytestmark = pytest.mark.no_device


def _now_iso():
    return "2026-08-17T02:05:00Z"


def _now_mono():
    return 0.0


def _ev(eid, *, channel="normal", sev="info", cat="task"):
    return {
        "eid": eid, "rid": "m20s-001", "channel": channel, "sev": sev,
        "cat": cat, "title": "t", "detail": {"k": eid}, "src": "p3_task",
        "ts": 100.0, "ts_sync": 1, "detected_at": "2026-08-17 10:00:00",
        "created_at": "2026-08-17T02:00:00Z",
    }


def _query(db_path, sql, args=()):
    """Read from a fresh aiosqlite reader on the same WAL file (a committed write
    from the subsystem's writer is visible to a new reader)."""
    async def q():
        async with aiosqlite.connect(db_path, isolation_level=None) as c:
            cur = await c.execute(sql, args)
            return await cur.fetchall()
    return asyncio.run(q())


def test_persists_and_marks_need_ack0_delivered(tmp_path):
    db = str(tmp_path / "record.db")
    subs = EventSubsystem("m20s-001", db, str(tmp_path / "d.jsonl"),
                          _now_iso, _now_mono)
    assert subs.start() is True
    try:
        fut = subs.submit_event(_ev("e1"), link_connected=True)
        fut.result(timeout=3)   # wait for the persist coroutine
        rows = _query(db, "SELECT eid, delivered FROM events WHERE eid='e1'")
        assert rows == [("e1", 1)]   # persisted AND delivered (need_ack=0 + up)
    finally:
        subs.stop()


def test_need_ack0_disconnected_persisted_not_delivered(tmp_path):
    db = str(tmp_path / "record.db")
    subs = EventSubsystem("m20s-001", db, str(tmp_path / "d.jsonl"),
                          _now_iso, _now_mono)
    assert subs.start() is True
    try:
        subs.submit_event(_ev("e1"), link_connected=False).result(timeout=3)
        rows = _query(db, "SELECT delivered FROM events WHERE eid='e1'")
        # MUTATION: mark delivered regardless of link -> this would be 1, and a
        # disconnected event would never reach the backfill queue.
        assert rows == [(0,)]
    finally:
        subs.stop()


def test_need_ack1_persisted_awaits_ack(tmp_path):
    db = str(tmp_path / "record.db")
    subs = EventSubsystem("m20s-001", db, str(tmp_path / "d.jsonl"),
                          _now_iso, _now_mono)
    assert subs.start() is True
    try:
        subs.submit_event(_ev("a1", channel="alarm", sev="alarm"),
                          link_connected=True).result(timeout=3)
        rows = _query(db, "SELECT delivered FROM events WHERE eid='a1'")
        assert rows == [(0,)]        # persisted, not delivered until acked
        # An ack marks it.
        subs.submit_ack("a1", "accepted")
        # submit_ack is fire-and-forget; poll briefly for the mark.
        import time
        for _ in range(30):
            r = _query(db, "SELECT delivered FROM events WHERE eid='a1'")
            if r == [(1,)]:
                break
            time.sleep(0.05)
        assert _query(db, "SELECT delivered FROM events WHERE eid='a1'") == [(1,)]
    finally:
        subs.stop()


def test_bad_path_degrades_to_noop(tmp_path):
    # A file where a directory is needed -> makedirs fails -> init fails -> the
    # subsystem stays DISABLED and submit is a silent no-op (never raises).
    blocker = tmp_path / "blocker"
    blocker.write_text("x")
    subs = EventSubsystem("m20s-001", str(blocker / "record.db"),
                          str(tmp_path / "d.jsonl"), _now_iso, _now_mono)
    assert subs.start() is False
    assert subs.enabled is False
    # No-op, returns None, does not raise (the voice loop survives).
    assert subs.submit_event(_ev("e1"), True) is None
    subs.submit_ack("e1", "accepted")
    subs.trigger_backfill()
    subs.stop()
