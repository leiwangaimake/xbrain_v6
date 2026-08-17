"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_record_dao.py
Brief: record.db DAO tests -- ch_seq density, dedup, need_ack union, cursor, JSONL degrade

Description:
Batch 1 of the event subsystem. Drives RecordDao against ONE in-memory aiosqlite
connection wrapped as all three roles (writer_normal / writer_full / reader): the
3-connection FS-d durability routing is a real-file integration concern (batch 7),
but every DAO RULE -- ch_seq density (SEQ-2), no-reset-on-restart (SEQ-3), dedup
merge consuming no ch_seq, the need_ack UNION, delivered rows leaving the backfill
scan, JSONL degrade on write failure -- is exercised here. Each load-bearing
assertion is paired with the mutation that turns it red (CLAUDE.md 3.3).
"""

import pytest
import pytest_asyncio
import aiosqlite

from xbrain.p5_gateway.persistence.base import RecordConn
from xbrain.p5_gateway.persistence.record_dao import RecordDao
from xbrain.p5_gateway.persistence.schema_record import (
    ALL_RECORD_STATEMENTS, SeqOrderViolation, advance_confirmed, need_ack,
)


pytestmark = pytest.mark.no_device


def _event(eid, *, channel="normal", sev="info", ts=100.0, dedup_key=None,
           dedup_window_s=None, cat="task", title="t"):
    """A minimal valid event dict. Only the fields the DAO reads are set; the
    optional snapshot columns default to absent (the DAO uses .get())."""
    ev = {
        "eid": eid, "rid": "m20s-001", "channel": channel, "sev": sev,
        "cat": cat, "title": title, "detail": {"k": "v"}, "src": "p3_task",
        "ts": ts, "ts_sync": 1, "detected_at": "2026-08-17 10:00:00",
        "created_at": "2026-08-17T02:00:00Z",
    }
    if dedup_key is not None:
        ev["dedup_key"] = dedup_key
        ev["dedup_window_s"] = dedup_window_s
    return ev


@pytest_asyncio.fixture
async def dao(tmp_path):
    """One :memory: connection playing all three roles. :memory: is per-connection
    so three real connections would be three DBs -- for LOGIC we share one."""
    # isolation_level=None to match production (base.py): the DAO owns its
    # transactions via explicit BEGIN IMMEDIATE.
    async with aiosqlite.connect(":memory:", isolation_level=None) as c:
        for stmt in ALL_RECORD_STATEMENTS:
            await c.execute(stmt)
        wn = RecordConn(role="writer_normal", path=":memory:", conn=c)
        wf = RecordConn(role="writer_full", path=":memory:", conn=c)
        rd = RecordConn(role="reader", path=":memory:", conn=c)
        yield RecordDao(wn, wf, rd, jsonl_path=str(tmp_path / "degrade.jsonl"))


# -- pure rules (no DB) -----------------------------------------------------

def test_need_ack_union_not_just_sev():
    # The whole point of the S3.3 union: an alarm-channel info event needs ack.
    assert need_ack("alarm", "info") is True          # MUTATION: sev-only -> False
    assert need_ack("normal", "fault") is True
    assert need_ack("normal", "info") is False
    assert need_ack("alarm", "alarm") is True


def test_need_ack_rejects_bad_closed_set():
    with pytest.raises(ValueError):
        need_ack("cloud", "info")
    with pytest.raises(ValueError):
        need_ack("normal", "critical")


def test_advance_confirmed_monotonic():
    assert advance_confirmed(5, 5) == 5     # idempotent ack
    assert advance_confirmed(5, 8) == 8
    with pytest.raises(SeqOrderViolation, match="rewind"):
        advance_confirmed(5, 3)             # MUTATION: allow rewind -> cloud re-drops


# -- ch_seq density (SEQ-2) --------------------------------------------------

@pytest.mark.asyncio
async def test_ch_seq_starts_at_one_and_is_dense(dao):
    r1 = await dao.insert_event(_event("e1"))
    r2 = await dao.insert_event(_event("e2"))
    r3 = await dao.insert_event(_event("e3"))
    assert (r1.status, r1.ch_seq) == ("inserted", 1)
    # MUTATION: an off-by-one or gap in _alloc_ch_seq breaks this dense run.
    assert (r2.ch_seq, r3.ch_seq) == (2, 3)


@pytest.mark.asyncio
async def test_ch_seq_is_per_channel(dao):
    a = await dao.insert_event(_event("n1", channel="normal"))
    b = await dao.insert_event(_event("a1", channel="alarm", sev="alarm"))
    c = await dao.insert_event(_event("n2", channel="normal"))
    # Each channel counts independently: normal 1,2 and alarm 1.
    assert (a.ch_seq, c.ch_seq) == (1, 2)
    assert b.ch_seq == 1


# -- dedup (S3.2) ------------------------------------------------------------

@pytest.mark.asyncio
async def test_dedup_merge_consumes_no_ch_seq(dao):
    r1 = await dao.insert_event(
        _event("d1", dedup_key="intrusion:42", dedup_window_s=300, ts=100.0))
    r2 = await dao.insert_event(
        _event("d2", dedup_key="intrusion:42", dedup_window_s=300, ts=110.0))
    r3 = await dao.insert_event(_event("d3"))     # different key -> new row
    assert r1.status == "inserted" and r1.ch_seq == 1
    assert r2.status == "merged"                  # MUTATION: new row -> "inserted"
    # ch_seq 2 goes to d3, NOT wasted on the merged d2 (merge consumes no seq).
    assert r3.ch_seq == 2
    # The merged event bumped dedup_count on the first row, added no new row.
    cur = await dao._rd.conn.execute("SELECT COUNT(*) FROM events")
    assert (await cur.fetchone())[0] == 2
    cur = await dao._rd.conn.execute(
        "SELECT dedup_count, last_ts FROM events WHERE eid = 'd1'")
    assert await cur.fetchone() == (2, 110.0)


@pytest.mark.asyncio
async def test_dedup_outside_window_is_new_row(dao):
    await dao.insert_event(
        _event("w1", dedup_key="k", dedup_window_s=5, ts=100.0))
    r2 = await dao.insert_event(
        _event("w2", dedup_key="k", dedup_window_s=5, ts=200.0))  # 100s > 5s
    # MUTATION: ignore the window -> this merges instead of inserting.
    assert r2.status == "inserted"


# -- need_ack frozen at insert ----------------------------------------------

@pytest.mark.asyncio
async def test_need_ack_written_by_union(dao):
    # alarm channel + info sev: the union says need_ack, sev-only would not.
    await dao.insert_event(_event("fr", channel="alarm", sev="info"))
    cur = await dao._rd.conn.execute(
        "SELECT need_ack FROM events WHERE eid = 'fr'")
    assert (await cur.fetchone())[0] == 1


# -- backfill scan + delivery -----------------------------------------------

@pytest.mark.asyncio
async def test_delivered_leaves_backlog(dao):
    await dao.insert_event(_event("b1", channel="alarm", sev="alarm"))
    await dao.insert_event(_event("b2", channel="alarm", sev="alarm"))
    before = await dao.backlog("alarm", limit=10)
    assert [e["eid"] for e in before] == ["b1", "b2"]
    n = await dao.mark_delivered(["b1"], batch="bf-1",
                                 delivered_at="2026-08-17T02:05:00Z")
    assert n == 1
    after = await dao.backlog("alarm", limit=10)
    # MUTATION: if delivered isn't set / the partial index ignores it, b1 stays.
    assert [e["eid"] for e in after] == ["b2"]


@pytest.mark.asyncio
async def test_confirmed_upto_advance_and_rewind(dao):
    await dao.insert_event(_event("c1", channel="alarm", sev="alarm"))
    upto = await dao.advance_confirmed_upto(
        "alarm", 1, backfill_at="2026-08-17T02:06:00Z")
    assert upto == 1
    cur = await dao.read_cursor("alarm")
    assert cur["confirmed_upto"] == 1
    with pytest.raises(SeqOrderViolation):
        await dao.advance_confirmed_upto("alarm", 0, backfill_at="x")


# -- SEQ-3: no reset across restart -----------------------------------------

@pytest.mark.asyncio
async def test_init_cursors_from_table_no_reset(dao):
    await dao.insert_event(_event("s1"))
    await dao.insert_event(_event("s2"))          # normal ch_seq now 1,2
    # Simulate restart: recompute next_ch_seq from the table.
    await dao.init_cursors_from_table()
    cur = await dao.read_cursor("normal")
    # MUTATION: reset to 1 -> next insert collides on (channel, ch_seq) UNIQUE.
    assert cur["next_ch_seq"] == 3
    r = await dao.insert_event(_event("s3"))
    assert r.ch_seq == 3


# -- JSONL degrade on DB failure (S-6) + rollback keeps SEQ-2 ----------------

@pytest.mark.asyncio
async def test_write_failure_degrades_to_jsonl_and_keeps_ch_seq(dao, tmp_path):
    import os

    await dao.insert_event(_event("g1"))          # ch_seq 1
    # Force a UNIQUE(eid) violation: reuse eid g1 with no dedup_key -> insert path
    # -> raises inside the tx -> rollback -> degrade to JSONL.
    r = await dao.insert_event(_event("g1"))
    assert r.status == "degraded"
    jsonl = dao._jsonl_path
    assert os.path.exists(jsonl)
    with open(jsonl, encoding="utf-8") as f:
        assert '"eid": "g1"' in f.read()
    # MUTATION: if the failed insert's ch_seq bump were NOT rolled back, g2 would
    # get ch_seq 3 (a gap). The rollback keeps it dense at 2.
    r2 = await dao.insert_event(_event("g2"))
    assert r2.ch_seq == 2
