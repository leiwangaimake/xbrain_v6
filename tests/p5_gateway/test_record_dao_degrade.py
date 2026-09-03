"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_record_dao_degrade.py
Brief: record.db insert -- ch_seq cursor desync self-heal + audible degrade

Description:
Observed on the robot, 2026-09-03: event_cursor.next_ch_seq for the `normal`
channel sat on a ch_seq the table already held. Every insert after that failed
UNIQUE(channel, ch_seq), degraded to the JSONL sidecar, and the cursor never
moved -- so the failure was permanent until p5 restarted (only
init_cursors_from_table recomputes it). Fifteen hours of events, the customer's
task audit trail among them, went to a file nobody was watching.

Two things made it that bad and both are covered here:
  * no recovery: the allocator returns the same used value forever;
  * no sound: the except swallowed the error without a single log line, so
    "persisting" and "dumping to a sidecar" looked identical at runtime.

Real sqlite (CLAUDE.md 7.2). A fake connection would return whatever the test
told it to and could not reproduce the UNIQUE violation that is the whole point.
"""

from __future__ import annotations

import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p5_gateway.persistence.base import RecordConn
from xbrain.p5_gateway.persistence.record_dao import RecordDao
from xbrain.p5_gateway.persistence.schema_record import ALL_RECORD_STATEMENTS

# INF-TS-1: 纯单测, 不碰设备(无 zenohd / 无底盘 / 无 ORIN 专属硬件).
pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def dao_conn(tmp_path):
    """Same shape as tests/p5_gateway/persistence/test_record_dao.py: one
    :memory: connection in all three roles, isolation_level=None so the DAO owns
    its transactions (the default level auto-begins one and the explicit
    BEGIN IMMEDIATE then deadlocks). Yields (dao, conn) because these cases have
    to corrupt the cursor behind the DAO's back."""
    async with aiosqlite.connect(":memory:", isolation_level=None) as c:
        for stmt in ALL_RECORD_STATEMENTS:
            await c.execute(stmt)
        wn = RecordConn(role="writer_normal", path=":memory:", conn=c)
        wf = RecordConn(role="writer_full", path=":memory:", conn=c)
        rd = RecordConn(role="reader", path=":memory:", conn=c)
        yield RecordDao(wn, wf, rd,
                        jsonl_path=str(tmp_path / "degrade.jsonl")), c


def _ev(eid, sev="info", cat="task", channel="normal"):
    return {"eid": eid, "rid": "gj-001", "channel": channel, "sev": sev,
            "cat": cat, "title": "t", "detail": {}, "src": "p3_task",
            "ts": 1.0, "ts_sync": 0, "detected_at": "2026-09-03 10:00:00",
            "created_at": "2026-09-03T02:00:00Z"}


async def _desync(conn, ch_seq, channel="normal"):
    """Put the cursor back on a ch_seq the table already holds -- the exact
    state found on the robot."""
    await conn.execute("UPDATE event_cursor SET next_ch_seq = ? "
                       "WHERE channel = ?", (ch_seq, channel))


@pytest.mark.asyncio
async def test_a_desynced_cursor_heals_instead_of_degrading_forever(dao_conn):
    """THE defect. Before the fix this returned 'degraded', and so did every
    event after it -- persistence was dead until p5 restarted.
    MUTATION: remove the resync+retry from insert_event -> red."""
    dao, conn = dao_conn
    r1 = await dao.insert_event(_ev("e-1"))
    assert r1.status == "inserted", r1
    await _desync(conn, r1.ch_seq)

    r2 = await dao.insert_event(_ev("e-2"))
    assert r2.status == "inserted", (
        "a desynced cursor still bricks persistence: %r" % (r2,))
    assert r2.ch_seq == r1.ch_seq + 1


@pytest.mark.asyncio
async def test_the_cursor_is_left_correct_for_the_next_event(dao_conn):
    """Healing one event is not enough: the invariant must hold afterwards, or
    every later event pays two attempts.
    MUTATION: retry without writing the cursor back -> red."""
    dao, conn = dao_conn
    r1 = await dao.insert_event(_ev("e-1"))
    await _desync(conn, r1.ch_seq)
    await dao.insert_event(_ev("e-2"))
    r3 = await dao.insert_event(_ev("e-3"))
    assert r3.status == "inserted"
    cur = await conn.execute("SELECT next_ch_seq FROM event_cursor "
                             "WHERE channel = 'normal'")
    nxt = (await cur.fetchone())[0]
    assert nxt == r3.ch_seq + 1, "cursor left behind at %r" % nxt


@pytest.mark.asyncio
async def test_a_recovered_insert_says_so_in_the_log(dao_conn, caplog):
    """A silent recovery hides a real fault (something desynced the cursor).
    MUTATION: drop the recovery warning -> red."""
    dao, conn = dao_conn
    r1 = await dao.insert_event(_ev("e-1"))
    await _desync(conn, r1.ch_seq)
    with caplog.at_level("WARNING"):
        await dao.insert_event(_ev("e-2"))
    assert any("resync" in r.getMessage() for r in caplog.records), (
        "the cursor was repaired without saying so")


@pytest.mark.asyncio
async def test_a_real_degrade_is_logged_not_silent(dao_conn, caplog):
    """*** The reason this went unnoticed for fifteen hours.

    A duplicate eid cannot be healed by a resync, so it degrades -- correctly.
    What was wrong is that it degraded WITHOUT A WORD, making "persisting" and
    "dumping to a sidecar file" indistinguishable at runtime.
    MUTATION: remove the degrade warning -> red.
    """
    dao, _conn = dao_conn
    await dao.insert_event(_ev("dup"))
    with caplog.at_level("WARNING"):
        r = await dao.insert_event(_ev("dup"))       # UNIQUE(eid)
    assert r.status == "degraded"
    msgs = [x.getMessage() for x in caplog.records]
    assert any("DEGRADED" in m for m in msgs), (
        "an event went to the JSONL sidecar silently: %r" % msgs)
    assert any("dup" in m for m in msgs), "the log does not name the lost eid"


@pytest.mark.asyncio
async def test_the_degraded_event_still_reaches_the_sidecar(dao_conn, tmp_path):
    """Logging must not replace the sidecar: the event is still evidence.
    MUTATION: log instead of writing the JSONL -> red."""
    dao, _conn = dao_conn
    await dao.insert_event(_ev("dup"))
    await dao.insert_event(_ev("dup"))
    text = (tmp_path / "degrade.jsonl").read_text(encoding="utf-8").strip()
    assert text, "the degraded event was not written to the sidecar"
    assert json.loads(text.splitlines()[-1])["eid"] == "dup"


@pytest.mark.asyncio
async def test_a_healthy_insert_neither_retries_nor_logs(dao_conn, caplog):
    """The healthy path must stay quiet -- a warning that fires on every event
    stops meaning anything.
    MUTATION: always resync before inserting -> red."""
    dao, _conn = dao_conn
    with caplog.at_level("WARNING"):
        for i in range(3):
            r = await dao.insert_event(_ev("e-%d" % i))
            assert r.status == "inserted"
    assert not caplog.records, [x.getMessage() for x in caplog.records]


# --------------------------------------- 并发: 同一连接上两条事件不能互相吃掉

@pytest.mark.asyncio
async def test_two_events_in_the_same_tick_both_land(dao_conn):
    """*** 实测丢过事件的那个竞态.

    三个 RecordConn 角色共享底层连接, 而每个写方法自己 BEGIN IMMEDIATE. 两条
    事件在同一毫秒到达时(accepted 与 started 就是), 两个协程在同一连接上交错:
    A 开了事务并在 await 处让出, B 也来 BEGIN -> "cannot start a transaction
    within a transaction"; 而 B 的 rollback 回滚的是[共享连接上的那个事务],
    把 A 已经写好的行一起带走.

    2026-09-03 真机: task-evt-reject-1 的 accepted 事件既不在 record.db 也不在
    JSONL 旁路里 -- 彻底丢失, 同一毫秒的 started 在. 游标脱节(此前只观察到
    症状, 没定位)也是同一个根因: A 的游标 UPDATE 被 B 的 rollback 带走, 而它
    的 INSERT 已经落了.

    MUTATION: 去掉 insert_event 的 _tx_lock -> 红(其中一条丢失或状态变成
    degraded).
    """
    import asyncio as _a

    dao, conn = dao_conn
    results = await _a.gather(dao.insert_event(_ev("e-a")),
                              dao.insert_event(_ev("e-b")))
    assert [r.status for r in results] == ["inserted", "inserted"], results

    cur = await conn.execute("SELECT eid FROM events ORDER BY eid")
    got = sorted(r[0] for r in await cur.fetchall())
    assert got == ["e-a", "e-b"], "并发写丢了事件: %r" % got
    # ch_seq 必须密集且不重号 -- 丢一条或撞一条都会在这里露出来.
    cur = await conn.execute("SELECT ch_seq FROM events ORDER BY ch_seq")
    seqs = [r[0] for r in await cur.fetchall()]
    assert seqs == list(range(1, len(seqs) + 1)), seqs


@pytest.mark.asyncio
async def test_a_burst_of_events_keeps_the_cursor_dense(dao_conn):
    """告警风暴时事件是成批来的. 密集不重号是补发游标能用的前提(SEQ-2).

    MUTATION: 去掉锁 -> 红(丢号 / 撞号 / 整批降级).
    """
    import asyncio as _a

    dao, conn = dao_conn
    n = 12
    results = await _a.gather(*[dao.insert_event(_ev("e-%02d" % i))
                                for i in range(n)])
    assert all(r.status == "inserted" for r in results), \
        [r.status for r in results]
    cur = await conn.execute("SELECT ch_seq FROM events ORDER BY ch_seq")
    seqs = [r[0] for r in await cur.fetchall()]
    assert seqs == list(range(1, n + 1)), seqs


@pytest.mark.asyncio
async def test_delivery_marking_does_not_race_with_inserts(dao_conn):
    """补发标记走的是同一个连接上的另一个事务. 与入库并发时同样会互相回滚.

    MUTATION: 去掉 mark_delivered 的 _tx_lock -> 红.
    """
    import asyncio as _a

    dao, conn = dao_conn
    await dao.insert_event(_ev("e-old"))
    await _a.gather(dao.insert_event(_ev("e-new")),
                    dao.mark_delivered(["e-old"], "bf-1", "2026-09-03T00:00:00Z"))
    cur = await conn.execute("SELECT eid, delivered FROM events ORDER BY eid")
    rows = {r[0]: r[1] for r in await cur.fetchall()}
    assert set(rows) == {"e-old", "e-new"}, rows
    assert rows["e-old"] == 1, "投递标记被并发写吃掉了"
