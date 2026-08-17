"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud.py
Brief: cloud uplink tests -- delivery mark, ack tracking, backfill runner (batch 4)

Description:
Drives DeliveryMarker / AckTracker / BackfillRunner against a REAL in-memory
RecordDao (so marking actually leaves the backfill index) plus a fake publisher
and a synthetic monotonic clock. Verifies the S3.5.1 delivery judgment, that an
ack marks an eid, and that a backfill pass emits begin/items/end, drains 4:1,
marks need_ack=0 delivered on send but leaves need_ack=1 for its ack, replays on
event/replay/{channel} (never the live key), and paces through the rate limiter.
Each load-bearing assertion is paired with the mutation that reddens it (3.3).
"""

import pytest
import pytest_asyncio
import aiosqlite

from xbrain.p5_gateway.persistence.base import RecordConn
from xbrain.p5_gateway.persistence.record_dao import RecordDao
from xbrain.p5_gateway.persistence.schema_record import ALL_RECORD_STATEMENTS
from xbrain.p5_gateway.reconnect.replay import RateLimiter
from xbrain.p5_gateway.uplink.cloud import (
    AckTracker, BackfillRunner, DeliveryMarker, ReconRunner, replay_key,
)


pytestmark = pytest.mark.no_device

RID = "m20s-001"


def _event(eid, *, channel, sev, cat="task", title="t"):
    return {
        "eid": eid, "rid": RID, "channel": channel, "sev": sev, "cat": cat,
        "title": title, "detail": {"k": eid}, "src": "p3_task",
        "ts": 100.0, "ts_sync": 1, "detected_at": "2026-08-17 10:00:00",
        "created_at": "2026-08-17T02:00:00Z",
    }


@pytest_asyncio.fixture
async def dao():
    async with aiosqlite.connect(":memory:", isolation_level=None) as c:
        for stmt in ALL_RECORD_STATEMENTS:
            await c.execute(stmt)
        wn = RecordConn(role="writer_normal", path=":memory:", conn=c)
        wf = RecordConn(role="writer_full", path=":memory:", conn=c)
        rd = RecordConn(role="reader", path=":memory:", conn=c)
        yield RecordDao(wn, wf, rd, jsonl_path="/tmp/xbrain_test_degrade.jsonl")


def _now_iso():
    return "2026-08-17T02:05:00Z"


class _FakePub:
    """Records every (key, data) put -- stands in for a Zenoh publisher."""

    def __init__(self):
        self.puts = []

    async def __call__(self, key, data):
        self.puts.append((key, data))


# --- DeliveryMarker (S3.5.1) ---

@pytest.mark.asyncio
async def test_need_ack0_connected_marks_delivered(dao):
    await dao.insert_event(_event("n1", channel="normal", sev="info"))
    dm = DeliveryMarker(dao, _now_iso)
    marked = await dm.after_persist("n1", need_ack=False, link_connected=True)
    assert marked is True
    assert await dao.backlog("normal", 10) == []   # left the backfill index


@pytest.mark.asyncio
async def test_need_ack0_disconnected_not_marked(dao):
    await dao.insert_event(_event("n1", channel="normal", sev="info"))
    dm = DeliveryMarker(dao, _now_iso)
    # MUTATION: mark regardless of link -> a disconnected event is wrongly
    # dropped from backfill and lost.
    marked = await dm.after_persist("n1", need_ack=False, link_connected=False)
    assert marked is False
    assert len(await dao.backlog("normal", 10)) == 1


@pytest.mark.asyncio
async def test_need_ack1_never_marked_at_persist(dao):
    await dao.insert_event(_event("a1", channel="alarm", sev="alarm"))
    dm = DeliveryMarker(dao, _now_iso)
    marked = await dm.after_persist("a1", need_ack=True, link_connected=True)
    assert marked is False


# --- AckTracker (S3.5.1) ---

@pytest.mark.asyncio
async def test_ack_ok_marks_delivered(dao):
    # 11 S8.4 result closed set is {ok, duplicate}. "ok" is the PRIMARY success
    # value the cloud sends. MUTATION (audit F9): if AckTracker used the command-
    # Ack model {accepted, ...}, "ok" would fall through and the event would never
    # be marked delivered -> re-sent forever.
    await dao.insert_event(_event("a1", channel="alarm", sev="alarm"))
    at = AckTracker(dao, _now_iso)
    assert await at.on_ack("a1", "ok") is True
    assert await dao.backlog("alarm", 10) == []


@pytest.mark.asyncio
async def test_ack_duplicate_marks_delivered(dao):
    # duplicate = the cloud already had this eid (idempotent re-delivery, S8.4).
    await dao.insert_event(_event("a1", channel="alarm", sev="alarm"))
    at = AckTracker(dao, _now_iso)
    assert await at.on_ack("a1", "duplicate") is True
    assert await dao.backlog("alarm", 10) == []


@pytest.mark.asyncio
async def test_ack_offcontract_result_does_not_mark(dao):
    await dao.insert_event(_event("a1", channel="alarm", sev="alarm"))
    at = AckTracker(dao, _now_iso)
    # A value outside {ok, duplicate} is NOT an ack -> leave delivered=0 for
    # re-send. MUTATION: mark on any result -> a non-ack would drop the event.
    assert await at.on_ack("a1", "rejected") is False
    assert len(await dao.backlog("alarm", 10)) == 1


# --- BackfillRunner (S3.5.2) ---

@pytest_asyncio.fixture
async def seeded(dao):
    # 2 normal/info (need_ack=0) + 2 alarm/alarm (need_ack=1).
    await dao.insert_event(_event("n1", channel="normal", sev="info"))
    await dao.insert_event(_event("n2", channel="normal", sev="info"))
    await dao.insert_event(_event("a1", channel="alarm", sev="alarm"))
    await dao.insert_event(_event("a2", channel="alarm", sev="alarm"))
    return dao


@pytest.mark.asyncio
async def test_backfill_emits_begin_items_end_on_replay_key(seeded):
    pub = _FakePub()
    clock = [0.0]
    async def sleep(s):
        clock[0] += s
    runner = BackfillRunner(seeded, RID, pub, RateLimiter(20), _now_iso)
    res = await runner.run(now_mono=lambda: clock[0], sleep=sleep)

    kinds = [d["kind"] for _, d in pub.puts]
    assert kinds.count("begin") == 2 and kinds.count("end") == 2
    assert kinds.count("item") == 4
    # R-3: everything went on event/replay/{channel}, NEVER the live key.
    assert all("/event/replay/" in k for k, _ in pub.puts)
    assert res["begins"] == 2 and res["ends"] == 2


@pytest.mark.asyncio
async def test_backfill_marks_need_ack0_but_leaves_need_ack1(seeded):
    pub = _FakePub()
    clock = [0.0]
    async def sleep(s):
        clock[0] += s
    runner = BackfillRunner(seeded, RID, pub, RateLimiter(20), _now_iso)
    await runner.run(now_mono=lambda: clock[0], sleep=sleep)
    # normal (need_ack=0) delivered on send -> gone; alarm (need_ack=1) waits ack.
    assert await seeded.backlog("normal", 10) == []
    # MUTATION: if the runner marked need_ack=1 too, the alarm would vanish before
    # any ack -- unacked loss.
    assert len(await seeded.backlog("alarm", 10)) == 2


@pytest.mark.asyncio
async def test_backfill_items_are_four_to_one(dao):
    # 8 alarm + 2 normal -> first five item channels are 4 alarm then 1 normal.
    for i in range(8):
        await dao.insert_event(_event(f"a{i}", channel="alarm", sev="alarm"))
    for i in range(2):
        await dao.insert_event(_event(f"n{i}", channel="normal", sev="info"))
    pub = _FakePub()
    clock = [0.0]
    async def sleep(s):
        clock[0] += s
    runner = BackfillRunner(dao, RID, pub, RateLimiter(20), _now_iso)
    await runner.run(now_mono=lambda: clock[0], sleep=sleep)
    item_chs = [d["channel"] for k, d in pub.puts if d["kind"] == "item"]
    assert item_chs[:5] == ["alarm", "alarm", "alarm", "alarm", "normal"]


@pytest.mark.asyncio
async def test_backfill_paces_through_rate_limiter(dao):
    # rate 2/s, 6 items -> the bucket empties and the runner must sleep + let the
    # clock advance. MUTATION: no rate limiting -> sleep never called.
    for i in range(6):
        await dao.insert_event(_event(f"a{i}", channel="alarm", sev="alarm"))
    pub = _FakePub()
    clock = [0.0]
    slept = []
    async def sleep(s):
        slept.append(s)
        clock[0] += s
    runner = BackfillRunner(dao, RID, pub, RateLimiter(2, capacity=2), _now_iso)
    await runner.run(now_mono=lambda: clock[0], sleep=sleep)
    assert len(slept) >= 1 and clock[0] > 0.0
    # All six still sent despite pacing.
    assert sum(1 for k, d in pub.puts if d["kind"] == "item") == 6


def test_replay_key_shape():
    assert replay_key("m20s-001", "alarm") == "xbrain/m20s-001/event/replay/alarm"


# --- ReconRunner (S3Y.3) ---

async def _noop_sleep(_s):
    return None


@pytest.mark.asyncio
async def test_recon_build_reqs_bounds(dao):
    # alarm holds ch_seq 1..3, normal holds 1..2.
    for i in range(3):
        await dao.insert_event(_event(f"a{i}", channel="alarm", sev="alarm"))
    for i in range(2):
        await dao.insert_event(_event(f"n{i}", channel="normal", sev="info"))
    rr = ReconRunner(dao, RID, _FakePub(), RateLimiter(1000.0), _now_iso)
    reqs = await rr.build_reqs(now_mono=lambda: 1.0)
    by_ch = {r["channel"]: r for r in reqs}
    assert by_ch["alarm"]["my_max_seq"] == 3 and by_ch["alarm"]["my_min_seq"] == 1
    assert by_ch["normal"]["my_max_seq"] == 2
    assert by_ch["alarm"]["req_id"] == "rc-1000-alarm"


@pytest.mark.asyncio
async def test_recon_rsp_resends_gap(dao):
    for i in range(3):                 # alarm ch_seq 1,2,3 (need_ack=1)
        await dao.insert_event(_event(f"a{i}", channel="alarm", sev="alarm"))
    pub = _FakePub()
    rr = ReconRunner(dao, RID, pub, RateLimiter(1000.0), _now_iso)
    await rr.build_reqs(now_mono=lambda: 1.0)          # req_id rc-1000-alarm
    res = await rr.on_rsp(
        {"req_id": "rc-1000-alarm", "channel": "alarm", "their_max_seq": 1},
        now_mono=lambda: 2.0, sleep=_noop_sleep)
    assert res["resent"] == 2                           # ch_seq 2,3
    kinds = [d["kind"] for _k, d in pub.puts]
    assert kinds == ["begin", "item", "item", "end"]
    keys = {k for k, _d in pub.puts}
    assert keys == {replay_key(RID, "alarm")}           # RC-1/R-3: replay key only
    # batch id prefixed rc- (RC-2), NOT bf-.
    assert all(d["batch_id"].startswith("rc-") for _k, d in pub.puts)


@pytest.mark.asyncio
async def test_recon_rsp_req_id_mismatch_discarded(dao):
    await dao.insert_event(_event("a0", channel="alarm", sev="alarm"))
    pub = _FakePub()
    rr = ReconRunner(dao, RID, pub, RateLimiter(1000.0), _now_iso)
    await rr.build_reqs(now_mono=lambda: 1.0)          # stores rc-1000-alarm
    # MUTATION: acting on a stale/foreign req_id lets a late old rsp drive a resend.
    res = await rr.on_rsp(
        {"req_id": "rc-999-alarm", "channel": "alarm", "their_max_seq": 0},
        now_mono=lambda: 2.0, sleep=_noop_sleep)
    assert res["discarded"] == "req_id_mismatch"
    assert pub.puts == []


@pytest.mark.asyncio
async def test_recon_rsp_nothing_missing(dao):
    await dao.insert_event(_event("a0", channel="alarm", sev="alarm"))  # ch_seq 1
    pub = _FakePub()
    rr = ReconRunner(dao, RID, pub, RateLimiter(1000.0), _now_iso)
    await rr.build_reqs(now_mono=lambda: 1.0)
    res = await rr.on_rsp(
        {"req_id": "rc-1000-alarm", "channel": "alarm", "their_max_seq": 1},
        now_mono=lambda: 2.0, sleep=_noop_sleep)
    assert res["resent"] == 0
    assert pub.puts == []


@pytest.mark.asyncio
async def test_recon_resend_marks_need_ack0_delivered(dao):
    # normal/info = need_ack=0: a recon resend is its delivery (like backfill).
    for i in range(2):
        await dao.insert_event(_event(f"n{i}", channel="normal", sev="info"))
    pub = _FakePub()
    rr = ReconRunner(dao, RID, pub, RateLimiter(1000.0), _now_iso)
    await rr.build_reqs(now_mono=lambda: 1.0)          # rc-1000-normal
    await rr.on_rsp(
        {"req_id": "rc-1000-normal", "channel": "normal", "their_max_seq": 0},
        now_mono=lambda: 2.0, sleep=_noop_sleep)
    # both left the backfill index (delivered=1 on send).
    assert await dao.backlog("normal", 10) == []
