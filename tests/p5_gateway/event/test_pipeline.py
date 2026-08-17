"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_pipeline.py
Brief: 7-stage pipeline + 11 S6.2 channel derivation tests (batch 2)

Description:
Two suites. channel_map: the 11 S6.2 category->channel map is COMPLETE against
EVENT_CATEGORY (the metatest that stops a new category slipping in channel-less),
and the per-detail overrides (fence/estop/device) win over the default. pipeline:
schema rejects off-contract events, the channel is DERIVED (producer value
overwritten), a merged event pushes nothing, a degraded event still reaches HMI,
and need_ack follows the S3.3 union. The DAO is faked so the pipeline's DECISION
logic is isolated from real SQL (the DAO itself is covered by batch 1). Each
load-bearing assertion is paired with the mutation that reddens it (3.3).
"""

import pytest

from xbrain.common.enums import EVENT_CATEGORY
from xbrain.p5_gateway.event.channel_map import (
    CATEGORY_CHANNEL, ChannelDerivationError, derive_channel, detail_type_of,
)
from xbrain.p5_gateway.event.pipeline import EventPipeline
from xbrain.p5_gateway.persistence.record_dao import InsertResult


pytestmark = pytest.mark.no_device


# ===== channel_map ==========================================================

def test_category_channel_is_complete_over_event_category():
    # The metatest: every closed-set category has a channel, and no extra. A
    # category added to sets.yaml without a channel here fails this (3.2).
    assert set(CATEGORY_CHANNEL) == set(EVENT_CATEGORY.values)


def test_category_defaults():
    assert derive_channel("intrusion") == "alarm"
    assert derive_channel("task") == "normal"
    assert derive_channel("rtk") == "alarm"
    assert derive_channel("payload") == "normal"     # default, no detail


def test_fence_per_kind_override():
    # fence default is alarm, but soft_enter rides normal (S6.2 fence sub-table).
    assert derive_channel("fence", {"kind": "soft_enter"}) == "normal"
    assert derive_channel("fence", {"kind": "breach"}) == "alarm"
    # MUTATION: drop the fence override -> soft_enter wrongly becomes alarm.


def test_estop_per_type_override():
    assert derive_channel("estop", {"type": "estop.soft"}) == "alarm"
    assert derive_channel("estop", {"type": "estop.hes_cleared"}) == "normal"
    assert derive_channel("estop", {"type": "estop.unlock"}) == "normal"


def test_device_offline_online_ride_alarm_together():
    # E-1: a device outage AND its recovery ride alarm, so the cloud never gets
    # stuck showing a recovered device as still-offline.
    for cat in ("payload", "ptz", "voice"):
        assert derive_channel(cat, {"type": "device_offline"}) == "alarm"
        assert derive_channel(cat, {"type": "device_online"}) == "alarm"
    # MUTATION: device_online back to normal -> its recovery can be dropped.


def test_detail_type_prefers_type_then_kind():
    assert detail_type_of({"type": "a", "kind": "b"}) == "a"
    assert detail_type_of({"kind": "b"}) == "b"
    assert detail_type_of(None) is None
    assert detail_type_of({}) is None


def test_unknown_category_raises():
    with pytest.raises(ChannelDerivationError):
        derive_channel("halfway")


# ===== pipeline =============================================================

class _FakeDao:
    """Records the event handed to insert_event and returns a preset result, so
    the pipeline's stage logic is tested without touching SQL."""

    def __init__(self, status="inserted", ch_seq=7):
        self._status = status
        self._ch_seq = ch_seq
        self.seen = []

    async def insert_event(self, ev):
        self.seen.append(dict(ev))
        return InsertResult(status=self._status, ch_seq=self._ch_seq,
                            channel=ev.get("channel"))


def _ev(**over):
    ev = {
        "eid": "e1", "rid": "m20s-001", "cat": "task", "sev": "info",
        "title": "t", "detail": {"k": "v"}, "src": "p3_task",
        "ts": 100.0, "detected_at": "2026-08-17 10:00:00",
        "created_at": "2026-08-17T02:00:00Z",
    }
    ev.update(over)
    return ev


@pytest.mark.asyncio
async def test_schema_drops_unknown_category():
    p = EventPipeline(_FakeDao())
    out = await p.process(_ev(cat="halfway"))
    assert out.dropped and "unknown_category" in out.reason


@pytest.mark.asyncio
async def test_schema_drops_bad_sev():
    p = EventPipeline(_FakeDao())
    out = await p.process(_ev(sev="critical"))
    assert out.dropped and "bad_sev" in out.reason


@pytest.mark.asyncio
async def test_schema_drops_missing_field():
    p = EventPipeline(_FakeDao())
    out = await p.process(_ev(title=None))
    assert out.dropped and "missing_field:title" in out.reason


@pytest.mark.asyncio
async def test_inserted_goes_to_cloud_and_hmi():
    dao = _FakeDao(status="inserted", ch_seq=9)
    out = await EventPipeline(dao).process(_ev())
    assert out.persisted and out.to_cloud and out.to_hmi
    assert out.ch_seq == 9 and not out.merged


@pytest.mark.asyncio
async def test_channel_is_derived_and_overwrites_producer():
    # Producer lies (channel=normal) on an intrusion (S6.2 -> alarm). The pipeline
    # overwrites it (S3.3). MUTATION: trust the producer -> DAO sees 'normal'.
    dao = _FakeDao()
    await EventPipeline(dao).process(_ev(cat="intrusion", channel="normal"))
    assert dao.seen[0]["channel"] == "alarm"


@pytest.mark.asyncio
async def test_merged_pushes_nothing():
    # S3.2: a dedup merge must not re-push to cloud/HMI.
    dao = _FakeDao(status="merged")
    out = await EventPipeline(dao).process(_ev(dedup_key="intrusion:42"))
    assert out.merged and out.persisted
    # MUTATION: if merged still set to_cloud/to_hmi, the cloud sees a duplicate.
    assert not out.to_cloud and not out.to_hmi


@pytest.mark.asyncio
async def test_degraded_hits_hmi_not_cloud():
    # DB write failed -> in JSONL. Live HMI yes (best-effort), live cloud no
    # (nothing durable to reference; replays later). Never persisted flag.
    dao = _FakeDao(status="degraded")
    out = await EventPipeline(dao).process(_ev(sev="fault", cat="health"))
    assert out.degraded and out.to_hmi
    assert not out.to_cloud and not out.persisted


@pytest.mark.asyncio
async def test_need_ack_union_on_alarm_channel_info():
    # fence.recovered: sev=info but channel=alarm -> need_ack via the union.
    dao = _FakeDao(status="inserted")
    out = await EventPipeline(dao).process(
        _ev(cat="fence", sev="info", detail={"kind": "recovered"}))
    assert out.channel == "alarm"
    assert out.need_ack is True    # MUTATION: sev-only union -> False
