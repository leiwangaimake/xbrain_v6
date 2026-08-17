"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cloud.py
Brief: 17 S3.5 cloud uplink -- delivery marking, ack tracking, backfill runner

Description:
P5 is NOT the real-time relay (17 S3.5.0): producers put events straight onto
event/{sev}/{cat}, and cloud/HMI/P5 all subscribe. P5's job is side-persistence +
backfill on reconnect. So this module is not a publisher of live events -- it is:

  DeliveryMarker  the S3.5.1 judgment for a freshly persisted event: a need_ack=0
                  event whose link was CONNECTED at insert is delivered=1 at once
                  (the producer's direct put reached the cloud); otherwise it
                  waits (need_ack=1 -> ack) or goes to the backfill queue.
  AckTracker      on event/ack (result accepted|duplicate) mark that eid
                  delivered. 'rejected' does NOT mark -- it will be re-sent.
  BackfillRunner  on link -> CONNECTED, freeze the backlog bounds, send a begin
                  per channel, drain both channels in 4:1 order through the rate
                  limiter, put each on event/replay/{channel} (R-3: NEVER on the
                  live event/{sev}/{cat} key), mark need_ack=0 delivered on send,
                  and send an end per channel.

Everything I/O is injected (put_fn / now_mono / sleep / now_iso), so the whole
orchestration -- ordering, 4:1 weighting, rate pacing, delivery marks -- tests
against an in-memory DAO and a fake publisher with a synthetic clock, no Zenoh.

need_ack follows the 17 S3.3 UNION (channel==alarm OR sev in {alarm,fault}), which
extends 11 S2.2.5's sev-only text -- the pending U18b revision. The union is what
stops an alarm-channel recovery (sev=info) from being dropped (E-1).
"""

from __future__ import annotations

from typing import Awaitable, Callable, Optional

from ..reconnect.replay import (
    RateLimiter, build_begin, build_end, build_item, weighted_interleave,
)


# EventAck.result closed set (11 S8.4). accepted/duplicate both mean "the cloud
# has it" (duplicate is the idempotent re-delivery, S2.3); rejected does not.
ACK_ACCEPTED = frozenset({"accepted", "duplicate"})


def replay_key(rid: str, channel: str) -> str:
    """The backfill key (17 S3.5.3). Published ONLY by p5, subscribed ONLY by the
    cloud (R-2: p5/HMI must not subscribe it, or a replay self-loops)."""
    return f"xbrain/{rid}/event/replay/{channel}"


class DeliveryMarker:
    """S3.5.1: decide delivered at persist time for events that need no ack."""

    def __init__(self, dao, now_iso: Callable[[], str]) -> None:
        self._dao = dao
        self._now_iso = now_iso

    async def after_persist(self, eid: str, need_ack: bool,
                            link_connected: bool) -> bool:
        """Mark delivered iff the event needs no ack AND the link was CONNECTED
        when it was persisted (the producer's direct put reached the cloud, S3.5.1
        row 2). A DEGRADED/DISCONNECTED link leaves it delivered=0 for backfill --
        DEGRADED counts as not-delivered on purpose (宁可重发一条, S3.5.1 row 3).
        need_ack events are never marked here; only their ack (AckTracker) marks
        them. Returns True if it marked."""
        if need_ack or not link_connected:
            return False
        await self._dao.mark_delivered([eid], None, self._now_iso())
        return True


class AckTracker:
    """S3.5.1: an event/ack from the cloud marks that eid delivered."""

    def __init__(self, dao, now_iso: Callable[[], str]) -> None:
        self._dao = dao
        self._now_iso = now_iso

    async def on_ack(self, eid: str, result: str) -> bool:
        """accepted/duplicate -> mark delivered. rejected -> leave delivered=0 so
        it is re-sent (idempotent by eid, S8.4). Returns True if it marked."""
        if result not in ACK_ACCEPTED:
            return False
        n = await self._dao.mark_delivered([eid], None, self._now_iso())
        return n > 0


class BackfillRunner:
    """S3.5.2: drive one backfill pass on link -> CONNECTED. Rate-limited 4:1
    replay of both channels' delivered=0 rows, on event/replay/{channel}."""

    def __init__(self, dao, rid: str, put_fn: Callable[[str, dict], Awaitable],
                 rate: RateLimiter, now_iso: Callable[[], str],
                 max_scan: int = 100000) -> None:
        self._dao = dao
        self._rid = rid
        self._put = put_fn
        self._rate = rate
        self._now_iso = now_iso
        # Upper bound on rows scanned per channel in one pass. Not a safety param;
        # a very deep backlog just takes multiple passes (each idempotent by eid).
        self._max_scan = max_scan

    async def run(self, now_mono: Callable[[], float],
                  sleep: Callable[[float], Awaitable]) -> dict:
        """One backfill pass. now_mono is CLOCK_MONOTONIC seconds (CLK-C1) for the
        batch id + rate pacing; sleep(secs) yields while waiting for a token (the
        wiring passes asyncio.sleep; tests pass a fake that advances the clock).
        Returns {'batch_id', 'sent': {...}, 'begins', 'ends'} for observability."""
        alarm = await self._dao.backlog("alarm", self._max_scan)
        normal = await self._dao.backlog("normal", self._max_scan)
        batch_id = f"bf-{int(now_mono() * 1000)}"
        sent = {"alarm": 0, "normal": 0}
        begins = ends = 0

        # (2) freeze the bounds + send a begin for each non-empty channel. New
        # events arriving after this snapshot go the live path, not this batch.
        for channel, backlog in (("alarm", alarm), ("normal", normal)):
            if not backlog:
                continue
            seqs = [r["ch_seq"] for r in backlog]
            await self._put(replay_key(self._rid, channel), build_begin(
                batch_id, channel, from_ch_seq=seqs[0], to_ch_seq=seqs[-1],
                count=len(backlog), ts_first=0.0, ts_last=0.0,
                outage_since_ts=0.0, outage_duration_s=0.0))
            begins += 1

        # (3)(4) drain both channels 4:1, each item through the rate limiter.
        for channel, row in weighted_interleave(alarm, normal):
            while not self._rate.take(now_mono()):
                # No token: yield and let the monotonic clock advance, then retry.
                await sleep(1.0 / max(self._rate.rate_eps, 1.0))
            await self._put(replay_key(self._rid, channel),
                            build_item(batch_id, channel, self._event_of(row, channel)))
            if not row["need_ack"]:
                # need_ack=0: sent == delivered (S3.5.2 step 4). Mark it out of the
                # backfill index now.
                await self._dao.mark_delivered([row["eid"]], batch_id, self._now_iso())
            # need_ack=1: leave delivered=0; the AckTracker marks it when the ack
            # arrives. An un-acked row is re-sent on the next pass (idempotent).
            sent[channel] += 1

        # (5) end per channel that had a backlog. Channels end independently.
        for channel, backlog in (("alarm", alarm), ("normal", normal)):
            if not backlog:
                continue
            cur = await self._dao.read_cursor(channel)
            await self._put(replay_key(self._rid, channel), build_end(
                batch_id, channel, sent=sent[channel], given_up=0,
                ts_first=0.0, ts_last=0.0,
                current_ch_seq=cur["next_ch_seq"] - 1))
            ends += 1

        return {"batch_id": batch_id, "sent": sent,
                "begins": begins, "ends": ends}

    def _event_of(self, row: dict, channel: str) -> dict:
        """Reconstruct the Event to embed in the replay item (R-1). Built from the
        backlog row's core S6.1 fields; channel is stamped so the three-way V-2
        check in build_item passes. Full-column round-trip (pose/media) is a
        wiring refinement -- the core identity + payload is here."""
        return {
            "eid": row["eid"], "channel": channel, "sev": row["sev"],
            "cat": row["cat"], "title": row["title"], "detail": row["detail"],
        }
