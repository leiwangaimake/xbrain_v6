"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: replay.py
Brief: 17 S3.5 backfill -- rate limiter + 4:1 weighted drain + EventReplay messages

Description:
The pure backfill logic (rewritten off the placeholder that keyed per-consumer
cursors on an event_seq and level in {info,warn,error}). The contract (17 S3.5.2)
backfills on TWO channels (normal / alarm), each with its own ch_seq cursor, and
drains them with a 4:1 weighted round robin so alarms go first WITHOUT starving
normal (U18 "互不阻塞" is two-directional).

Three pieces, all pure so they test with injected time and no Zenoh:
  * RateLimiter    token bucket at backfill.rate_eps_total (20/s, S3.5.2). A
                   reconnect dumping the whole backlog at once would swamp the
                   link and crowd out the control plane, so every replayed item
                   must take a token first. Monotonic clock only (CLK-C1) --
                   now_mono is passed in, never read here.
  * weighted_interleave  the 4:1 order: 4 alarm then 1 normal per round, but once
                   a channel empties the other takes the whole remaining order
                   (A empty -> N drains fully; N empty -> A drains fully). Within
                   a channel, strict ch_seq ascending (S3.5.2 因果顺序).
  * build_begin / build_item / build_end   the EventReplay message shapes
                   (17 S3.5.3). build_item passes the event through VERBATIM (R-1):
                   rewriting eid/ts/channel would break G-2 (HMI and cloud同源).

What is NOT here (batch 4, needs real time + Zenoh + acks): the runner that
freezes the batch bounds from the DAO, actually sends on event/replay/{channel},
waits for acks (need_ack rows), retries (max_deliver_tries=5 -> delivered=-1 +
fault), and advances confirmed_upto. This module is the deterministic core that
runner drives.
"""

from __future__ import annotations

from typing import Optional


# 17 S3.5.2 weight: 4 alarm items for every 1 normal item, while both have a
# backlog. NOT a code default for a safety param -- it is a scheduling ratio, and
# the real value is injected by the runner from configs (backfill.weight).
DEFAULT_ALARM_WEIGHT = 4


class RateLimiter:
    """Token bucket. Refills at `rate_eps` tokens/second up to `capacity`; take()
    spends one token if available. now_mono is CLOCK_MONOTONIC seconds passed in
    by the caller (CLK-C1: this class reads no clock of its own, so it is testable
    with a synthetic timeline and cannot be fooled by a wall-clock step)."""

    def __init__(self, rate_eps: float, capacity: Optional[float] = None) -> None:
        if rate_eps <= 0:
            raise ValueError(f"rate_eps must be > 0, got {rate_eps}")
        self._rate = rate_eps
        # Capacity defaults to one second of tokens: a short reconnect burst is
        # allowed, but not an unbounded dump (which is the whole point, S3.5.2).
        self._cap = capacity if capacity is not None else rate_eps
        self._tokens = self._cap
        self._last: Optional[float] = None

    def take(self, now_mono: float) -> bool:
        """True (and spend a token) if the bucket has one, else False. Refills
        based on elapsed monotonic time since the last call."""
        if self._last is None:
            self._last = now_mono
        elapsed = now_mono - self._last
        if elapsed > 0:
            self._tokens = min(self._cap, self._tokens + elapsed * self._rate)
            self._last = now_mono
        if self._tokens >= 1.0:
            self._tokens -= 1.0
            return True
        return False


def weighted_interleave(alarm: list, normal: list,
                        alarm_weight: int = DEFAULT_ALARM_WEIGHT) -> list:
    """Return [(channel, item), ...] draining both lists in a 4:1 (alarm:normal)
    round robin. Each list is assumed already in ch_seq ascending order and is
    emitted in that order. When alarm empties, normal takes the whole rest of the
    order (and vice versa), so an empty alarm queue does NOT throttle normal to
    1-per-round (S3.5.2: 'A empty -> N 独占全部配额')."""
    if alarm_weight < 1:
        raise ValueError(f"alarm_weight must be >= 1, got {alarm_weight}")
    ai = ni = 0
    out: list = []
    while ai < len(alarm) or ni < len(normal):
        for _ in range(alarm_weight):
            if ai < len(alarm):
                out.append(("alarm", alarm[ai]))
                ai += 1
        if ni < len(normal):
            out.append(("normal", normal[ni]))
            ni += 1
    return out


def build_begin(batch_id: str, channel: str, from_ch_seq: int, to_ch_seq: int,
                count: int, ts_first: float, ts_last: float,
                outage_since_ts: float, outage_duration_s: float) -> dict:
    """EventReplay 'begin' (17 S3.5.3): announces this channel's frozen batch
    bounds so the cloud knows the range and can detect a missing ch_seq. ts_span
    is display-only (DBF-3), never an ordering key."""
    return {
        "kind": "begin", "batch_id": batch_id, "channel": channel,
        "from_ch_seq": from_ch_seq, "to_ch_seq": to_ch_seq, "count": count,
        "ts_span": {"first": ts_first, "last": ts_last},
        "outage": {"since_ts": outage_since_ts, "duration_s": outage_duration_s},
    }


def build_item(batch_id: str, channel: str, event: dict) -> dict:
    """EventReplay 'item' (17 S3.5.3 R-1): the original event embedded VERBATIM.
    channel must equal the key segment and event.channel (three-way, V-2). The
    event dict is not copied-and-edited -- any rewrite of eid/ts/channel breaks
    G-2 (HMI and cloud must see identical content)."""
    if event.get("channel") not in (None, channel):
        raise ValueError(
            f"replay item channel mismatch: key {channel!r} vs "
            f"event.channel {event.get('channel')!r}")
    return {"kind": "item", "batch_id": batch_id, "channel": channel,
            "event": event}


def build_end(batch_id: str, channel: str, sent: int, given_up: int,
              ts_first: float, ts_last: float, current_ch_seq: int) -> dict:
    """EventReplay 'end' (17 S3.5.2 step 5): this channel drained. Reports how many
    were sent vs given up (delivered=-1 after max_deliver_tries) and the current
    realtime ch_seq so the cloud knows where live resumes. Channels end
    independently -- neither waits for the other."""
    return {
        "kind": "end", "batch_id": batch_id, "channel": channel,
        "sent": sent, "given_up": given_up,
        "ts_span": {"first": ts_first, "last": ts_last},
        "current_ch_seq": current_ch_seq,
    }
