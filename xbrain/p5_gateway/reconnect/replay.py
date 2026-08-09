"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: replay.py
Brief: GWY-P5-03 disconnect replay + R-1..3 + ordering O-1..5

Description:
17 S5 disconnect / reconnect:

  Two channels: cloud, hmi. Each has its OWN cursor -- 15 §5
  "one cursor per consumer" so a lagging HMI cannot slow the
  cloud, and vice versa.

  R-1  replay batch size <= max_batch_events (from configs, no
       default in code per CLAUDE.md 3.1)
  R-2  replay never advances the cursor past the current head
  R-3  replay is idempotent (already-delivered seqs may be re-sent;
       the consumer dedupes locally)

Ordering rules O-1..5:
  O-1  within a single event_seq, events are delivered strictly
       in ascending event_seq order
  O-2  across channels, order is INDEPENDENT (cloud may be ahead
       of hmi and vice versa)
  O-3  a delayed cloud may lag behind by up to `reconnect_window_s`
       before p5 stops accepting new events into its buffer
  O-4  after the buffer is full, oldest 'info' events are dropped
       first (never 'warn' or 'error')
  O-5  reconnect resumes from LAST persisted cursor, not from
       'newest'; the operator explicitly opts into 'skip old'
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ReplayBatch:
    consumer: str
    from_seq: int
    to_seq: int          # inclusive
    events: tuple


class ReplayViolation(Exception):
    pass


def build_replay_batch(consumer: str,
                        cursor: int,
                        head_seq: int,
                        events: list,
                        max_batch: int) -> ReplayBatch:
    """R-1 R-2: batch from cursor+1 up to min(head_seq, cursor+max_batch).
    Idempotent: safe to call repeatedly."""
    if head_seq < cursor:
        raise ReplayViolation(
            f"head_seq={head_seq} behind cursor={cursor}")
    from_seq = cursor + 1
    to_seq = min(head_seq, cursor + max_batch)
    filtered = tuple(e for e in events if from_seq <= e["event_seq"] <= to_seq)
    return ReplayBatch(consumer=consumer, from_seq=from_seq,
                         to_seq=to_seq, events=filtered)


def drop_oldest_info_first(buffer: list, target_size: int) -> int:
    """O-4: shrink buffer to target_size by evicting oldest 'info'
    events. Never evict 'warn' or 'error'. Returns count dropped.
    If dropping every info still leaves buffer over target, we STOP
    (never drop warn/error)."""
    dropped = 0
    i = 0
    while len(buffer) > target_size and i < len(buffer):
        if buffer[i].get("level") == "info":
            buffer.pop(i)
            dropped += 1
        else:
            i += 1
    return dropped
