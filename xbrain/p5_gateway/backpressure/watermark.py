"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: watermark.py
Brief: GWY-P5-05 backpressure (queue watermark ladder + info-drop-only + explicit report)

Description:
17 S7 memory queue watermarks:

  low  <= level <= high  normal
  level > high           enter backpressure: drop new 'info'
                          events at ingress; keep 'warn' / 'error'
  level > overflow       enter overflow: drop new 'info'; TRIM
                          existing 'info' from the tail of the queue

record.db backlog:
  when the DB writer lags too far, only 'info' rows are pruned;
  never 'warn' / 'error'.

Backpressure state changes ALWAYS publish an audit event so the
operator sees why event counts dropped.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class BackpressureState(str, Enum):
    NORMAL = "normal"
    ELEVATED = "elevated"
    OVERFLOW = "overflow"


@dataclass(frozen=True)
class BackpressureThresholds:
    low: int
    high: int
    overflow: int


def classify(queue_len: int, thr: BackpressureThresholds) -> BackpressureState:
    """Bucket the current queue length. `low` is the recovery-from-
    elevated boundary; while transitioning down we won't jitter."""
    if queue_len > thr.overflow:
        return BackpressureState.OVERFLOW
    if queue_len > thr.high:
        return BackpressureState.ELEVATED
    return BackpressureState.NORMAL


def should_drop_at_ingress(new_event_level: str,
                             state: BackpressureState) -> bool:
    """Return True if the ingress path should drop this new event.
    'warn' and 'error' are NEVER dropped."""
    if state == BackpressureState.NORMAL:
        return False
    return new_event_level == "info"


def trim_info_from_queue(queue: list, target_len: int) -> int:
    """Overflow response: trim 'info' events from the OLDEST end
    of the queue until target_len is reached, or all info exhausted."""
    dropped = 0
    i = 0
    while len(queue) > target_len and i < len(queue):
        if queue[i].get("level") == "info":
            queue.pop(i)
            dropped += 1
        else:
            i += 1
    return dropped
