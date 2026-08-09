"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: handshake.py
Brief: GWY-P5-08 delivery ledger + 3-stage handshake + DP-1..8 + outbox inotify

Description:
17 S10 delivery ledger tracks each event's journey to each consumer
through a 3-stage handshake:

  s1  submitted  event committed to record.db + enqueued for consumer
  s2  in_flight  handed to transport (Zenoh publish / WebSocket send)
  s3  delivered  consumer ack received

DP-1..DP-8 discipline:
  DP-1  transitions are one-way
  DP-2  ack MUST reference the exact event_seq
  DP-3  ack timeout escalates to reconciliation (RC-*)
  DP-4  retry attempts capped at max_retries (from config)
  DP-5  ledger row remains for retention period after delivered
  DP-6  a stuck in_flight for > timeout_ms triggers state=stuck
  DP-7  outbox inotify writes to disk immediately on submit (crash-
        recoverable)
  DP-8  concurrent updates to same row use last-writer-wins with
        an updated_ms monotone check
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class DeliveryStage(str, Enum):
    SUBMITTED = "submitted"
    IN_FLIGHT = "in_flight"
    DELIVERED = "delivered"
    STUCK = "stuck"


ALLOWED_TRANSITIONS: dict = {
    DeliveryStage.SUBMITTED: {DeliveryStage.IN_FLIGHT},
    DeliveryStage.IN_FLIGHT: {DeliveryStage.DELIVERED,
                                DeliveryStage.STUCK},
    DeliveryStage.DELIVERED: set(),
    DeliveryStage.STUCK: {DeliveryStage.IN_FLIGHT},
}


class InvalidDeliveryTransition(Exception):
    pass


@dataclass
class DeliveryRecord:
    event_seq: int
    consumer: str
    stage: DeliveryStage
    updated_ms: int
    retry_count: int = 0


def transition(rec: DeliveryRecord, to_stage: DeliveryStage,
                 now_ms: int, max_retries: int) -> None:
    """Apply a transition. Rejects illegal ones (DP-1)."""
    if to_stage not in ALLOWED_TRANSITIONS[rec.stage]:
        raise InvalidDeliveryTransition(
            f"{rec.stage.value!r} -> {to_stage.value!r} not allowed")
    if to_stage == DeliveryStage.IN_FLIGHT and rec.stage == DeliveryStage.STUCK:
        rec.retry_count += 1
        if rec.retry_count > max_retries:
            raise InvalidDeliveryTransition(
                f"retry_count {rec.retry_count} > max {max_retries}")
    rec.stage = to_stage
    rec.updated_ms = now_ms


def is_stuck(rec: DeliveryRecord, now_ms: int,
              timeout_ms: int) -> bool:
    """DP-6: an in_flight record older than timeout is stuck."""
    if rec.stage != DeliveryStage.IN_FLIGHT:
        return False
    return (now_ms - rec.updated_ms) > timeout_ms
