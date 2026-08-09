"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ws_protocol.py
Brief: GWY-P5-10 HMI WebSocket 9 down + 5 up + ack + SS-1..4

Description:
17 S11 HMI WebSocket protocol has closed sets in BOTH directions:

  DOWN (p5 -> HMI) 9 message kinds:
    state_snapshot / state_delta / event / telemetry / approval /
    task_status / dock_status / link_status / prompt

  UP (HMI -> p5) 5 message kinds:
    ack / cmd / query / approve / reject

Any message with a `kind` outside these sets is rejected (never
routed). SS-1..4 session invariants:

  SS-1  each up-message carries a msg_id; p5 acks with (msg_id, ok/err)
  SS-2  the HMI is expected to ack every down-message within
        hmi_ack_timeout_ms or the session is treated as broken
  SS-3  a session with no successful ack for keepalive_timeout_ms
        is closed and WD-1..6 deadman fires
  SS-4  cmd/query/approve/reject go into a rate-limited bucket;
        overrun -> return E_RATE_LIMIT (never queue up)
"""

from __future__ import annotations

from dataclasses import dataclass


DOWN_MESSAGE_KINDS = frozenset({
    "state_snapshot", "state_delta", "event", "telemetry", "approval",
    "task_status", "dock_status", "link_status", "prompt",
})


UP_MESSAGE_KINDS = frozenset({
    "ack", "cmd", "query", "approve", "reject",
})


class UnknownMessageKind(Exception):
    pass


def classify_down(msg: dict) -> str:
    kind = msg.get("kind")
    if kind not in DOWN_MESSAGE_KINDS:
        raise UnknownMessageKind(f"down kind {kind!r}")
    return kind


def classify_up(msg: dict) -> str:
    kind = msg.get("kind")
    if kind not in UP_MESSAGE_KINDS:
        raise UnknownMessageKind(f"up kind {kind!r}")
    return kind


@dataclass
class RateLimitBucket:
    """SS-4: token bucket, replenished at configured rate. On overrun
    return False; caller must NOT queue (queueing would let the
    operator create back-pressure by hammering)."""
    capacity: int
    tokens: int
    fill_rate_per_ms: float
    last_refill_ms: int

    def try_take(self, now_ms: int) -> bool:
        elapsed = max(0, now_ms - self.last_refill_ms)
        self.tokens = min(self.capacity,
                            self.tokens + int(elapsed * self.fill_rate_per_ms))
        self.last_refill_ms = now_ms
        if self.tokens <= 0:
            return False
        self.tokens -= 1
        return True
