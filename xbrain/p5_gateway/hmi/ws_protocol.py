"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ws_protocol.py
Brief: GWY-P5-10 HMI WebSocket 9 down + 5 up + ack + SS-1..4

Description:
The WebSocket envelope kinds this implementation uses, plus the SS-4 rate
bucket.

*** READ THIS BEFORE CITING ANYTHING IN THIS FILE. Corrected 2026-08-20.

The original docstring attributed four session invariants SS-1..SS-4 to 17 S11
and quoted a code E_RATE_LIMIT. Neither survives a check against the contract:

  * 17 S11's SS-1..SS-4 say something completely different (SS-1 session_id is
    minted once per P5 start; SS-2 the first frame after a connect must be a
    full snapshot; SS-3 the server declares a client dead at 30 s; SS-4 the
    deadman fallback when a session closes). The msg_id / ack-every-downstream /
    keepalive / rate-bucket set written here appears nowhere in the document
    set.
  * E_RATE_LIMIT does not exist. `grep -rn E_RATE_LIMIT docs/` returns nothing,
    and 11 S13 -- the closed set -- has no such member. Anything over the bucket
    answers E_BUSY with detail.reason="rate_limited" (see hmi/uplink.py).

So the two closed sets below are an IMPLEMENTATION RECORD of the kinds this
build sends and accepts, not a contract citation. The authoritative upstream
whitelist is 11 S12.1.1 (frozen item F-8) and it keys off `type`, not `kind`:
estop | goto | exit_broadcast | geo | task. That set lives in hmi/uplink.py,
which is what the WS reader actually consults.

Of the nine downstream kinds, this build sends three -- state_snapshot,
state_delta and ack. The rest are unimplemented names.

RateLimitBucket is the one piece here with no such problem: a token bucket that
refuses rather than queues. Queueing would let an operator build back-pressure
by holding a button, which is the property worth keeping whatever the invariant
ends up being numbered.
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
