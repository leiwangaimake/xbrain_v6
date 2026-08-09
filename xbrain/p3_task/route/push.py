"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: push.py
Brief: BIZ-P3-11 cmd/motion/route push (RP-1..4 triggers + RA-1..3 ack window)

Description:
15 S5 route push protocol. P3 pushes cmd/motion/route to P1 in FOUR
scenarios:
  RP-1  task dispatch (new route)
  RP-2  resume (may include a remap)
  RP-3  geo change requires re-push
  RP-4  suspend-cancel required to restart

Each push carries a route_seq (monotonic per task); P1 acks with
RA-1 (accepted), RA-2 (rejected + reason), or RA-3 (timeout on
the ack window, default 2s). Any ack path other than these three
means the protocol contract is broken; we abort the task rather
than guess.

The route is chunked when its waypoint count exceeds
CHUNK_SIZE=32; each chunk carries (route_seq, chunk_ix, total_chunks)
so P1 can detect gaps.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


CHUNK_SIZE = 32
DEFAULT_ACK_TIMEOUT_MS = 2000


class RoutePushTrigger(str, Enum):
    RP1_DISPATCH = "RP-1"
    RP2_RESUME = "RP-2"
    RP3_GEO_CHANGE = "RP-3"
    RP4_SUSPEND_CANCEL = "RP-4"


class RouteAck(str, Enum):
    RA1_ACCEPTED = "RA-1"
    RA2_REJECTED = "RA-2"
    RA3_TIMEOUT = "RA-3"


@dataclass(frozen=True)
class RouteChunk:
    task_id: str
    route_seq: int
    chunk_ix: int
    total_chunks: int
    waypoints: tuple      # ((x, y, heading), ...)
    trigger: str


def chunk_waypoints(waypoints, chunk_size: int = CHUNK_SIZE):
    """Split waypoints into chunk_size batches. Empty list -> zero
    chunks (caller should not push at all)."""
    if not waypoints:
        return ()
    chunks = []
    for i in range(0, len(waypoints), chunk_size):
        chunks.append(tuple(waypoints[i:i + chunk_size]))
    return tuple(chunks)


def build_route_push(task_id: str,
                      route_seq: int,
                      waypoints,
                      trigger: str) -> tuple:
    """Return the ordered tuple of RouteChunk to publish. Each
    chunk carries (chunk_ix, total_chunks) so P1 can reject
    incomplete sequences without extra state."""
    parts = chunk_waypoints(waypoints)
    total = len(parts)
    return tuple(
        RouteChunk(task_id=task_id, route_seq=route_seq,
                    chunk_ix=i, total_chunks=total,
                    waypoints=part, trigger=trigger)
        for i, part in enumerate(parts))


class AckContractViolation(Exception):
    """Ack payload was neither RA-1 / RA-2 / RA-3 shape."""


def classify_ack(ack_code: str) -> RouteAck:
    """Only RA-1/2/3 are legal ack values. Anything else -> task
    abort (CLAUDE.md 3.5: closed-set outputs must not degrade)."""
    try:
        return RouteAck(ack_code)
    except ValueError:
        raise AckContractViolation(f"unknown ack code {ack_code!r}")
