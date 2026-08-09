"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: probe.py
Brief: GWY-P5-12 estop link end-to-end probe (link_probe <=10 ms shortest path)

Description:
17 S13 dedicated estop probe path. This is separate from the
regular telemetry heartbeat because the estop chain has a hard
budget: <= 10 ms round-trip from p5 through the RT plane to
chassis_relay and back.

The probe:
  * uses a dedicated Zenoh key on the RT plane
  * has NO logging in the hot path (would blow the budget)
  * uses a single fixed-size payload (16 bytes) to avoid GC
    allocation
  * fires at 20 Hz -- one probe per P1 control tick

Missing 3 consecutive probes -> raise LinkProbeMissed;
caller should escalate to systemwide estop.
"""

from __future__ import annotations

from dataclasses import dataclass


PROBE_PAYLOAD_BYTES = 16
PROBE_BUDGET_MS = 10
PROBE_RATE_HZ = 20
MISSED_THRESHOLD = 3


@dataclass
class ProbeStats:
    sent: int = 0
    received: int = 0
    missed_streak: int = 0
    worst_rtt_ms: float = 0.0


class LinkProbeMissed(Exception):
    """3 consecutive misses -> escalate to system estop."""


def record_send(stats: ProbeStats) -> None:
    stats.sent += 1


def record_receive(stats: ProbeStats, rtt_ms: float) -> None:
    stats.received += 1
    stats.missed_streak = 0
    if rtt_ms > stats.worst_rtt_ms:
        stats.worst_rtt_ms = rtt_ms
    if rtt_ms > PROBE_BUDGET_MS:
        raise LinkProbeMissed(
            f"probe rtt {rtt_ms:.2f}ms exceeds budget {PROBE_BUDGET_MS}ms")


def record_miss(stats: ProbeStats) -> None:
    stats.missed_streak += 1
    if stats.missed_streak >= MISSED_THRESHOLD:
        raise LinkProbeMissed(
            f"missed {stats.missed_streak} consecutive probes")
