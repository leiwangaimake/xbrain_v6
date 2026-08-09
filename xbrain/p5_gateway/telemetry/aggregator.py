"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: aggregator.py
Brief: GWY-P5-07 telemetry (4 classes + weak-link downsample + ring buffer)

Description:
17 S9 telemetry has FOUR classes:

  system     cpu / mem / disk / temperature / uptime
  link       rtt, loss, tx/rx bytes per second
  perf       queue depths, per-stage latency P50/P95/P99
  business   task counts by state, event counts by category

Each class is sampled at a configured cadence (from configs, no
default per CLAUDE.md 3.1). Under weak-link conditions (rtt above
threshold OR loss above threshold), the uplink cadence is DOWNSAMPLED
by a factor of 4 (fixed; not tunable to avoid drift).

Ring buffer: retains last N samples per class for HMI-side charting;
older samples evicted in O(1).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum


class TelemetryClass(str, Enum):
    SYSTEM = "system"
    LINK = "link"
    PERF = "perf"
    BUSINESS = "business"


TELEMETRY_CLASSES = frozenset(TelemetryClass)

WEAK_LINK_DOWNSAMPLE = 4


@dataclass(frozen=True)
class Sample:
    class_: str
    mono_ms: int
    fields: dict


def is_weak_link(rtt_ms: float, loss_pct: float,
                  rtt_threshold_ms: float,
                  loss_threshold_pct: float) -> bool:
    """Weak-link trigger fires if either RTT OR loss exceeds its
    threshold."""
    return rtt_ms > rtt_threshold_ms or loss_pct > loss_threshold_pct


def uplink_cadence_ms(base_cadence_ms: int, weak_link: bool) -> int:
    """Under weak link, downsample by factor 4."""
    return base_cadence_ms * WEAK_LINK_DOWNSAMPLE if weak_link else base_cadence_ms


class TelemetryRing:
    def __init__(self, capacity_per_class: int) -> None:
        self._capacity = capacity_per_class
        self._rings: dict = {c.value: deque(maxlen=capacity_per_class)
                              for c in TelemetryClass}

    def append(self, sample: Sample) -> None:
        if sample.class_ not in self._rings:
            raise ValueError(f"unknown class {sample.class_!r}")
        self._rings[sample.class_].append(sample)

    def snapshot(self, class_: str):
        if class_ not in self._rings:
            raise ValueError(f"unknown class {class_!r}")
        return list(self._rings[class_])
