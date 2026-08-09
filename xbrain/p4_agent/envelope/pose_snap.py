"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: pose_snap.py
Brief: GWY-P4-25 -- PS-1..PS-6 position snapshot ring buffer + still_1s_ok

Description:
16 S8.6 pose snapshot on intent commit. When P4 dispatches an intent
whose semantics depend on the robot's CURRENT position (e.g., 'record
this fence corner'), the pose_snap block is attached to the intent
envelope so the consumer knows the position at ISSUE time, not at
receive time.

PS-1: ring buffer capacity N poses (typical 100 = 1s @ 100 Hz)
PS-2: pose is EMA + variance over last 1 s
PS-3: still_1s_ok = variance below threshold for a continuous 1 s
       window (else the operator was walking; snapshot might be
       across two different points)
PS-4: snapshot INCLUDES ts_iso for audit, MONOTONIC ms for math
PS-5: if buffer < 1s of history, still_1s_ok=False
PS-6: consumer refuses intent with still_1s_ok=False when the intent
       requires a stable position (recording, waypoint mark)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    heading_deg: float
    mono_ms: int


@dataclass
class PoseRing:
    """Fixed-capacity ring; oldest evicted on write."""
    capacity: int
    _buf: List[Pose] = field(default_factory=list)

    def push(self, p: Pose) -> None:
        self._buf.append(p)
        if len(self._buf) > self.capacity:
            self._buf.pop(0)

    def window(self, since_mono_ms: int) -> List[Pose]:
        return [p for p in self._buf if p.mono_ms >= since_mono_ms]

    def is_still_1s(self, now_mono_ms: int,
                    variance_threshold_meters: float = 0.05) -> bool:
        """PS-3: variance of x/y over last 1 s below threshold."""
        window = self.window(now_mono_ms - 1000)
        # PS-5: less than 1 s of data -> not still.
        if len(window) < 2:
            return False
        span = window[-1].mono_ms - window[0].mono_ms
        if span < 900:    # < 900 ms is still 'less than 1 s'
            return False
        mean_x = sum(p.x for p in window) / len(window)
        mean_y = sum(p.y for p in window) / len(window)
        var_x = sum((p.x - mean_x) ** 2 for p in window) / len(window)
        var_y = sum((p.y - mean_y) ** 2 for p in window) / len(window)
        return (var_x + var_y) ** 0.5 < variance_threshold_meters


@dataclass(frozen=True)
class PoseSnap:
    """PoseSnap block for intent envelope."""
    x: float
    y: float
    heading_deg: float
    mono_ms: int
    still_1s_ok: bool
