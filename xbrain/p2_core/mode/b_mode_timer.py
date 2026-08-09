"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: b_mode_timer.py
Brief: BIZ-P2-13 -- B-mode timeout enforcement (BCT-1..BCT-6)

Description:
14 S5.6.1 T-BCAST-MAX (= p2_core.yaml.mode.b_cast_max_duration_s,
pinned 300 s). Rules:

  BCT-1: monotonic clock timing (never reset by audio frames /
         heartbeats)
  BCT-2: NEVER reset mid-broadcast (a heartbeat renewing this timer
         would let a stuck broadcast run forever)
  BCT-3: on timeout, force exit to IDLE (auto trigger, TriggerKind.TIMEOUT)
  BCT-5: covers audio-control cloud broadcast (no exemption for
         'cloud initiated' broadcasts)
  BCT-6: single instance (one broadcast at a time)

This module owns the TIMER only. Actual mode transition on timeout
is orchestrated by the SM.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional


@dataclass
class BModeTimer:
    """Monotonic-clock B-mode timeout tracker.

    * Deliberately holds no clock inside; caller supplies mono_ms
    (CLK-C1: monotonic clock ONLY for age/timeout, not wall-clock).
    """
    max_duration_s: float
    started_mono_ms: Optional[int] = None

    def start(self, now_mono_ms: int) -> None:
        """Start (or restart from IDLE) the timer. Only called on the
        IDLE -> BROADCAST transition. BCT-6: caller must ensure only
        one instance is active (check state != BROADCAST first)."""
        self.started_mono_ms = now_mono_ms

    def stop(self) -> None:
        """Clear the timer. Called on ANY exit from BROADCAST (rule
        engine, timeout, safety, user command)."""
        self.started_mono_ms = None

    def elapsed_ms(self, now_mono_ms: int) -> int:
        """Ms since start. Returns 0 if not running."""
        if self.started_mono_ms is None:
            return 0
        return max(0, now_mono_ms - self.started_mono_ms)

    def expired(self, now_mono_ms: int) -> bool:
        """True iff the timer is running AND elapsed >= max_duration_s.
        BCT-1: uses monotonic ms only."""
        if self.started_mono_ms is None:
            return False
        return self.elapsed_ms(now_mono_ms) >= int(self.max_duration_s * 1000)

    # BCT-2: no reset() method here on purpose. A heartbeat renewing
    # the timer would let a stuck broadcast run forever. To restart,
    # caller must first stop() (which requires BROADCAST -> IDLE) and
    # then start() again.
