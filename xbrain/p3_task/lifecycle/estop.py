"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: estop.py
Brief: BIZ-P3-20 P3 lifecycle S10 + estop ES-1..3 + S11.3 no auto-resume

Description:
15 S11.1 emergency stop handling:

  ES-1  freeze scheduling immediately (no new tasks dispatched)
  ES-2  suspend running task (kind='estop')
  ES-3  wait for explicit unfreeze signal from p2 (do NOT auto-resume
        after some timeout)

15 S11.3 is emphatic that p3 does NOT auto-resume from an estop
condition; that always waits for a human. This is CLAUDE.md 3.6
territory: no toggle exists to bypass it.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class EstopController:
    """Track the freeze state. All transitions are explicit;
    there is no time-based unfreeze path."""
    frozen: bool = False
    freeze_reason: str = ""

    def freeze(self, reason: str) -> None:
        """ES-1: idempotent; keep the first reason."""
        if self.frozen:
            return
        self.frozen = True
        self.freeze_reason = reason

    def unfreeze(self, source: str) -> None:
        """ES-3: only p2 explicit signals may unfreeze.
        source must be 'p2_operator' -- any other origin raises."""
        if source != "p2_operator":
            raise PermissionError(
                f"unfreeze source {source!r} not authorized")
        self.frozen = False
        self.freeze_reason = ""

    def scheduling_permitted(self) -> bool:
        return not self.frozen
