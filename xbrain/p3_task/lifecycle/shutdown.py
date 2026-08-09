"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: shutdown.py
Brief: BIZ-P3-29 shutdown coordination (SystemCommand{poweroff} -> STOPPING)

Description:
15 S12 shutdown sequence:

  step 1  P5 receives SystemCommand{poweroff}
  step 2  P3 marks shutdown_pending on its state row
  step 3  P3 refuses to admit new tasks (state stays STOPPING)
  step 4  P3 waits for running task to reach a safe boundary
          (either the current step completes or falls into a
           configured 'wait_for_power_off' window)
  step 5  P3 flushes DB (checkpoint + fsync), exits cleanly

STOPPING is a lifecycle state distinct from the task state machine:
tasks in-flight keep advancing, but no new dispatch is issued.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ShutdownController:
    shutdown_pending: bool = False
    wait_for_power_off: bool = False
    admit_allowed: bool = True

    def request(self) -> None:
        """Called on SystemCommand{poweroff}. Idempotent."""
        self.shutdown_pending = True
        self.admit_allowed = False

    def enter_wait_window(self) -> None:
        """Reached the wait_for_power_off configured window."""
        if not self.shutdown_pending:
            raise RuntimeError(
                "cannot enter wait_for_power_off without pending shutdown")
        self.wait_for_power_off = True

    def can_admit_new_task(self) -> bool:
        return self.admit_allowed
