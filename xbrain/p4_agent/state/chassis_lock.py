"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: chassis_lock.py
Brief: Is the chassis locked? -- from state/robot, the same rule as the HMI lock button and P2's lock event (99 U199)
Description:
What problem this solves
------------------------
The commx live run of 2026-11-14 (chassis never unlocked) answered "前进2米"
with "好的,前进2米" and, a moment later, "底盘锁定中, 现在不能动"; and
"现在允许运动吗" with "系统允许运动" -- health/summary knows nothing about the
lock. P4 did not subscribe state/robot, so it could not tell.

Rule (11 S9.12.2, as p5 data_readers.chassis_lock_group and p2 chassis_lock_events)
---------------------------------------------------------------------------------
  * locked = hes_lock OR timeout_lock; the derived `locked` field is NOT read
    (11 S4.1 marks it compat-only);
  * no frame, or a frame missing either lock field -> None (unknown). A
    missing lock is never defaulted to False or True: the caller then sends
    the command and lets the executor decide, exactly as before.
Freshness is the caller's: state/robot is 10 Hz, and make_state_fn hands over
only a recent frame (orchestrator_turn.ROBOT_MAX_AGE_MS).
"""
from __future__ import annotations  # postponed annotations

from typing import Any, Optional  # type hints only

#: 11 S4.1 / S9.12.2: the two Tier 1 locks of state/robot.
LOCK_FIELDS = ("hes_lock", "timeout_lock")


def chassis_locked(robot: Any) -> Optional[bool]:
    """True / False from a state/robot value (S3.0 envelope or bare), None when unknown."""
    if isinstance(robot, dict) and isinstance(robot.get("data"), dict):  # enveloped
        robot = robot["data"]  # the RobotState body
    if not isinstance(robot, dict):  # never received, or stale (dropped upstream)
        return None  # unknown
    hes_lock, timeout_lock = robot.get("hes_lock"), robot.get("timeout_lock")  # the two locks
    if not isinstance(hes_lock, bool) or not isinstance(timeout_lock, bool):  # a field missing
        return None  # never default a lock
    return hes_lock or timeout_lock  # either lock holds the chassis


__all__ = ["chassis_locked", "LOCK_FIELDS"]
