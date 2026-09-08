"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: dynamic.py
Brief: Dynamic distance rule + wait budget (P3 -- 20 S5.3/S5.3A)

Description:
Dynamic-obstacle handling: the two-threshold stop/resume distance rule (S5.3) and
the wait budget (RNS-N-16, S5.3A) that bounds how long the robot stands still.

Two-threshold (A-DYN-2): stop_dist and resume_dist are DIFFERENT. A single
threshold plus range noise gives stop-go-stop chatter at the boundary; resume >
stop is the hysteresis.

Only-in-corridor (A-DYN-1): the robot stops only when a dynamic obstacle occupies
the SELECTED corridor, not when any dynamic target is in view. Side traffic with a
clear path ahead must not stop it.

Wait budget (RNS-N-16): the timer is bound to the STATE (blocked), not to a
track_id. A rotating blocker (A leaves, B arrives) would reset a per-track timer
each swap and the budget would never fire; a state-bound timer accumulates while
blocked regardless of who is blocking, reporting the CURRENT blocker on timeout.
person blocking reports only -- hailing/dispersal is the upper layer's job.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

from .types import NavFailReason, NavFailure


class DynamicAction(str, Enum):
    RUN = "run"      # corridor clear (or obstacle beyond resume) -> proceed
    SLOW = "slow"    # dynamic obstacle in corridor, r_near > stop_dist -> slow
    STOP = "stop"    # dynamic obstacle in corridor, r_near < stop_dist -> stop


def distance_action(
    in_corridor: bool,
    r_near_m: float,
    prev_action: DynamicAction,
    stop_dist_m: float,
    resume_dist_m: float,
) -> DynamicAction:
    """Two-threshold stop/resume with hysteresis (20 S5.3, A-DYN-1/2).

    - not in the selected corridor -> RUN (A-DYN-1: side traffic does not stop us).
    - in corridor, r_near < stop_dist -> STOP.
    - in corridor, stop_dist <= r_near <= resume_dist -> hold prev (hysteresis
      band): if we were stopped, stay stopped; if running/slowing, SLOW.
    - r_near > resume_dist -> RUN.

    The band between stop and resume is where the two thresholds matter (A-DYN-2):
    with resume==stop, boundary noise flips RUN/STOP every tick."""
    if not in_corridor:
        return DynamicAction.RUN
    if r_near_m < stop_dist_m:
        return DynamicAction.STOP
    if r_near_m > resume_dist_m:
        return DynamicAction.RUN
    # in the hysteresis band [stop_dist, resume_dist]:
    if prev_action == DynamicAction.STOP:
        return DynamicAction.STOP   # stay stopped until past resume_dist
    return DynamicAction.SLOW


class WaitBudget:
    """Bounds the time spent stopped for a dynamic obstacle (RNS-N-16, S5.3A). The
    timer is STATE-bound: it accumulates while in the stopped state, resets when
    the stop clears, and fires blocked_by_dynamic on timeout regardless of which
    track was blocking. A rotating blocker cannot reset it (that is the point)."""

    def __init__(self, wait_budget_s: float) -> None:
        self._budget_ms = int(wait_budget_s * 1000)
        self._waiting_since_ms: Optional[int] = None

    def tick(self, is_stopped: bool, now_ms: int,
             blocking_track_id: Optional[int],
             blocking_class: Optional[str]) -> Optional[NavFailure]:
        """One tick. is_stopped is the current dynamic-stop state (from
        distance_action). Returns a failure when the accumulated stopped time
        exceeds the budget. Reset happens when is_stopped goes False."""
        if not is_stopped:
            self._waiting_since_ms = None   # cleared: stop resolved
            return None
        if self._waiting_since_ms is None:
            self._waiting_since_ms = now_ms   # entered the stopped state
            return None
        if now_ms - self._waiting_since_ms > self._budget_ms:
            return NavFailure(
                reason=NavFailReason.BLOCKED_BY_DYNAMIC,
                detail={"track_id": blocking_track_id,
                        "class_name": blocking_class})
        return None

    def waiting(self) -> bool:
        return self._waiting_since_ms is not None
