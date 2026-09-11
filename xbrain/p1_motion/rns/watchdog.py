"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: watchdog.py
Brief: Global progress watchdog (P5 -- 20 S7.3A / RNS-N-15)

Description:
Fills step 3 of the convergence proof (RNS-T-1 lemma L1): the reactive layer
(follow/thread/detour) can loop on itself -- twin-gap oscillation, U-trap in-and-
out, or any form we did not foresee -- without ever hitting a wall, so S7.2's
geometric entry never triggers and wall-follow can't reach it. The watchdog does
not target a specific trap shape (that always misses the next one); it watches
the one common symptom: arc-length progress not rising.

Timer domain (RNS-N-15): accumulates only in {follow, thread, detour}, NOT during
wait (waiting for a dynamic obstacle is not being lost, S5.3A) and NOT when the
argmax limiter is heading/RTK (those push v to zero for their own reasons and
have their own failure paths, S8.2/S3.2.1). Only geometry/UNKNOWN stall counts.

Two branches on trigger (S7.3A):
  - a BLOCKED boundary within lookahead exists -> escalate to wall-follow.
  - no boundary -> report watchdog_no_progress (the stall source is not an
    obstacle -- UNKNOWN/fence/health capped v to zero; wall-follow has nothing to
    hug). This branch trades completeness for boundedness and is filed honestly
    in the S7A.4 weakening table.
  - branch THREE (20 S7.3A v1.41, hot-battery leg04 record in S4A.12): the
    escalation itself is not bounded -- _enter_wall may answer with a
    small-obstacle DETOUR subgoal instead of hugging, the subgoal is adopted
    and dropped every tick with the pose frozen, the timer never resets, and
    the S7.6 wall criteria never get to run: 600 s hang, no report. So once
    the timer exceeds report_after_windows * T_w the watchdog reports no
    matter what boundary exists. Wall-follow ticks do not count, so a real
    hug is never cut short by it.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

from .types import NavFailReason, NavFailure


class WatchdogResult(str, Enum):
    OK = "ok"                        # progress within window, keep going
    ESCALATE_WALL = "escalate_wall"  # trigger: a boundary exists -> wall-follow
    REPORT_NO_PROGRESS = "report"    # trigger: no boundary -> fail (branch two)


# limiter sources that do NOT count as "lost" (they have own failure paths,
# S7.3A). Referenced from 12 S6.8's limiter attribution closed set, not invented.
_NON_WATCHDOG_LIMITERS = frozenset({"heading", "rtk"})


class ProgressWatchdog:
    """Watches arc-length progress s* (RNS-N-15). Reset when s* rises by delta_w;
    trigger when the timer exceeds window_s without that rise."""

    def __init__(self, window_ms: int, min_progress_m: float,
                 report_after_windows: int) -> None:
        # report_after_windows: branch three budget in T_w units (20 S7.3A
        # v1.41); injected from rns.yaml like every other calibration value,
        # never defaulted here (CLAUDE.md 3.1).
        self._window_ms = window_ms
        self._delta_w = min_progress_m
        self._report_after_ms = window_ms * int(report_after_windows)
        self._anchor_s: Optional[float] = None
        self._timer_ms = 0
        self.exhausted = False           # branch three fired (for the detail)

    def tick(
        self, s_star: float, dt_ms: int, mode: str, is_waiting: bool,
        argmax_limiter: Optional[str], boundary_exists: bool,
    ) -> WatchdogResult:
        """One tick. mode is the run state (follow/thread/detour or other). The
        timer only runs in the reactive modes, not waiting, not when a heading/
        RTK limiter is binding."""
        if self._anchor_s is None:
            self._anchor_s = s_star
        # progress reset: s* rose by delta_w.
        if s_star - self._anchor_s >= self._delta_w:
            self._anchor_s = s_star
            self._timer_ms = 0
            self.exhausted = False
            return WatchdogResult.OK
        # timer domain: only reactive modes, not waiting, not heading/RTK limited.
        counts = (mode in ("follow", "thread", "detour")
                  and not is_waiting
                  and argmax_limiter not in _NON_WATCHDOG_LIMITERS)
        if counts:
            self._timer_ms += dt_ms
        if self._timer_ms > self._report_after_ms:
            # branch three (A-CVG-7): escalation had its windows and the arc
            # progress still did not rise -> bounded report, boundary or not.
            self.exhausted = True
            return WatchdogResult.REPORT_NO_PROGRESS
        if self._timer_ms > self._window_ms:
            # trigger: escalate if there's a wall to hug, else report (branch 2).
            return (WatchdogResult.ESCALATE_WALL if boundary_exists
                    else WatchdogResult.REPORT_NO_PROGRESS)
        return WatchdogResult.OK


def no_progress_failure(argmax_limiter: Optional[str],
                        exhausted: bool = False) -> NavFailure:
    """The branch-two / branch-three failure (S7.3A): stall with no boundary
    to hug, or escalation exhausted. detail carries the argmax limiter source
    so the operator sees WHY it stalled (UNKNOWN / fence / health) and whether
    the escalation budget ran out (branch three, v1.41)."""
    return NavFailure(NavFailReason.WATCHDOG_NO_PROGRESS,
                      detail={"limiter": argmax_limiter,
                              "escalations_exhausted": bool(exhausted)})
