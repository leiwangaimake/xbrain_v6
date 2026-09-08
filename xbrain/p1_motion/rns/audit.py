"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: audit.py
Brief: Ring audit buffer + terminal-report assembly (P6 -- 20 S9.0/S9.3)

Description:
Two P6 jobs:
  1. the failure/terminal report assembly (S9.0.2/S9.0.3): every module failure
     path (route deviation, wall-follow, watchdog, dynamic, RTK) produces a
     NavFailure; this file is where the ONE-shot report discipline lives -- report
     exactly once, then the mission clears to IDLE, and the module NEVER auto-
     retries (A-FAIL-1/2). A route_rev replacement ends the mission WITHOUT a
     failure report (A-FAIL-3, recorded superseded).
  2. the ring audit buffer (S9.3, RNS-M-2): detour decisions (chosen candidate,
     excluded ones and why, wall-follow enter/exit/side) go into a ring buffer
     drained by a NON-realtime thread -- the loop does NO file I/O. Without the
     audit, "why did it not go" is unanswerable in the field.

The report-once discipline is the A-FAIL-1 core: a second report for the same
terminal event, or a wrong reason (e.g. watchdog branch-two reported as
blocked_by_dynamic), is the defect. This file gates that.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Deque, List, Optional

from .types import NavFailure


class Outcome(str, Enum):
    """A mission's terminal event (20 S9.0.3). ARRIVED/FAILED are reported;
    CANCELLED/SUPERSEDED are audit-only (no failure report)."""
    ARRIVED = "arrived"
    FAILED = "failed"
    CANCELLED = "cancelled"    # cmd/motion/route op=clear -> IDLE, no failure
    SUPERSEDED = "superseded"  # route_rev replacement -> new line, no failure


@dataclass(frozen=True)
class AuditRecord:
    """One audit entry. t_mono_ms is CLOCK_MONOTONIC (RNS-CLK-1). kind is a short
    tag; detail is free-form for observability (never consumed by control)."""
    t_mono_ms: int
    kind: str
    detail: dict


class RingAudit:
    """Fixed-capacity ring buffer (S9.3, RNS-M-2). append() is O(1) and never
    allocates beyond the bound (the deque is maxlen-capped); a non-realtime thread
    calls drain(). The control loop only appends -- no I/O, no blocking."""

    def __init__(self, capacity: int) -> None:
        self._buf: Deque[AuditRecord] = deque(maxlen=capacity)

    def append(self, rec: AuditRecord) -> None:
        self._buf.append(rec)   # O(1); oldest evicted at capacity (maxlen)

    def drain(self) -> List[AuditRecord]:
        """Called by the non-realtime mover thread. Returns and clears the
        current contents; the loop keeps appending meanwhile."""
        out = list(self._buf)
        self._buf.clear()
        return out

    def __len__(self) -> int:
        return len(self._buf)


class TerminalReporter:
    """Enforces report-once (A-FAIL-1) and no-auto-retry (A-FAIL-2). A mission
    reports its terminal event exactly once; after that the reporter is 'spent'
    and refuses further reports until reset for a NEW mission."""

    def __init__(self) -> None:
        self._reported = False

    def report_failure(self, failure: NavFailure) -> Optional[NavFailure]:
        """Report a failure ONCE. A second call for the same mission returns None
        (already reported) -- the caller must not emit twice (A-FAIL-1). mutant:
        drop the spent gate -> two reports for one event -> reddens."""
        if self._reported:
            return None
        self._reported = True
        return failure

    def report_arrival(self) -> Optional[Outcome]:
        if self._reported:
            return None
        self._reported = True
        return Outcome.ARRIVED

    def is_spent(self) -> bool:
        """A-FAIL-2: once spent (terminal reported), is_active must be false and
        no new output until a NEW mission resets this. No auto-retry."""
        return self._reported

    def reset_for_new_mission(self) -> None:
        """A NEW mission (fresh route/goto) resets the reporter. This is the ONLY
        way to un-spend it -- there is no auto-retry path that resets it."""
        self._reported = False


def on_route_superseded(audit: RingAudit, t_mono_ms: int,
                        old_rev: int, new_rev: int) -> Outcome:
    """route_rev replacement (A-FAIL-3, S9.0.3): the current mission ends WITHOUT
    a failure report; the new line takes over from FOLLOW. Records superseded in
    the audit. mutant: report the old line's deviation as max_deviation at the
    swap instant -> a false failure on a normal path update -> reddens."""
    audit.append(AuditRecord(t_mono_ms=t_mono_ms, kind="superseded",
                             detail={"old_rev": old_rev, "new_rev": new_rev}))
    return Outcome.SUPERSEDED
