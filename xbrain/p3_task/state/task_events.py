"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_events.py
Brief: 11 S6.2 task events -- task state transition -> event/{sev}/task

Description:
The 11 S6.2 `task` row is info/warn, channel normal, and fires on 任务 accept /
reject / start / complete / fail. P3 already drives every task state transition
through the scheduler's on_transition callback (runtime/main_wiring _make_publish),
so this module is the PURE mapping from (to_state, reason) to the task event kind +
severity -- the wiring emits event/{sev}/task whenever this returns non-None.

A validate_fail carries a non-empty `reason` (the scheduler only sets reason on a
rejection), so reason-set is the reject case (warn). The terminal + lifecycle states
map per S6.2: started/completed are info, failed/aborted/cancelled are warn.

Not every transition is an event: pending (the queued head) and suspended/resumed
are internal bookkeeping, not the S6.2 accept/reject/start/complete/fail set, so they
return None -- otherwise the event stream would carry churn the operator does not
need. When P3's executor lands, succeeded/failed transitions start flowing through
the same callback and get their events for free.
"""

from __future__ import annotations

from typing import Optional, Tuple


TASK_CATEGORY = "task"


# to_state (15 S12 task state closed set) -> (kind, sev). Absent states (pending,
# suspended, ...) produce no event.
_STATE_EVENT = {
    "ready": ("accepted", "info"),      # validated + admitted into the queue
    "running": ("started", "info"),
    "succeeded": ("completed", "info"),
    "failed": ("failed", "warn"),
    "aborted": ("aborted", "warn"),
    "cancelled": ("cancelled", "warn"),
}


def task_event_for_transition(to_state: str,
                              reason: str) -> Optional[Tuple[str, str]]:
    """Return (kind, sev) for a task state transition, or None when the transition
    warrants no 11 S6.2 task event. A non-empty reason is a validate-fail reject
    (warn); otherwise map the state."""
    if reason:
        return ("rejected", "warn")
    return _STATE_EVENT.get(to_state)
