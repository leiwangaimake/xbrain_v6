"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: loop.py
Brief: BIZ-P3-9 scheduling loop + priority table + preemption (§6.1 §6.3)

Description:
The scheduler picks the next task from the 'queued' pool by
(priority DESC, submit_seq ASC). Same-priority tasks run in
submission order (FIFO). Preemption semantics (15 §6.3):
  * If a higher-priority task arrives while a lower-priority task is
    running AND the lower one is 'preemptible' (a policy on the row),
    the running task is suspended (kind='system').
  * If the running task is 'non-preemptible', the newer one waits.

This module holds ONLY the picking + preemption decision. The
actual state mutation goes through TasksDAO.update_state + the
transition table -- both must agree, or the transition rejects.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ScheduleCandidate:
    task_id: str
    priority: int
    submit_seq: int
    preemptible: bool


@dataclass(frozen=True)
class ScheduleDecision:
    """Pick + optional preemption. If preempt is not None, the
    scheduler must suspend that running task first."""
    pick_task_id: str = ""
    preempt: str = ""
    reason: str = ""


def pick_next(queued: list) -> ScheduleCandidate | None:
    """Highest priority, then earliest submit_seq. Returns None if
    the queued list is empty."""
    if not queued:
        return None
    return sorted(queued, key=lambda c: (-c.priority, c.submit_seq))[0]


def decide(queued: list,
             running: ScheduleCandidate | None) -> ScheduleDecision:
    """Given the queued pool and (optionally) a running task,
    decide what to do next."""
    winner = pick_next(queued)
    if winner is None:
        return ScheduleDecision(reason="empty_queue")
    if running is None:
        return ScheduleDecision(
            pick_task_id=winner.task_id, reason="start_fresh")
    if winner.priority <= running.priority:
        return ScheduleDecision(
            reason="running_still_highest")
    if not running.preemptible:
        return ScheduleDecision(
            reason="running_non_preemptible")
    return ScheduleDecision(
        pick_task_id=winner.task_id,
        preempt=running.task_id,
        reason="preempted_by_higher_priority")
