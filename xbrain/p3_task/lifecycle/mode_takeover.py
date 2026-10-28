"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: mode_takeover.py
Brief: 15 S4.1A -- B / D mode takes over motion: yield the running task (mode_takeover), resume it after

Description:
What problem this solves
------------------------
When P2 enters B (cloud broadcast) or D (alarm) mode it tells P1 to hold the
robot still (cmd/motion/behavior hold, 99 U90 R-8 (a) + U99). The task layer
has to follow: the running patrol / goto / return_home must stop counting as
"running" -- otherwise state/task tells the cloud the robot is patrolling
while it stands still for a broadcast, and the scheduler would dispatch the
next task straight into the hold. Until 2026-10-28 P3 had no subscription to
state/mode at all, so none of this happened.

The rules (15 S4.1A, 15 S3.2, TSK-04)
-------------------------------------
  * trigger: state/mode.motion_behavior != "normal" (P2 publishes "hold" for
    B / D since 99 U99; the doc's older face_target_* wording is superseded
    by R-8 (a) and corrected in 15 S4.1A in the same batch);
  * only a RUNNING patrol / goto / return_home is suspended -- other types
    (charge docking, standby, teach) are left alone, exactly the 15 S4.1A
    list;
  * kind = yielding, reason = mode_takeover (CR-8 pairing): it resumes BY
    ITSELF when motion_behavior returns to "normal" -- a broadcast lasts
    seconds to minutes and nobody stands by to press "resume";
  * P3 does NOT send hold itself: P2 already drives P1, a second hold source
    would fight it (15 S4.1A "why step 1 does not send hold");
  * no new task is dispatched while the takeover lasts (15 S4.1A table, "不启
    动新任务") -- the caller skips its scheduler tick, the same mechanism the
    estop freeze uses.

Boundaries: the DB write + transition check + commit only. The caller decides
when (edge of motion_behavior), publishes state/task and events from the
return values, and skips the scheduler tick. Same transaction discipline as
estop_suspend.py: BEGIN IMMEDIATE ... commit / rollback here, never an open
implicit transaction left behind (that once broke every later submit).
"""

from __future__ import annotations  # postponed annotations

from typing import List, Optional, Tuple  # type hints only

from xbrain.p3_task.state.machine import apply_transition, validate_suspend_fields  # the 15 S3 state machine

#: 15 S4.1A / S3.2 attribution: yielding (auto-resume) + mode_takeover.
TAKEOVER_SUSPEND_KIND = "yielding"  # auto-resume kind (15 S3.2)
TAKEOVER_SUSPEND_REASON = "mode_takeover"  # CR-8 pairs it with yielding
#: 15 S4.1A: the task types a B / D takeover suspends. Verbatim list.
TAKEOVER_TASK_TYPES = frozenset({"patrol", "goto", "return_home"})


async def suspend_running_for_takeover(dao, conn, now_mono_ms: int
                                       ) -> Optional[Tuple[str, str, str]]:
    """Suspend the running patrol / goto / return_home (at most one, 15 S6.1).

    Returns (task_id, "suspended", "mode_takeover") for the caller to publish,
    or None when there is nothing to suspend. Idempotent: once suspended, no
    task is running and the next call is a no-op.
    """
    rows = await dao.list_by_priority()                 # (id, prio, seq, state)
    running = [tid for tid, _p, _s, state in rows if state == "running"]
    if not running:  # idle robot
        return None                                     # nothing running
    task_id = running[0]                                # scheduler invariant
    row = await dao.fetch_by_id(task_id)  # need its task_type
    if row is None or row.task_type not in TAKEOVER_TASK_TYPES:  # 15 S4.1A list
        return None                                     # e.g. charge docking
    result = apply_transition("running", "suspend")     # legal edge check
    validate_suspend_fields(result.to_state, TAKEOVER_SUSPEND_KIND,
                            TAKEOVER_SUSPEND_REASON)    # CR-8 pairing check
    await conn.execute("BEGIN IMMEDIATE")               # same as estop_suspend
    try:  # one transaction
        # preempt_task writes kind=yielding itself (it cannot be mis-paired).
        await dao.preempt_task(task_id, TAKEOVER_SUSPEND_REASON, now_mono_ms)  # kind fixed to yielding
        await conn.commit()  # committed, never left open
    except Exception:  # any DB failure
        await conn.rollback()  # leave the connection usable
        raise  # the caller logs it
    return task_id, result.to_state, TAKEOVER_SUSPEND_REASON


async def resume_after_takeover(dao, conn, now_mono_ms: int
                                ) -> List[Tuple[str, str]]:
    """Put every mode_takeover-yielded task back to ready (15 S3.2).

    Returns [(task_id, "ready"), ...] for the caller to publish; the scheduler
    then dispatches as usual. Tasks yielding for "preempted" are NOT touched:
    their resume condition is a different one (the preempting task ending).
    """
    rows = await dao.list_yielding()            # (id, prio, seq, reason)
    mine = [tid for tid, _p, _s, reason in rows  # only this module's yields
            if reason == TAKEOVER_SUSPEND_REASON]  # not "preempted"
    if not mine:  # no takeover leftovers
        return []                               # nothing yielded to a mode
    out: List[Tuple[str, str]] = []  # (task_id, new state)
    await conn.execute("BEGIN IMMEDIATE")  # write lock now, as estop_suspend
    try:  # one transaction
        for tid in mine:  # usually exactly one
            result = apply_transition("suspended", "resume")   # -> ready
            await dao.resume_task(tid, now_mono_ms)            # clears kind/reason
            out.append((tid, result.to_state))  # for the caller to publish
        await conn.commit()  # committed, never left open
    except Exception:  # any DB failure
        await conn.rollback()  # leave the connection usable
        raise  # the caller logs it
    return out  # empty = nothing resumed


__all__ = ["suspend_running_for_takeover", "resume_after_takeover",
           "TAKEOVER_SUSPEND_KIND", "TAKEOVER_SUSPEND_REASON",
           "TAKEOVER_TASK_TYPES"]
