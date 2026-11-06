"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: boot_recover.py
Brief: 15 S10 INIT -- suspend every task left `running` by an unclean shutdown (11 S9.14)

Description:
The problem. The scheduler dispatches only while NOTHING is `running`
(15 S6.1, schedule/driver.py phase 2). A `running` row seen at start-up can
only be left over from a process that died without suspending it (power cut,
crash, kill -9) -- a clean stop never leaves one. Until 2026-11-06 nothing
handled that row, although two comments in this package already said "a P3
restart suspends every running task (15 S10 INIT)". The robot measured what
that costs: after a restart mid-goto the dead row stayed `running`, every new
task sat in `ready` for ever, and the operator heard "好的,去1号测试点" for a
goto that would never start and never be reported as failed.

What it does (15 S10 INIT row, 11 S9.14 power-on recovery): every unclosed
`running` row -> `suspended` with suspend_kind = passive and suspend_reason =
unclean_shutdown, through the state machine (running --suspend--> suspended)
and validate_suspend_fields (CR-8: unclean_shutdown pairs with passive), in
ONE committed transaction. passive means it never resumes by itself: 15 S10
"不自动重启任务", 11 S9.14 "绝不能假设它还在跑" -- the operator resumes it
("继续" picks the newest passive suspension, 99 U111), and the resume then
re-plans from where the robot is (15 S7.3).

What it does NOT do:
  * publish. The caller turns the returned ids into state/task, the 11 S6.2
    task event and the 11 S9.14 system event, once its publishers exist.
  * touch P1. 15 S11 "P3 崩溃" row: P1 keeps (and finishes) its geometry while
    P3 is down and is not degraded to hold; on a whole-machine power-up P1 has
    no mission anyway. So no cmd/motion/route clear is sent from here.
  * look at any other state. ready / scheduled / suspended rows are already
    what they say and the scheduler handles them.

Traps:
  * Suspending only the FIRST running row. The single-running invariant is
    the scheduler's, and the rows here are exactly the ones written when that
    invariant may not have held (a crash between two writes); every one is
    suspended, or one dead row is still enough to block dispatch.
  * Leaving the transaction open. TasksDAO methods execute without committing;
    an open implicit transaction makes every later BEGIN IMMEDIATE (each
    cmd/task submit) fail -- the ES-2 bug of 2026-09-01, see estop_suspend.py.
"""

from __future__ import annotations  # postponed annotations

from typing import List  # the suspended ids

from xbrain.p3_task.state.machine import apply_transition, validate_suspend_fields  # the graph

#: 11 S4.4 / S9.14 attribution. passive: waits for a human (15 S10 "不自动重启").
BOOT_SUSPEND_KIND = "passive"
#: The closed-set reason for exactly this case (common/enums/sets.yaml).
BOOT_SUSPEND_REASON = "unclean_shutdown"


async def suspend_unclosed_running(dao, conn, now_mono_ms: int) -> List[str]:
    """Suspend every task still `running` at start-up; return their ids
    (empty when the last shutdown was clean). Committed before returning."""
    rows = await dao.list_by_priority()  # (task_id, prio, seq, state)
    running = [tid for tid, _p, _s, state in rows if state == "running"]
    if not running:
        return []  # clean shutdown: nothing to do, no transaction opened
    # The graph decides that running -> suspended is legal, and CR-8 that
    # unclean_shutdown pairs with passive, BEFORE anything is written: a
    # mismatch raises here with a readable message instead of a sqlite CHECK.
    to_state = apply_transition("running", "suspend").to_state
    validate_suspend_fields(to_state, BOOT_SUSPEND_KIND, BOOT_SUSPEND_REASON)
    # IMMEDIATE takes the write lock at once, the same as every other P3 write
    # path, so start-up cannot interleave with a submit arriving early.
    await conn.execute("BEGIN IMMEDIATE")
    try:
        for task_id in running:  # all of them, see Traps
            await dao.suspend_task(task_id, BOOT_SUSPEND_KIND,
                                   BOOT_SUSPEND_REASON, now_mono_ms)
        await conn.commit()
    except Exception:
        # Nothing half-applied: either every dead row is suspended or none is,
        # and the exception reaches the caller (start-up must not hide it).
        await conn.rollback()
        raise
    return running


__all__ = ["suspend_unclosed_running", "BOOT_SUSPEND_KIND", "BOOT_SUSPEND_REASON"]
