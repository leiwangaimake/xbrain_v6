"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: driver.py
Brief: BIZ-P3-42 scheduler tick -- drive the task machine (validate + dispatch)

Description:
The step that MOVES a recorded task through the machine (15 S5 / S6). Before
this, a task recorded by the ingest sat at 'pending' forever -- the state
machine, preconditions, and pick_next were pure functions with no driver.
scheduler_tick is that driver, one atomic pass over the task table:

  1. VALIDATE each 'pending' task (15 S5 TSK-14): run the preconditions that
     apply BEFORE route expansion -- V-1 type, V-2 priority, V-5 mission
     parses. A failure -> validate_fail -> 'failed' (with the code as the
     reason); all pass -> validate_ok -> 'ready'. (V-3 energy and V-6 step
     count need an expanded route/soc, which is the execution-wiring batch, so
     they are NOT run here -- running V-6 on an unexpanded patrol would fail a
     valid task for having 0 steps.)
  2. DISPATCH: if NOTHING is 'running', pick the highest-priority 'ready' task
     (priority DESC, submit_seq ASC -- the 15 S6.1 order) and move it
     'ready' -> 'running'. One running task at a time (single robot).

Each transition goes through apply_transition (the graph is the authority) +
TasksDAO.update_state, and calls on_transition(task_id, to_state, reason) so
the caller can publish state/task + an event. The whole tick is one
transaction: a reader on another connection sees either the pre-tick or the
post-tick state, never a half-applied pass. It reads no clock (now_mono_ms
injected).

Boundary (NOT here): route expansion (mission_json -> waypoints/total_steps),
the actual robot execution (cmd/motion to P1) and the running -> done
completion that a P1 status reports. A dispatched task correctly sits at
'running' until that execution wiring lands.
"""
from __future__ import annotations

from typing import Awaitable, Callable, List, Tuple

from xbrain.p3_task.state.machine import apply_transition
from xbrain.p3_task.state.preconditions import (
    check_v1_type, check_v2_priority, check_v5_mission_parses,
)


# on_transition(task_id, to_state, reason) -> awaitable. reason is '' on a
# clean transition, or the failing precondition code on a validate_fail.
OnTransition = Callable[[str, str, str], Awaitable[None]]


def validate_pending(row) -> Tuple[str, str]:
    """Run the pre-expansion preconditions on a pending task. Returns
    (event, reason): ('validate_ok', '') if all pass, else
    ('validate_fail', '<code>: <detail>')."""
    for check in (
        check_v1_type(row.task_type),
        check_v2_priority(row.priority),
        check_v5_mission_parses(row.mission_json),
    ):
        if check is not None:
            return "validate_fail", f"{check.code}: {check.detail}"
    return "validate_ok", ""


async def scheduler_tick(conn, dao, *, now_mono_ms: int,
                         on_transition: OnTransition) -> List[Tuple[str, str, str]]:
    """One scheduler pass. Returns the transitions made as
    (task_id, from_state, to_state). See module docstring."""
    made: List[Tuple[str, str, str]] = []

    # -- phase 1: validate every pending task --
    rows = await dao.list_by_priority()          # (task_id, priority, seq, state)
    for task_id, _prio, _seq, state in rows:
        if state != "pending":
            continue
        full = await dao.fetch_by_id(task_id)
        if full is None:                          # vanished between calls
            continue
        event, reason = validate_pending(full)
        result = apply_transition("pending", event)
        await dao.update_state(task_id, result.to_state, now_mono_ms)
        made.append((task_id, "pending", result.to_state))
        await on_transition(task_id, result.to_state, reason)

    # -- phase 2: dispatch one ready task if nothing is running --
    # Re-read so the tasks just validated to 'ready' are visible (read-your-
    # writes on this connection, still inside the tick's transaction).
    rows2 = await dao.list_by_priority()
    if not any(state == "running" for _i, _p, _s, state in rows2):
        ready = [(tid, p, s) for tid, p, s, state in rows2 if state == "ready"]
        if ready:
            # priority DESC, submit_seq ASC (15 S6.1).
            winner = sorted(ready, key=lambda r: (-r[1], r[2]))[0][0]
            result = apply_transition("ready", "dispatch")
            await dao.update_state(winner, result.to_state, now_mono_ms)
            made.append((winner, "ready", result.to_state))
            await on_transition(winner, result.to_state, "")

    # One commit closes the whole tick atomically (see module docstring).
    await conn.commit()
    return made


# Motion result closed set (11 S3.5 relative_move/status terminal values). A
# 'succeeded' completes the task; 'aborted'/'rejected' fail it. 'accepted'/
# 'running' are in-flight and drive no task transition.
_MOTION_TERMINAL = {"succeeded": "complete", "aborted": "fail",
                    "rejected": "fail"}


async def apply_motion_result(conn, dao, task_id: str, result: str, *,
                              now_mono_ms: int,
                              on_transition: OnTransition) -> bool:
    """Close a running task on the P1 motion result (11 S3.5). 'succeeded' ->
    running -> done; 'aborted'/'rejected' -> running -> failed. Returns True if
    a transition was made.

    This is the lifecycle-closing half of execution. It is a pure step the
    (future) motion-status subscriber calls; it does NOT subscribe or emit
    cmd/motion -- P1 executing a real path + reporting this status is the
    execution-wiring milestone (P1 today is an ad-hoc-motion MVP). An in-flight
    'accepted'/'running', or a result for a task that is not running, is a
    no-op (not an error: a late status after cancel is legal)."""
    event = _MOTION_TERMINAL.get(result)
    if event is None:
        return False                              # accepted/running: in-flight
    full = await dao.fetch_by_id(task_id)
    if full is None or full.state != "running":
        return False                              # already terminal / cancelled
    transition = apply_transition("running", event)
    await dao.update_state(task_id, transition.to_state, now_mono_ms)
    await conn.commit()
    await on_transition(task_id, transition.to_state, f"motion:{result}")
    return True
