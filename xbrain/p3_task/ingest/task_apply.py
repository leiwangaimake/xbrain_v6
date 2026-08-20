"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_apply.py
Brief: cmd/task action dispatch -- submit / cancel / pause / resume / clear_queue

Description:
The five S7.2 actions against task.db, and the one entry point P3's wiring
calls for a cmd/task frame. Same shape as cmd/geo's dispatcher: parse ->
apply -> ack, and every outcome answers on cmd/task/ack.

What each action is allowed to do, and the rule that is easy to get wrong:

  submit       mint a task. Idempotent on task.task_id (S7.2): a repeat returns
               `duplicate` and does NOT re-execute.
  cancel       any live state -> cancelled. Already terminal -> `duplicate`
               (NOT an error: the operator asked for a state it is already in).
               Absent -> E_NOT_FOUND.
  pause        ONLY from running (S7.2). Anything else -> E_TASK_STATE.
  resume       ONLY from suspended. Anything else -> E_TASK_STATE.
  clear_queue  empties the WAITING states and `ready`. *** It must NOT touch
               running or suspended -- S7.2 says so, and the reason is that an
               operator clearing a backlog is not asking to stop the robot that
               is currently driving. It also does NO duplicate check: it is a
               set operation on whatever the queue holds at that instant, so
               "the same command twice" legitimately means two different sets.

*** pause writes suspend_kind / suspend_reason, and must.

The tasks table pairs them with the state by CHECK ((state='suspended') =
(suspend_kind IS NOT NULL)), so a pause that only wrote the state would be
rejected by sqlite. An operator pause is kind=passive / reason=operator_pause;
CR-8 pairs `yielding` exclusively with preempted / mode_takeover, which is why
an operator pause may not use it.
"""
from __future__ import annotations

import json
import logging
from typing import Any, Dict, List, Optional, Tuple

from xbrain.common.errors import (
    E_INTERNAL, E_NOT_FOUND, E_TASK_STATE,
)
from xbrain.p3_task.ingest.id_alloc import next_submit_seq
from xbrain.p3_task.ingest.task_command import (
    TaskCommand, TaskCommandError, parse_task_command, task_ack,
)
from xbrain.p3_task.ingest.task_row import task_row_from_command
from xbrain.p3_task.state.machine import (
    InvalidTransition, TERMINAL_STATES, apply_transition,
)

_logger = logging.getLogger("xbrain.p3.task_cmd")

#: The states clear_queue empties (S7.2: "不影响 running 与 suspended").
#: `ready` is included -- it is queued-and-validated, not running.
_QUEUE_STATES = ("pending", "scheduled", "blocked", "ready")

#: An operator-initiated pause. 11 S4.4 / CR-8: `yielding` pairs ONLY with
#: preempted / mode_takeover, so a human pressing pause is passive.
_PAUSE_KIND = "passive"
_PAUSE_REASON = "operator_pause"


class TaskContext:
    """The live handles an applier may touch."""

    def __init__(self, task_conn, dao=None) -> None:
        self.task_conn = task_conn
        self.dao = dao


async def handle_task_payload(payload: Dict[str, Any], ctx: TaskContext, *,
                              now_mono_ms: int, created_at: str = "",
                              on_transition=None) -> Dict[str, Any]:
    """Run one cmd/task frame end to end and return the ack body to publish.

    Never raises: a malformed frame from a browser must not stop the loop that
    also drives task scheduling. created_at is the wall-clock ISO the task
    panel shows as the dispatch time (15 S9.5, a display value -- ages and
    timeouts still use now_mono_ms).
    """
    raw_id = payload.get("cmd_id") if isinstance(payload, dict) else None
    cmd_id = raw_id if isinstance(raw_id, str) else ""
    try:
        cmd = parse_task_command(payload)
    except TaskCommandError as exc:
        _logger.warning("p3 cmd/task refused (%s): %s", exc.code, exc)
        return task_ack(cmd_id, "rejected", exc.code, {"reason": str(exc)})
    try:
        return await _dispatch(cmd, ctx, now_mono_ms, created_at,
                               on_transition)
    except TaskCommandError as exc:
        return task_ack(cmd.cmd_id, "rejected", exc.code,
                        exc.detail if exc.detail is not None
                        else {"reason": str(exc)})
    except Exception as exc:              # noqa: BLE001
        _logger.error("p3 cmd/task %s failed: %s", cmd.action, exc)
        return task_ack(cmd.cmd_id, "error", E_INTERNAL, {"reason": str(exc)})


async def _dispatch(cmd: TaskCommand, ctx: TaskContext, now_mono_ms: int,
                    created_at: str, on_transition) -> Dict[str, Any]:
    if cmd.action == "clear_queue":
        return await _clear_queue(cmd, ctx, now_mono_ms, on_transition)
    if cmd.action == "submit":
        return await _submit(cmd, ctx, now_mono_ms, created_at)
    return await _transition_one(cmd, ctx, now_mono_ms, on_transition)


#: action -> (machine event, the states it is legal from, the code to refuse
#: with). Held as data so a new action cannot be added without stating both.
_ACTION_EVENT = {
    "cancel": ("cancel", None),                 # None = any non-terminal state
    "pause": ("suspend", ("running",)),         # S7.2: only running
    "resume": ("resume", ("suspended",)),       # S7.2: only suspended
}


async def _submit(cmd: TaskCommand, ctx: TaskContext, now_mono_ms: int,
                  created_at: str = "") -> Dict[str, Any]:
    """S7.2 submit: mint the task, idempotent on task.task_id.

    Unlike the p4 recorder path, the id arrives WITH the command (the sender
    minted it, which is what makes the idempotency key meaningful across a
    redelivery), so nothing is allocated here beyond submit_seq.

    *** route_id lands in tasks.route_geo_id -- the column that has been NULL on
    every task so far. That is why geo_refs has to match on the spoken NAME as
    well as the id (see its module docstring): with the contract path in use,
    the id match starts working and the name match becomes the fallback it was
    meant to be, rather than the only thing that finds anything.
    """
    body = cmd.task or {}
    task_id = cmd.task_id
    existing = await ctx.task_conn.execute(
        "SELECT state FROM tasks WHERE task_id=?", (task_id,))
    row = await existing.fetchone()
    if row is not None:
        # S7.2: a repeat returns duplicate and does NOT re-execute.
        return task_ack(cmd.cmd_id, "duplicate", "OK",
                        {"task_id": task_id,
                         "applied": {"state": row[0], "changed": False}})
    task_row = task_row_from_command(
        cmd, submit_seq=await next_submit_seq(ctx.task_conn),
        now_mono_ms=now_mono_ms, created_at=created_at)
    await ctx.task_conn.execute("BEGIN IMMEDIATE")
    try:
        await ctx.dao.insert(task_row)
        await ctx.task_conn.commit()
    except Exception:
        await ctx.task_conn.rollback()
        raise
    return task_ack(cmd.cmd_id, "accepted", "OK",
                    {"task_id": task_id,
                     "applied": {"state": task_row.state,
                                 "type": task_row.task_type,
                                 "route_id": task_row.route_geo_id or None}})


async def _transition_one(cmd: TaskCommand, ctx: TaskContext, now_mono_ms: int,
                          on_transition) -> Dict[str, Any]:
    """cancel / pause / resume on one existing task."""
    event, legal_from = _ACTION_EVENT[cmd.action]
    cur = await ctx.task_conn.execute(
        "SELECT state FROM tasks WHERE task_id=?", (cmd.task_id,))
    row = await cur.fetchone()
    if row is None:
        raise TaskCommandError(E_NOT_FOUND,
                               f"task {cmd.task_id!r} does not exist",
                               {"task_id": cmd.task_id})
    state = row[0]
    if cmd.action == "cancel" and state in TERMINAL_STATES:
        # S7.2: already terminal -> duplicate, not an error. The operator asked
        # for a state the task is already in; reporting a failure would send
        # them looking for a problem that does not exist.
        return task_ack(cmd.cmd_id, "duplicate", "OK",
                        {"task_id": cmd.task_id,
                         "applied": {"state": state, "changed": False}})
    if legal_from is not None and state not in legal_from:
        raise TaskCommandError(
            E_TASK_STATE,
            f"{cmd.action} needs state in {list(legal_from)}, task is {state!r}",
            {"task_id": cmd.task_id, "state": state})
    try:
        result = apply_transition(state, event)
    except InvalidTransition as exc:
        raise TaskCommandError(E_TASK_STATE, str(exc),
                               {"task_id": cmd.task_id, "state": state})
    if result.idempotent:
        return task_ack(cmd.cmd_id, "duplicate", "OK",
                        {"task_id": cmd.task_id,
                         "applied": {"state": state, "changed": False}})
    await _write_state(ctx.task_conn, cmd, result.to_state, now_mono_ms)
    if on_transition is not None:
        # Same seam the scheduler uses, so an HMI-driven change reaches
        # state/task and the S6.2 task events exactly like a scheduled one.
        await on_transition(cmd.task_id, result.to_state, cmd.reason)
    # AP-1/AP-2: `applied` carries the resulting STATE, which reads as a whole
    # sentence on its own ("task t-... is now cancelled") rather than {ok:true}.
    return task_ack(cmd.cmd_id, "accepted", "OK",
                    {"task_id": cmd.task_id,
                     "applied": {"state": result.to_state,
                                 "from": result.from_state, "changed": True}})


async def _write_state(conn, cmd: TaskCommand, to_state: str,
                       now_mono_ms: int) -> None:
    """One state write, with the suspend fields kept consistent.

    The tasks DDL pairs suspend_kind / suspend_reason with the suspended state
    (non-null IFF suspended), so both directions have to be written here: a
    pause that set only the state, or a resume that left the fields behind,
    is rejected by sqlite -- and would surface as an internal error on an
    operation the operator has every reason to expect works.
    """
    if to_state == "suspended":
        await conn.execute(
            "UPDATE tasks SET state=?, suspend_kind=?, suspend_reason=?, "
            " updated_ms=? WHERE task_id=?",
            (to_state, _PAUSE_KIND, _PAUSE_REASON, now_mono_ms, cmd.task_id))
    else:
        await conn.execute(
            "UPDATE tasks SET state=?, suspend_kind=NULL, suspend_reason=NULL, "
            " updated_ms=? WHERE task_id=?",
            (to_state, now_mono_ms, cmd.task_id))
    await conn.commit()


async def _clear_queue(cmd: TaskCommand, ctx: TaskContext, now_mono_ms: int,
                       on_transition) -> Dict[str, Any]:
    """S7.2 clear_queue: cancel everything waiting, touch nothing running.

    No duplicate check, by contract: this is a set operation on the queue as it
    stands, so the same command twice legitimately clears two different sets.
    """
    marks = ", ".join("?" for _ in _QUEUE_STATES)
    params: List[Any] = list(_QUEUE_STATES)
    sql = f"SELECT task_id, state, source FROM tasks WHERE state IN ({marks})"
    if cmd.filter and isinstance(cmd.filter.get("source"), str):
        sql += " AND source = ?"
        params.append(cmd.filter["source"])
    cur = await ctx.task_conn.execute(sql, params)
    rows = await cur.fetchall()
    cleared: List[str] = []
    for task_id, state, _source in rows:
        try:
            result = apply_transition(state, "cancel")
        except InvalidTransition:
            # A queue state with no cancel arrow would be a graph change; skip
            # it rather than abort the whole sweep, and it simply does not
            # appear in cleared_ids -- which is the honest report.
            continue
        await ctx.task_conn.execute(
            "UPDATE tasks SET state=?, updated_ms=? WHERE task_id=?",
            (result.to_state, now_mono_ms, task_id))
        cleared.append(task_id)
    await ctx.task_conn.commit()
    if on_transition is not None:
        for task_id in cleared:
            await on_transition(task_id, "cancelled", cmd.reason or "clear_queue")
    # S7.2: cleared_ids MUST be reported, and an empty sweep is still accepted.
    return task_ack(cmd.cmd_id, "accepted", "OK",
                    {"cleared_ids": cleared,
                     "applied": {"cleared": len(cleared)}})
