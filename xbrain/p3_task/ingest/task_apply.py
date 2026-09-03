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
from dataclasses import replace
from typing import Any, Dict, List, Optional, Tuple

from xbrain.common.errors import (
    E_INTERNAL, E_NOT_FOUND, E_TASK_STATE,
)
from xbrain.p3_task.ingest.id_alloc import next_submit_seq, next_task_id
from xbrain.p3_task.ingest.task_command import (
    TaskCommand, TaskCommandError, parse_task_command, task_ack,
)
from xbrain.p3_task.ingest.task_row import task_row_from_command
# compute_duration_sec 与 driver 共用一份: 15 S9.5 的口径(跨重启写
# NULL, 不回退墙钟差值)只能有一个实现, 抄第二份必然漂.
from xbrain.p3_task.schedule.driver import compute_duration_sec
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
                              date_str: str = "",
                              finished_at: str = "", boot_id: str = "",
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
    # 11 S2.3: the receiver de-duplicates on cmd_id. Checked before anything
    # is applied, so a redelivery replays the first answer instead of acting a
    # second time -- which for submit would mint a task nobody asked for.
    try:
        replay = await _replay_if_seen(ctx, cmd.cmd_id)
        if replay is not None:
            return replay
        return await _dispatch(cmd, ctx, now_mono_ms, created_at, date_str,
                               on_transition, finished_at=finished_at,
                               boot_id=boot_id)
    except TaskCommandError as exc:
        return task_ack(cmd.cmd_id, "rejected", exc.code,
                        exc.detail if exc.detail is not None
                        else {"reason": str(exc)})
    except Exception as exc:              # noqa: BLE001
        _logger.error("p3 cmd/task %s failed: %s", cmd.action, exc)
        return task_ack(cmd.cmd_id, "error", E_INTERNAL, {"reason": str(exc)})


async def _dispatch(cmd: TaskCommand, ctx: TaskContext, now_mono_ms: int,
                    created_at: str, date_str: str,
                    on_transition, *, finished_at: str = "",
                    boot_id: str = "") -> Dict[str, Any]:
    if cmd.action == "clear_queue":
        # S7.2: clear_queue does NO duplicate check -- it is a set operation on
        # whatever the queue holds at that instant, so it is deliberately NOT
        # written to the cmd log either.
        return await _clear_queue(cmd, ctx, now_mono_ms, on_transition)
    if cmd.action == "submit":
        return await _submit(cmd, ctx, now_mono_ms, created_at, date_str)
    return await _transition_one(cmd, ctx, now_mono_ms, on_transition,
                                 finished_at=finished_at, boot_id=boot_id)


#: action -> (machine event, the states it is legal from, the code to refuse
#: with). Held as data so a new action cannot be added without stating both.
_ACTION_EVENT = {
    "cancel": ("cancel", None),                 # None = any non-terminal state
    "pause": ("suspend", ("running",)),         # S7.2: only running
    "resume": ("resume", ("suspended",)),       # S7.2: only suspended
}


async def _replay_if_seen(ctx: TaskContext,
                          cmd_id: str) -> Optional[Dict[str, Any]]:
    """The first answer for a cmd_id already applied, or None (11 S2.3)."""
    cur = await ctx.task_conn.execute(
        "SELECT result, code, detail_json FROM task_cmd_log WHERE cmd_id=?",
        (cmd_id,))
    row = await cur.fetchone()
    if row is None:
        return None
    detail = json.loads(row[2]) if row[2] else {}
    detail = dict(detail)
    detail["replayed"] = True
    # result is `duplicate`, not the original `accepted`: the sender must be
    # able to tell a second delivery from a second effect.
    return task_ack(cmd_id, "duplicate", row[1], detail)


async def _write_cmd_log(conn, cmd: TaskCommand, task_id: Optional[str],
                         result: str, code: str,
                         detail: Optional[Dict[str, Any]],
                         now_mono_ms: int) -> None:
    """Record the outcome INSIDE the caller's open transaction (S2.3)."""
    await conn.execute(
        "INSERT INTO task_cmd_log (cmd_id, action, task_id, result, code, "
        " detail_json, applied_ms) VALUES (?, ?, ?, ?, ?, ?, ?)",
        (cmd.cmd_id, cmd.action, task_id, result, code,
         None if detail is None else json.dumps(detail, ensure_ascii=False),
         now_mono_ms))


async def _submit(cmd: TaskCommand, ctx: TaskContext, now_mono_ms: int,
                  created_at: str = "", date_str: str = "") -> Dict[str, Any]:
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
    task_id = cmd.task_id
    if task_id:
        # The sender supplied one (the party-A cloud path does). Business-level
        # de-duplication on top of the cmd_id one: the same task pushed twice
        # under two different cmd_ids is still one task.
        existing = await ctx.task_conn.execute(
            "SELECT state FROM tasks WHERE task_id=?", (task_id,))
        row = await existing.fetchone()
        if row is not None:
            return task_ack(cmd.cmd_id, "duplicate", "OK",
                            {"task_id": task_id,
                             "applied": {"state": row[0], "task_id": task_id,
                                         "changed": False}})
    await ctx.task_conn.execute("BEGIN IMMEDIATE")
    try:
        if not task_id:
            # S7.2 (corrected 2026-08-20): P3 allocates. Inside the transaction
            # because next_task_id reads max(NNN) for the day -- allocating
            # outside it would let two frames in the same tick take the same id.
            task_id = await next_task_id(ctx.task_conn, date_str)
            cmd = replace(cmd, task_id=task_id)
        task_row = task_row_from_command(
            cmd, submit_seq=await next_submit_seq(ctx.task_conn),
            now_mono_ms=now_mono_ms, created_at=created_at)
        await ctx.dao.insert(task_row)
        detail = {"task_id": task_id,
                  "applied": {"state": task_row.state,
                              # AP-2: the id is IN applied, not only alongside
                              # it -- a sender that omitted one learns what it
                              # got without having to read another field.
                              "task_id": task_id,
                              "type": task_row.task_type,
                              "route_id": task_row.route_geo_id or None}}
        await _write_cmd_log(ctx.task_conn, cmd, task_id, "accepted", "OK",
                             detail, now_mono_ms)
        await ctx.task_conn.commit()
    except Exception:
        await ctx.task_conn.rollback()
        raise
    return task_ack(cmd.cmd_id, "accepted", "OK", detail)


async def _transition_one(cmd: TaskCommand, ctx: TaskContext, now_mono_ms: int,
                          on_transition, *,
                          finished_at: str = "", boot_id: str = "") -> Dict[str, Any]:
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
    await _write_state(ctx.task_conn, cmd, result.to_state, now_mono_ms,
                       finished_at=finished_at, boot_id=boot_id)
    detail = {"task_id": cmd.task_id,
              "applied": {"state": result.to_state,
                          "from": result.from_state, "changed": True}}
    await _write_cmd_log(ctx.task_conn, cmd, cmd.task_id, "accepted", "OK",
                         detail, now_mono_ms)
    await ctx.task_conn.commit()
    if on_transition is not None:
        # Same seam the scheduler uses, so an HMI-driven change reaches
        # state/task and the S6.2 task events exactly like a scheduled one.
        await on_transition(cmd.task_id, state, result.to_state, cmd.reason)
    # AP-1/AP-2: `applied` carries the resulting STATE, which reads as a whole
    # sentence on its own ("task t-... is now cancelled") rather than {ok:true}.
    return task_ack(cmd.cmd_id, "accepted", "OK", detail)


async def _write_state(conn, cmd: TaskCommand, to_state: str,
                       now_mono_ms: int, *, finished_at: str = "",
                       boot_id: str = "") -> None:
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
    elif to_state in TERMINAL_STATES:
        # *** 终态还要写 finished_at 与 duration_sec (15 S9.5), NO 不能只写 state.
        # 原实现走的是下面那条通用 UPDATE, 于是[凡是被取消的任务终态审计列全空]
        # -- 2026-09-03 甲方停掉一条任务, 库里 started_at 有值而 finished_at 与
        # duration_sec 都是 NULL, 上报给云端的 summary.duration_sec 因此是 0.0.
        # finish_task 只被 apply_motion_result(执行完成路径)调用过, 而那条路
        # 因为执行器未建从没跑过 -- 于是这两列在真机上一直是空的.
        cur = await conn.execute(
            "SELECT started_mono, started_boot FROM tasks WHERE task_id=?",
            (cmd.task_id,))
        row = await cur.fetchone()
        started_mono = row[0] if row else None
        started_boot = row[1] if row else None
        await conn.execute(
            "UPDATE tasks SET state=?, suspend_kind=NULL, suspend_reason=NULL, "
            " updated_ms=?, finished_at=?, duration_sec=? WHERE task_id=?",
            (to_state, now_mono_ms, finished_at or None,
             compute_duration_sec(started_mono, started_boot,
                                  now_mono_ms, boot_id),
             cmd.task_id))
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
            await on_transition(task_id, state, "cancelled",
                                cmd.reason or "clear_queue")
    # S7.2: cleared_ids MUST be reported, and an empty sweep is still accepted.
    return task_ack(cmd.cmd_id, "accepted", "OK",
                    {"cleared_ids": cleared,
                     "applied": {"cleared": len(cleared)}})
