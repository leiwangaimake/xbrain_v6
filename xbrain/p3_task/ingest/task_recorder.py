"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_recorder.py
Brief: BIZ-P3-41 record a cmd/task payload into task.db (dedup + transaction)

Description:
The recording STEP of the voice/text -> task.db chain (15 S3.3). Given a
decoded cmd/task payload (as PB4 publishes it), this records a new task row,
under a single BEGIN IMMEDIATE transaction, with the 15 S3.3 idempotency rule:

  * If the payload carries an explicit task_id (the party-A cloud path, whose
    id is the dedup key TSK-12) and that id already exists -> DUPLICATE: do
    NOT insert, do NOT change the existing row, return the existing state.
  * Otherwise (a voice/text task has no external id) allocate a fresh
    t-YYYYMMDD-NNN id + submit_seq and INSERT at state='pending' (the head of
    the state machine: recorded, not yet validated -- 15 S3.3 insert-then-
    validate; validation/scheduling is PB6).

Why a transaction: the dup check and the insert must be one atomic step on the
single db writer, or two near-simultaneous copies of the same cmd/task (a
Zenoh redelivery) could both pass the check and both insert. BEGIN IMMEDIATE
takes the write lock up front (15 S9.1).

This module does NOT open the connection, run the loop, or subscribe -- it is
a pure async step the db-thread loop (PB5 wiring) calls with a live conn/dao.
It reads no clock: date_str (for the id) and now_mono_ms are injected.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional

from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.ingest.id_alloc import next_submit_seq, next_task_id
from xbrain.p3_task.ingest.voice_task import task_row_from_request


@dataclass(frozen=True)
class RecordOutcome:
    """What happened to one cmd/task payload."""
    kind: str            # 'recorded' | 'duplicate' | 'skipped'
    task_id: str = ""    # the recorded (or existing duplicate) id
    state: str = ""      # the row's state
    reason: str = ""     # why skipped (not a task-create)


async def record_task_from_payload(
    conn,
    dao: TasksDAO,
    payload: Mapping[str, Any],
    *,
    date_str: str,
    now_mono_ms: int,
    priority: int = 50,
    trace_id: str = "",
) -> RecordOutcome:
    """Record one cmd/task payload. Returns a RecordOutcome. See module doc.

    payload is the decoded cmd/task frame; the task fields live under
    payload['task_request'] (PB4). A frame without a task_request is a control
    or device command, not a task-create -> skipped."""
    # A control/device frame (light, PTZ, pause) has no task_request -- it acts
    # on hardware or an existing task, it does not mint one. Skip, don't error:
    # cmd/task legitimately carries both creates and non-creates.
    treq = payload.get("task_request")
    if not treq:
        return RecordOutcome(kind="skipped", reason="no task_request")

    # Idempotency (TSK-12): the dedup KEY is the task_id, and only the party-A
    # cloud path carries one. A voice task has none (each utterance is new), so
    # explicit_id is None there and no dup check applies.
    explicit_id: Optional[str] = payload.get("task_id") or treq.get("task_id")

    # One atomic step: take the write lock, check the dup, insert.
    await conn.execute("BEGIN IMMEDIATE")
    try:
        if explicit_id:
            existing = await dao.fetch_by_id(explicit_id)
            if existing is not None:
                # Already recorded: do not re-execute, do not mutate (15 S3.3).
                await conn.rollback()
                return RecordOutcome(kind="duplicate", task_id=explicit_id,
                                     state=existing.state)
            task_id = explicit_id
        else:
            # Voice/text: allocate a fresh id (no external dedup key).
            task_id = await next_task_id(conn, date_str)
        submit_seq = await next_submit_seq(conn)
        # trace_id: prefer the frame's, fall back to the caller's (never empty
        # -- it is a NOT NULL column). The recorder does not invent one beyond
        # this fallback.
        tid = treq.get("trace_id") or payload.get("trace_id") or trace_id \
            or task_id
        row = task_row_from_request(
            treq, task_id=task_id, submit_seq=submit_seq, priority=priority,
            now_mono_ms=now_mono_ms, trace_id=tid)
        await dao.insert(row)
        await conn.commit()
        return RecordOutcome(kind="recorded", task_id=task_id, state=row.state)
    except Exception:
        # Any failure rolls the whole step back -- never a half-written task.
        await conn.rollback()
        raise
