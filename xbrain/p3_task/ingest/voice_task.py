"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: voice_task.py
Brief: GWY-P4-40 (32.H) -- record a voice/text cmd/task into task.db (unified)

Description:
15 S12: P3 is the sole writer of task.db. A voice or text task request
(built by P4 task_request.to_task_request) is recorded here into the SAME
`tasks` table and the SAME TaskRow shape as a party-A cloud task (CHK-0-38)
-- one schema, one queue, one scheduler. A spoken 'start patrol' and a
cloud PATROL differ only in the mission_json.source field, not in table or
columns.

Why unification matters: two schemas for 'a task' would mean two code paths
for state transitions, suspend/resume, retention, and the scheduler -- and
the moment they drift, a voice task and a cloud task behave differently
under the same estop or the same charge interrupt. One TaskRow keeps the
lifecycle single-sourced.

The row is built as state='pending' (the head of the 15 S12 state closed
set): a newly recorded task waits in the queue for the scheduler to start
it. total_steps/current_step are 0 until the mission is expanded (P3 route
layer); the request carries the intent + slots, not yet a route.

This module builds the TaskRow (pure) and offers an async recorder that
inserts it via TasksDAO. created_ms/updated_ms are MONOTONIC ms supplied by
the caller (CLK-C1); the ingest never reads a clock itself.
"""
from __future__ import annotations

import json
from typing import Any, Mapping

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.persistence.schema_task import TASK_SOURCES, TASK_TYPES


class VoiceTaskIngestError(RuntimeError):
    """A voice/text task request is not recordable (bad task_type)."""


# request 'source' (the CHANNEL a task arrived on) -> the tasks.source column
# closed set (15 S9.5: cloud|wecom|local|auto|charge, the scheduler priority
# axis). Voice and on-site text are 'local'; a cloud/wecom text keeps its
# channel. The detail ('voice'/'text') is preserved in mission_json.source.
_CHANNEL_TO_SOURCE = {
    "voice": "local", "text": "local", "hmi": "local", "local": "local",
    "cloud": "cloud", "wecom": "wecom",
}

# Default resume_policy per task_type (15 S7.5 / 11 S4.4 lines on abort/manual):
# teach -> manual (operator decides, S3.4); follow -> abort (target already
# lost); everything else is resumable -> continue. Resolved at admission and
# frozen on the row.
_RESUME_POLICY_BY_TYPE = {"teach": "manual", "follow": "abort"}


def default_resume_policy(task_type: str) -> str:
    return _RESUME_POLICY_BY_TYPE.get(task_type, "continue")


def task_row_from_request(
    request: Mapping[str, Any],
    *,
    task_id: str,
    submit_seq: int,
    priority: int,
    now_mono_ms: int,
    trace_id: str,
) -> TaskRow:
    """Convert a P4 cmd/task request into the unified TaskRow.

    request is the dict from task_request.to_task_request: task_type
    (7-value closed set) + intent + id + slots + source (the channel). The
    task_type is re-checked against the closed set here (the DB CHECK would
    also reject it, but failing with the value is clearer); mission_json
    carries the intent + slots + channel so the scheduler and any audit can
    see WHERE the task came from and WHAT it asked for. source / trace_id /
    resume_policy are the 15 S9.5 NOT NULL columns: source is the channel
    mapped to the closed set, resume_policy is the per-type default, trace_id
    threads the cmd -> task -> event chain (supplied by the caller from the
    intent envelope, never invented here)."""
    task_type = request.get("task_type")
    if task_type not in TASK_TYPES:
        raise VoiceTaskIngestError(
            "task_type %r not in the 15 S12 closed set %s"
            % (task_type, sorted(TASK_TYPES)))
    channel = request.get("source")
    source = _CHANNEL_TO_SOURCE.get(channel, "local")
    if source not in TASK_SOURCES:                 # defensive; map is closed
        source = "local"
    mission = {
        "source": channel,                     # 'voice' | 'text' (detail)
        "intent": request.get("intent"),       # fine registry name (CS-A1)
        "id": request.get("id"),               # 18 id (B02, ...)
        "slots": request.get("slots", {}),
    }
    # command_text (15 S9.5A.4): the raw command the task was created from --
    # the ASR transcript (post normalisation) or the typed text, threaded from
    # P4 to_task_request. '' when absent -> DAO stores NULL. Party-A REQUIRES it
    # for incident traceability; it is a first-class column, not a mission field.
    command_text = request.get("text") or ""
    return TaskRow(
        task_id=task_id,
        task_type=task_type,
        state="pending",                        # head of the state closed set
        priority=priority,
        submit_seq=submit_seq,
        mission_json=json.dumps(mission, ensure_ascii=False,
                                separators=(",", ":")),
        total_steps=0,                          # expanded later by route layer
        current_step=0,
        step_status_json="[]",
        created_ms=now_mono_ms,
        updated_ms=now_mono_ms,
        source=source,
        command_text=command_text,
        trace_id=trace_id,
        resume_policy=default_resume_policy(task_type),
    )


async def record_voice_task(
    dao: TasksDAO,
    request: Mapping[str, Any],
    *,
    task_id: str,
    submit_seq: int,
    priority: int,
    now_mono_ms: int,
    trace_id: str,
) -> TaskRow:
    """Build + INSERT the voice/text task via the same DAO as party-A.

    Returns the recorded TaskRow. The insert goes through TasksDAO into the
    `tasks` table, so the schema is literally the party-A schema."""
    row = task_row_from_request(
        request, task_id=task_id, submit_seq=submit_seq,
        priority=priority, now_mono_ms=now_mono_ms, trace_id=trace_id)
    await dao.insert(row)
    return row
