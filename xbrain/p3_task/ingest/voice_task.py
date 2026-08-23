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


# *** task_row_from_request / record_voice_task were DELETED on 2026-08-23.
#
# They mapped p4_agent's private `task_request` frame onto a TaskRow. Since
# batch 15 nothing emits that shape -- p4_agent, the HMI and the cloud all send
# the 11 S7.2 TaskCommand, which task_row.py maps instead. Keeping them would
# have left a second, unreachable way to admit a task, and CLAUDE.md 9.3 is
# explicit that a spare door gets removed rather than left ajar.
#
# What stays here is what the contract path still uses: VoiceTaskIngestError
# (the shared ingest error) and default_resume_policy.

