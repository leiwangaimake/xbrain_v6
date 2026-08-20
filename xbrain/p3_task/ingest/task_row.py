"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_row.py
Brief: 11 S7.2 TaskCommand.task -> the 15 S9.5 TaskRow

Description:
The contract's submit body and the tasks table are two different shapes, and
this is the one place that maps between them. Kept apart from voice_task.py
(which maps p4_agent's private request shape) on purpose: they are two SOURCES,
not two spellings, and folding them into one function with branches is how the
two silently grow apart.

*** route_id -> tasks.route_geo_id is the notable one.

That column has existed since the schema was written and has been NULL on every
task ever recorded -- the voice path had no geo_id to put there, only the name
the operator spoke. It is why geo_refs (11 S7.9.4, the impact set behind a
delete confirmation) has to match on the NAME as well as the id: with nothing
in the column, an id-only match answered "referenced by nothing" for a route
three tasks were about to run.

With the contract path in use the column finally carries a real geo_id, and the
name match goes back to being the fallback it was meant to be. That is also why
p4_agent is being migrated to this shape (2026-08-20 decision): the resolution
from a spoken name to a geo_id belongs on the sender's side, where the
GeoManifest and the operator are both available to disambiguate.
"""
from __future__ import annotations

import json
from typing import Any, Dict

from xbrain.p3_task.dao.tasks_dao import TaskRow
from xbrain.p3_task.ingest.voice_task import (
    VoiceTaskIngestError, default_resume_policy,
)
from xbrain.p3_task.persistence.schema_task import TASK_SOURCES, TASK_TYPES

#: 15 S9.5 priority bounds. The DB CHECK enforces it too; refusing here names
#: the value instead of surfacing an opaque IntegrityError.
_PRIORITY_MIN, _PRIORITY_MAX = 0, 100

#: 11 S7.2's envelope `source` is the CHANNEL a command arrived on; 15 S9.5's
#: tasks.source is the five-value ORIGIN class the scheduler ranks on. They are
#: not the same axis, so the mapping is written out from 15 S4.2's table, whose
#: row reads 现场语音 / HMI -> local.
#:
#: *** Written as an explicit table, NOT as "anything unknown becomes local".
#: 11 S13.6 forbids interpreting an off-set value as the nearest thing, and a
#: silent fallback here would put a cloud task at local priority (40 instead of
#: 80) the first time the cloud spelled its channel differently -- a real
#: demotion, invisible, on the axis the scheduler orders by.
_CHANNEL_TO_SOURCE = {
    "cloud": "cloud",
    "wecom": "wecom",
    "hmi": "local",          # 15 S4.2: 现场语音 / HMI -> local
    "voice": "local",
    "text": "local",
    "local": "local",
    "charge": "charge",
}

#: 15 S4.2 priority per origin. Used when the sender omitted task.priority --
#: 50 was a made-up middle value that sits between cloud (80) and wecom (60),
#: so an omitted priority used to outrank a wecom task and lose to a cloud one
#: regardless of where it actually came from.
_PRIORITY_BY_SOURCE = {"cloud": 80, "wecom": 60, "local": 40, "auto": 20,
                       "charge": 95}


def task_row_from_command(cmd, *, submit_seq: int,
                          now_mono_ms: int, created_at: str = "") -> TaskRow:
    """Map a parsed S7.2 submit command onto a TaskRow.

    Raises VoiceTaskIngestError (the shared ingest error) on a value outside a
    closed set, rather than letting the DB CHECK reject it later -- the same
    reasoning as the voice path: failing with the offending value is what makes
    the sender's mistake fixable.
    """
    body: Dict[str, Any] = cmd.task or {}
    task_type = body.get("type")
    if task_type not in TASK_TYPES:
        raise VoiceTaskIngestError(
            "task.type %r not in the 15 S12 closed set %s"
            % (task_type, sorted(TASK_TYPES)))
    # source first: the default priority depends on it.
    channel = cmd.source or "local"
    source = _CHANNEL_TO_SOURCE.get(channel)
    if source is None:
        raise VoiceTaskIngestError(
            "source %r is not a known channel %s (15 S4.2 maps channels onto "
            "the five-value tasks.source; an unknown one is refused, never "
            "mapped to the nearest)" % (channel, sorted(_CHANNEL_TO_SOURCE)))
    if source not in TASK_SOURCES:            # guard: the map and 15 S9.5 agree
        raise VoiceTaskIngestError(
            "channel %r maps to %r, which is not in %s"
            % (channel, source, sorted(TASK_SOURCES)))
    priority = body.get("priority", _PRIORITY_BY_SOURCE[source])
    if not isinstance(priority, int) or isinstance(priority, bool) \
            or not _PRIORITY_MIN <= priority <= _PRIORITY_MAX:
        raise VoiceTaskIngestError(
            "task.priority %r outside [%d, %d]"
            % (priority, _PRIORITY_MIN, _PRIORITY_MAX))
    resume_policy = body.get("resume_policy") or default_resume_policy(task_type)
    # mission_json (15 S5.10) is IMMUTABLE after admission and is what the
    # scheduler and any audit read to see what was asked for. The contract's
    # params ride in it verbatim; `source` records the channel detail.
    mission = {"source": source, "params": body.get("params") or {}}
    for passthrough in ("intent", "id", "slots", "text"):
        # p4_agent's provenance fields, carried inside task.params by the
        # migrated sender. Lifted to the top of mission_json so the shape the
        # scheduler reads is identical whichever sender produced it.
        value = (body.get("params") or {}).get(passthrough)
        if value is not None:
            mission[passthrough] = value
    return TaskRow(
        task_id=cmd.task_id,
        task_type=task_type,
        state="pending",                       # head of the S4.4 closed set
        priority=priority,
        submit_seq=submit_seq,
        mission_json=json.dumps(mission, ensure_ascii=False,
                                separators=(",", ":")),
        total_steps=0,                         # expanded later by the route layer
        current_step=0,
        step_status_json="[]",
        created_ms=now_mono_ms,
        updated_ms=now_mono_ms,
        source=source,
        # 15 S9.5A.4: the raw command text, for party-A incident traceability.
        command_text=(body.get("params") or {}).get("text") or "",
        created_at=created_at,
        # trace_id threads cmd -> task -> event and is NOT NULL. The command's
        # own cmd_id is the honest default: it is what an auditor would follow
        # back from the task to the frame that created it.
        trace_id=body.get("trace_id") or cmd.cmd_id,
        resume_policy=resume_policy,
        # *** The column that was NULL on every task until now. See the module
        # docstring on what that cost geo_refs.
        route_geo_id=body.get("route_id") or "",
    )
