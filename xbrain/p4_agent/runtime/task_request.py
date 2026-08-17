"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_request.py
Brief: GWY-P4-40 (32.H) -- voice/text action intent -> cmd/task request

Description:
16 / 15 S12: P4 does NOT write task.db (P3 is the sole writer of the three
DBs, CLAUDE.md 0.1). When a VOICE or TEXT command is a task-family action,
P4 publishes a cmd/task REQUEST; P3 records it into the SAME task.db schema
as a party-A cloud task (CHK-0-38). This module builds that request.

Two things it gets right, each with a mutation test:
  * Only TASK-family action intents become tasks (16 / 15 S12). A G-class
    query or a J-class chitchat NEVER enters the task queue -- it is
    answered, not scheduled. to_task_request returns None for those.
  * The intent name it emits is a REAL registry intent (CS-A1: the 128
    closed set). The party-A coarse patrol maps to the fine registry intent
    patrol_route (B02), NOT a fabricated 'schedule_patrol' that no consumer
    knows. Resolving through the registry makes a bad mapping raise, not
    silently emit an id outside the closed set.

Boundary: this does not dispatch, publish, or schedule. It maps
(intent_name, slots) -> a request dict (or None). The orchestrator
publishes it on cmd/task; P3 (task_ingest, this batch) records + schedules.
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from xbrain.p4_agent.registry.intents import IntentRegistry


# Intent NAME -> task.db task_type (15 S12 TASK_TYPES 7-value closed set).
# ONLY the intents that CREATE a task are here. Task-CONTROL intents
# (pause/resume/cancel/skip/stop_follow) act on an EXISTING task via a
# state transition, not a new row, so they are deliberately absent -- they
# are not 'create a task' and must not mint one.
_TASK_CREATE_INTENTS: Dict[str, str] = {
    "goto_waypoint":      "goto",         # B01
    "patrol_route":       "patrol",       # B02 (party-A coarse patrol -> here)
    "patrol_schedule":    "patrol",       # B03
    "patrol_repeat":      "patrol",       # B04
    "return_home":        "return_home",  # B08
    "goto_dock":          "charge",       # B09 (docking is a charge task)
    "follow_target":      "follow",       # B11
    "record_route_start": "teach",        # F01 (route recording is a teach task)
    "record_fence_start": "teach",        # F07 (fence recording is a teach task)
}

# 15 S12 TASK_TYPES -- duplicated as the validation target so a mapping
# value that drifts out of the closed set is caught here (not only at the
# DB CHECK constraint downstream).
_TASK_TYPES = frozenset({
    "patrol", "goto", "charge", "return_home", "standby", "teach", "follow",
})


class TaskRequestError(RuntimeError):
    """A task-create mapping is inconsistent with the registry / closed set."""


def is_task_create_intent(intent_name: str) -> bool:
    """True iff this intent MINTS a new task (vs a query/chitchat/control)."""
    return intent_name in _TASK_CREATE_INTENTS


def to_task_request(
    intent_name: str,
    registry: IntentRegistry,
    *,
    slots: Mapping[str, Any],
    source: str,
    text: str = "",
) -> Optional[Dict[str, Any]]:
    """Build the cmd/task request for a voice/text action, or None.

    Returns None when the intent is NOT a task-create action (a G query, a
    J chitchat, an ad-hoc motion, a payload/ptz command) -- those never
    enter the task queue (15 S12; criterion 3).

    For a task-create intent, returns a dict carrying the fine registry
    intent name + id + the 7-value task_type + slots + source + text. The
    intent is resolved through the registry (CS-A1): a mapping key that is not
    one of the 128 raises rather than emitting an id no consumer knows.

    source is 'voice' or 'text' -- the channel the command arrived on, so
    the task record and telemetry can tell a spoken task from a typed one.

    text is the raw command string the turn acted on (ASR transcript post
    normalisation, or the typed text). It rides through to P3 and lands in
    tasks.command_text for party-A incident traceability (15 S9.5A.4 /
    17 S6.8.4 field 3). Defaulted to '' so a caller that has no text (or a
    test) still builds a valid request -- P3 stores '' as NULL.
    """
    task_type = _TASK_CREATE_INTENTS.get(intent_name)
    if task_type is None:
        return None    # not a task-family create intent
    # CS-A1: the emitted intent MUST be in the 128 registry. by_name raises
    # on an unknown name, so a mapping that pointed at 'schedule_patrol'
    # (not in the closed set) fails here instead of shipping a dead id.
    entry = registry.by_name(intent_name)
    if task_type not in _TASK_TYPES:
        # Defensive: a mapping value outside the closed set would be
        # rejected by the DB CHECK anyway, but fail early with the key.
        raise TaskRequestError(
            "intent %r maps to task_type %r not in %s"
            % (intent_name, task_type, sorted(_TASK_TYPES)))
    return {
        "task_type": task_type,
        "intent": entry.name,
        "id": entry.id,
        "slots": dict(slots),
        "source": source,
        "text": text,          # raw command -> tasks.command_text (15 S9.5A.4)
    }


def assert_mapping_covered_by_registry(registry: IntentRegistry) -> None:
    """Startup meta-check (CS-A1): every task-create mapping key is a real
    registry intent, and every value is in the task_type closed set. A
    fabricated key (schedule_patrol) or a drifted value fails startup, not
    a live voice turn."""
    for name, task_type in _TASK_CREATE_INTENTS.items():
        registry.by_name(name)          # raises if name not in 128
        if task_type not in _TASK_TYPES:
            raise TaskRequestError(
                "task-create mapping %r -> %r: value not in TASK_TYPES"
                % (name, task_type))
