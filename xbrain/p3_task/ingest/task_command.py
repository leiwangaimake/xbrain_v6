"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_command.py
Brief: cmd/task TaskCommand envelope -- the 11 S7.2 contract shape

Description:
P3 is the sole subscriber of cmd/task, and its publishers are the HMI, the
cloud (via P5, which is the only forwarder since v0.7.8) and p4_agent. This
module parses the S7.2 envelope they all send.

*** Why this exists: until 2026-08-20 P3's cmd/task receiver understood only
p4_agent's PRIVATE shape -- it read payload['task_request'] and skipped any
frame without it, in as many words. So a frame in the CONTRACT shape
({action:"submit", task:{...}}) was dropped on the floor: silently, with no ack,
by a process whose logs said nothing. That closed the HMI's W2/W7 and the
cloud's forwarded tasks at the same time, and neither would have looked like a
bug from the sending side.

*** For cancel / pause / resume, task_id is required and there is deliberately
NO "omit = the current task".
S7.2 gives four reasons and the first is the one that bites: the queue is LIVE.
Between the operator seeing "A is running" on the HMI and the command arriving,
A may have finished and B started -- the shorthand would pause the wrong task,
and nothing in the log would show that it happened. The other three: voice adds
0.5-2 s of ASR + LLM + confirmation on top; idempotency requires a resent cmd_id
to mean the same thing it meant the first time; and an audit has to be able to
reconstruct which task the operator meant.

submit is the exception, and for an unrelated reason: its task_id may be OMITTED
(S7.2, corrected 2026-08-20), because the form is t-YYYYMMDD-NNN with a per-day
sequence only P3 holds. That is not the "current task" shorthand -- nothing is
being guessed at; a new id is minted and returned in the ack.

Boundaries: this parses and validates. It opens no db, reads no clock, and
applies nothing -- the appliers live in task_apply.py, the same split as
cmd/geo's.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from xbrain.common.enums import TASK_ACTION
from xbrain.common.errors import E_SCHEMA

#: Actions that name one existing task, so task_id is required (S7.2 field
#: table). submit carries it inside `task`; clear_queue is a set operation.
_NEEDS_TASK_ID = frozenset({"cancel", "pause", "resume"})


class TaskCommandError(ValueError):
    """A cmd/task payload is refused. Carries the closed-set code so the ack
    maps to a real E_* rather than free text, plus an optional detail."""

    def __init__(self, code: str, message: str,
                 detail: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(message)
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class TaskCommand:
    cmd_id: str
    action: str
    task_id: Optional[str]
    task: Optional[Dict[str, Any]]        # submit only
    resume_mode: Optional[str]            # resume only
    filter: Optional[Dict[str, Any]]      # clear_queue only
    reason: str
    source: str


def _goto_has_destination(task: Dict[str, Any]) -> bool:
    """True iff a goto task names a destination in a form a producer sends.

    11 S7.2 writes the requirement as "waypoint_id or target". The three
    producers carry it in different places, and every one of them must keep
    working, so each form is listed with its sender:
      * HMI W2 (11 W2 row)     params.waypoint_id, or params.lat + params.lon
      * 11 S7.2 wording        params.target
      * cloud GOTO_KEYPOINT    task.route_id and/or params.waypoints[] (the
                               gateway passes the v2.0 plan through; P3 resolves
                               it -- see p5 inbound/task_router._goto)
      * voice B01              params.waypoint_id, resolved by p4 against the
                               geo catalogue (16 S8.1)
    A key that is present but empty (blank name, empty list, half a position)
    is not a destination.

    *** A spoken NAME alone (params.slots.waypoint) is NOT a destination
    (changed 2026-10-25). It was accepted until then on the belief that P3
    resolves names; it does not -- geo_refs only computes the delete-impact set
    (11 S7.9.4) and dispatch pushes a route only from route_geo_id. A
    name-only goto was therefore stored and never moved, and on 2026-10-25 the
    tier-2 LLM filled that name with invented text ("到这边=w").
    """
    params = task.get("params") if isinstance(task.get("params"), dict) else {}

    def _text(v: Any) -> bool:
        return isinstance(v, str) and bool(v.strip())

    def _num(v: Any) -> bool:
        return isinstance(v, (int, float)) and not isinstance(v, bool)

    if _text(task.get("route_id")) or _text(params.get("waypoint_id")):
        return True
    if _num(params.get("lat")) and _num(params.get("lon")):
        return True
    target = params.get("target")
    if isinstance(target, dict) and _num(target.get("lat")) and _num(target.get("lon")):
        return True
    waypoints = params.get("waypoints")
    return isinstance(waypoints, list) and len(waypoints) > 0


def _patrol_has_route(task: Dict[str, Any]) -> bool:
    """True iff a patrol task carries a non-blank route_id (11 S7.2).

    Only the top-level task.route_id counts: it is the one field
    task_row_from_command maps to tasks.route_geo_id, which is what dispatch
    reads. A route NAME in params.slots.route is provenance, not a route.
    B04 patrol_repeat (16 S6.7.3 [5]: route taken from the current task by
    P3) is refused too: nothing in P3 fills it yet, and an unfilled patrol is
    exactly the empty task this check exists to stop.
    """
    route_id = task.get("route_id")
    return isinstance(route_id, str) and bool(route_id.strip())


def parse_task_command(payload: Dict[str, Any]) -> TaskCommand:
    """Validate a cmd/task payload (11 S7.2) or raise TaskCommandError."""
    if not isinstance(payload, dict):
        raise TaskCommandError(E_SCHEMA, "task command payload is not an object")
    cmd_id = payload.get("cmd_id")
    if not isinstance(cmd_id, str) or not cmd_id:
        # cmd_id is the idempotency key (S2.3). Without it a redelivered frame
        # is indistinguishable from a second intent, and S7.2's submit rule
        # ("a repeat returns duplicate and does NOT re-execute") cannot hold.
        raise TaskCommandError(E_SCHEMA, "task command missing cmd_id")
    action = payload.get("action")
    if action not in TASK_ACTION:
        raise TaskCommandError(E_SCHEMA, f"unknown task action {action!r}")
    task = payload.get("task")
    task_id = payload.get("task_id")
    if action == "submit":
        if not isinstance(task, dict):
            raise TaskCommandError(E_SCHEMA, "submit requires a task body")
        if task.get("type") == "goto" and not _goto_has_destination(task):
            # 11 S7.2 task table: goto REQUIRES a destination ("waypoint_id 或
            # target"). Refused here whoever the sender is: on 2026-10-24 p4
            # sent a goto with params.slots {} (B01 hit on the single character
            # 到), P3 accepted it, and the operator heard "收到" for a task with
            # nowhere to go.
            raise TaskCommandError(
                E_SCHEMA, "goto requires a destination (11 S7.2: waypoint_id "
                "or target)")
        if task.get("type") == "patrol" and not _patrol_has_route(task):
            # 11 S7.2 task table: patrol REQUIRES route_id. P3's dispatch
            # pushes geometry only from route_geo_id, so a patrol without one
            # was stored, set "running" and moved nothing. 2026-10-25 .1.26:
            # four such patrols (route "候选", invented by the tier-2 LLM) were
            # queued from voice, one of them running.
            raise TaskCommandError(
                E_SCHEMA, "patrol requires route_id (11 S7.2)")
        inner = task.get("task_id")
        # S7.2 as corrected 2026-08-20: task.task_id is OPTIONAL. It has to be,
        # because the id form is t-YYYYMMDD-NNN and NNN is a per-day sequence
        # only P3 knows -- p4_agent and p5_gateway are both listed publishers of
        # this key (S2.2) and neither can produce a legal one. Omitted here means
        # "P3 allocates"; the idempotency key is cmd_id either way (S2.3).
        if inner is not None and (not isinstance(inner, str) or not inner):
            raise TaskCommandError(
                E_SCHEMA, "task.task_id must be a non-empty string when given")
        if isinstance(task_id, str) and task_id and inner and task_id != inner:
            # S7.2: on submit the authority is task.task_id, and if both appear
            # they must be equal.
            # Refused rather than resolved by precedence -- two ids in one frame
            # means the sender is confused about which task it is submitting,
            # and picking one would bury that.
            raise TaskCommandError(
                E_SCHEMA,
                "task_id %r and task.task_id %r disagree" % (task_id, inner))
        task_id = inner or (task_id if isinstance(task_id, str) else None)
    elif action in _NEEDS_TASK_ID:
        if not isinstance(task_id, str) or not task_id:
            raise TaskCommandError(
                E_SCHEMA,
                f"action {action!r} requires task_id (S7.2 forbids "
                "'omit = the current task')")
    else:
        # clear_queue: a set operation, no task_id.
        task_id = None
    return TaskCommand(
        cmd_id=cmd_id, action=action, task_id=task_id,
        task=task if isinstance(task, dict) else None,
        resume_mode=payload.get("resume_mode")
        if isinstance(payload.get("resume_mode"), str) else None,
        filter=payload.get("filter")
        if isinstance(payload.get("filter"), dict) else None,
        reason=payload.get("reason") or "",
        source=payload.get("source") or "")


def task_ack(cmd_id: str, result: str, code: str = "OK",
             detail: Optional[Dict[str, Any]] = None,
             message: str = "") -> Dict[str, Any]:
    """Build a cmd/task/ack body (11 S7.7 Ack).

    result is accepted | rejected | duplicate | error. `detail.applied` is
    required by AP-1 for anything at confirmation level >= L1 and must be
    independently readable (AP-2) -- so the appliers put the resulting STATE in
    it, not "ok: true".
    """
    ack: Dict[str, Any] = {"schema": "task_ack_v1", "cmd_id": cmd_id,
                           "result": result, "code": code}
    if message:
        ack["message"] = message
    if detail is not None:
        ack["detail"] = detail
    return ack
