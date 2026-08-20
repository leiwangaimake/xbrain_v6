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

*** task_id is required and there is deliberately NO "omit = the current task".
S7.2 gives four reasons and the first is the one that bites: the queue is LIVE.
Between the operator seeing "A is running" on the HMI and the command arriving,
A may have finished and B started -- the shorthand would pause the wrong task,
and nothing in the log would show that it happened. The other three: voice adds
0.5-2 s of ASR + LLM + confirmation on top; idempotency requires a resent cmd_id
to mean the same thing it meant the first time; and an audit has to be able to
reconstruct which task the operator meant.

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
        inner = task.get("task_id")
        if not isinstance(inner, str) or not inner:
            raise TaskCommandError(E_SCHEMA, "submit requires task.task_id")
        if isinstance(task_id, str) and task_id and task_id != inner:
            # S7.2: on submit the authority is task.task_id, and if both appear
            # they must be equal.
            # Refused rather than resolved by precedence -- two ids in one frame
            # means the sender is confused about which task it is submitting,
            # and picking one would bury that.
            raise TaskCommandError(
                E_SCHEMA,
                "task_id %r and task.task_id %r disagree" % (task_id, inner))
        task_id = inner
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


def looks_like_p4_shape(payload: Dict[str, Any]) -> bool:
    """True for p4_agent's legacy private frame (a `task_request` member).

    Kept ONLY for the transition: p4 is being changed to emit the S7.2 shape in
    the next batch, and this predicate is what lets the receiver tell "an old
    p4 frame" from "a malformed contract frame" while both can arrive. It is
    NOT a second accepted shape -- see task_apply.handle_task_payload, which
    logs a deprecation on this path so the transition cannot go quiet.
    """
    return isinstance(payload, dict) and isinstance(
        payload.get("task_request"), dict)
