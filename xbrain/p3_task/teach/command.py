"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: command.py
Brief: cmd/teach TeachCommand envelope (11 S12A.4)

Description:
Parses and validates the cmd/teach payload into a TeachCommand, or refuses it
with a closed-set code. Same division as geo_command.py: the envelope is
checked here, the session machine decides whether the action is legal NOW, and
the runtime performs it.

Two rules from S12A.4 that are refusals rather than conveniences:

  * session_id is REQUIRED for every action except start / mark_once / query,
    and a mismatch against the live session is E_TEACH_STATE. The contract's reason is to keep a
    stale command from landing in a new session, and that is a real sequence: an operator says
    "finish", the ack is lost, they say it again, and by then a second session
    has begun. Without the id check the second recording is finished by a
    command meant for the first.

  * issuer.channel is carried but NOT used to refuse. 18 marks the record_*
    intents as A-channel only, and S12A.4 says a non-local start is ACCEPTED
    with warn: non_local_issuer rather than denied. Recording is already gated
    by the seven arming checks (which include the e-stop path); adding a
    channel denial here would refuse a cloud-initiated recording that is
    physically just as safe.

The mark_once path (F06 / F10) is parsed here too although it opens no session:
it is the same message on the same key, and S12A.8 keeps it there deliberately
so a single-point capture and a route recording cannot drift apart.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from xbrain.common.enums import TEACH_ACTION
from xbrain.common.errors import E_SCHEMA

#: Actions that must carry a session_id (S12A.4 field table). start mints one,
#: mark_once opens none, query may ask about whatever session is current.
_NEEDS_SESSION = frozenset(TEACH_ACTION.values) - {"start", "mark_once",
                                                   "query"}

#: S12A.4: name_hint and save.name are <= 32 characters (18 VD-1).
_MAX_NAME_LEN = 32

#: The two kinds a session records, and the two a mark_once captures. They are
#: disjoint on purpose: a waypoint/dock is one point (S12A.8), a route/fence is
#: a sequence, and the machinery differs all the way down.
SESSION_KINDS = frozenset({"route", "fence"})
MARK_ONCE_KINDS = frozenset({"waypoint", "dock"})


class TeachCommandError(ValueError):
    """A cmd/teach payload is not a well-formed TeachCommand."""

    def __init__(self, code: str, message: str,
                 detail: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(message)
        self.code = code
        self.detail = detail


@dataclass(frozen=True)
class TeachCommand:
    cmd_id: str
    action: str
    session_id: Optional[str]
    issuer_src: str
    issuer_channel: str
    issuer_ip: Optional[str]
    #: Per-action payloads, already shape-checked. None when absent.
    start: Optional[Dict[str, Any]] = None
    mark: Optional[Dict[str, Any]] = None
    mark_once: Optional[Dict[str, Any]] = None
    undo_count: int = 1
    save: Optional[Dict[str, Any]] = None
    reason: str = ""


def _name(value: Any, field: str, required: bool) -> Optional[str]:
    if value is None:
        if required:
            raise TeachCommandError(E_SCHEMA, f"{field} is required")
        return None
    if not isinstance(value, str) or not value.strip():
        raise TeachCommandError(E_SCHEMA, f"{field} must be a non-empty string")
    text = value.strip()
    if len(text) > _MAX_NAME_LEN:
        raise TeachCommandError(
            E_SCHEMA, f"{field} exceeds {_MAX_NAME_LEN} characters")
    return text


def parse_teach_command(payload: Dict[str, Any]) -> TeachCommand:
    """Validate a cmd/teach payload (11 S12A.4) or raise TeachCommandError."""
    if not isinstance(payload, dict):
        raise TeachCommandError(E_SCHEMA, "teach payload is not an object")
    cmd_id = payload.get("cmd_id")
    if not isinstance(cmd_id, str) or not cmd_id:
        raise TeachCommandError(E_SCHEMA, "teach command missing cmd_id")
    action = payload.get("action")
    if action not in TEACH_ACTION:
        raise TeachCommandError(E_SCHEMA, f"unknown teach action {action!r}")
    session_id = payload.get("session_id")
    if action in _NEEDS_SESSION:
        if not isinstance(session_id, str) or not session_id:
            raise TeachCommandError(
                E_SCHEMA, f"action {action!r} requires a session_id")
    elif not isinstance(session_id, str):
        session_id = None
    issuer = payload.get("issuer")
    if not isinstance(issuer, dict):
        # issuer.src drives orphan detection (S12A.11): without it, a session
        # whose issuer died can never be recognised as orphaned and would hold
        # the global single-session slot until the process restarts.
        raise TeachCommandError(E_SCHEMA, "teach command missing issuer")
    src = issuer.get("src")
    channel = issuer.get("channel")
    if not isinstance(src, str) or not src:
        raise TeachCommandError(E_SCHEMA, "issuer.src is required")
    if not isinstance(channel, str) or not channel:
        raise TeachCommandError(E_SCHEMA, "issuer.channel is required")
    start = mark = mark_once = save = None
    if action == "start":
        start = _parse_start(payload.get("start"))
    elif action == "mark":
        raw = payload.get("mark")
        mark = raw if isinstance(raw, dict) else {}
    elif action == "mark_once":
        mark_once = _parse_mark_once(payload.get("mark_once"))
    elif action == "save":
        save = _parse_save(payload.get("save"))
    undo_count = 1
    if action == "undo":
        raw = (payload.get("undo") or {}).get("count", 1)
        if not isinstance(raw, int) or isinstance(raw, bool) or raw < 1:
            raise TeachCommandError(E_SCHEMA, "undo.count must be >= 1")
        # The 1..10 cap is applied by the recorder, which also knows how many
        # points exist; refusing 11 here would be a second, different limit.
        undo_count = raw
    return TeachCommand(
        cmd_id=cmd_id, action=action, session_id=session_id,
        issuer_src=src, issuer_channel=channel,
        issuer_ip=issuer.get("op_ip") if isinstance(issuer.get("op_ip"), str)
        else None,
        start=start, mark=mark, mark_once=mark_once, undo_count=undo_count,
        save=save, reason=payload.get("reason") or "")


def _parse_start(raw: Any) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise TeachCommandError(E_SCHEMA, "start requires a start block")
    kind = raw.get("kind")
    if kind not in SESSION_KINDS:
        raise TeachCommandError(
            E_SCHEMA,
            f"start.kind must be one of {sorted(SESSION_KINDS)}, got {kind!r}")
    require_fix = raw.get("require_fix", "rtk_fixed")
    if require_fix not in ("rtk_fixed", "rtk_float"):
        # S12A.4 allows exactly these two. Accepting 'single' would let a route
        # be recorded at metre-scale error and then driven as if surveyed.
        raise TeachCommandError(
            E_SCHEMA, f"start.require_fix {require_fix!r} is not allowed")
    return {"kind": kind,
            "name_hint": _name(raw.get("name_hint"), "start.name_hint", False),
            "sample": raw.get("sample") if isinstance(raw.get("sample"), dict)
            else None,
            "require_fix": require_fix,
            "max_duration_s": raw.get("max_duration_s")}


def _parse_mark_once(raw: Any) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise TeachCommandError(E_SCHEMA, "mark_once requires a mark_once block")
    kind = raw.get("kind")
    if kind not in MARK_ONCE_KINDS:
        raise TeachCommandError(
            E_SCHEMA,
            f"mark_once.kind must be one of {sorted(MARK_ONCE_KINDS)}")
    capture_heading = bool(raw.get("capture_heading", kind == "dock"))
    if kind == "dock" and not capture_heading:
        # S12A.8: a dock's handover orientation IS the captured heading, so a
        # dock without it is not a dock. Refused rather than defaulted quietly.
        raise TeachCommandError(
            E_SCHEMA, "mark_once(dock) requires capture_heading=true")
    return {"kind": kind,
            "name": _name(raw.get("name"), "mark_once.name", True),
            "capture_heading": capture_heading,
            "overwrite": bool(raw.get("overwrite", False))}


def _parse_save(raw: Any) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise TeachCommandError(E_SCHEMA, "save requires a save block")
    return {"name": _name(raw.get("name"), "save.name", True),
            "overwrite": bool(raw.get("overwrite", False)),
            # S12A.7 constraint 1: saving is not activating. The default is
            # false and stays false unless the sender asks -- and an activating
            # save is refused when the robot is outside the new fence.
            "activate": bool(raw.get("activate", False))}


def teach_ack(cmd_id: str, result: str, code: str = "OK",
              detail: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Build a cmd/teach/ack body (S12A.4, reusing the S7.7 Ack shape)."""
    ack: Dict[str, Any] = {"schema": "teach_ack_v1", "cmd_id": cmd_id,
                           "result": result, "code": code}
    if detail is not None:
        ack["detail"] = detail
    return ack
