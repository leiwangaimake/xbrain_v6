"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: teach_request.py
Brief: F01-F10 voice intents -> cmd/teach TeachCommand (11 S12A.1)

Description:
The recording half of the F class. 11 S12A.1 maps the fifteen F intents onto two
keys, and this module owns the ten that go to cmd/teach; geo_request.py owns the
five that go to cmd/geo.

*** What this replaces. F01 and F07 used to be mapped to a task_type of "teach"
and published on cmd/task -- i.e. recording was modelled as a TASK. 11 S12A.2
settles it the other way: a recording is a SESSION owned by P3, with its own
state machine, its own key and its own ack. The task model could not express
mark / undo / pause / finish / save at all, so F02-F06 and F08-F10 had no
outlet whatsoever -- an operator could start a recording and then had no way to
stop, name, or discard it. That is the gap this closes.

session_id: every action except start and mark_once must carry the id of the
session it means (S12A.4). P4 does not invent it -- it reads the live
state/teach broadcast. When there is no session, the command is not built and
the caller answers "there is no recording in progress" instead of sending a
command that P3 would refuse with E_TEACH_STATE.

Boundary: this builds a payload. It does not publish, does not confirm (L1/L2
levels are the orchestrator's), and does not decide whether the operator is
allowed to record -- the seven arming checks live in P3, which is the only side
that can see health, e-stop path and battery.
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

#: 18 intent name -> (teach action, kind). kind is None for the actions that
#: operate on whatever session is open. Transcribed from the S12A.1 table.
_TEACH_INTENTS: Dict[str, tuple] = {
    "record_route_start": ("start", "route"),        # F01
    "record_route_stop": ("finish", None),           # F02
    "record_route_save": ("save", None),             # F03
    "record_route_cancel": ("discard", None),        # F04
    "record_route_mark": ("mark", None),             # F05
    "record_waypoint": ("mark_once", "waypoint"),    # F06
    "record_fence_start": ("start", "fence"),        # F07
    "record_fence_stop": ("finish", None),           # F08
    "record_fence_save": ("save", None),             # F09
    "record_dock": ("mark_once", "dock"),            # F10
}

#: Actions that need a live session id (S12A.4). Kept as data so a new mapping
#: row cannot forget it -- the failure would be a command P3 refuses with a code
#: the operator cannot act on.
_NEEDS_SESSION = frozenset({"finish", "save", "discard", "mark", "undo",
                            "pause", "resume"})

#: The issuer.channel value per P4 source. S12A.4 records it for audit and for
#: the non_local_issuer warning; it never denies (see command.py).
_CHANNEL = {"voice": "local_voice", "text": "cloud_text"}


class TeachRequestError(RuntimeError):
    """The intent maps to a teach action that cannot be built from this turn."""


def is_teach_intent(intent_name: str) -> bool:
    return intent_name in _TEACH_INTENTS


def to_teach_command(intent_name: str, *, slots: Mapping[str, Any],
                     cmd_id: str, source: str = "voice",
                     session_id: Optional[str] = None,
                     op_ip: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """Build the cmd/teach payload for an F01-F10 turn, or None.

    None means "this intent is not a teach intent". A teach intent that cannot
    be completed raises TeachRequestError with a reason the caller can speak --
    the two cases are different and must not collapse: the first is "not my
    family", the second is "yours, but I need something you did not say".
    """
    entry = _TEACH_INTENTS.get(intent_name)
    if entry is None:
        return None
    action, kind = entry
    if action in _NEEDS_SESSION and not session_id:
        raise TeachRequestError("no recording session is open")
    payload: Dict[str, Any] = {
        "cmd_id": cmd_id,
        "action": action,
        "issuer": {"src": "p4_agent",
                   "channel": _CHANNEL.get(source, "local_voice")},
    }
    if op_ip:
        payload["issuer"]["op_ip"] = op_ip
    if session_id and action in _NEEDS_SESSION:
        payload["session_id"] = session_id
    if action == "start":
        start: Dict[str, Any] = {"kind": kind}
        # name_hint is optional and only used for the spoken restatement and the
        # draft name; the real name arrives with F03/F09. Taken from the slot
        # when the operator said one ("start recording the east gate route").
        hint = slots.get("name")
        if isinstance(hint, str) and hint.strip():
            start["name_hint"] = hint.strip()
        payload["start"] = start
    elif action == "save":
        name = slots.get("name")
        if not isinstance(name, str) or not name.strip():
            # F03/F09 carry a `name` slot; without it there is nothing to save
            # under. Asking again is right -- inventing a name would leave the
            # operator with an object they cannot refer to by voice.
            raise TeachRequestError("save needs a name")
        # activate stays FALSE for a voice save even on a fence: 11 S12A.7
        # constraint 1 makes enabling a separate L2 action (F15). A spoken
        # "save the fence" must not change where the robot may go.
        payload["save"] = {"name": name.strip(), "overwrite": False,
                           "activate": False}
    elif action == "mark_once":
        name = slots.get("name")
        if not isinstance(name, str) or not name.strip():
            raise TeachRequestError("a keypoint needs a name")
        payload["mark_once"] = {
            "kind": kind, "name": name.strip(),
            # S12A.8: a dock's handover orientation IS the captured heading, so
            # F10 always captures it. F06 captures it too when it is available
            # -- a keypoint with a heading can be approached the same way twice.
            "capture_heading": True, "overwrite": False}
    elif action == "discard":
        payload["reason"] = "operator_cancel"
    return payload


def session_id_from_state(teach_state: Optional[Mapping[str, Any]]) -> Optional[str]:
    """The live session id from a state/teach body, or None.

    Only a session that is actually running counts. A `closed` session lingers
    for 60 s for idempotent queries (S12A.3), and treating that as current would
    send finish/save at a recording that has already ended.
    """
    if not isinstance(teach_state, Mapping):
        return None
    session = teach_state.get("session")
    if not isinstance(session, Mapping):
        return None
    if session.get("state") in ("recording", "paused", "finalizing", "arming"):
        sid = session.get("session_id")
        return sid if isinstance(sid, str) and sid else None
    return None
