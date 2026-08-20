"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_command.py
Brief: cmd/geo GeoCommand parse + validate + ack + dispatch shell (11 S7.9)

Description:
P3 is the SINGLE WRITER of the four geographic object kinds (11 S7.9): cloud, the
HMI (origin=hmi) and on-site voice/teach (origin=voice/teach) all send a
GeoCommand on cmd/geo, and ONLY p3_task applies it, then acks on cmd/geo/ack. This
module is the parse + validate + ack half plus the dispatch SHELL; the per-action
appliers (upsert/delete/rename/set_state/get/list/refs/resync) land action by
action on top of it. Until an action is wired it returns a clean rejected ack with
E_UNSUPPORTED, NEVER a silent no-op or a partial write -- an unwired command must
be visibly refused so a caller never believes a route was recorded that was not.

What this file does NOT do: it reads no clock (idempotency keys off cmd_id, not
time), opens no db (the applier is handed a live conn), and decides no channel
policy beyond the origin closed set (the S7.9.5 permission matrix -- which origin
may do which action -- is a later batch). Optimistic concurrency (base_rev, S7.9.2)
and chunking (S7.9.3) also land with the write actions that need them.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

# 11 S7.9.1 closed sets. Kept as module frozensets for now (a fence_role-style
# sets.yaml promotion is a follow-up, tracked with the other geo enums); an
# off-set value is REFUSED here, never passed through (CLAUDE.md 3.5 spirit).
GEO_ACTIONS = frozenset({
    "upsert", "delete", "rename", "set_state", "get", "list", "refs", "resync",
})
GEO_TYPES = frozenset({"route", "waypoint", "dock", "fence"})
GEO_ORIGINS = frozenset({"cloud", "hmi", "voice", "teach"})
# Actions that name a single object (need type + geo_id). list/resync are set-wide.
_OBJECT_ACTIONS = frozenset({"upsert", "delete", "rename", "set_state", "get",
                             "refs"})


class GeoCommandError(ValueError):
    """A cmd/geo payload is not a well-formed GeoCommand (11 S7.9.1). Carries the
    closed-set error `code` so the ack maps to a real E_* rather than free text."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class GeoCommand:
    cmd_id: str
    action: str
    origin: str
    type: Optional[str]          # None for list/resync (set-wide)
    geo_id: Optional[str]        # None for list/resync
    base_rev: Optional[int]      # optimistic-concurrency anchor (S7.9.2)
    force: bool
    obj: Optional[Dict[str, Any]]
    chunk: Optional[Dict[str, Any]]


def parse_geo_command(payload: Dict[str, Any]) -> GeoCommand:
    """Validate a cmd/geo payload into a GeoCommand (11 S7.9.1) or raise
    GeoCommandError(E_SCHEMA). Only the envelope is checked here -- the GeoObject
    body (obj) is validated by the upsert applier against 11 S7.8, and base_rev is
    arbitrated by the applier against the stored rev (S7.9.2)."""
    if not isinstance(payload, dict):
        raise GeoCommandError("E_SCHEMA", "geo command payload is not an object")
    cmd_id = payload.get("cmd_id")
    if not isinstance(cmd_id, str) or not cmd_id:
        raise GeoCommandError("E_SCHEMA", "geo command missing cmd_id")
    action = payload.get("action")
    if action not in GEO_ACTIONS:
        raise GeoCommandError("E_SCHEMA", f"unknown geo action {action!r}")
    origin = payload.get("origin")
    if origin not in GEO_ORIGINS:
        raise GeoCommandError("E_SCHEMA", f"unknown geo origin {origin!r}")
    gtype = payload.get("type")
    geo_id = payload.get("geo_id")
    if action in _OBJECT_ACTIONS:
        if gtype not in GEO_TYPES:
            raise GeoCommandError("E_SCHEMA", f"unknown geo type {gtype!r}")
        if not isinstance(geo_id, str) or not geo_id:
            raise GeoCommandError("E_SCHEMA",
                                  f"action {action!r} needs a geo_id")
    base_rev = payload.get("base_rev")
    if base_rev is not None and not isinstance(base_rev, int):
        raise GeoCommandError("E_SCHEMA", "base_rev must be an integer")
    obj = payload.get("obj")
    if action == "upsert" and not isinstance(obj, dict):
        raise GeoCommandError("E_SCHEMA", "upsert requires an obj body")
    return GeoCommand(
        cmd_id=cmd_id, action=action, origin=origin,
        type=gtype if action in _OBJECT_ACTIONS else None,
        geo_id=geo_id if action in _OBJECT_ACTIONS else None,
        base_rev=base_rev, force=bool(payload.get("force", False)),
        obj=obj if isinstance(obj, dict) else None,
        chunk=payload.get("chunk") if isinstance(payload.get("chunk"), dict)
        else None)


def geo_ack(cmd_id: str, result: str, code: str = "OK",
            detail: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Build a cmd/geo/ack body (11 S7.9.4, reusing the S7.7 Ack shape). result is
    accepted | rejected | duplicate; code is a closed-set E_* (or OK)."""
    ack: Dict[str, Any] = {"schema": "geo_ack_v1", "cmd_id": cmd_id,
                           "result": result, "code": code}
    if detail is not None:
        ack["detail"] = detail
    return ack
