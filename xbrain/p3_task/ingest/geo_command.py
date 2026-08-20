"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_command.py
Brief: cmd/geo GeoCommand envelope parse + S7.9.5 channel permission (11 S7.9)

Description:
P3 is the SINGLE WRITER of the four geographic object kinds (11 S7.9): cloud, the
HMI (origin=hmi) and on-site voice/teach (origin=voice) all send a GeoCommand on
cmd/geo, and ONLY p3_task applies it, then acks on cmd/geo/ack. This module is
the envelope half -- parse + validate + channel permission + ack shape. The
per-action appliers live in geo_apply.py.

Two refusals happen HERE, before any applier sees the command, and both are
security-shaped rather than convenience-shaped:

  * an off-closed-set action / type / origin is E_SCHEMA (CLAUDE.md 3.5: no
    silent pass-through, no "interpret the unknown value as something close");
  * a command whose origin is not permitted for that operation is
    E_CHANNEL_DENIED per the 11 S7.9.5 matrix, checked cell by cell.

Why the channel check is in the envelope and not in each applier: U23 decided the
HMI is NOT authenticated, so the CHANNEL is the only authorisation boundary the
system has (CH-1: origin is the sole discriminant). One applier that forgot to
consult the matrix would be a hole, and a hole in this particular fence means an
un-authenticated browser can delete the camp's keep-in fence. One gate, one
place, ahead of the dispatch.

What this file does NOT do: it reads no clock (idempotency keys off cmd_id, not
time), opens no db (the applier is handed a live conn), and does not check the
confirm LEVEL (L1/L2). Levels are the operator-facing half and belong to the
originating channel -- 17 S12A.9 W4 makes P5 refuse an unconfirmed delete with
E_CONFIRM_REQUIRED before it ever mints a GeoCommand; the GeoCommand schema
(S7.9.1) carries no confirm field at all, so P3 cannot re-derive it and must not
pretend to. Optimistic concurrency (base_rev, S7.9.2) and chunking (S7.9.3) live
with the write appliers that need them.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, Optional, Tuple

# The closed sets are imported, never re-typed (CLAUDE.md 3.5). GEO_ORIGIN in
# particular is generated from the S7.9.5 matrix HEADER, so if a channel column
# is ever added to the contract, the set and the matrix below move together.
from xbrain.common.enums import GEO_ACTION, GEO_ORIGIN, GEO_TYPE
from xbrain.common.errors import E_CHANNEL_DENIED, E_SCHEMA

# Actions that name a single object (need type + geo_id). list/resync are set-wide.
_OBJECT_ACTIONS = frozenset({"upsert", "delete", "rename", "set_state", "get",
                             "refs"})

# 11 S7.9.5 read row: get / list / refs are open to every channel including
# wecom -- they have no side effect, so the matrix treats them like the G-class
# queries. Named here because three separate cells below key off "is this a read".
_READ_ACTIONS = frozenset({"get", "list", "refs"})

# The write channels: every origin except wecom. E-mode (WeChat) is not built
# this period (VOI-50) and ALL its write cells are a deliberate no, not an
# oversight -- CH-4 forbids inheriting hmi's permissions for it, because U23's
# "channel is the boundary" holds only on the camp's own encrypted network and
# wecom is a PUBLIC channel.
_WRITE_ORIGINS = frozenset({"cloud", "hmi", "voice"})
# Cloud only. The three irreversible-or-sweeping operations (CMD-34): deleting a
# fence, forcing an overwrite past a rev conflict, and a full resync.
_CLOUD_ONLY = frozenset({"cloud"})


class GeoCommandError(ValueError):
    """A cmd/geo command is refused. Carries the closed-set error `code` so the
    ack maps to a real E_* rather than free text, plus an optional `detail` dict
    for the codes whose usefulness IS the detail: E_GEO_CONFLICT must return the
    local {rev, content_hash, updated_by, updated_ts} for the sender to retry
    against (S7.9.2 step 3), and a delete refusal returns refs (S7.9.4). Default
    None rather than {} so an ack carries no empty detail key."""

    def __init__(self, code: str, message: str,
                 detail: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(message)
        self.code = code
        self.detail = detail


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
        raise GeoCommandError(E_SCHEMA, "geo command payload is not an object")
    cmd_id = payload.get("cmd_id")
    if not isinstance(cmd_id, str) or not cmd_id:
        raise GeoCommandError(E_SCHEMA, "geo command missing cmd_id")
    action = payload.get("action")
    if action not in GEO_ACTION:
        raise GeoCommandError(E_SCHEMA, f"unknown geo action {action!r}")
    origin = payload.get("origin")
    # An origin outside the matrix columns is refused here rather than mapped to
    # the nearest column. "teach" is the concrete case: it used to appear in the
    # S7.9.1 example and has NO column in S7.9.5, so honouring it would invent a
    # permission row. Teach-recorded objects travel as origin=voice with
    # created_by=teach (S7.8.2).
    if origin not in GEO_ORIGIN:
        raise GeoCommandError(E_SCHEMA, f"unknown geo origin {origin!r}")
    gtype = payload.get("type")
    geo_id = payload.get("geo_id")
    if action in _OBJECT_ACTIONS:
        if gtype not in GEO_TYPE:
            raise GeoCommandError(E_SCHEMA, f"unknown geo type {gtype!r}")
        if not isinstance(geo_id, str) or not geo_id:
            raise GeoCommandError(E_SCHEMA,
                                  f"action {action!r} needs a geo_id")
    base_rev = payload.get("base_rev")
    if base_rev is not None and not isinstance(base_rev, int):
        raise GeoCommandError(E_SCHEMA, "base_rev must be an integer")
    obj = payload.get("obj")
    if action == "upsert" and not isinstance(obj, dict):
        raise GeoCommandError(E_SCHEMA, "upsert requires an obj body")
    return GeoCommand(
        cmd_id=cmd_id, action=action, origin=origin,
        type=gtype if action in _OBJECT_ACTIONS else None,
        geo_id=geo_id if action in _OBJECT_ACTIONS else None,
        base_rev=base_rev, force=bool(payload.get("force", False)),
        obj=obj if isinstance(obj, dict) else None,
        chunk=payload.get("chunk") if isinstance(payload.get("chunk"), dict)
        else None)


def allowed_origins(action: str, gtype: Optional[str],
                    force: bool) -> Tuple[FrozenSet[str], str]:
    """The 11 S7.9.5 cell for this (action, type, force), as (origins, cell_name).

    Returned rather than asserted so the caller can name the violated cell in the
    ack -- "delete fence is cloud-only" is actionable where "denied" is not. The
    cell name is also what the per-cell test cases parametrise over, so a row
    silently dropping out of this function shows up as a missing case rather than
    as a permissive default.
    """
    if force:
        # The force row spans every action: a forced overwrite is cloud-only no
        # matter what it forces (S7.9.5 "force: true 强制覆盖"). Checked FIRST so
        # a forced hmi upsert cannot be waved through by the upsert row below.
        return _CLOUD_ONLY, "force"
    if action in _READ_ACTIONS:
        return frozenset(GEO_ORIGIN), "read"
    if action == "resync":
        return _CLOUD_ONLY, "resync"
    if action == "delete" and gtype == "fence":
        # The one cell where hmi and voice are BOTH denied while they may delete
        # every other type: an operator cannot talk or click the camp's keep-in
        # fence away (CMD-34 / 18 F13).
        return _CLOUD_ONLY, "delete_fence"
    if action == "delete":
        return _WRITE_ORIGINS, "delete_object"
    if action == "upsert":
        # The matrix splits upsert into two rows (route/waypoint/dock, and
        # fence). Both carry the same THREE channels; they differ only in the
        # operator-side confirm level (L1 vs L2, CMD-19), which is P5/P4's gate,
        # not this one. Kept as one cell here on purpose: writing two cells with
        # identical origin sets would look like a difference that does not exist.
        return _WRITE_ORIGINS, "upsert"
    if action == "rename":
        return _WRITE_ORIGINS, "rename"
    if action == "set_state":
        # S7.9.5 states this row as "set_state fence -> active" only; the other
        # types and the other target states are NOT written down. Treated as the
        # same three channels, which is the conservative reading in the one
        # direction that matters: set_state is reversible (it flips a lifecycle
        # flag, it destroys nothing), and the irreversible neighbours (delete
        # fence, force, resync) each have their own explicit cloud-only cell
        # above. Refusing the unwritten cells outright would instead break F15
        # for every non-fence type with no contract line calling for that.
        return _WRITE_ORIGINS, "set_state"
    # Unreachable while GEO_ACTION and the cells above agree. It is a raise and
    # not a permissive default because the failure mode of the other choice is a
    # newly added action that every channel may run.
    raise GeoCommandError(E_SCHEMA, f"action {action!r} has no S7.9.5 cell")


def check_channel(cmd: GeoCommand) -> None:
    """Enforce the 11 S7.9.5 channel matrix, or raise E_CHANNEL_DENIED.

    CH-1: origin is the SOLE discriminant. Not the peer IP, not a client_id, not
    anything else about the connection -- U23 settled that the channel itself is
    the permission boundary, and CH-2 forbids P5 from relabelling an HMI write as
    cloud to get past this function.
    """
    origins, cell = allowed_origins(cmd.action, cmd.type, cmd.force)
    if cmd.origin not in origins:
        raise GeoCommandError(
            E_CHANNEL_DENIED,
            "origin %r may not run %s (11 S7.9.5 cell %r allows %s)"
            % (cmd.origin, cmd.action, cell, sorted(origins)))


def geo_ack(cmd_id: str, result: str, code: str = "OK",
            detail: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Build a cmd/geo/ack body (11 S7.9.4, reusing the S7.7 Ack shape). result is
    accepted | rejected | duplicate; code is a closed-set E_* (or OK)."""
    ack: Dict[str, Any] = {"schema": "geo_ack_v1", "cmd_id": cmd_id,
                           "result": result, "code": code}
    if detail is not None:
        ack["detail"] = detail
    return ack
