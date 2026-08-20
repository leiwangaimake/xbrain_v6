"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_apply.py
Brief: cmd/geo action dispatch -- envelope -> per-action applier -> ack (11 S7.9)

Description:
The one entry point P3's wiring calls for a cmd/geo frame. It runs the fixed
order the contract implies and nothing else:

  parse envelope (S7.9.1)  ->  channel matrix (S7.9.5)  ->  applier  ->  ack

and it converts every refusal into a REJECTED ack carrying a closed-set code.
That last part is the reason this file exists as a seam rather than as four
lines inside the wiring loop: a cmd/geo that is dropped on the floor and a
cmd/geo that was applied are indistinguishable to the sender, and the sender in
this system is an operator who just said "save this as the east gate route". A
command must always come back with an answer, including when the answer is "this
action is not built yet".

Actions arrive in batches. An action with no registered applier answers
rejected + E_NOT_IMPLEMENTED and writes NOTHING -- never a silent no-op that a
caller reads as success, and never a partial write. This is deliberate and is
what the accompanying mutation test pins: an applier table that answered
"accepted" for an unbuilt action would let a route be reported saved that no db
row exists for, which is the exact fail-silent shape CLAUDE.md 3.2 catalogues.

Boundaries: it opens no connections (the live geo/fence/task conns are passed
in), reads no clock (now_ms is injected -- wall-clock ms for the audit columns,
per 15 S9.3), and publishes nothing (the wiring owns the ack publisher). It also
does not enforce confirm levels; see the note in geo_command.py.
"""
from __future__ import annotations

import logging
from typing import Any, Awaitable, Callable, Dict, Optional

from xbrain.common.errors import E_INTERNAL, E_NOT_IMPLEMENTED
from xbrain.p3_task.ingest.geo_command import (
    GeoCommand, GeoCommandError, check_channel, geo_ack, parse_geo_command,
)

_logger = logging.getLogger("xbrain.p3.geo")


class GeoContext:
    """The live handles an applier may touch, passed as one object so adding a
    handle later does not re-shape every applier signature.

    task_conn is here because the delete path must reach task.db: GC-1..GC-7
    (15 S9) turn a geo edit into task state changes, and refs (S7.9.4) counts the
    tasks that reference an object. geo/fence stay separate connections because
    15 S9 splits them into two physical files.
    """

    def __init__(self, geo_conn, fence_conn, task_conn=None) -> None:
        self.geo_conn = geo_conn
        self.fence_conn = fence_conn
        self.task_conn = task_conn


# action -> applier. Populated by the per-action batches; an absent action is a
# clean refusal, not a crash and not a pretend-success. Appliers are async and
# return the ack detail dict (or None), never the whole ack -- shaping the ack in
# one place keeps result/code consistent across actions.
Applier = Callable[[GeoCommand, GeoContext, int], Awaitable[Optional[Dict[str, Any]]]]
APPLIERS: Dict[str, Applier] = {}


def register_applier(action: str, fn: Applier) -> None:
    """Bind an applier to an action. Refuses to overwrite an existing binding:
    two modules claiming the same action is a merge accident, and whichever
    imported last would silently win."""
    if action in APPLIERS:
        raise ValueError(f"applier for {action!r} already registered")
    APPLIERS[action] = fn


async def handle_geo_payload(payload: Dict[str, Any], ctx: GeoContext,
                             *, now_ms: int) -> Dict[str, Any]:
    """Run one cmd/geo payload end to end and return the ack body to publish.

    Never raises: every failure path becomes a rejected ack. An exception that
    escaped here would kill the P3 wiring loop, and P3 is the process that also
    holds task scheduling -- one malformed geo frame from a browser must not
    stop patrol tasks from being dispatched.
    """
    # cmd_id is needed for the ack even when parsing failed, so it is read
    # defensively before validation. An unusable cmd_id (missing / not a string)
    # becomes the empty string: the ack is then unroutable by the sender, which
    # is honest -- it says "something arrived that I could not attribute".
    raw_id = payload.get("cmd_id") if isinstance(payload, dict) else None
    cmd_id = raw_id if isinstance(raw_id, str) else ""
    try:
        cmd = parse_geo_command(payload)
        check_channel(cmd)
    except GeoCommandError as exc:
        _logger.warning("p3 cmd/geo refused (%s): %s", exc.code, exc)
        return geo_ack(cmd_id, "rejected", exc.code, {"reason": str(exc)})
    applier = APPLIERS.get(cmd.action)
    if applier is None:
        # Not built yet. Say so; do not touch the db.
        _logger.info("p3 cmd/geo action %r not wired yet (cmd_id=%s)",
                     cmd.action, cmd.cmd_id)
        return geo_ack(cmd.cmd_id, "rejected", E_NOT_IMPLEMENTED,
                       {"action": cmd.action,
                        "reason": "action has no applier in this build"})
    try:
        detail = await applier(cmd, ctx, now_ms)
    except GeoCommandError as exc:
        # The applier's own refusals (conflict, not found, invalid geometry)
        # carry their code the same way the envelope's do.
        _logger.info("p3 cmd/geo %s rejected (%s): %s", cmd.action, exc.code, exc)
        # An applier that supplied no structured detail still owes the sender the
        # reason text -- an ack with a bare code sends the operator to the logs.
        return geo_ack(cmd.cmd_id, "rejected", exc.code,
                       exc.detail if exc.detail is not None
                       else {"reason": str(exc)})
    except Exception as exc:              # noqa: BLE001
        # An applier bug must not be reported as success, and must not take the
        # loop down. E_INTERNAL is the honest code: the command was well formed
        # and permitted, and we failed to apply it.
        _logger.error("p3 cmd/geo %s failed: %s", cmd.action, exc)
        return geo_ack(cmd.cmd_id, "rejected", E_INTERNAL, {"reason": str(exc)})
    return geo_ack(cmd.cmd_id, "accepted", "OK", detail)
