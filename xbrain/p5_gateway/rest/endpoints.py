"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: endpoints.py
Brief: /api/fences* E_DEGRADED precondition (17 S6.9 P5F-2 / 11 S9A.11)

Description:
The problem this solves. The 17 S6.5 read-only REST surface has one hard
precondition that is NOT a plain read: /api/fences* must return 503 E_DEGRADED
(never 200 + empty set) while the fence geometry is unsynchronised (P5F-2).
fences_endpoint() is that decision, and the live HMI web server (web_server.py
build_app) calls it for exactly that.

What USED to be here, and why it is gone (2026-08-14). This module also carried
a check_readonly() guard over a READONLY_ENDPOINTS whitelist (self-labelled
"GWY-P5-13", citing "17 S12"). That was removed because it was BOTH dead and
wrong: dead -- nothing live imported it (web_server uses only fences_endpoint),
only a test did; wrong -- its 8-entry list (telemetry/tasks/dock/link/...) never
matched the frozen contract (11 S12.2 / 17 S6.5) NOR its own work-ticket, which
cites the S6.5 set. Rewriting a dead, stale constant would only fake alignment
(3.2). The FULL REST-surface reconciliation -- which register wins (11 S12.2 vs
17 S6.5 genuinely differ), rebuilding the endpoint set to it, and the real
GWY-P5-13 acceptance (read-only guard, E_DEGRADED triples, events sort key) --
is deferred to the actual GWY-P5-13 implementation, tracked in NEXT.md S7.1.

Boundary. REST is read-only on purpose: every state change goes through the
WebSocket cmd path (rate-limited + audited, 17 S6.5). The read-only-ness is a
contract property (F-8 frozen surface), NOT enforced here by a runtime guard --
the contract's runtime refuse-startup guards (HW-1/HW-2, 11 S12.1.4) govern the
WS UPLINK whitelist, not this REST read surface.
"""

from __future__ import annotations

from dataclasses import dataclass  # the response pair
from typing import Any, Dict, Mapping, Optional, Sequence  # signatures

from xbrain.common.errors import E_DEGRADED


@dataclass(frozen=True)
class DegradedResponse:
    status: int
    body: dict


def fence_set_summary(meta: Optional[Mapping[str, Any]],
                      fences: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    """One 11 S9A.11 FenceSet summary row: fence_set_id / rev / name / active /
    polygons / vertices / updated_ts. P5 holds only the ACTIVE set (cmd/fence
    carries the set every consumer runs on), so active is always true and the
    list has one row. Header fields cmd/fence did not carry stay None, never
    invented; polygons / vertices are counts, the geometry itself is
    /api/fences/active."""
    m = meta or {}  # no header seen yet -> the header fields stay None
    return {"fence_set_id": m.get("fence_set_id"), "rev": m.get("rev"),  # identity
            "name": m.get("name"), "active": True,  # P5 holds only the active set
            "polygons": len(fences),  # a count; geometry is /api/fences/active
            "vertices": sum(len(f.get("vertices") or []) for f in fences),  # total
            "updated_ts": m.get("updated_ts")}  # wall clock, display only (B5)


def fences_endpoint(fence_db_degraded: bool,
                    meta: Optional[Mapping[str, Any]] = None,
                    fences: Optional[Sequence[Mapping[str, Any]]] = None):
    """/api/fences: 503 + E_DEGRADED while the fence cache is not synced (17
    S6.9 P5F-2), else the 11 S9A.11 summary list.

    ~~Until 2026-11-12 a synced cache answered {"fences": []}~~: a 200 empty
    set is exactly what P5F-2 forbids for the unsynced case, and here it was
    returned for the synced one -- a client could not tell "no fences" from
    "fences exist". The summary now comes from the same cmd/fence cache as
    /api/fences/active."""
    if fence_db_degraded:  # unsynced: say so, never a 200 empty set (P5F-2)
        return DegradedResponse(status=503,
                                  body={"error": E_DEGRADED})
    rows = [fence_set_summary(meta, fences)] if fences else []  # one active set or none
    return DegradedResponse(status=200, body={"fences": rows})  # the S9A.11 list
