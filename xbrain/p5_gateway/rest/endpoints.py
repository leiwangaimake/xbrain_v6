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

from dataclasses import dataclass

from xbrain.common.errors import E_DEGRADED


@dataclass(frozen=True)
class DegradedResponse:
    status: int
    body: dict


def fences_endpoint(fence_db_degraded: bool):
    """/api/fences precondition: 503 + E_DEGRADED when the fence
    DB is in degraded write mode."""
    if fence_db_degraded:
        return DegradedResponse(status=503,
                                  body={"error": E_DEGRADED})
    return DegradedResponse(status=200, body={"fences": []})
