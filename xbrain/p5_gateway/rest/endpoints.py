"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: endpoints.py
Brief: GWY-P5-13 REST 8 read-only endpoints + /api/fences* E_DEGRADED precondition

Description:
17 S12 REST surface: EIGHT read-only endpoints. The API is small on
purpose -- state changes go through the WebSocket cmd path (which is
rate-limited and audited). REST is for dashboards + external tools.

  GET /api/health
  GET /api/telemetry/{class}
  GET /api/tasks
  GET /api/tasks/{task_id}
  GET /api/events?since={seq}&limit={n}
  GET /api/dock/status
  GET /api/link/status
  GET /api/fences

/api/fences is the exception: if the p3 fence.db is in
DegradedWriteMode, the endpoint returns HTTP 503 with body
{"error": "E_DEGRADED"}. This surfaces the degraded state to any
caller instead of returning stale or partial data.
"""

from __future__ import annotations

from dataclasses import dataclass


READONLY_ENDPOINTS = frozenset({
    "/api/health",
    "/api/telemetry/{class}",
    "/api/tasks",
    "/api/tasks/{task_id}",
    "/api/events",
    "/api/dock/status",
    "/api/link/status",
    "/api/fences",
})


class EndpointNotAllowed(Exception):
    """Attempted to write via REST -- forbidden by design."""


@dataclass(frozen=True)
class DegradedResponse:
    status: int
    body: dict


def check_readonly(method: str, path_template: str) -> None:
    """Only GET on read-only endpoints. Any other verb -> refuse."""
    if method != "GET":
        raise EndpointNotAllowed(
            f"{method} on {path_template!r}: REST is read-only")
    if path_template not in READONLY_ENDPOINTS:
        raise EndpointNotAllowed(
            f"unknown endpoint {path_template!r}")


def fences_endpoint(fence_db_degraded: bool):
    """/api/fences precondition: 503 + E_DEGRADED when the fence
    DB is in degraded write mode."""
    if fence_db_degraded:
        return DegradedResponse(status=503,
                                  body={"error": "E_DEGRADED"})
    return DegradedResponse(status=200, body={"fences": []})
