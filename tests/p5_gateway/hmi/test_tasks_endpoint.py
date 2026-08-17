"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_tasks_endpoint.py
Brief: GET /api/tasks route -- relays P3 query/tasks, honest gating (11 S12.2A)

Description:
Guards the HMI GET /api/tasks route: it relays the provider's query_tasks (which
P5 backs with a Zenoh get() to P3), validates scope, clamps limit, and -- for a
provider with no query_tasks wired -- returns available:false rather than 500.
The HTTP query string uses '&' (standard); the route turns that into the ';'
zenoh selector inside the client, so this test drives the route with '&'. Each
check names its mutation (CLAUDE.md 3.3).
"""
from __future__ import annotations

import os

# repo root = four levels up: hmi -> p5_gateway -> tests -> /opt/xbrain_v6.
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
_STATIC = os.path.join(_REPO, "xbrain", "p5_gateway", "hmi", "static")

_MIN_WEB = {"push_hz": 2, "map": {}, "font": {}, "layout": {},
            "fence": {}, "route": {}, "waypoint": {}}


class _BaseProvider:
    """Minimal provider (no query_tasks) -- the gated case."""

    def snapshot_inputs(self):
        return {"fences": None, "routes": None, "waypoints": None,
                "enu_origin": None, "pose": None, "tasks": None, "mode": None,
                "link": None, "health": None, "events": None}

    def fence_degraded(self):
        return True

    def rest_inputs(self):
        return {"health": None, "bit": None, "routes": None, "docks": None,
                "metrics": None, "approval_pending": None}


class _ProviderWithTasks(_BaseProvider):
    """Wired provider: query_tasks records the args and returns a page (the real
    one does a Zenoh get() to P3's queryable; the route does not care which)."""

    def __init__(self):
        self.last = None

    def query_tasks(self, scope, limit, before):
        self.last = (scope, limit, before)
        return {"tasks": [{"task_id": "t-1", "state": scope}],
                "has_more": False, "next_before": None}


def _client(provider):
    from fastapi.testclient import TestClient
    from xbrain.p5_gateway.hmi.web_server import build_app
    return TestClient(build_app(_MIN_WEB, provider, lambda: None, _STATIC))


def test_api_tasks_relays_provider_query():
    # The route hands scope/limit/before to query_tasks and returns its page.
    # MUTATION: dropping the route -> 404; ignoring the provider -> stale/empty.
    p = _ProviderWithTasks()
    resp = _client(p).get("/api/tasks?scope=history&limit=10")
    assert resp.status_code == 200
    body = resp.json()
    assert body["tasks"][0]["task_id"] == "t-1"
    assert p.last == ("history", 10, None)


def test_api_tasks_passes_before_cursor():
    p = _ProviderWithTasks()
    _client(p).get("/api/tasks?scope=history&limit=5&before=42")
    assert p.last == ("history", 5, 42)


def test_api_tasks_bad_scope_is_400():
    # Only current|history are valid. MUTATION: passing an unknown scope through
    # would let P3 raise / answer nothing; the route must reject it up front.
    resp = _client(_ProviderWithTasks()).get("/api/tasks?scope=bogus")
    assert resp.status_code == 400


def test_api_tasks_limit_clamped():
    # limit is clamped [1,500] at the edge. MUTATION: no clamp -> a client asks
    # for the whole 30-day table in one reply.
    p = _ProviderWithTasks()
    _client(p).get("/api/tasks?scope=current&limit=99999")
    assert p.last[1] == 500
    _client(p).get("/api/tasks?scope=current&limit=0")
    assert p.last[1] == 1


def test_api_tasks_provider_without_query_tasks_is_available_false():
    # A provider that never wired query_tasks (MVP / legacy) -> available:false
    # empty, NEVER 500. MUTATION: calling provider.query_tasks without getattr ->
    # AttributeError -> 500.
    resp = _client(_BaseProvider()).get("/api/tasks?scope=current")
    assert resp.status_code == 200
    body = resp.json()
    assert body["available"] is False and body["tasks"] == []


def test_api_tasks_defaults_to_current():
    p = _ProviderWithTasks()
    _client(p).get("/api/tasks")
    assert p.last[0] == "current"
