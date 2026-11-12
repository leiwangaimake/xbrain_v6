"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_rest_endpoints_w8.py
Brief: HMI-W8 17 S6.5 read-only endpoint set + honest availability

Description:
Guards W8: the HMI web server serves the full 17 S6.5 (== 11 S12.2) read-only
endpoint set, and each endpoint reports availability HONESTLY -- available:false
+ empty when its source is not wired, the authoritative payload when it is. The
trap W8 must not fall into is a 200 empty body a client cannot tell from "no
data" (the P5F-2 / 3.2 fail-silent lesson): every claim below is paired with the
mutant that would turn it red (3.3).

Two layers:
  * the readers (rest_list_endpoint / rest_object_endpoint) -- the availability
    logic, tested with plain values;
  * the routes -- a TestClient build_app smoke test proving all six endpoints
    exist and thread the provider's rest_inputs through, including the getattr
    fallback for a provider that predates rest_inputs.
"""

from __future__ import annotations

import os

import pytest

from xbrain.p5_gateway.hmi import data_readers as D

# -- readers: honest availability -------------------------------------------

def test_list_endpoint_none_is_unavailable_not_empty():
    # None source -> available:false, NOT a 200 empty set posing as authoritative.
    # RED MUTANT: return available:true on None -> a client reads "no routes
    # exist" when the truth is "not wired" (the P5F-2 fail-silent).
    body = D.rest_list_endpoint(None, "routes")
    assert body == {"available": False, "routes": []}


def test_list_endpoint_wired_passes_through():
    # A wired source -> available:true + the list.
    # RED MUTANT: hardcode [] -> real routes vanish.
    body = D.rest_list_endpoint([{"id": "r1"}], "routes")
    assert body["available"] is True
    assert body["routes"] == [{"id": "r1"}]


def test_object_endpoint_none_is_unavailable_null():
    # /api/health|bit|metrics with no source -> available:false + null payload.
    # RED MUTANT: default value to {} -> an empty object reads as a real snapshot.
    body = D.rest_object_endpoint(None, "health")
    assert body == {"available": False, "health": None}


def test_object_endpoint_passthrough_not_recomputed():
    # P5 RELAYS P2's payload verbatim (G-2 same-source), does not reshape it.
    # RED MUTANT: project/rename fields -> a second, drifting truth source.
    factor = {"level": 1, "factor": 0.8, "items": {"cpu": "ok"}}
    body = D.rest_object_endpoint(factor, "health")
    assert body == {"available": True, "health": factor}


# -- routes: full 17 S6.5 set via TestClient --------------------------------

# repo root = four levels up: hmi -> p5_gateway -> tests -> /opt/xbrain_v6.
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
_STATIC = os.path.join(_REPO, "xbrain", "p5_gateway", "hmi", "static")

# build_ui_config only requires the six presentation groups to be present
# (it passes them through verbatim), so a minimal dict is a valid hmi.web.
_MIN_WEB = {"push_hz": 2, "map": {}, "font": {}, "layout": {},
            "fence": {}, "route": {}, "waypoint": {}}


class _Provider:
    """Fake with health wired (like P2 health/factor) and the geo-gated sources
    left None -- mirrors the MVP wiring so the test asserts the real behaviour."""

    def snapshot_inputs(self):
        return {"fences": None, "routes": None, "waypoints": None,
                "enu_origin": None, "pose": None, "tasks": None, "mode": None,
                "link": None, "health": {"level": 0}, "events": None}

    def fence_degraded(self):
        return True

    def rest_inputs(self):
        return {"health": {"level": 0}, "bit": None, "routes": None,
                "docks": None, "metrics": None, "approval_pending": None}


class _LegacyProvider(_Provider):
    """A provider that predates rest_inputs (the getattr fallback path)."""
    rest_inputs = None  # shadow the method -> not callable


def _client(provider):
    from fastapi.testclient import TestClient

    from xbrain.p5_gateway.hmi.web_server import build_app
    app = build_app(_MIN_WEB, provider, lambda: None, _STATIC)
    return TestClient(app)


@pytest.mark.parametrize("path,key,available", [
    ("/api/routes", "routes", False),          # no fresh geo broadcast in this fake
    ("/api/docks", "docks", False),            # same source
    ("/api/health", "health", True),           # P2 health/factor wired
    ("/api/bit", "bit", False),                # no health/bit yet
    ("/api/metrics", "metrics", False),        # aggregator gated
    ("/api/approval/pending", "pending", False),  # L3 queue gated
])
def test_s65_endpoint_exists_and_reports_honestly(path, key, available):
    # Every 17 S6.5 endpoint must exist (200, not 404) and report the true
    # availability of its source. RED MUTANT: drop the route -> 404; or hardcode
    # available:true -> the gated ones lie.
    resp = _client(_Provider()).get(path)
    assert resp.status_code == 200
    body = resp.json()
    assert body["available"] is available
    assert key in body


def test_legacy_provider_without_rest_inputs_degrades_not_crashes():
    # A provider missing rest_inputs must serve the extra endpoints as
    # available:false, never 500. RED MUTANT: call provider.rest_inputs()
    # directly (no getattr) -> AttributeError -> 500.
    resp = _client(_LegacyProvider()).get("/api/metrics")
    assert resp.status_code == 200
    assert resp.json()["available"] is False


class _GeoProvider(_Provider):
    """Fake with a fresh geo broadcast and a synced fence cache."""

    _FENCES = [{"geo_id": "f-1", "name": "area", "role": "allow",
                "vertices": [{"lat": 1, "lon": 2}] * 4},
               {"geo_id": "f-2", "name": "no-go", "role": "forbid",
                "vertices": [{"lat": 1, "lon": 2}] * 3}]

    def snapshot_inputs(self):
        d = super().snapshot_inputs()
        d["fences"] = self._FENCES
        return d

    def fence_degraded(self):
        return False

    def fence_set_meta(self):
        return {"fence_set_id": "fs-site", "rev": 7, "name": "site", "updated_ts": 1.5}

    def rest_inputs(self):
        d = super().rest_inputs()
        d["routes"] = [{"geo_id": "r-1", "name": "1号路径", "points": [], "kind": "recorded"}]
        d["waypoints"] = [{"geo_id": "w-1", "name": "1号充电站", "type": "dock"},
                          {"geo_id": "w-2", "name": "2号测试点", "type": "poi"}]
        d["docks"] = [d["waypoints"][0]]
        return d


def test_fences_returns_the_s9a11_summary_when_synced():
    """11 S9A.11: /api/fences is the FenceSet summary list (2026-11-12; was a
    fixed {"fences": []} even when synced). MUTATION: back to [] -> red; counts
    from the wrong field -> red."""
    resp = _client(_GeoProvider()).get("/api/fences")
    assert resp.status_code == 200
    assert resp.json() == {"fences": [{
        "fence_set_id": "fs-site", "rev": 7, "name": "site", "active": True,
        "polygons": 2, "vertices": 7, "updated_ts": 1.5}]}
    degraded = _client(_Provider()).get("/api/fences")
    assert degraded.status_code == 503  # P5F-2: never 200 [] while unsynced


def test_routes_and_docks_come_from_the_geo_cache():
    """17 S6.5: routes + keypoints, and the charging stations (type-dock
    keypoints, 99 U119). MUTATION: routes still None -> available false -> red;
    waypoints not attached -> red."""
    c = _client(_GeoProvider())
    r = c.get("/api/routes").json()
    assert r["available"] is True and r["routes"][0]["geo_id"] == "r-1"
    assert [w["geo_id"] for w in r["waypoints"]] == ["w-1", "w-2"]
    d = c.get("/api/docks").json()
    assert d == {"available": True, "docks": [{"geo_id": "w-1", "name": "1号充电站",
                                               "type": "dock"}]}
