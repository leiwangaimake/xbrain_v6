"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_cache.py
Brief: P5 geo cache + shape for the HMI map (11 S7.10A)

Description:
Pins the P5 geo relay: the cache serves the last state/geo/objects payload until a
generous stale window (P3 down) then None; geo_layers reshapes it into the routes/
waypoints the frontend renderGeo needs -- waypoints get geom:[e_m,n_m] (so toXY
places them without an enu_origin), routes pass points through as recorded. Each
check names the mutation it reddens (CLAUDE.md 3.3); the load-bearing one is the
geom-array shape (a {e_m,n_m} object renders NOTHING -- toXY returns null).
"""
from __future__ import annotations

from xbrain.p5_gateway.geo.cache import (
    GEO_STALE_AFTER_MS, GeoCache, geo_layers,
)

_PAYLOAD = {
    "schema": "geo_objects_v1", "catalog_rev": 130,
    "waypoints": [{"geo_id": "w-1", "name": "东门岗亭", "e_m": 12.0, "n_m": -3.5}],
    "routes": [{"geo_id": "r-1", "name": "营区日常",
                "points": [[20.4, 8.1], [12.0, -3.5]]}],
    "docks": [{"geo_id": "d-01", "name": "1号桩", "e_m": -40.0, "n_m": 22.0}],
}


# -- GeoCache ------------------------------------------------------------------

def test_cache_serves_fresh_then_stale_none():
    c = GeoCache()
    assert c.snapshot(1000) is None                 # never received -> None
    c.on_update(_PAYLOAD, 1000)
    assert c.snapshot(1000) is _PAYLOAD             # fresh -> the payload
    # Past the stale window (P3 clearly down) -> None, map greys, no stale set.
    # MUTATION: dropping the stale check shows a dead geo set forever.
    assert c.snapshot(1000 + GEO_STALE_AFTER_MS + 1) is None


# -- geo_layers reshape --------------------------------------------------------

def test_none_payload_is_unavailable():
    # No payload -> (None, None): the snapshot marks those layers unavailable
    # (grey), never an empty-set-as-authoritative.
    assert geo_layers(None) == (None, None)


def test_waypoint_reshaped_to_geom_array():
    routes, wps = geo_layers(_PAYLOAD)
    # geom is an [e_m, n_m] ARRAY -- toXY places only an array (or {lat,lon}), NOT
    # separate e_m/n_m fields. MUTATION: passing {e_m,n_m} through renders nothing.
    assert wps[0]["geom"] == [12.0, -3.5]
    assert wps[0]["name"] == "东门岗亭" and wps[0]["recorded"] is True


def test_route_points_and_kind():
    routes, wps = geo_layers(_PAYLOAD)
    assert routes[0]["name"] == "营区日常"
    assert routes[0]["points"] == [[20.4, 8.1], [12.0, -3.5]]
    # kind 'recorded' -> the recorded-route layer, NOT the yellow realtime trail
    # (that is the live pose trace, not a stored route). MUTATION: 'realtime' would
    # draw a stored route as the live trail.
    assert routes[0]["kind"] == "recorded"


def test_waypoint_without_coords_skipped():
    # A row missing e_m/n_m cannot be placed -> dropped, not rendered at (0,0).
    payload = {"waypoints": [{"geo_id": "w-x", "name": "坏点"}], "routes": []}
    _routes, wps = geo_layers(payload)
    assert wps == []
