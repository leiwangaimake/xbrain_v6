"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cache.py
Brief: P5 geo cache + shape for the HMI map (state/geo/objects, 11 S7.10A)

Description:
P3 broadcasts the full geo geometry (routes/keypoints/docks) on state/geo/objects
(11 S7.10A, 变更即发 + 0.1 Hz). P5 caches the last payload here and, at snapshot
time, reshapes it into the routes/waypoints lists the frontend renderGeo consumes
-- P5 NEVER reads geo.db (11 S7843). Geo is semi-static, so the last-known payload
is served until a generous stale window (P3 clearly down) rather than blanked on a
brief silence.

The shape conversion is the load-bearing detail: GeoObjects (v1.5 PLAN A) carries
a keypoint's position as SEPARATE lat/lon fields, and the frontend's toXY projects
a {lat,lon} object through enu_origin; so waypoints are reshaped to geom:{lat,lon}
and routes' points (already {lat,lon} objects from P3) pass through. A wrong shape
here renders nothing (toXY returns null) -- exactly the bug that looks like "no
geo". Route points must stay {lat,lon} OBJECTS: a bare [a,b] array would be read as
ENU metres and land the route in the wrong frame.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

# Geo publishes every 5 s (11 S7.10A GEO_PUBLISH_PERIOD_S); treat it as gone only
# after several missed cycles -- geo is static, a brief hiccup must not blank the
# map. NOT a safety gate (unlike the fence P5F-2 degrade), just a liveness bound.
GEO_STALE_AFTER_MS = 30000


@dataclass
class GeoCache:
    """Holds the last state/geo/objects payload P5 received."""
    payload: Optional[Dict[str, Any]] = None
    updated_ms: int = 0

    def on_update(self, payload: Dict[str, Any], now_ms: int) -> None:
        self.payload = payload
        self.updated_ms = now_ms

    def snapshot(self, now_ms: int,
                 stale_after_ms: int = GEO_STALE_AFTER_MS
                 ) -> Optional[Dict[str, Any]]:
        """The current geo payload, or None when never received / gone stale
        (P3 down). None -> the map greys those layers, never a fabricated set."""
        if self.payload is None:
            return None
        if (now_ms - self.updated_ms) > stale_after_ms:
            return None
        return self.payload


def geo_layers(payload: Optional[Dict[str, Any]]
               ) -> Tuple[Optional[List[Dict[str, Any]]],
                          Optional[List[Dict[str, Any]]]]:
    """Reshape a GeoObjects payload into (routes, waypoints) for the HMI snapshot.

    Returns (None, None) when there is no payload -> the snapshot marks those
    layers unavailable (grey), never an empty-set-as-authoritative. Waypoints get
    geom:{lat,lon} (so toXY projects them through enu_origin); routes keep their
    [{lat,lon},...] points and are tagged kind:"recorded" (the yellow realtime
    layer is the live pose trail, not a stored route).
    """
    if not payload:
        return None, None
    waypoints = [
        {"name": w.get("name"),
         "geom": {"lat": w["lat"], "lon": w["lon"]},
         "recorded": True}                          # a stored keypoint = recorded
        for w in payload.get("waypoints", [])
        if w.get("lat") is not None and w.get("lon") is not None
    ]
    routes = [
        {"name": r.get("name"),
         "points": r.get("points") or [],
         "kind": "recorded"}
        for r in payload.get("routes", [])
    ]
    return routes, waypoints
