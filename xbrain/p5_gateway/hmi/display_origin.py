"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: display_origin.py
Brief: The HMI map's display origin = the robot's current GPS fix (99 U192)

Description:
User ruling 2026-11-13: the HMI draws every route, keypoint, dock and fence
around an origin taken from the robot's own GPS position -- any fix level
counts except no fix -- and NOT from common.geo.enu_origin.

Why the change: since 99 U183 P3 broadcasts the (possibly empty) FenceSet on
cmd/fence carrying common.geo.enu_origin, and P5 adopted that as the
"authoritative" HMI origin. On this robot the value is the simulation
placeholder from configs/sites/sim.yaml (31.2304, 121.4737), about 10 km from
the yard, so from 2026-11-13 18:00 everything -- robot included -- was drawn
some 60 000 px off-screen while the data itself was correct.

What this module decides: whether one state/pose body can serve as the
display origin. The caller (runtime/main_wiring _on_state_pose) adopts the
first body that can and keeps it for the life of the P5 process, so the
grid does not slide under the operator while the robot drives.

What it does NOT do: it does not touch P1 / P3 coordinates. Those still use
common.geo.enu_origin for their own local frame; every geometry the HMI
draws arrives as WGS84 lat/lon, so the display origin only has to be shared
by the browser's own projection, not with any other process.
"""
from __future__ import annotations  # postponed annotations

from typing import Any, Dict, Optional  # type hints

#: The only fix that cannot place the robot (11 S3.2.1). Every other reported
#: level -- rtk_fixed, rtk_float, dgps, single -- is good enough for a
#: display origin (user: "除了无解, 其他级别的定位都有效").
NO_FIX = "no_fix"


def _num(v: Any) -> bool:
    """A real number (bool is an int in Python and is refused by name)."""
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def origin_from_pose(pose: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """{lat, lon, alt} from one state/pose data body, or None when it cannot
    place the robot: no body, no fix type, no_fix, or a missing coordinate."""
    if not isinstance(pose, dict):
        return None  # nothing heard yet
    fix = pose.get("fix_type")
    if not isinstance(fix, str) or not fix or fix == NO_FIX:
        return None  # unknown or no fix: a position would be invented
    lat, lon = pose.get("lat"), pose.get("lon")
    if not _num(lat) or not _num(lon):
        return None  # a fix type without coordinates is not a position
    alt = pose.get("alt")
    return {"lat": float(lat), "lon": float(lon),
            "alt": float(alt) if _num(alt) else None}  # alt is display-only
