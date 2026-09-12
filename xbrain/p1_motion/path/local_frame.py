"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: local_frame.py
Brief: WGS84 <-> local ENU metres about common.geo.enu_origin (11 S10.1 / S3.5A)

Description:
Why this exists. RNS (20 S2) works in a metric plane: pose_xy in metres and a
polyline in metres. The wire carries WGS84: rt/gnss/fix gives lat/lon
(11 S3.2) and cmd/motion/route points are frame "wgs84" (11 S3.5A). One
mapping must serve BOTH the pose and the route, or the robot follows a line
drawn in a frame other than the one it is located in.

Which origin. 11 S9A (FenceSet, verbatim "各进程不得各自选原点") makes the site
anchor a single system-wide value: common.geo.enu_origin, filled per site at
L4 (configs/sites/<site_id>.yaml) and expanded by the freeze line into the p1
snapshot (configs/p1_motion.yaml carries the ${common.geo.enu_origin.*}
references). p1 never anchors on its first fix or on a route's first point:
two processes anchored on different first samples disagree by metres, and
nothing reports it.

Projection. Equirectangular about the origin:
    x_east  = (lon - lon0) * cos(lat0) * M_PER_DEG
    y_north = (lat - lat0) * M_PER_DEG
with M_PER_DEG = 111320.0 -- the constant p3 already uses (teach/validate.py),
so the two sides project alike. Inside a site (FV-7 keeps fence vertices within
20 km of the origin) the scale error of this projection is far below RTK
accuracy. Heading needs no conversion: 11 S3.3 heading_rad is already ENU (east
0, CCW positive), which is exactly the yaw of this x/y plane.

What it does NOT do: no altitude, no odometry fusion, no re-anchoring. A null or
out-of-range origin component raises with the key path (CLAUDE.md 3.1). It is
NOT replaced by (0, 0): that would place the site in the Gulf of Guinea and make
every route "unreachable" with no error anywhere.
"""
from __future__ import annotations

import math
from typing import Any, Tuple

#: metres per degree of latitude (and of longitude at the equator); identical to
#: p3 teach/validate.py so route validation and route following share one scale.
M_PER_DEG = 111320.0

#: a site closer than this to a pole has cos(lat0) ~ 0 and the inverse mapping
#: divides by it; refuse rather than emit garbage longitudes.
_MIN_COS_LAT = 1e-6


class LocalFrameError(ValueError):
    """enu_origin missing, null, or out of range -- refuse and name the key."""


def _check_deg(name: str, value: Any, limit: float) -> float:
    """A finite number within +/-limit, or LocalFrameError naming the key. bool
    is excluded explicitly: True would otherwise pass as 1.0 degrees."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LocalFrameError(
            "common.geo.enu_origin.%s is not a number: %r" % (name, value))
    v = float(value)
    if not math.isfinite(v) or abs(v) > limit:
        raise LocalFrameError(
            "common.geo.enu_origin.%s out of range: %r" % (name, value))
    return v


class LocalFrame:
    """The site's local ENU plane. Construct once from the resolved config; the
    pose bridge and the route intake call the SAME instance."""

    def __init__(self, lat0_deg: Any, lon0_deg: Any) -> None:
        self._lat0 = _check_deg("lat", lat0_deg, 90.0)
        self._lon0 = _check_deg("lon", lon0_deg, 180.0)
        self._cos = math.cos(math.radians(self._lat0))
        if self._cos < _MIN_COS_LAT:
            raise LocalFrameError(
                "common.geo.enu_origin.lat too close to a pole: %r" % (lat0_deg,))

    @property
    def origin(self) -> Tuple[float, float]:
        """(lat0, lon0) in degrees, for logs and the pose stub."""
        return (self._lat0, self._lon0)

    def to_xy(self, lat_deg: Any, lon_deg: Any) -> Tuple[float, float]:
        """WGS84 -> (x_east_m, y_north_m). Inputs are validated like the origin
        so a null lat from a no-fix frame cannot become NaN metres downstream.
        mutant: swap the two outputs -> a route heading north walks east ->
        test_to_xy_axes red."""
        lat = _check_deg("lat", lat_deg, 90.0)
        lon = _check_deg("lon", lon_deg, 180.0)
        return ((lon - self._lon0) * self._cos * M_PER_DEG,
                (lat - self._lat0) * M_PER_DEG)

    def to_latlon(self, x_east_m: float, y_north_m: float) -> Tuple[float, float]:
        """(x_east_m, y_north_m) -> WGS84; the exact inverse of to_xy."""
        return (self._lat0 + y_north_m / M_PER_DEG,
                self._lon0 + x_east_m / (self._cos * M_PER_DEG))


def frame_from_config(enu_origin: Any) -> LocalFrame:
    """common.geo.enu_origin mapping -> LocalFrame. The mapping form is what the
    resolved snapshot carries ({lat, lon, alt}); alt is not used here. A missing
    mapping or a null component raises naming the component."""
    if not isinstance(enu_origin, dict):
        raise LocalFrameError(
            "common.geo.enu_origin missing or not a mapping: %r" % (enu_origin,))
    return LocalFrame(enu_origin.get("lat"), enu_origin.get("lon"))
