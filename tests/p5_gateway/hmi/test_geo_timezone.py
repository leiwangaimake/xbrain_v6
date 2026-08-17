"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_timezone.py
Brief: GPS-derived HMI footer-clock timezone (17 S6.10.2 v1.3)

Description:
Pins timezone_for_fix and its wiring into build_snapshot: the footer clock must
follow the robot's position (Japan -> Tokyo, China -> Beijing, Malaysia -> KL)
and fall back to common.timezone when there is no usable fix. Every check names
the mutation it reddens. The load-bearing ones are the (0,0) Null Island guard
(tzfpy maps 0,0 to Etc/GMT -- a real-looking zone -- so dropping the guard
silently pins a zeroed pose to GMT, 3.2 fail-silent) and the LON-first arg order
(swapping the args returns a plausible-but-wrong zone, not an error).
"""

from __future__ import annotations

import math

import pytest

from xbrain.p5_gateway.hmi import data_readers as dr
from xbrain.p5_gateway.hmi import geo_timezone as gtz

# The real reverse lookups need tzfpy present; the pure fallback / degrade paths
# do not. Skip only the lookup assertions when the wheel is absent (it is in
# tests/requirements.txt and installed on the ORIN, so this normally runs).
requires_tzfpy = pytest.mark.skipif(
    not gtz._HAVE_TZFPY, reason="tzfpy not installed on this host")

# A fallback that is a REAL zone and DIFFERENT from every expected lookup below,
# so 'returned the fallback' is always distinguishable from 'looked it up'.
FALLBACK = "Asia/Shanghai"


@requires_tzfpy
def test_real_lookups_by_position():
    # Japan / China / Malaysia -> their own zones. MUTATION: a body that just
    # returns fallback_tz (or a hardcoded single zone) reddens on at least two of
    # these three, because all three differ.
    assert gtz.timezone_for_fix(34.69, 135.50, FALLBACK) == "Asia/Tokyo"     # 大阪
    assert gtz.timezone_for_fix(39.90, 116.40, FALLBACK) == "Asia/Shanghai"  # 北京
    assert gtz.timezone_for_fix(3.14, 101.69, FALLBACK) == "Asia/Kuala_Lumpur"


@requires_tzfpy
def test_lon_first_arg_order():
    # Osaka with a fallback that is NOT Tokyo. tzfpy is get_tz(LON, LAT); the
    # MUTATION _get_tz(lat, lon) passes lat=135.50 as a latitude -- out of tzfpy's
    # range -> '' -> fallback. So asserting Tokyo (not the KL fallback) reddens a
    # swapped arg order.
    assert gtz.timezone_for_fix(34.69, 135.50, "Asia/Kuala_Lumpur") == "Asia/Tokyo"


@requires_tzfpy
def test_null_island_is_no_fix():
    # (0,0) is a zeroed pose, not a real fix. tzfpy maps it to Etc/GMT.
    # MUTATION: drop the exact-(0,0) guard -> this returns 'Etc/GMT' and reddens.
    assert gtz.timezone_for_fix(0.0, 0.0, FALLBACK) == FALLBACK


@requires_tzfpy
def test_out_of_range_falls_back():
    # Past the poles / dateline, or NaN/inf -> garbage coordinate -> fallback.
    # MUTATION: drop the range guard -> tzfpy returns '' and (since '' or FALLBACK
    # still yields FALLBACK) these happen to pass; the guard's real value is being
    # explicit + total against a future tzfpy that raises, so keep them pinned.
    assert gtz.timezone_for_fix(91.0, 10.0, FALLBACK) == FALLBACK
    assert gtz.timezone_for_fix(10.0, 200.0, FALLBACK) == FALLBACK
    assert gtz.timezone_for_fix(math.nan, 10.0, FALLBACK) == FALLBACK
    assert gtz.timezone_for_fix(10.0, math.inf, FALLBACK) == FALLBACK


def test_no_fix_falls_back():
    # No coordinate at all (pose group's None) -> fallback, no tzfpy call needed.
    # MUTATION: a body that calls _get_tz(None, None) crashes instead of falling
    # back.
    assert gtz.timezone_for_fix(None, None, FALLBACK) == FALLBACK
    assert gtz.timezone_for_fix(34.69, None, FALLBACK) == FALLBACK
    assert gtz.timezone_for_fix(None, 135.50, FALLBACK) == FALLBACK


def test_fallback_none_returns_none():
    # fallback_tz itself may be None (common.timezone unset) -> return None so the
    # frontend uses the browser zone. MUTATION: substituting a hardcoded 'UTC'
    # here would hide an unset common.timezone.
    assert gtz.timezone_for_fix(None, None, None) is None


def test_missing_tzfpy_degrades_to_fallback(monkeypatch):
    # Host without tzfpy: even a valid fix must degrade to the fallback rather
    # than crash. MUTATION: ignoring _HAVE_TZFPY and calling _get_tz (which is
    # None) raises TypeError.
    monkeypatch.setattr(gtz, "_HAVE_TZFPY", False)
    assert gtz.timezone_for_fix(34.69, 135.50, FALLBACK) == FALLBACK


# -- wiring into build_snapshot (top-level `timezone` field) ------------------

@requires_tzfpy
def test_build_snapshot_timezone_from_pose():
    # The snapshot's top-level timezone follows the pose fix. MUTATION: dropping
    # the field, or computing it from something other than pose lat/lon, reddens.
    snap = dr.build_snapshot(pose={"lat": 34.69, "lon": 135.50},
                             site_timezone=FALLBACK)
    assert snap["timezone"] == "Asia/Tokyo"


def test_build_snapshot_timezone_fallback_no_pose():
    # No pose -> the fallback (common.timezone) so the clock still ticks.
    snap = dr.build_snapshot(site_timezone=FALLBACK)
    assert snap["timezone"] == FALLBACK


@requires_tzfpy
def test_build_snapshot_timezone_fallback_null_island():
    # A zeroed pose must NOT pin the snapshot zone to Etc/GMT (the null-island
    # guard, exercised end-to-end through build_snapshot).
    snap = dr.build_snapshot(pose={"lat": 0.0, "lon": 0.0},
                             site_timezone=FALLBACK)
    assert snap["timezone"] == FALLBACK
