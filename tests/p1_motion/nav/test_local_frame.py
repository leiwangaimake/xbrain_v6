"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_local_frame.py
Brief: LocalFrame axes, inverse and null-origin refusal (11 S10.1 / CLAUDE.md 3.1)

Description:
Three things can go wrong with a site frame and each has a test: the axes
(east/north swapped or mirrored sends the robot the wrong way), the inverse
(the pose stub needs to_latlon to be the exact inverse of to_xy) and the null
origin (a null component must raise naming the key, never become 0.0 --
CLAUDE.md 3.1's fail-silent trap in geographic form).
"""
from __future__ import annotations

import math

import pytest

from xbrain.p1_motion.path.local_frame import (M_PER_DEG, LocalFrame,
                                               LocalFrameError,
                                               frame_from_config)

pytestmark = pytest.mark.no_device

LAT0, LON0 = 31.2304, 121.4737


def test_to_xy_axes():
    """North is +y, east is +x, with cos(lat0) on the east scale.
    mutant: swap the two outputs -> red."""
    f = LocalFrame(LAT0, LON0)
    x, y = f.to_xy(LAT0 + 0.001, LON0)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.001 * M_PER_DEG, rel=1e-9)
    x, y = f.to_xy(LAT0, LON0 + 0.001)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert x == pytest.approx(0.001 * M_PER_DEG * math.cos(math.radians(LAT0)), rel=1e-9)


def test_to_latlon_is_the_inverse():
    f = LocalFrame(LAT0, LON0)
    for x, y in ((0.0, 0.0), (123.4, -56.7), (-2000.0, 1500.0)):
        lat, lon = f.to_latlon(x, y)
        bx, by = f.to_xy(lat, lon)
        assert bx == pytest.approx(x, abs=1e-6)
        assert by == pytest.approx(y, abs=1e-6)


@pytest.mark.parametrize("origin", [
    {"lat": None, "lon": LON0, "alt": 4.0},
    {"lat": LAT0, "lon": None, "alt": 4.0},
    {"lat": "31.2", "lon": LON0},
    {"lat": True, "lon": LON0},
    {"lat": 91.0, "lon": LON0},
    {"lat": LAT0, "lon": float("nan")},
    None,
])
def test_null_or_bad_origin_refuses_and_names_the_key(origin):
    """mutant: default a null lat to 0.0 -> the site moves to the equator and
    every route is unreachable with no error -> red."""
    with pytest.raises(LocalFrameError) as ei:
        frame_from_config(origin)
    assert "common.geo.enu_origin" in str(ei.value)


def test_bad_point_refused_not_nan():
    f = LocalFrame(LAT0, LON0)
    with pytest.raises(LocalFrameError):
        f.to_xy(None, LON0)
