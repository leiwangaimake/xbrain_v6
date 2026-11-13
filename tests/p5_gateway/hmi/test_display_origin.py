"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_display_origin.py
Brief: 99 U192 -- the HMI display origin comes from the robot's GPS fix, any level but no_fix

Description:
User ruling 2026-11-13: the HMI draws everything around the robot's own GPS
position, taken from state/pose whenever the fix is anything but no_fix, and
not around common.geo.enu_origin (the sim placeholder on this robot pushed the
whole map ~10 km off-screen). These cases pin which pose bodies may become the
origin and that the P5 wiring adopts one once and keeps it. Each names the
mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import inspect  # static wiring check (the callback is a closure)

import pytest  # parametrize

from xbrain.p5_gateway.hmi.display_origin import origin_from_pose  # under test
from xbrain.p5_gateway.runtime import main_wiring  # the wiring that calls it

pytestmark = pytest.mark.no_device  # pure logic


@pytest.mark.parametrize("fix", ["rtk_fixed", "rtk_float", "dgps", "single"])
def test_every_fix_but_no_fix_places_the_origin(fix):
    """MUTATION: accept only rtk_fixed (the old 'good fix' habit) -> float /
    dgps / single refused -> red."""
    got = origin_from_pose({"fix_type": fix, "lat": 31.1584, "lon": 121.3912, "alt": 4.0})
    assert got == {"lat": 31.1584, "lon": 121.3912, "alt": 4.0}  # taken as is


@pytest.mark.parametrize("pose", [
    None,                                                   # nothing heard
    {"fix_type": "no_fix", "lat": 31.1, "lon": 121.3},      # no fix: refused
    {"lat": 31.1, "lon": 121.3},                            # fix type unknown
    {"fix_type": "", "lat": 31.1, "lon": 121.3},            # empty fix type
    {"fix_type": "single", "lat": None, "lon": 121.3},      # no latitude
    {"fix_type": "single", "lat": 31.1, "lon": True},       # a bool is not a number
])
def test_poses_that_cannot_place_the_robot_give_no_origin(pose):
    """MUTATION: drop the no_fix check -> an invented position becomes the
    origin -> red; drop the coordinate check -> red."""
    assert origin_from_pose(pose) is None  # never invented


def test_altitude_is_optional():
    """A missing / bad altitude still places the map (it is display-only).
    MUTATION: require alt -> None -> red."""
    assert origin_from_pose({"fix_type": "dgps", "lat": 31.1, "lon": 121.3})["alt"] is None


def test_the_wiring_adopts_once_from_gps_and_never_from_cmd_fence():
    """MUTATION: re-adopt on every pose (the grid slides) -> red; put the
    cmd/fence origin back into enu_origin -> red."""
    src = inspect.getsource(main_wiring)  # the closures live in this module
    assert 'if hmi_state.get("enu_origin") is None:' in src  # adopt once
    assert 'o = origin_from_pose(hmi_state["pose"])' in src  # from GPS
    assert 'hmi_state["enu_origin"] = {"lat": eo.get("lat")' not in src  # not from cmd/fence
