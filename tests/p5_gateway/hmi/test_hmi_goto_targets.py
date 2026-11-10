"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_hmi_goto_targets.py
Brief: HMI W2 tap-to-go: only route points and keypoints, with a radius (99 U155)

Description:
Until 2026-11-10 an HMI goto had never worked on the robot: the browser sent a
bare lat/lon, P3 refuses a coordinate goto without an arrival radius (it will
not invent one), so P3 acknowledged "accepted" and the task failed at dispatch.
The user then narrowed the targets to route points and keypoints. These cases
pin P5's half of that rule (uplink.build_goto_command) and the two plumbing
pieces it needs: geo_layers keeping geo_id, ui_config forwarding the goto group.

Each assertion names the mutant that turns it red (CLAUDE.md 3.3).
"""

from __future__ import annotations

import pytest

from xbrain.common.errors import E_CONFIG_INVALID, E_DEGRADED, E_NOT_FOUND, E_SCHEMA
from xbrain.p5_gateway.geo.cache import geo_layers
from xbrain.p5_gateway.hmi import uplink
from xbrain.p5_gateway.hmi.ui_config import build_ui_config

pytestmark = pytest.mark.no_device

_TARGETS = {
    "waypoints": [{"geo_id": "w-1", "name": "1号测试点",
                   "geom": {"lat": 31.158330, "lon": 121.391398}}],
    "routes": [{"geo_id": "r-1", "name": "1号路径",
                "points": [{"lat": 31.158574983, "lon": 121.391269996},
                           {"lat": 31.158580000, "lon": 121.391300000}]}],
}


def _goto(targets=_TARGETS, radius=1.0, **kw):
    frame = {"type": "goto", "req_id": "g1"}
    frame.update(kw)
    return uplink.build_goto_command(frame, targets, radius)


def test_keypoint_goes_out_by_id_with_a_readable_command_text():
    # RED MUTANT: send the keypoint's lat/lon instead of waypoint_id -> P3
    # needs a radius it does not have and the task dies at dispatch.
    built = _goto(waypoint_id="w-1")
    params = built.payload["task"]["params"]
    assert params == {"waypoint_id": "w-1", "text": "HMI 点击前往 关键点 1号测试点"}


def test_unknown_keypoint_is_refused():
    # RED MUTANT: forward any waypoint_id -> P3 fails it later, after "sent".
    ref = _goto(waypoint_id="w-404")
    assert isinstance(ref, uplink.UplinkRefusal) and ref.code == E_NOT_FOUND


def test_route_point_goes_out_with_the_vertex_coordinates_and_the_radius():
    # The frame is 0.1 m off the vertex (a float round trip in the browser);
    # what goes out is the VERTEX, to the last digit, plus the radius.
    # RED MUTANT: forward the frame's own coordinates -> lat differs below.
    built = _goto(lat=31.158574983 + 0.1 / 111320.0, lon=121.391269996)
    p = built.payload["task"]["params"]
    assert p["lat"] == 31.158574983 and p["lon"] == 121.391269996
    assert p["arrive_radius_m"] == 1.0
    assert p["text"].startswith("HMI 点击前往 路径 1号路径 上的点")


def test_a_point_off_every_route_is_refused():
    # 0.5 m away from the nearest vertex: not produced by a snap.
    # RED MUTANT: drop the route check -> an arbitrary map tap is forwarded,
    # which is exactly what the user ruled out.
    ref = _goto(lat=31.158574983 + 0.5 / 111320.0, lon=121.391269996)
    assert isinstance(ref, uplink.UplinkRefusal) and ref.code == E_SCHEMA
    assert ref.detail["reason"] == "goto_target_not_on_route"


def test_no_geo_layer_means_no_goto():
    # Nothing to check the tap against -> refused, never forwarded on faith.
    # RED MUTANT: treat targets=None as "anything goes".
    ref = _goto(targets=None, waypoint_id="w-1")
    assert isinstance(ref, uplink.UplinkRefusal) and ref.code == E_DEGRADED


@pytest.mark.parametrize("radius", [None, 0, -1.0, True])
def test_route_point_without_a_configured_radius_is_refused(radius):
    # RED MUTANT: fall back to a literal radius -> an invented value travels
    # to P3 as if it were configured (CLAUDE.md 3.1).
    ref = _goto(radius=radius, lat=31.158574983, lon=121.391269996)
    assert isinstance(ref, uplink.UplinkRefusal) and ref.code == E_CONFIG_INVALID


def test_keypoint_goto_needs_no_configured_radius():
    # The keypoint's own arrival radius applies (P3 reads it from geo.db).
    built = _goto(radius=None, waypoint_id="w-1")
    assert "arrive_radius_m" not in built.payload["task"]["params"]


def test_geo_layers_keep_geo_id_on_keypoints_and_routes():
    # RED MUTANT: drop geo_id again -> the browser can only send coordinates.
    routes, wps = geo_layers({
        "waypoints": [{"geo_id": "w-1", "name": "a", "lat": 1.0, "lon": 2.0}],
        "routes": [{"geo_id": "r-1", "name": "b", "points": []}]})
    assert wps[0]["geo_id"] == "w-1" and routes[0]["geo_id"] == "r-1"


def test_ui_config_forwards_the_goto_group_and_tolerates_its_absence():
    base = {"map": {}, "font": {}, "layout": {}, "fence": {}, "route": {},
            "waypoint": {}}
    assert build_ui_config(base)["goto"] == {}
    cfg = build_ui_config(dict(base, goto={"pin_diameter_px": 20, "snap_px": 14}))
    assert cfg["goto"] == {"pin_diameter_px": 20, "snap_px": 14}
