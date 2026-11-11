"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_map_marks_js.py
Brief: HMI map marks (user 2026-11-11) -- keypoint white dot + red ring 1.5x, robot arrow red 3x

Description:
Pins the user's display rules (2026-11-11, two rounds the same day):
  * keypoint (位置点): a true circle, filled bright white, bright red ring, no
    gap between them; 1.5x the old size, then twice that. The old mark's
    outer radius was 4.0 px (r 3.5 + half of a 1 px stroke); the new one is
    radius_px + ring_px / 2 = 12.0 px.
  * robot: bright red; 3x the old size, then halved, then a third larger
    (half-length 9 -> 27 -> 13.5 -> 18 px).
  * grid lines one step darker: #555555 -> #2a2a2a (half the brightness).
The values live in configs/p5_gateway.yaml hmi.web.waypoint / robot (17
S6.10.2 U6 / U9) and reach the browser through ui_config; hmi.js markStyle
reads them, with presentation fallbacks of the same values.

How it runs: the block between the `// <map-marks>` markers is cut out of
hmi.js and evaluated under node, like test_battery_view_js.py. The robot has
no node, so there the node cases SKIP; the yaml, ui_config and marker checks
run everywhere.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess

import pytest
import yaml

from xbrain.p5_gateway.hmi.ui_config import build_ui_config

_ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "..")
_HMI_JS = os.path.join(_ROOT, "xbrain", "p5_gateway", "hmi", "static", "hmi.js")
_YAML = os.path.join(_ROOT, "configs", "p5_gateway.yaml")
_BEGIN, _END = "// <map-marks>", "// </map-marks>"
_OLD_OUTER_PX = 3.5 + 0.5  # old keypoint: r 3.5 px + half of a 1 px stroke
_OLD_ARROW_PX = 9  # old robot arrow half-length


def _block() -> str:
    with open(_HMI_JS, encoding="utf-8") as f:
        src = f.read()
    start = src.index(_BEGIN)
    return src[start:src.index(_END, start)]


def _node(expr: str):
    node = shutil.which("node")
    if node is None:
        pytest.skip("node not installed (the robot has none)")
    out = subprocess.run([node, "-e", _block() + "\nprocess.stdout.write(JSON.stringify("
                          + expr + "));\n"], capture_output=True, text=True,
                         timeout=30, check=True)
    return json.loads(out.stdout)


def _web() -> dict:
    with open(_YAML, encoding="utf-8") as f:
        return yaml.safe_load(f)["hmi"]["web"]


def test_the_configured_marks_are_the_users_sizes_and_colours():
    """MUTATION: radius 11.0 -> 5.0 (first round) -> red; robot 18 -> 13.5 ->
    red; ring colour not red / fill not white -> red; grid back to #555555 ->
    red."""
    web = _web()
    w, r = web["waypoint"], web["robot"]
    assert w["radius_px"] + w["ring_px"] / 2 == pytest.approx(1.5 * 2 * _OLD_OUTER_PX)
    assert w["ring_px"] > 0 and w["fill_color"].lower() == "#ffffff"
    assert w["ring_color"].lower() == "#ff1f1f" and r["color"].lower() == "#ff1f1f"
    assert r["size_px"] == pytest.approx(3 * _OLD_ARROW_PX * 0.5 * 4 / 3)
    assert web["map"]["grid"]["line_color"].lower() == "#2a2a2a"


def test_ui_config_hands_both_groups_to_the_browser():
    """MUTATION: drop the robot group in build_ui_config -> {} -> red."""
    cfg = build_ui_config(_web())
    assert cfg["waypoint"]["radius_px"] == _web()["waypoint"]["radius_px"]
    assert cfg["robot"] == _web()["robot"]


def test_mark_style_reads_the_config_and_falls_back_to_the_same_look():
    """MUTATION: ignore cfg.waypoint.radius_px -> the configured 7 is lost ->
    red; a zero radius accepted -> invisible keypoints -> red; fallback robot
    size 27 -> red."""
    got = _node('[markStyle({waypoint: {radius_px: 7, ring_px: 3, fill_color: "#fefefe",'
                ' ring_color: "#ee0000"}, robot: {size_px: 30, color: "#dd0000"}}),'
                ' markStyle({}), markStyle({waypoint: {radius_px: 0}, robot: {size_px: -1}})]')
    assert got[0] == {"kpRadiusPx": 7, "kpRingPx": 3, "kpFill": "#fefefe",
                      "kpRing": "#ee0000", "robotPx": 30, "robotColor": "#dd0000"}
    fallback = {"kpRadiusPx": 11, "kpRingPx": 2, "kpFill": "#ffffff",
                "kpRing": "#ff1f1f", "robotPx": 18, "robotColor": "#ff1f1f"}
    assert got[1] == fallback
    assert (got[2]["kpRadiusPx"], got[2]["robotPx"]) == (11, 18)


def test_the_arrow_scales_with_its_size_and_turns_with_the_heading():
    """The tip sits s ahead of the centre (north at a = 0, east at a = pi/2).
    MUTATION: a hard-coded size inside robotArrowPoints -> red."""
    pts = _node("[robotArrowPoints(10, 20, 0, 27), robotArrowPoints(0, 0, Math.PI / 2, 9)]")
    assert pts[0][0] == pytest.approx([10, -7])  # tip 27 north (screen up) of (10, 20)
    assert math.hypot(*pts[1][0]) == pytest.approx(9)
    assert pts[1][0][0] == pytest.approx(9) and abs(pts[1][0][1]) < 1e-9
