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
  * 2026-11-12: every disc twice as large again (outer 12 -> 24 px), the red
    ring twice as thick (2 -> 4 px), the disc number scaled with the disc, the
    robot arrow twice as large (18 -> 36 px); a route is drawn as the polyline
    alone, as thick as the old route-point dots were wide (7 px).
  * grid lines one step darker: #555555 -> #2a2a2a (half the brightness).
The values live in configs/p5_gateway.yaml hmi.web.waypoint / robot (17
S6.10.2 U6 / U9) and reach the browser through ui_config; hmi.js markStyle
reads them, with presentation fallbacks of the same values.

How it runs: the block between the `// <map-marks>` markers is cut out of
hmi.js and evaluated under node, like test_battery_view_js.py. The robot has
no node, so there the node cases SKIP; the yaml, ui_config and marker checks
run everywhere.
"""

from __future__ import annotations  # postponed annotations

import json  # node input / output
import math  # arrow tip distance
import os  # paths
import shutil  # find node
import subprocess  # run node

import pytest  # skip / approx / parametrize
import yaml  # the real config file

from xbrain.p5_gateway.hmi.ui_config import build_ui_config  # yaml -> browser config

_ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "..")  # repo root
_HMI_JS = os.path.join(_ROOT, "xbrain", "p5_gateway", "hmi", "static", "hmi.js")  # frontend
_YAML = os.path.join(_ROOT, "configs", "p5_gateway.yaml")  # the source, not the snapshot
_BEGIN, _END = "// <map-marks>", "// </map-marks>"  # the pure block's markers
_OLD_OUTER_PX = 3.5 + 0.5  # old keypoint: r 3.5 px + half of a 1 px stroke
_OLD_ARROW_PX = 9  # old robot arrow half-length


def _block() -> str:
    """The <map-marks> block of hmi.js (pure functions, no DOM)."""
    with open(_HMI_JS, encoding="utf-8") as f:  # utf-8: Chinese numerals inside
        src = f.read()  # the real frontend
    start = src.index(_BEGIN)  # block start
    return src[start:src.index(_END, start)]  # up to the end marker


def _node(expr: str):
    """Evaluate expr after the block under node and return its JSON value."""
    node = shutil.which("node")  # the robot has none
    if node is None:  # skip rather than pass: nothing was checked
        pytest.skip("node not installed (the robot has none)")  # visible skip
    out = subprocess.run([node, "-e", _block() + "\nprocess.stdout.write(JSON.stringify("  # print
                          + expr + "));\n"], capture_output=True, text=True,  # the result
                         timeout=30, check=True)  # a JS error fails the test
    return json.loads(out.stdout)  # back to Python


def _web() -> dict:
    """hmi.web of the source yaml (17 S6.10.2)."""
    with open(_YAML, encoding="utf-8") as f:  # Chinese comments inside
        return yaml.safe_load(f)["hmi"]["web"]  # the group ui_config reads


def test_the_configured_marks_are_the_users_sizes_and_colours():
    """MUTATION: radius 22.0 -> 11.0 (the previous round) -> red; robot 36 -> 18
    -> red; ring colour not red / fill not white -> red; grid back to #555555 ->
    red."""
    web = _web()  # the configured values
    w, r = web["waypoint"], web["robot"]  # U6, U9
    # 1.5x, 2x (2026-11-11), 2x again (2026-11-12)
    assert w["radius_px"] + w["ring_px"] / 2 == pytest.approx(1.5 * 2 * 2 * _OLD_OUTER_PX)  # 24
    assert w["ring_px"] == 4.0 and w["fill_color"].lower() == "#ffffff"  # ring 2 -> 4
    assert w["ring_color"].lower() == "#ff1f1f" and r["color"].lower() == "#ff1f1f"  # bright red
    assert r["size_px"] == pytest.approx(3 * _OLD_ARROW_PX * 0.5 * 4 / 3 * 2)  # 36
    assert web["map"]["grid"]["line_color"].lower() == "#2a2a2a"  # one step darker


def test_ui_config_hands_both_groups_to_the_browser():
    """MUTATION: drop the robot group in build_ui_config -> {} -> red."""
    cfg = build_ui_config(_web())  # what /api/hmi/ui_config returns
    assert cfg["waypoint"]["radius_px"] == _web()["waypoint"]["radius_px"]  # passed through
    assert cfg["robot"] == _web()["robot"]  # whole group


def test_mark_style_reads_the_config_and_falls_back_to_the_same_look():
    """MUTATION: ignore cfg.waypoint.radius_px -> the configured 7 is lost ->
    red; a zero radius accepted -> invisible keypoints -> red; fallback robot
    size 18 (the previous round) -> red."""
    got = _node('[markStyle({waypoint: {radius_px: 7, ring_px: 3, fill_color: "#fefefe",'  # all set
                ' ring_color: "#ee0000"}, robot: {size_px: 30, color: "#dd0000"}}),'  # robot too
                ' markStyle({}), markStyle({waypoint: {radius_px: 0}, robot: {size_px: -1}})]')  # none / bad
    assert {k: got[0][k] for k in ("kpRadiusPx", "kpRingPx", "kpFill", "kpRing",  # configured
                                   "robotPx", "robotColor")} == {  # wins
        "kpRadiusPx": 7, "kpRingPx": 3, "kpFill": "#fefefe",  # waypoint
        "kpRing": "#ee0000", "robotPx": 30, "robotColor": "#dd0000"}  # robot
    fallback = {"kpRadiusPx": 22, "kpRingPx": 4, "kpFill": "#ffffff",  # same look as the yaml
                "kpRing": "#ff1f1f", "dockFill": "#ffd400", "homeFill": "#22c55e",  # colours
                "discOutline": "#1a1a1a", "discOutlinePx": 1, "numColor": "#000000",  # disc edge
                "robotPx": 36, "robotColor": "#ff1f1f"}  # arrow
    assert got[1] == fallback  # nothing configured
    assert (got[2]["kpRadiusPx"], got[2]["robotPx"]) == (22, 36)  # zero / negative refused


def test_the_arrow_scales_with_its_size_and_turns_with_the_heading():
    """The tip sits s ahead of the centre (north at a = 0, east at a = pi/2).
    MUTATION: a hard-coded size inside robotArrowPoints -> red."""
    pts = _node("[robotArrowPoints(10, 20, 0, 27), robotArrowPoints(0, 0, Math.PI / 2, 9)]")  # two
    assert pts[0][0] == pytest.approx([10, -7])  # tip 27 north (screen up) of (10, 20)
    assert math.hypot(*pts[1][0]) == pytest.approx(9)  # size honoured
    assert pts[1][0][0] == pytest.approx(9) and abs(pts[1][0][1]) < 1e-9  # pointing east


# -- 2026-11-12: charging-point / return-home discs ---------------------------
@pytest.mark.parametrize("w,kind,num", [  # name -> disc kind and printed number
    ({"type": "dock", "name": "1号充电站"}, "dock", "1"),  # Arabic digit
    ({"type": "dock", "name": "二号充电站"}, "dock", "2"),  # Chinese numeral
    ({"type": "dock", "name": "十二号充电桩"}, "dock", "12"),  # Chinese tens
    ({"type": "dock", "name": "120号充电桩"}, "dock", "120"),  # three digits
    ({"type": "dock", "name": "东门充电站"}, "dock", ""),  # no number: empty disc
    ({"type": "dock", "name": "东门充电桩3"}, "dock", ""),  # not "<N>号" at the start
    ({"type": "home", "name": "返航终点站"}, "home", ""),  # green, never numbered
    ({"type": "poi", "name": "3号测试点"}, "poi", ""),  # only docks get numbers
    ({"name": "1号充电站"}, "poi", ""),  # no type: never guessed from the name
])
def test_waypoint_mark_kind_and_number(w, kind, num):
    """The number is the one in the name, never invented; only a dock gets one.
    MUTATION: number from any digit (not N号) / Chinese numerals not converted /
    a numberless dock gets "1" / type ignored -> red."""
    assert _node("waypointMark(" + json.dumps(w, ensure_ascii=False) + ")") == {  # one call
        "kind": kind, "num": num}  # exact


def test_number_font_shrinks_past_two_digits_and_grows_with_the_disc():
    """MUTATION: one size for all -> a 3-digit number overflows the disc -> red;
    the outer radius ignored -> the doubled disc keeps a 14 px number -> red."""
    assert _node('[numFontPx("1", 12), numFontPx("12", 12), numFontPx("120", 12)]') == [14, 14, 10]  # 12 px disc
    assert _node('[numFontPx("1", 24), numFontPx("12", 24), numFontPx("120", 24)]') == [28, 28, 20]  # 24 px disc


def test_a_route_is_the_polyline_alone_as_thick_as_the_old_dots():
    """User 2026-11-12. MUTATION: the per-point dot loop back -> red; the
    recorded line left at 1 px -> red."""
    with open(_HMI_JS, encoding="utf-8") as f:  # source check: renderGeo needs a DOM
        src = f.read()  # whole frontend
    assert "dot(pxy[0], pxy[1], rline.color)" not in src  # no per-point dots
    assert "const DOT_PX = 3.5;" in src  # the old dot radius the width is taken from
    # lineStroke draws every line at 2x its configured width (2026-08-18), so a
    # 7 px screen line -- the old dot diameter, 2 x DOT_PX -- is configured 3.5
    assert "(line.width_px || 1.4) * 2" in src  # the doubling the yaml relies on
    assert 2 * _web()["route"]["recorded"]["line"]["width_px"] == pytest.approx(2 * 3.5)  # 7 px


def test_the_disc_colours_are_configured():
    """MUTATION: dock not yellow / home not green / outline over 1 px -> red."""
    w = _web()["waypoint"]  # U6
    assert w["dock_fill_color"].lower() == "#ffd400"  # yellow
    assert w["home_fill_color"].lower() == "#22c55e"  # green
    assert w["disc_outline_px"] == 1.0 and w["disc_outline_color"].lower() == "#1a1a1a"  # thin dark edge


def test_the_goto_pin_is_a_waypoint_disc_and_the_disc_is_tappable():
    """User 2026-11-12: the blinking target disc is as large as a waypoint disc,
    and (the disc having grown past snap_px) a tap anywhere on a disc picks it.
    MUTATION: pin back to 20 -> red; keypoint hit = snap_px only -> a tap 20 px
    from the centre misses -> red; hit ignoring a larger snap_px -> red."""
    web = _web()  # source yaml
    w = web["waypoint"]  # U6
    assert web["goto"]["pin_diameter_px"] == pytest.approx(2 * (w["radius_px"] + w["ring_px"] / 2))  # 48
    with open(_HMI_JS, encoding="utf-8") as f:  # the fallback must match too
        assert "const GOTO_PIN_PX_FALLBACK = 48;" in f.read()  # same look without config
    got = _node('[keypointHitPx(14, {waypoint: {radius_px: 22, ring_px: 4}}),'  # disc 24 > 14
                ' keypointHitPx(30, {waypoint: {radius_px: 22, ring_px: 4}})]')  # snap larger
    assert got == [24, 30]  # the larger of the two
