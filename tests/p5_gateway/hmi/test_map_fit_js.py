"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_map_fit_js.py
Brief: 99 U200 -- HMI first view: centred on the robot, nearby objects only, 1 m grid reachable

Description:
Field 2026-11-14: once two fences had been recorded at the office, the HMI's
one-shot fit (fitGeoToData) ran for the first time on this site. It fitted to
every route and waypoint in geo.db, the customer site ~15 km away included, so
the view centre lay between the two sites, the robot was off screen, and full
zoom-in still showed ~220 m (5 m grid; the 1 m cell never came).
Pinned here:
  * objects farther than FIT_RADIUS_M from the centre are left out;
  * the centre is the robot's current position (the origin when no pose);
  * zoom-in reaches a FIT_MIN_WINDOW_M window, so the 1 m cell is drawn;
  * the snapshot handler re-centres the default window on the robot once.

How it runs: toXY and the fit block are cut out of hmi.js and evaluated under
node with a stub DOM, like test_map_marks_js.py. The robot has no node, so
there the node cases SKIP; the wiring check runs everywhere.
"""

from __future__ import annotations  # postponed annotations

import json  # node output
import os  # paths
import shutil  # find node
import subprocess  # run node

import pytest  # skip

_ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "..")  # repo root
_HMI_JS = os.path.join(_ROOT, "xbrain", "p5_gateway", "hmi", "static", "hmi.js")  # frontend
_ORIGIN = {"lat": 31.158439, "lon": 121.391463}  # the office origin of 2026-11-14

pytestmark = pytest.mark.no_device  # pure, no robot


def _src() -> str:
    with open(_HMI_JS, encoding="utf-8") as f:  # utf-8: Chinese comments inside
        return f.read()  # the real frontend


def _cut(src: str, start: str, end: str) -> str:
    """The text from start up to and including end."""
    a = src.index(start)  # block start
    return src[a:src.index(end, a) + len(end)]  # through the end marker


def _fit(geo: dict, pose) -> dict:
    """Run the real fitGeoToData under node; return baseVB and zoomBounds."""
    node = shutil.which("node")  # the robot has none
    if node is None:  # skip rather than pass: nothing was checked
        pytest.skip("node not installed (the robot has none)")  # visible skip
    src = _src()  # the frontend
    js = "\n".join([
        "const view = { zoom: 1, pan: { x: 0, y: 0 } };",  # module state hmi.js declares
        "let baseVB = [-100, -65, 240, 145];",  # the config default window
        "let zoomBounds = { min: 0.65, max: 16, wheel: 0.12 };",  # configs/p5_gateway.yaml
        "const $ = () => ({ clientWidth: 1600, clientHeight: 1000 });",  # stub DOM
        "function applyView() {}",  # no SVG here
        _cut(src, "  function toXY(pt, origin) {", "\n    return null;\n  }"),  # projection
        _cut(src, "  let geoFitted = false;", "    geoFitted = true;\n    applyView();\n  }"),  # the fit
        "fitGeoToData(%s, %s, %s);" % (json.dumps(geo), json.dumps(_ORIGIN), json.dumps(pose)),
        "process.stdout.write(JSON.stringify({ vb: baseVB, zb: zoomBounds }));",
    ])
    out = subprocess.run([node, "-e", js], capture_output=True, text=True, timeout=30, check=True)
    return json.loads(out.stdout)  # back to Python


def _geo(far: bool) -> dict:
    """One 40 m fence around the origin, one office route, and (far) the customer site."""
    d = 0.00018  # ~20 m in latitude
    ring = [{"lat": _ORIGIN["lat"] + a * d, "lon": _ORIGIN["lon"] + b * d} for a, b in
            ((-1, -1), (-1, 1), (1, 1), (1, -1))]  # the fence
    routes = [{"points": [{"lat": _ORIGIN["lat"], "lon": _ORIGIN["lon"]}]}]  # office route
    wps = [{"geom": {"lat": _ORIGIN["lat"], "lon": _ORIGIN["lon"]}}]  # office waypoint
    if far:  # the customer site, ~15 km away
        routes.append({"points": [{"lat": 31.0916, "lon": 121.5336}]})
        wps.append({"geom": {"lat": 31.0916, "lon": 121.5338}})
    return {"fences": {"available": True, "items": [{"vertices": ring}]},
            "routes": {"available": True, "items": routes},
            "waypoints": {"available": True, "items": wps}}


def test_another_site_does_not_widen_the_first_view():  # field: ~14 km window
    """MUTATION: drop the FIT_RADIUS_M filter -> the window is kilometres wide -> red."""
    near, both = _fit(_geo(False), None), _fit(_geo(True), None)  # same office content
    assert both["vb"] == near["vb"]  # the far site changes nothing
    assert near["vb"][2] < 200  # an office-sized window, metres not kilometres


def test_first_view_is_centred_on_the_robot():  # field: robot off screen
    """MUTATION: centre on the origin -> the centre misses the robot by ~20 m -> red."""
    pose = {"lat": _ORIGIN["lat"] - 0.00018, "lon": _ORIGIN["lon"]}  # ~20 m south
    vb = _fit(_geo(True), pose)["vb"]  # the fitted window
    cx, cy = vb[0] + vb[2] / 2, vb[1] + vb[3] / 2  # its centre
    assert abs(cx) < 0.5 and abs(cy - 19.9) < 0.5  # on the robot (north is -y)


def test_full_zoom_in_reaches_the_1m_grid():  # field: 5 m grid at full zoom
    """MUTATION: keep the old ceil(width / 200) bound on a wide fit -> window > 47 m -> red."""
    out = _fit(_geo(True), None)  # the fit
    assert out["vb"][2] / out["zb"]["max"] <= 47  # 1 m cell >= 42 px on a 2000 px map


def test_default_window_is_moved_onto_the_robot_once():  # no fences: no fit
    """MUTATION: drop the call -> the default window stays on the origin -> red."""
    src = _src()  # the frontend
    assert "centerOnRobotOnce(snap.pose, origin);" in src  # wired in the snapshot handler
    assert "function centerOnRobotOnce(pose, origin) {" in src  # and defined
