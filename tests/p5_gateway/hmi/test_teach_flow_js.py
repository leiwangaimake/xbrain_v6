"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_teach_flow_js.py
Brief: HMI recording-trail flow effect -- the moving bright segment (17 S6.10.2 U10)

Description:
The user chose the "流光" effect from a demo on 2026-11-12: a short bright
segment runs along the pink recording polyline from its start to the robot,
one trip per period, looping. hmi.js computes the segment in the pure
`// <teach-flow>` block (teachFlowSegment); these cases cut that block out and
run it under node, like test_goto_reason_js.py (the robot has no node, so
they SKIP there). They pin:
  * the segment is length_px screen pixels long, converted with the current
    metres per pixel -- it looks the same at every zoom;
  * it starts at the polyline's start and ends at the robot (the last vertex)
    within one period, entering and leaving rather than popping in;
  * nothing is drawn for a degenerate line.
What they do NOT check: the colour / glow / rAF scheduling, which only the
browser shows (checked by eye on the robot).
"""

from __future__ import annotations  # postponed annotations

import json  # node input / output
import os  # paths
import shutil  # find node
import subprocess  # run node

import pytest  # skip

_JS = os.path.join(os.path.dirname(__file__), "..", "..", "..",  # repo root
                   "xbrain", "p5_gateway", "hmi", "static", "hmi.js")  # the frontend
_BEGIN, _END = "// <teach-flow>", "// </teach-flow>"  # block markers
_LINE = [[0, 0], [10, 0], [10, 10]]  # an L: 20 m long, the robot at (10, 10)


def _seg(pts, mpp, length_px, period_s, now_ms):
    """teachFlowSegment(...) evaluated under node."""
    node = shutil.which("node")  # the robot has none
    if node is None:  # skip rather than pass: nothing was checked
        pytest.skip("node not installed (the robot has none)")  # visible skip
    with open(_JS, encoding="utf-8") as f:  # the real frontend
        src = f.read()  # whole file
    start = src.index(_BEGIN)  # block start
    block = src[start:src.index(_END, start)]  # the pure function only
    expr = "teachFlowSegment(%s)" % ",".join(json.dumps(a) for a in  # one call
                                             (pts, mpp, length_px, period_s, now_ms))
    out = subprocess.run([node, "-e", block + "\nprocess.stdout.write(JSON.stringify("  # print
                          + expr + "));\n"], capture_output=True, text=True,  # result
                         timeout=30, check=True)  # a JS error fails the test
    return json.loads(out.stdout)  # back to Python


def _len(seg):
    """Polyline length of a returned segment, metres."""
    return sum(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5  # edge length
               for a, b in zip(seg, seg[1:]))  # consecutive pairs


def test_the_segment_is_length_px_screen_pixels_at_any_zoom():
    """60 px at 0.05 m/px = 3 m; at 0.1 m/px = 6 m. MUTATION: use length_px as
    metres (ignore mpp) -> 60 m, clamped to the 20 m line -> red."""
    mid = 0.5 * 1400  # half a period: the head is mid-line, nothing clipped
    assert abs(_len(_seg(_LINE, 0.05, 60, 1.4, mid)) - 3.0) < 1e-6  # zoomed in
    assert abs(_len(_seg(_LINE, 0.10, 60, 1.4, mid)) - 6.0) < 1e-6  # zoomed out


def test_it_runs_from_the_start_to_the_robot_and_loops():
    """Early in the period the segment touches the start; late, it ends on the
    robot; one period later it is back at the start. MUTATION: head runs
    total -> 0 (robot to start) -> the early segment sits at the robot -> red."""
    early = _seg(_LINE, 0.05, 60, 1.4, 0.05 * 1400)  # 5 % in
    late = _seg(_LINE, 0.05, 60, 1.4, 0.93 * 1400)  # 93 % in
    again = _seg(_LINE, 0.05, 60, 1.4, 1.05 * 1400)  # next period, 5 % in
    assert early[0] == [0, 0]  # entering at the start of the line
    assert late[-1] == [10, 10]  # leaving at the robot
    assert len(again) == len(early) and all(  # the loop repeats (float-close:
        abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6  # 1.05 periods mod 1.4
        for a, b in zip(again, early))  # leaves a tiny rounding difference)


def test_a_degenerate_line_draws_nothing():
    """One point, or all points on one spot -> []. MUTATION: drop the
    `a1 > a0` loop guard -> at the start of a period two coincident points
    come back instead of [] -> red."""
    assert _seg([[3, 3]], 0.05, 60, 1.4, 100) == []  # one vertex
    assert _seg([[3, 3], [3, 3]], 0.05, 60, 1.4, 100) == []  # zero length
    # the instant a period starts the segment has not entered yet (a0 = a1 = 0):
    # nothing, not two coincident points
    assert _seg(_LINE, 0.05, 60, 1.4, 0) == []  # head at the start
