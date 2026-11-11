"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_battery_view_js.py
Brief: HMI battery gauge cells (99 U160) -- hmi.js batteryCellView under node

Description:
Pins the user's rules for the toolbar battery gauges (2026-11-11): level
>= 50 green ("ok"), 20-49 yellow ("warn"), < 20 red ("low"), with the two
boundaries checked exactly (50 is green, 20 is yellow); inside the bar a
present pack shows the percentage for 5 s and the temperature for 2 s,
alternating; an empty slot always reads 未插入 and missing data 未知, with no
fill; the charging mark is shown.

How it runs: the block between the `// <battery-view>` markers is cut out of
hmi.js and evaluated under node, like test_estop_view_js.py. The robot has no
node, so there the view tests SKIP; the marker check runs everywhere.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess

import pytest

_HMI_JS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "xbrain",
                       "p5_gateway", "hmi", "static", "hmi.js")
_BEGIN = "// <battery-view>"
_END = "// </battery-view>"


def _block() -> str:
    with open(_HMI_JS, encoding="utf-8") as f:
        src = f.read()
    start = src.index(_BEGIN)
    return src[start:src.index(_END, start)]


def _views(cases):
    """cases: [[pack, phase_ms], ...] -> batteryCellView(pack, phase_ms) each."""
    node = shutil.which("node")
    if node is None:
        pytest.skip("node not installed (the robot has none); markers still checked")
    script = (_block() + "\nconst cases = " + json.dumps(cases) + ";\n"
              "process.stdout.write(JSON.stringify("
              "cases.map(([p, t]) => batteryCellView(p, t))));\n")
    out = subprocess.run([node, "-e", script], capture_output=True, text=True,
                         timeout=30, check=True)
    return json.loads(out.stdout)


def _present(level, **kw):
    pack = {"state": "present", "level_pct": level, "temp_c": 33.2, "charging": False}
    pack.update(kw)
    return pack


def test_markers_present():
    block = _block()
    assert "function batteryCellView" in block
    assert "BATTERY_PCT_MS = 5000" in block and "BATTERY_TEMP_MS = 2000" in block


@pytest.mark.parametrize("level, cls", [
    (100, "ok"), (50, "ok"),          # >= 50 green, 50 itself is green
    (49, "warn"), (20, "warn"),       # 20..49 yellow, 20 itself is yellow
    (19, "low"), (0, "low"),          # < 20 red
])
def test_colour_bands_and_boundaries(level, cls):
    # RED MUTANTS: `> 50` (50 turns yellow) or `> 20` (20 turns red).
    (v,) = _views([[_present(level), 0]])
    assert v["cls"] == cls
    assert v["text"] == "%d%%" % level
    assert v["width"] == level


@pytest.mark.parametrize("phase_ms, text", [
    (0, "90%"), (4999, "90%"),             # percentage for the first 5 s
    (5000, "33.2℃"), (6999, "33.2℃"),     # then temperature for 2 s
    (7000, "90%"), (12000, "33.2℃"),      # and the 7 s cycle repeats
])
def test_percent_five_seconds_then_temperature_two(phase_ms, text):
    # User 2026-11-11: percentage 5 s, temperature 2 s, alternating.
    # RED MUTANTS: swap the two durations, or never show the temperature.
    (v,) = _views([[_present(90), phase_ms]])
    assert v["text"] == text
    assert v["cls"] == "ok" and v["width"] == 90   # fill keeps the level colour


def test_no_temperature_keeps_the_percentage():
    # A present pack without a temperature reading must not blank the bar.
    (v,) = _views([[_present(90, temp_c=None), 6000]])
    assert v["text"] == "90%"


@pytest.mark.parametrize("phase_ms", [0, 6000])
def test_absent_slot_always_reads_not_inserted(phase_ms):
    # RED MUTANT: treat absent like present -> a red 0 % bar / -273 C.
    (v,) = _views([[{"state": "absent", "level_pct": None}, phase_ms]])
    assert v["cls"] == "absent" and v["text"] == "未插入" and v["width"] == 0


@pytest.mark.parametrize("pack", [
    {}, {"state": "unknown"}, {"state": "present", "level_pct": None},
])
def test_unknown_reads_unknown_never_a_number(pack):
    (v,) = _views([[pack, 6000]])
    assert v["cls"] == "unknown" and v["text"] == "未知" and v["width"] == 0


def test_charging_mark():
    (v,) = _views([[_present(60, charging=True), 0]])
    assert v["text"] == "⚡60%"
