"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_battery_view_js.py
Brief: HMI battery gauge cells (99 U160) -- hmi.js batteryCellView under node

Description:
Pins the user's colour rule for the toolbar battery gauges (2026-11-11):
level >= 50 green ("ok"), 20-49 yellow ("warn"), < 20 red ("low"), with the
two boundaries checked exactly (50 is green, 20 is yellow). Also pins that an
empty slot reads 未插入 and missing data reads 未知 with no fill, and that the
charging mark is shown.

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
    node = shutil.which("node")
    if node is None:
        pytest.skip("node not installed (the robot has none); markers still checked")
    script = (_block() + "\nconst cases = " + json.dumps(cases) + ";\n"
              "process.stdout.write(JSON.stringify(cases.map(batteryCellView)));\n")
    out = subprocess.run([node, "-e", script], capture_output=True, text=True,
                         timeout=30, check=True)
    return json.loads(out.stdout)


def _present(level, **kw):
    pack = {"state": "present", "level_pct": level, "temp_c": 33.2, "charging": False}
    pack.update(kw)
    return pack


def test_markers_present():
    assert "function batteryCellView" in _block()


@pytest.mark.parametrize("level, cls", [
    (100, "ok"), (50, "ok"),          # >= 50 green, 50 itself is green
    (49, "warn"), (20, "warn"),       # 20..49 yellow, 20 itself is yellow
    (19, "low"), (0, "low"),          # < 20 red
])
def test_colour_bands_and_boundaries(level, cls):
    # RED MUTANTS: `> 50` (50 turns yellow) or `> 20` (20 turns red).
    (v,) = _views([_present(level)])
    assert v["cls"] == cls
    assert v["text"] == "%d%%" % level
    assert v["width"] == level


def test_temperature_is_shown_with_one_decimal():
    (v,) = _views([_present(90, temp_c=33.24)])
    assert v["temp"] == "33.2℃"


def test_absent_slot_reads_not_inserted_with_no_fill():
    # RED MUTANT: treat absent like present -> a red 0 % bar.
    (v,) = _views([{"state": "absent", "level_pct": None}])
    assert v["cls"] == "absent" and v["text"] == "未插入"
    assert v["width"] == 0 and v["temp"] == "--"


@pytest.mark.parametrize("pack", [
    {}, {"state": "unknown"}, {"state": "present", "level_pct": None},
])
def test_unknown_reads_unknown_never_a_number(pack):
    (v,) = _views([pack])
    assert v["cls"] == "unknown" and v["text"] == "未知" and v["width"] == 0


def test_charging_mark():
    (v,) = _views([_present(60, charging=True)])
    assert v["text"] == "⚡60%"
