"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_estop_view_js.py
Brief: HMI ESTOP button states (17 S6.3) -- hmi.js estopView under node

Description:
Guards the four ESTOP outcomes the frontend decides (17 S6.3 table, NAV-64, 11
S12.3): ok -> armed; degraded -> still pressable with a warning; down or unknown
-> greyed with the hardware-stop sentence; and 2 s without any downlink ->
greyed whatever estop_path last said. The last one is the load-bearing row: P5
that has died cannot send "I died", so before this the button kept the last
"ok" and stayed armed with nothing behind it.

How it runs. estopView lives in the browser bundle, so the test cuts the block
between the `// <estop-view>` markers out of hmi.js and evaluates it under node.
Each assertion names the mutant that turns it red (CLAUDE.md 3.3).

Boundary and the honest limit: the robot has no node, so there this file SKIPS
and the assertions only run on a host that has node. The markers themselves are
checked without node (test_markers_present), so a refactor that drops them shows
up red everywhere rather than as a silent skip.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess

import pytest

_HMI_JS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "xbrain",
                       "p5_gateway", "hmi", "static", "hmi.js")
_BEGIN = "// <estop-view>"
_END = "// </estop-view>"


def _block() -> str:
    with open(_HMI_JS, encoding="utf-8") as f:
        src = f.read()
    start = src.index(_BEGIN)
    end = src.index(_END, start)
    return src[start:end]


def _views(cases):
    """Run estopView(path, age) for each case under node; return the results."""
    node = shutil.which("node")
    if node is None:
        pytest.skip("node not installed (the robot has none); markers still checked")
    script = (_block() + "\nconst cases = " + json.dumps(cases) + ";\n"
              "process.stdout.write(JSON.stringify("
              "cases.map(([p, a]) => estopView(p, a))));\n")
    out = subprocess.run([node, "-e", script], capture_output=True, text=True,
                         timeout=30, check=True)
    return json.loads(out.stdout)


def test_markers_present():
    # RED MUTANT: delete or rename either marker -> _block() raises ValueError.
    # Without this, losing the markers would turn every other test into a skip
    # on a node-less host and into an error only where node exists.
    block = _block()
    assert "function estopView" in block
    assert "LINK_DEAD_MS = 2000" in block


def test_fresh_ok_is_armed():
    # RED MUTANT: return enabled:false for "ok" -> the button never arms.
    (v,) = _views([["ok", 100]])
    assert v["enabled"] is True and v["armed"] is True
    assert v["notice"] == ""
    assert v["link"]["cls"] == "ok"


def test_degraded_stays_pressable_with_warning():
    # The bug this fixes: degraded used to grey the button, taking the stop
    # away while the path still carried it. 17 S6.3: "可点击 + 黄色警示".
    # RED MUTANT: fold degraded into the down branch -> enabled False.
    (v,) = _views([["degraded", 100]])
    assert v["enabled"] is True
    assert v["armed"] is False
    assert v["noticeCls"] == "warn" and "延迟高" in v["notice"]
    assert v["link"]["cls"] == "warn"


def test_down_and_unknown_are_greyed_with_hardware_sentence():
    # NAV-64 / 11 S12.3: greyed AND the hardware-stop sentence shown.
    # RED MUTANT: treat null as ok (the "armed on faith" fail-silent) -> the
    # second case enables; drop "硬件急停" from the text -> notice assert red.
    down, unknown, typo = _views([["down", 100], [None, 100], ["OK", 100]])
    for v in (down, unknown, typo):
        assert v["enabled"] is False
        assert "硬件急停" in v["notice"]
        assert v["noticeCls"] == "bad"


def test_two_seconds_without_downlink_greys_even_when_last_said_ok():
    # The row 17 S6.3 calls the most important one. At exactly 2000 ms the link
    # is still alive; one ms later it is dead -- and a last "ok" must not keep
    # the button armed.
    # RED MUTANT: compare with >= instead of > -> 2000 greys (first case red);
    # drop the age check -> 2001 stays armed (second case red); treat a missing
    # age (no downlink yet) as fresh -> third case red.
    at, past, never = _views([["ok", 2000], ["ok", 2001], ["ok", None]])
    assert at["enabled"] is True
    for v in (past, never):
        assert v["enabled"] is False
        assert v["armed"] is False
        assert "硬件急停" in v["notice"]
        assert v["link"]["text"] == "链路中断"


def test_garbage_age_is_treated_as_dead():
    # NaN compares false with everything; written as !(age <= limit) so it
    # lands on the dead side. RED MUTANT: write the test as age > limit ->
    # NaN passes as fresh and the button arms.
    (v,) = _views([["ok", float("nan")]])
    assert v["enabled"] is False
