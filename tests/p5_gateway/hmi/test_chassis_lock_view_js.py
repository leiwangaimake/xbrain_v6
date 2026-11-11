"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chassis_lock_view_js.py
Brief: HMI chassis-lock button states (99 U159) -- hmi.js chassisLockView under node

Description:
Guards the three button states the frontend draws from the state/robot read-back
(17 S6.10.1A): locked (hes_lock or timeout_lock) -> deep red "底盘已锁定" and
clickable; unlocked -> teal "底盘已解锁", clickable but with no action (the
contract has no lock command); no read-back -> grey "底盘状态未知", disabled.
The load-bearing row is the last: a missing state/robot must never be drawn as
teal, or a chassis nobody has heard from looks safe to drive.

How it runs. chassisLockView lives in the browser bundle, so the test cuts the
block between the `// <chassis-lock-view>` markers out of hmi.js and evaluates
it under node, the same way test_estop_view_js.py does. Each assertion names the
mutant that turns it red.

Boundary and the honest limit: the robot has no node, so there this file SKIPS
and the view assertions only run on a host that has node. The markers are
checked without node, so dropping them shows up red everywhere.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess

import pytest

_HMI_JS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "xbrain",
                       "p5_gateway", "hmi", "static", "hmi.js")
_BEGIN = "// <chassis-lock-view>"
_END = "// </chassis-lock-view>"


def _block() -> str:
    with open(_HMI_JS, encoding="utf-8") as f:
        src = f.read()
    start = src.index(_BEGIN)
    end = src.index(_END, start)
    return src[start:end]


def _views(cases):
    """Run chassisLockView(cl) for each case under node; return the results."""
    node = shutil.which("node")
    if node is None:
        pytest.skip("node not installed (the robot has none); markers still checked")
    script = (_block() + "\nconst cases = " + json.dumps(cases) + ";\n"
              "process.stdout.write(JSON.stringify(cases.map(chassisLockView)));\n")
    out = subprocess.run([node, "-e", script], capture_output=True, text=True,
                         timeout=30, check=True)
    return json.loads(out.stdout)


def test_markers_present():
    # RED MUTANT: delete or rename either marker -> _block() raises ValueError.
    assert "function chassisLockView" in _block()


def test_locked_is_deep_red_and_clickable():
    # RED MUTANT: return cls "unlocked" for locked:true -> the operator sees
    # teal over a locked chassis and never finds the unlock.
    (v,) = _views([{"available": True, "locked": True,
                    "hes_lock": False, "timeout_lock": True}])
    assert v["cls"] == "locked" and v["text"] == "底盘已锁定"
    assert v["enabled"] is True


def test_hes_lock_says_release_the_hardware_stop_first():
    # S9.3.4 E-2: enable does not clear hes_lock while the stop is pressed.
    # RED MUTANT: drop the hes_lock branch of the title -> the operator gets
    # the generic hint and keeps clicking an unlock that cannot work.
    (v,) = _views([{"available": True, "locked": True,
                    "hes_lock": True, "timeout_lock": False}])
    assert v["cls"] == "locked"
    assert "硬件急停" in v["title"]


def test_unlocked_is_teal():
    # RED MUTANT: map locked:false to "locked" -> red after an unlock.
    (v,) = _views([{"available": True, "locked": False,
                    "hes_lock": False, "timeout_lock": False}])
    assert v["cls"] == "unlocked" and v["text"] == "底盘已解锁"


@pytest.mark.parametrize("cl", [
    {},                                              # no group at all
    {"available": False, "locked": None},            # P5 has no fresh state/robot
    {"available": True, "locked": None},             # malformed: no boolean
])
def test_unknown_is_grey_and_disabled_never_teal(cl):
    # RED MUTANT: treat a missing lock as false (`!!cl.locked`) -> teal
    # "unlocked" over a chassis nobody has heard from.
    (v,) = _views([cl])
    assert v["cls"] == "unknown" and v["text"] == "底盘状态未知"
    assert v["enabled"] is False
