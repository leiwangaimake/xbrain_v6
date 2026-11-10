"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_flow_js.py
Brief: HMI task-card state flow (99 U155) -- hmi.js taskFlow under node

Description:
Every task card shows 已下发 -> 执行中 -> outcome (user 2026-11-10). The mapping
from the 12-value TaskState closed set (11 S4.4) to that flow is a pure
function; these cases cut it out of hmi.js by its markers and run it under node.
The load-bearing case is the closed-set sweep: a value with no place in the flow
would leave a card showing the wrong step -- a failed task reading "执行中".

Same arrangement and the same honest limit as test_estop_view_js.py: the robot
has no node, so the node cases skip there; the marker check runs everywhere.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess

import pytest

_HMI_JS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "xbrain",
                       "p5_gateway", "hmi", "static", "hmi.js")


def _block() -> str:
    with open(_HMI_JS, encoding="utf-8") as f:
        src = f.read()
    start = src.index("// <task-flow>")
    return src[start:src.index("// </task-flow>", start)]


def _flows(states):
    node = shutil.which("node")
    if node is None:
        pytest.skip("node not installed (the robot has none); markers still checked")
    script = (_block() + "\nprocess.stdout.write(JSON.stringify("
              + json.dumps(states) + ".map(taskFlow)));\n")
    out = subprocess.run([node, "-e", script], capture_output=True, text=True,
                         timeout=30, check=True)
    return json.loads(out.stdout)


def test_markers_present():
    # RED MUTANT: lose a marker -> ValueError here, not a silent skip.
    assert "function taskFlow" in _block()


def test_every_closed_set_value_lands_on_the_right_step():
    # RED MUTANT: forget a terminal value (say interrupted) -> it falls to the
    # unknown branch at step 0 and a dead task reads as just submitted.
    states = ["pending", "ready", "scheduled", "blocked", "running", "suspended",
              "done", "cancelled", "failed", "interrupted", "needs_review",
              "wait_for_power_off"]
    at = {s: f["at"] for s, f in zip(states, _flows(states), strict=True)}
    assert [at[s] for s in ("pending", "ready", "scheduled", "blocked")] == [0, 0, 0, 0]
    assert at["running"] == 1 and at["suspended"] == 1
    for s in ("done", "cancelled", "failed", "interrupted", "needs_review",
              "wait_for_power_off"):
        assert at[s] == 2, s


def test_outcomes_are_coloured_by_how_they_ended():
    # RED MUTANT: colour cancelled like a failure, or done like a failure.
    done, cancelled, failed = _flows(["done", "cancelled", "failed"])
    assert done["endCls"] == "ok" and cancelled["endCls"] == "muted"
    assert failed["endCls"] == "bad" and failed["end"] == "失败"


def test_unknown_value_is_shown_not_swallowed():
    (f,) = _flows(["teleporting"])
    assert f["at"] == 0 and f["first"] == "teleporting"
