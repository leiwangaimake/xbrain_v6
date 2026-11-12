"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_goto_reason_js.py
Brief: HMI goto refusal wording -- U173 task_running, P5's 2 s timeout, fallbacks

Description:
Pins the `// <goto-reason>` block of hmi.js (cut out and run under node, like
test_map_marks_js.py; the robot has no node, so these SKIP there):
  * 99 U173 (user 2026-11-12): an HMI goto while a task runs is refused by P3
    with E_BUSY {reason: task_running}; the banner must tell the operator it
    will not interrupt the task and how to go anyway (soft estop, tap again,
    press the on-screen 确认前往);
  * P5's own E_TIMEOUT (11 S12.1.4, detail.item downstream_ack) reads as "no
    answer from the robot", not the bare code;
  * anything unmapped still shows something (reason_text, reason, code).
"""

from __future__ import annotations  # postponed annotations

import json  # node input / output
import os  # paths
import shutil  # find node
import subprocess  # run node

import pytest  # skip

_JS = os.path.join(os.path.dirname(__file__), "..", "..", "..",  # repo root
                   "xbrain", "p5_gateway", "hmi", "static", "hmi.js")  # the frontend
_BEGIN, _END = "// <goto-reason>", "// </goto-reason>"  # block markers


def _reason(ack: dict) -> str:
    """reasonText(ack) evaluated under node."""
    node = shutil.which("node")  # the robot has none
    if node is None:  # skip rather than pass: nothing was checked
        pytest.skip("node not installed (the robot has none)")  # visible skip
    with open(_JS, encoding="utf-8") as f:  # the real frontend
        src = f.read()  # whole file
    start = src.index(_BEGIN)  # block start
    block = src[start:src.index(_END, start)]  # pure functions only
    expr = "reasonText(" + json.dumps(ack, ensure_ascii=False) + ")"  # one call
    out = subprocess.run([node, "-e", block + "\nprocess.stdout.write(JSON.stringify("  # print
                          + expr + "));\n"], capture_output=True, text=True,  # result
                         timeout=30, check=True)  # a JS error fails the test
    return json.loads(out.stdout)  # back to Python


def test_a_running_task_is_not_interrupted_and_the_banner_says_how_to_go():
    """U173. MUTATION: drop the task_running row -> the banner shows the raw
    "task_running" -> red; name a button that is not on screen -> red."""
    text = _reason({"result": "rejected", "code": "E_BUSY",  # P3's refusal
                    "detail": {"reason": "task_running", "task_id": "t-1"}})  # U173 shape
    assert "不会打断" in text and "软件急停" in text  # what and how
    assert "确认前往" in text  # the real button label (index.html gotoSend)


def test_p5_timeout_reads_as_no_answer():
    """MUTATION: drop the downstream_ack branch -> "E_TIMEOUT" shown -> red."""
    assert _reason({"result": "error", "code": "E_TIMEOUT",  # P5-made
                    "detail": {"item": "downstream_ack"}}) == "机器人未回应, 请稍后再试"  # wording


def test_unmapped_refusals_still_say_something():
    """MUTATION: return "" for an unmapped reason -> a blank banner -> red."""
    assert _reason({"code": "E_X", "detail": {"reason": "new_reason"}}) == "new_reason"  # raw reason
    assert _reason({"code": "E_X"}) == "E_X"  # at least the code
    assert _reason({"code": "E_X", "detail": {"reason_text": "why"}}) == "why"  # builder text
