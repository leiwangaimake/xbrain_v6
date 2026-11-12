"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_actions_js.py
Brief: HMI task buttons -- every queued task can be cancelled (99 U174)

Description:
User ruling 2026-11-12 (99 U174): tasks waiting in the queue can be cancelled
by voice / text and by a button in the HMI current-task list. The button has
existed since the W7 controls (11 S12.1.1); nothing tested it. This pins:
  * each queued state offers exactly "cancel" (a 取消任务 button, sent as W7
    task{action: cancel, task_id});
  * the frontend's queued set is P3's queue set (_QUEUE_STATES, the states
    clear_queue empties), so a state P3 adds cannot silently lose its button;
  * running / suspended / terminal keep their own buttons (pause, resume, none).
The `// <task-actions>` block is cut out of hmi.js and run under node (SKIP on
the robot, which has no node); the P3 comparison reads hmi.js as text.
"""

from __future__ import annotations  # postponed annotations

import json  # node output
import os  # paths
import re  # read the JS array literal
import shutil  # find node
import subprocess  # run node

import pytest  # skip

from xbrain.p3_task.ingest import task_apply  # P3's queue states

_JS = os.path.join(os.path.dirname(__file__), "..", "..", "..",  # repo root
                   "xbrain", "p5_gateway", "hmi", "static", "hmi.js")  # frontend
_BEGIN, _END = "// <task-actions>", "// </task-actions>"  # block markers


def _block() -> str:
    """The <task-actions> block of hmi.js."""
    with open(_JS, encoding="utf-8") as f:  # the real frontend
        src = f.read()  # whole file
    start = src.index(_BEGIN)  # block start
    return src[start:src.index(_END, start)]  # pure functions only


def test_the_queued_set_is_p3s_queue_set():
    """MUTATION: drop "blocked" from QUEUED -> a blocked task has no button
    -> red; P3 adding a queue state without the HMI -> red."""
    m = re.search(r"const QUEUED = (\[[^\]]*\]);", _block())  # the literal
    assert m is not None  # still declared in the block
    assert set(json.loads(m.group(1))) == set(task_apply._QUEUE_STATES)  # same set


def test_every_queued_task_can_be_cancelled_and_the_others_keep_theirs():
    """MUTATION: queued -> [] (the first version of the table) -> red; cancel
    offered on a finished task -> red."""
    node = shutil.which("node")  # the robot has none
    if node is None:  # skip rather than pass
        pytest.skip("node not installed (the robot has none)")  # visible skip
    states = ["pending", "scheduled", "ready", "blocked", "running", "suspended",  # live
              "done", "failed", "cancelled"]  # terminal
    out = subprocess.run([node, "-e", _block() + "\nprocess.stdout.write(JSON.stringify("  # print
                          + json.dumps(states) + ".map(taskActions)));\n"],  # one call each
                         capture_output=True, text=True, timeout=30, check=True)  # JS errors fail
    got = dict(zip(states, json.loads(out.stdout)))  # state -> buttons
    for s in ("pending", "scheduled", "ready", "blocked"):  # queued (U174)
        assert got[s] == ["cancel"], s  # one 取消任务 button
    assert got["running"] == ["pause", "cancel"]  # running
    assert got["suspended"] == ["resume", "cancel"]  # suspended
    assert got["done"] == got["failed"] == got["cancelled"] == []  # terminal: none
