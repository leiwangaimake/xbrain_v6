"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p3_wiring_record.py
Brief: p3 wiring -- cmd/task frames with no contract action are logged, not recorded

Description:
*** Rewritten 2026-08-23. This file used to test `_record_one`, the wiring hook
that turned p4_agent's private `task_request` frame into a task row. Since batch
15 nothing emits that shape -- p4_agent, the HMI and the cloud all send the
11 S7.2 TaskCommand, which goes to handle_task_payload -- so `_record_one` and
the recorder behind it were deleted rather than left as a second, unreachable
way to admit a task (CLAUDE.md 9.3).

Two of its three tests died with it (record-and-reflect, survive-a-bad-row);
both are now covered on the contract path by tests/p3_task/test_task_command.py
and test_p3_pipeline.py. The third -- "a control frame is skipped, not
recorded" -- is the behaviour that survived, and it is what this file guards.

Why it still matters: a few voice intents route to cmd/task but have no S7.2
action that can express them (B10 skip_waypoint -- the five-action closed set
has no `skip`; H04 reload_config -- 18 says verbatim it belongs on cmd/config).
Those frames must land nowhere AND be visible. Before this they were dropped
silently, which is indistinguishable from working.
"""
from __future__ import annotations

import logging

import pytest

from xbrain.p3_task.runtime.main_wiring import _log_non_contract_frame

pytestmark = pytest.mark.no_device


def test_a_non_contract_frame_records_nothing():
    """*** Returns 0 -- the caller's recorded counter must not move.

    MUTATION: return 1 and the p3 heartbeat starts claiming it recorded tasks
    that are not in task.db, which is the one number an operator uses to tell
    whether voice is landing.
    """
    assert _log_non_contract_frame({"schema": "p4_intent_v1",
                                    "intent_id": "B10"}) == 0


def test_the_frame_is_logged_with_its_intent_id(caplog):
    """*** Logged BY INTENT ID, so the gap is visible as a specific gap.

    "p3 dropped something" is not actionable; "B10 has no contract action" is.

    MUTATION: drop the log line (or log without the id) and these frames go
    back to vanishing silently -- which is exactly how B10/H04 stayed invisible
    until the 2026-08-21 audit counted them.
    """
    with caplog.at_level(logging.INFO):
        _log_non_contract_frame({"schema": "p4_intent_v1", "intent_id": "H04"})
    # getMessage(), not .message: the logger formats lazily, so .message is the
    # raw "%s" template and the id lives in r.args. Asserting on .message would
    # pass for a log line that never actually names the intent.
    assert any("H04" in r.getMessage() for r in caplog.records)


def test_a_malformed_frame_does_not_raise():
    """A frame that is not even a dict must not take down the P3 loop -- that
    loop also drives task scheduling."""
    assert _log_non_contract_frame("not a dict") == 0
    assert _log_non_contract_frame(None) == 0
