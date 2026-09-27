"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_estop_cmd_fields.py
Brief: the cmd/estop audit triple (cmd_id / reason / src_role) on p5's two publishing points

Description:
What this pins, and why the defect it covers was invisible.

11 S7.1 EstopCommand carries four fields and three of them are audit only:
cmd_id (the idempotency key quadruped echoes into EstopAck), reason (free text)
and src_role (a five-value set). All three are optional on the wire -- S7.1
says so in as many words, because cmd/estop is the one key that must execute
even when it cannot be parsed (S3.0.1 fail-safe). That is exactly what let them
go unfilled: nothing rejects a frame without them, nothing logs a warning, and
the robot stops either way. The cost surfaces two layers later:

  * with no cmd_id, quadruped answers "anonymous" (rt_bridge HandleEstop), and
    S7.1's FOUR parallel initiators -- HMI button, cloud, p4_agent, p5 itself --
    all share ONE ack key. Every real ack on the bus is then called anonymous
    and no reader can say which request it answers.
  * with no reason / src_role, 11 S4.1 last_soft_estop = {epoch, reason,
    src_role, age_ms} is permanently half empty (quadruped stores what arrives
    and publishes null for what does not), so the object whose whole purpose is
    "3.2 s ago, by the HMI" cannot distinguish an HMI stop from a voice one.

So the assertions here are about fields whose ABSENCE breaks nothing locally.
That is why they are asserted on the frame each publishing point actually
builds, and why the src_role values are checked against the contract's own row
rather than against a list copied into this file (see
test_the_src_role_copy_below_still_matches_the_contract -- a copied closed set
lags, and this one is copied on purpose because cmd/estop must never grow a
validating gate: S3.0.1 makes rejecting an estop over a bad field the failure
mode the whole key is shaped to avoid).

The cloud path's own frame is asserted in test_cloud_bridge.py (it needs the
bridge harness); this file covers the HMI button and the shared vocabulary.

Mutations verified red on 2026-09-27, each named at its assertion.
"""

import os
import re

import pytest

from xbrain.p5_gateway.runtime.main_wiring import (
    hmi_estop_cmd_id, hmi_estop_frame,
)


pytestmark = pytest.mark.no_device


ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONTRACT = os.path.join(ROOT, "docs", "11-接口契约.md")

#: 11 S7.1 EstopCommand.src_role. Copied here rather than exported from
#: xbrain/common/enums: those sets exist so a runtime can REFUSE an out-of-set
#: value, and cmd/estop is the one key 11 S3.0.1 exempts from validation --
#: a stop must happen even when the frame is unparseable. Putting src_role in
#: that machinery would invite a gate on the estop path that must not exist.
#: The copy cannot rot silently: the test below re-reads the contract row.
SRC_ROLES = ("hmi", "cloud", "voice", "agent", "test")

#: The verbatim anchor of the row that defines the set (NUM-4: section + text,
#: never a line number).
_SRC_ROLE_ANCHOR = "| `src_role` | string |"


def _contract_src_roles():
    """The src_role values as 11 S7.1's own field-table row spells them."""
    with open(CONTRACT, encoding="utf-8") as fh:
        for line in fh:
            if line.startswith(_SRC_ROLE_ANCHOR):
                # The members are backticked words joined by an ESCAPED pipe
                # (`hmi` \| `cloud` \| ...), so splitting the row on "|" cuts
                # the list apart -- the cells cannot be indexed. Every
                # backticked lowercase token on the row is either the field
                # name or a member, so the field name is the only exclusion.
                return tuple(t for t in re.findall(r"`([a-z_]+)`", line)
                             if t != "src_role")
    raise AssertionError("11 S7.1 src_role row not found (anchor %r)"
                         % _SRC_ROLE_ANCHOR)


def test_the_src_role_copy_below_still_matches_the_contract():
    """The remedy for "a copy always lags" (11 S1.1.6's own words).

    mutant: drop "test" from SRC_ROLES -> red; equally red if the contract's
    row grows a sixth role and nobody comes back here.
    """
    assert set(_contract_src_roles()) == set(SRC_ROLES)


# -- the HMI button (W1) ------------------------------------------------------


def test_the_hmi_estop_frame_carries_all_three_audit_fields():
    # mutant: drop any one of the three keys from hmi_estop_frame -> red.
    # Before 2026-09-27 the frame was {"type","action"} only, so all three
    # were absent and nothing anywhere noticed.
    frame = hmi_estop_frame("h-estop-ab12cd-1")
    assert frame["cmd_id"] == "h-estop-ab12cd-1"
    assert frame["reason"] == "operator_hmi"
    assert frame["src_role"] == "hmi"


def test_the_hmi_src_role_is_hmi_and_is_in_the_closed_set():
    # 11 S12.1.1 W1 names this one verbatim ("P5 补填 src_role: hmi"), and HW-5
    # forbids P5 relabelling an HMI action as cloud.
    # mutant: src_role = "cloud" -> the first assert red (HW-5); mutant:
    # src_role = "operator" -> the second (not a member).
    frame = hmi_estop_frame("h-estop-ab12cd-1")
    assert frame["src_role"] == "hmi"
    assert frame["src_role"] in SRC_ROLES


def test_the_hmi_frame_still_says_action_stop():
    # 11 S7.1: stop is the only legal action since v0.3, and S7.1.2 makes a
    # misspelling execute as stop anyway -- so a wrong value here would never
    # show up as a failure, only as a frame that disagrees with the contract.
    # mutant: action = "estop" -> red.
    assert hmi_estop_frame("x")["action"] == "stop"
    assert hmi_estop_frame("x")["type"] == "estop"


def test_the_hmi_cmd_id_is_unique_per_press_and_across_restarts():
    """boot + seq, not a bare counter.

    A bare seq restarts at 0 on every p5 restart, so press #1 after a restart
    would reuse press #1's cmd_id -- and quadruped's 11 S7.1.1 idempotency rule
    answers a repeated cmd_id with result="duplicate". An operator's real second
    stop would then be reported as a duplicate of a stop from a previous boot.
    mutant: return "h-estop-%d" % seq (drop boot) -> the cross-boot assert red.
    """
    # Same boot, two presses: different ids.
    assert hmi_estop_cmd_id("ab12cd", 1) != hmi_estop_cmd_id("ab12cd", 2)
    # Two boots, the same press number: still different ids.
    assert hmi_estop_cmd_id("ab12cd", 1) != hmi_estop_cmd_id("ef34gh", 1)
    # The h- namespace 11 S12.1.1 W1 uses for the HMI side, so a reader of
    # cmd/estop/ack can tell an HMI ack from the cloud's (c-) one.
    assert hmi_estop_cmd_id("ab12cd", 1).startswith("h-")


def test_the_sender_publishes_the_builder_and_builds_nothing_itself():
    """The builder is worthless if the closure still writes its own dict.

    _estop_sender is a closure inside _start_hmi (it needs the publisher), so
    this reads p5's real source the same way test_chassis_events reads the
    wiring: what is asserted is that the <=10 ms W1 path routes through the
    builder and carries no second copy of the frame.
    MUTATION: put a literal {"type": "estop", ...} back in the closure -> red.
    """
    from xbrain.p5_gateway.runtime import main_wiring
    src = open(main_wiring.__file__, encoding="utf-8").read()
    body = src.split("def _estop_sender()", 1)[1].split("\n    # 11 S12.1.1", 1)[0]
    assert "hmi_estop_frame(cmd_id)" in body
    assert "hmi_estop_cmd_id(" in body
    # No second place that BUILDS the audit fields: one writer, one vocabulary.
    # The quoted forms are what a dict literal / json key would look like; the
    # bare word still appears in the log line, which is a report, not a second
    # source of the value.
    assert '"src_role"' not in body
    assert "operator_hmi" not in body
    assert '"type": "estop"' not in body
