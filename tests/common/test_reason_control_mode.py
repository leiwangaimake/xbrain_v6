"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_reason_control_mode.py
Brief: CFG-CM-16 -- the reason (4-value) and control_mode (single-value jog)
       closed sets, with the mutations the criterion names

Description:
CFG-CM-16 adds two closed sets from 11 S R2.6-e / R2.6-f:
  * reason (task/progress motion/teleop block reason): rotation_blocked,
    lateral_clearance_unavailable, teleop_stale, deadman_released.
  * control_mode (direct-control mode): currently the single value jog.
The doc-diff against these sets lives in test_closed_sets.py (they are two more
entries in the symmetric-difference sweep). This file holds the CFG-CM-16
behaviour the criterion names verbatim: an off-set value MUST raise, and
control_mode must NOT be silently degraded to jog.
"""

import pytest

from xbrain.common.enums import CONTROL_MODE, REASON, ClosedSetViolation


def test_reason_is_the_four_r2_6_e_values():
    """reason is exactly the 11 S R2.6-e set, no more, no less."""
    assert set(REASON.values) == {
        "rotation_blocked", "lateral_clearance_unavailable",
        "teleop_stale", "deadman_released"}


def test_control_mode_is_the_single_value_jog():
    """control_mode is exactly {jog} today (11 S R2.6-f 现仅 jog)."""
    assert set(CONTROL_MODE.values) == {"jog"}


def test_control_mode_accepts_jog():
    """The one legal value passes through unchanged."""
    assert CONTROL_MODE.parse("jog") == "jog"


def test_off_contract_control_mode_raises_not_degrades_to_jog():
    """*** The CFG-CM-16 mutation, verbatim: a control_mode other than jog must
    raise, never be degraded to jog (11 S R2.6-f: no degrade-to-jog).

    Mutation: a parse that fell back to jog on an unknown value (the exact
    fail-silent R2.6-f forbids) would let this pass -- so this asserts the raise.
    """
    with pytest.raises(ClosedSetViolation):
        CONTROL_MODE.parse("walk")
    with pytest.raises(ClosedSetViolation):
        CONTROL_MODE.parse("auto")
    with pytest.raises(ClosedSetViolation):
        CONTROL_MODE.parse("nav")


def test_reason_accepts_each_member_and_rejects_others():
    """Every member parses; an off-set reason raises (no silent pass-through)."""
    for value in ("rotation_blocked", "lateral_clearance_unavailable",
                  "teleop_stale", "deadman_released"):
        assert REASON.parse(value) == value
    with pytest.raises(ClosedSetViolation):
        REASON.parse("route_deleted")          # a real reason string, but not THIS set
    with pytest.raises(ClosedSetViolation):
        REASON.parse("low_battery")            # a suspend_reason value, not a reason value


def test_reason_is_not_suspend_reason():
    """*** reason and suspend_reason are DIFFERENT sets (the confusion CFG-CM-16
    research had to resolve). A suspend_reason value must not be a reason value,
    and vice versa -- else the two would collapse and one closed set would police
    the wrong field."""
    from xbrain.common.enums import SUSPEND_REASON
    assert set(REASON.values).isdisjoint(set(SUSPEND_REASON.values))
