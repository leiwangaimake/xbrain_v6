"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_bit.py
Brief: bit tests -- bit

Description:
BIZ-P2-23 -- BIT report + BIT-G1 guard tests.
"""


import pytest

from xbrain.p2_core.bit.report import (
    BitConfigViolation, BitItemReport, BitReport, BitResult,
    check_bit_g1,
)
from xbrain.p2_core.health.items import HealthState


pytestmark = pytest.mark.no_device


# --- Report result computation ------------------------------------

def test_all_ok_is_pass():
    r = BitReport(items=[
        BitItemReport(item="chassis", state=HealthState.OK),
        BitItemReport(item="ptz", state=HealthState.OK),
    ])
    assert r.result() == BitResult.PASS


def test_fatal_fail_is_fail():
    r = BitReport(items=[
        BitItemReport(item="chassis", state=HealthState.FAIL),
    ])
    assert r.result() == BitResult.FAIL


def test_non_fatal_fail_is_degraded_not_fail():
    r = BitReport(items=[
        BitItemReport(item="ptz", state=HealthState.FAIL),
    ])
    assert r.result() == BitResult.DEGRADED


def test_non_blocking_fatal_does_not_push_to_fail():
    """CFG-42: non_blocking_items with a fatal fail does NOT push
    the report to FAIL. This is the setup that BIT-G1 exists to
    prevent (a defect-in-config)."""
    r = BitReport(
        items=[BitItemReport(item="chassis", state=HealthState.FAIL)],
        non_blocking=["chassis"],
    )
    # Runtime result: not FAIL, because chassis is on the non-blocking
    # list. This is the DEFECT case; BIT-G1 must prevent it at config load.
    assert r.result() != BitResult.FAIL


def test_skipped_fatal_does_not_push_to_fail():
    r = BitReport(
        items=[BitItemReport(item="cam_rgbd", state=HealthState.FAIL)],
        skipped=["cam_rgbd"],
    )
    assert r.result() != BitResult.FAIL


# --- BIT-G1 startup guard ------------------------------------------

def test_bit_g1_accepts_empty_lists():
    check_bit_g1(non_blocking_items=[], skip_items=[])


def test_bit_g1_accepts_non_fatal_items():
    """warn / degraded items ARE allowed in these lists."""
    check_bit_g1(non_blocking_items=["ptz_home"], skip_items=["speech_preset"])


def test_bit_g1_rejects_fatal_in_non_blocking():
    """chassis is fatal; putting it in non_blocking_items is a fail-
    silent hazard."""
    with pytest.raises(BitConfigViolation) as ei:
        check_bit_g1(non_blocking_items=["chassis"], skip_items=[])
    msg = str(ei.value)
    assert "chassis" in msg
    assert "fatal" in msg.lower()


def test_bit_g1_rejects_fatal_in_skip():
    with pytest.raises(BitConfigViolation) as ei:
        check_bit_g1(non_blocking_items=[], skip_items=["cam_rgbd"])
    assert "cam_rgbd" in str(ei.value)


def test_bit_g1_rejects_multiple_and_names_all():
    """When both lists have fatal items, error names both."""
    with pytest.raises(BitConfigViolation) as ei:
        check_bit_g1(
            non_blocking_items=["chassis", "ptz"],   # ptz is not fatal
            skip_items=["cam_rgbd"],
        )
    msg = str(ei.value)
    assert "chassis" in msg
    assert "cam_rgbd" in msg
