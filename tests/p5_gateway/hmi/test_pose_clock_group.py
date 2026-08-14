"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_pose_clock_group.py
Brief: data_readers pose/clock groups carry RTK heading status + sync (18-C)

Description:
Asserts the HMI projection surfaces the RTK heading fields p1 now publishes
(heading_source / heading_level) and the clock sync group, and that both are
fail-safe when absent: no pose -> available False + every field None; no clock ->
sync False (CLK-A3), never a fabricated 'synced'. build_snapshot must expose a
clock group so the frontend time-sync indicator has a source.
"""

from __future__ import annotations

from xbrain.p5_gateway.hmi import data_readers as dr


def test_pose_group_carries_heading_status():
    pose = {
        "heading_rad": 1.0, "heading_valid": True,
        "heading_source": "dual_antenna", "heading_level": 1,
        "speed_mps": 0.4,
    }
    g = dr.pose_group(pose)
    assert g["available"] is True
    assert g["heading_source"] == "dual_antenna"
    assert g["heading_level"] == 1
    assert g["heading_valid"] is True


def test_pose_group_none_is_no_fix():
    g = dr.pose_group(None)
    assert g["available"] is False
    assert g["heading_source"] is None
    assert g["heading_level"] is None
    assert g["heading_valid"] is False


def test_clock_group_passthrough_and_failsafe():
    g = dr.clock_group({"sync": True, "source": "rtk"})
    assert g["available"] is True and g["sync"] is True and g["source"] == "rtk"
    # Fail-safe: no clock -> sync False (CLK-A3), never fabricated True.
    ng = dr.clock_group(None)
    assert ng["available"] is False and ng["sync"] is False and ng["source"] == "none"


def test_build_snapshot_exposes_clock_group():
    snap = dr.build_snapshot(clock={"sync": False, "source": "none"})
    assert "clock" in snap
    assert snap["clock"]["sync"] is False
