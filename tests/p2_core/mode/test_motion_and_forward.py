"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_motion_and_forward.py
Brief: BIZ-P2-26/29 mode_motion -> BehaviorCommand map + B-mode cloud audio forward

Description:
*** Brief 由占位串改写(2026-08-23). 原值是按路径自动生成的
"mode tests -- motion and forward" -- 既没说清本文件测什么, 也无法据以索引任务号, 于是 P2 是唯一
无法自动提取证据映射的子系统(CLAUDE.md 2.5 要求 Brief 一行说清).
BIZ-P2-26 + P2-29 -- motion_behavior mapper + B-mode forward tests.
"""


import pytest

from xbrain.p2_core.mode.motion_behavior import (
    BehaviorCommand, MotionBehaviorParams,
    command_for_mode, command_for_target_lost,
)


pytestmark = pytest.mark.no_device


def _mm_cfg():
    return {
        "d_alarm": {"behavior": "face_target_stop",
                     "params": {"keep_dist_m": 3.0, "max_speed_mps": 1.0,
                                 "stop_at_fence": True}},
        "b_cast":  {"behavior": "face_target_follow",
                     "params": {"keep_dist_m": 3.0, "max_speed_mps": 1.0,
                                 "stop_at_fence": True}},
    }


def test_command_for_d_alarm_maps_correctly():
    cmd = command_for_mode("d_alarm", _mm_cfg())
    assert cmd.behavior == "face_target_stop"
    assert cmd.keep_dist_m == 3.0
    assert cmd.max_speed_mps == 1.0
    assert cmd.stop_at_fence is True


def test_command_for_b_cast_maps_correctly():
    cmd = command_for_mode("b_cast", _mm_cfg())
    assert cmd.behavior == "face_target_follow"


def test_command_for_unknown_mode_returns_none():
    assert command_for_mode("dialog_a", _mm_cfg()) is None


def test_command_for_target_lost_returns_hold():
    cmd = command_for_target_lost("hold")
    assert cmd.behavior == "hold"


def test_command_for_target_lost_rejects_bad_value():
    with pytest.raises(ValueError):
        command_for_target_lost("magic")


# --- B-mode forward ---

