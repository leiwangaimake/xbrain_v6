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
    command_for_mode,
    command_for_target_lost,
)

pytestmark = pytest.mark.no_device


def _mm_cfg():
    """同 configs/p2_core.yaml 的 mode_motion 形状.

    *** 2026-10-08 (99 U90 / R-8 (a)): behavior 由 face_target_stop /
    face_target_follow 改为 hold. 夹具必须跟着真配置走 -- 一个还在用旧值的
    夹具会让这组用例继续绿, 而它们钉的是一个[建不出来]的行为
    (11 S3.5 把两个 face_target_* 标 (x), 逐字"需航向闭环对准目标").
    """
    return {
        "d_alarm": {"behavior": "hold",
                     "params": {"keep_dist_m": 3.0, "max_speed_mps": 1.0,
                                 "stop_at_fence": True}},
        "b_cast":  {"behavior": "hold",
                     "params": {"keep_dist_m": 3.0, "max_speed_mps": 1.0,
                                 "stop_at_fence": True}},
    }


def test_command_for_d_alarm_maps_correctly():
    """d_alarm -> hold, 其余参数原样透传.

    MUTATION: command_for_mode 把 behavior 写死成某个值 -> 第一条红.
    """
    cmd = command_for_mode("d_alarm", _mm_cfg())
    assert cmd.behavior == "hold"
    # 这两个参数当前[无消费者](只被 face_target_follow 读, 该值已不在闭集),
    # 但映射必须原样带出来 -- 它们是 14 S5.6.3 的已决值, 航向闭环建成后
    # face_target_* 回来时要用. 透传坏了会在那天才发现.
    assert cmd.keep_dist_m == 3.0
    assert cmd.max_speed_mps == 1.0
    assert cmd.stop_at_fence is True


def test_command_for_b_cast_maps_correctly():
    cmd = command_for_mode("b_cast", _mm_cfg())
    assert cmd.behavior == "hold"


def test_an_out_of_set_behavior_raises_and_is_not_downgraded():
    """闭集越界必抛, NO 不降级解释(3.5 / 11 S13.6).

    *** 为什么这条特别重要. 一个配成 face_target_stop 的块, 若被静默放行,
    会让一条"机器人该停下"的指令变成[无人消费的报文]-- P1 侧没有任何东西
    能执行 face_target_*(需航向闭环, 11 S3.5 标 (x)). 那是 fail-silent:
    配置看着有, 报文也发了, 而机器人不停.
    MUTATION: 删掉 command_for_mode 里的闭集检查 -> 本条红.
    """
    bad = _mm_cfg()
    bad["d_alarm"]["behavior"] = "face_target_stop"
    with pytest.raises(ValueError, match="face_target"):
        command_for_mode("d_alarm", bad)
    # 反向: 闭集内的另一个值放行, 否则上面的抛可能只是"什么都抛".
    ok = _mm_cfg()
    ok["d_alarm"]["behavior"] = "resume_task"
    assert command_for_mode("d_alarm", ok).behavior == "resume_task"


def test_command_for_unknown_mode_returns_none():
    assert command_for_mode("dialog_a", _mm_cfg()) is None


def test_command_for_target_lost_returns_hold():
    cmd = command_for_target_lost("hold")
    assert cmd.behavior == "hold"


def test_command_for_target_lost_rejects_bad_value():
    with pytest.raises(ValueError):
        command_for_target_lost("magic")


# --- B-mode forward ---

