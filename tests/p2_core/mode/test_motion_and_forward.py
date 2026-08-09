"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_motion_and_forward.py
Brief: mode tests -- motion and forward

Description:
BIZ-P2-26 + P2-29 -- motion_behavior mapper + B-mode forward tests.
"""


import pytest

from xbrain.p2_core.mode.b_mode_forward import (
    BroadcastForwarder, on_grant, on_mode_exit, should_forward,
)
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

def test_forward_drops_when_b_not_active():
    fw = BroadcastForwarder()
    # b_mode_active default False.
    assert not should_forward(fw, chunk_gen=5)


def test_forward_drops_stale_gen():
    """G-3: chunk with gen < current_gen is stale."""
    fw = BroadcastForwarder()
    on_grant(fw, new_gen=10)
    assert should_forward(fw, chunk_gen=10)
    assert should_forward(fw, chunk_gen=11)
    assert not should_forward(fw, chunk_gen=9)   # stale


def test_forward_drops_after_mode_exit():
    fw = BroadcastForwarder()
    on_grant(fw, new_gen=1)
    on_mode_exit(fw)
    assert not should_forward(fw, chunk_gen=1)
