"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_voice_hold.py
Brief: A04 "原地待命" holds P1 until "继续"; a mode's hold is not the operator's to release

Description:
99 U111. A04 used to send a legacy frame P2 refused (E_SCHEMA) while the
robot said "好的,原地待命". Now hold / resume are MotionIntent values P2
answers before the gates, keeping its own VoiceHold that P1 sees through
cmd/motion/behavior. A move while held is refused with a reason the speaker
can say. Each case names the mutant it kills.
"""
from __future__ import annotations

import json  # wire frames

import pytest  # marker

from xbrain.p2_core.runtime import main_wiring as mw  # the handler under test
from xbrain.p2_core.runtime.motion_intent_wiring import VoiceHold  # the state

pytestmark = pytest.mark.no_device  # pure


class _Health:
    """The aggregate stub: the hold path never asks it."""

    def build_summary(self, cfg):
        return {}  # never consulted before the hold check


def _frame(intent, slots=None):
    """One enveloped MotionIntent, as P4 sends it."""
    return json.dumps({"v": 1, "src": "p4_agent",
                       "data": {"cmd_id": "mi-1", "intent": intent,
                                "auth_level": "L0", "slots": slots or {},
                                "channel": "mic_local"}}).encode("utf-8")


def _run(intent, hold, mode_holds=False, slots=None):
    """The ack P2 returns; nothing is published on these paths."""
    return mw._handle_motion_intent(_frame(intent, slots), None, {}, _Health(),
                                    None, None, voice_hold=hold,
                                    mode_holds=mode_holds)  # the real handler


def test_hold_then_resume():
    """MUTATION: run hold through the gates (G-2 refuses an unknown intent)."""
    hold = VoiceHold()  # off
    ack = _run("hold", hold)  # 原地待命
    assert ack["result"] == "accepted" and hold.on  # held
    assert ack["detail"]["applied"]["behavior"] == "hold"  # AP-1
    ack = _run("resume", hold)  # 继续
    assert ack["result"] == "accepted" and not hold.on  # released
    assert ack["detail"]["applied"]["behavior"] == "resume_task"  # P1 moves again


def test_a_move_while_held_is_refused_with_its_reason():
    """MUTATION: let it through -- P1 accepts then refuses, the operator hears
    nothing about why the robot did not move."""
    hold = VoiceHold(on=True)  # held
    ack = _run("move_forward", hold, slots={"distance_m": 1.0})  # 前进一米
    assert ack["result"] == "rejected" and ack["code"] == "E_BUSY"  # refused
    assert ack["detail"] == {"reason": "voice_hold"} and hold.on  # still held


def test_a_mode_hold_is_not_the_operators_to_release():
    """During a broadcast / alarm, "继续" must not move the robot.
    MUTATION: one shared switch -- the mode's hold is lifted by voice."""
    hold = VoiceHold()  # no operator hold
    ack = _run("resume", hold, mode_holds=True)  # 继续 during B / D
    assert ack["result"] == "rejected" and ack["code"] == "E_BUSY"  # refused
    assert ack["detail"] == {"reason": "mode_hold"}  # said why


def test_releasing_the_operator_hold_during_a_mode_keeps_p1_held():
    hold = VoiceHold(on=True)  # operator hold + a mode
    ack = _run("resume", hold, mode_holds=True)  # 继续
    assert not hold.on  # the operator's hold is gone
    assert ack["detail"]["applied"]["behavior"] == "hold"  # the mode still holds


def test_resume_with_nothing_held_changes_nothing():
    ack = _run("resume", VoiceHold())  # 继续, nothing held
    assert ack["result"] == "accepted" and ack["detail"]["changed"] is False
