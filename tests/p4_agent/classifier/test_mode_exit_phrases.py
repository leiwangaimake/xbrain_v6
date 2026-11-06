"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mode_exit_phrases.py
Brief: "退出报警模式" / "退出喊话模式" must route to the EXIT intents (C02 / C04), not the enter intents

Description:
Field test 2026-10-29 (R018): "退出报警模式" could not leave alarm mode. Offline
on the production intents.yaml it routed to C01 enter_alarm: the long-phrase
layer is longest-first (16 S5.2), and C01's "报警模式" ties C02's "退出报警"
at four characters, so C01 won and the robot was told to ENTER alarm again.
"退出喊话模式" had the same tie against C03's "喊话模式". The fix adds the
six-character full phrases to C02 / C04 so they win on length. These cases run
the production orchestrator on the production configs, both channels.
MUTATION: drop "退出报警模式" from C02 -> routes to C01 -> the first case fails.
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # the turn path
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # chitchat presets

pytestmark = pytest.mark.no_device  # pure in-process


def _orch():  # production orchestrator
    """The production orchestrator over the production content files."""
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))  # chitchat presets
    return build_orchestrator(reg, cc)  # no LLM needed


@pytest.mark.parametrize("channel, modality", [("cloud", "text"), ("mic_local", "voice")])
@pytest.mark.parametrize("text, intent, action", [
    ("退出报警模式", "C02", "exit_alarm"), ("关闭报警模式", "C02", "exit_alarm"),
    ("退出报警", "C02", "exit_alarm"), ("进入报警模式", "C01", "set_voice_mode"),
    ("退出喊话模式", "C04", "exit_broadcast"), ("退出喊话", "C04", "exit_broadcast")])
def test_mode_phrases_route_to_the_right_direction(text, intent, action, channel, modality):  # one row
    """Enter stays enter, exit becomes exit, on both entry channels."""
    d = _orch().handle_turn(text, OrchestratorSession(), 1000,
                            channel=channel, modality=modality)  # the turn
    assert d.intent_id == intent  # routed to the right intent
    modes = [p for k, p in decision_to_publishes(d) if k == "cmd/mode"]  # the mode command
    assert modes and modes[0]["action"] == action  # and the right action goes out
