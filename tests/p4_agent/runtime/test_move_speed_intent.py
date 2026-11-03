"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_move_speed_intent.py
Brief: "移动速度 / 行走速度 快一点 / 慢一点 / 最快 / 最慢" reach A16 and go out as ModeCommand set_move_speed; bare "快一点" stays A13
Description:
User ruling 2026-11-03 (99 U106): the move speed for forward / back / strafe
is set by cloud voice / text like the turn speed; phrases carry 移动速度 /
行走速度, so bare "快一点" keeps meaning A13 and "机器人转速..." keeps meaning
A15. These cases run the production orchestrator on the production configs:
  * each phrase -> A16, one cmd/mode frame with the right move_speed and no
    motion frame, and a spoken ack naming the direction;
  * bare "快一点" -> A13, "机器人转速快一点" -> A15 (no capture by A16);
  * the mode request refuses a missing level instead of guessing.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.mode_request import ModeRequestError, to_mode_command  # A16 frame
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # turn path
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # dialog state
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets

pytestmark = pytest.mark.no_device  # pure in-process


def _orch():  # production orchestrator
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production registry
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as fh:  # production presets
        cc = yaml.safe_load(fh)  # chitchat
    return build_orchestrator(reg, ChitchatResponder(cc), l2_timeout_ms=8000)  # real presets


def _turn(text):  # one cloud text turn -> (decision, {key: payload}, spoken)
    d = _orch().handle_turn(text, OrchestratorSession(), 1000, channel="cloud", modality="text")  # one turn
    pubs = decision_to_publishes(d)  # what would go out
    by_key = {k.split("/", 1)[-1] if k.startswith("xbrain/") else k: p for k, p in pubs}  # bare keys
    spoken = [p["text"] for k, p in pubs if k.endswith("audio/speak")]  # TTS
    return d, by_key, spoken  # decision, frames, speech


@pytest.mark.parametrize("text, level, said", [  # A16 phrases
    ("移动速度快一点", "faster", "好的,移动速度调快一点"),  # step up
    ("移动速度慢一点", "slower", "好的,移动速度调慢一点"),  # step down
    ("移动速度最快", "max", "好的,移动速度调到最快"),  # to max
    ("移动速度调到最慢", "min", "好的,移动速度调到最慢"),  # to min
    ("行走速度快一点", "faster", "好的,移动速度调快一点"),  # 行走速度 form
    ("机器人移动速度慢一点", "slower", "好的,移动速度调慢一点"),  # with a body word in front
])
def test_move_speed_phrases_send_set_move_speed(text, level, said):  # A16 -> cmd/mode
    """MUTATION: drop "A16" from INTENT_TO_KEY -> no cmd/mode frame -> fails."""
    d, pubs, spoken = _turn(text)  # run the turn
    assert d.intent_id == "A16" and d.kind == "dispatch"  # classified and dispatched
    mode = [p for k, p in pubs.items() if k.endswith("cmd/mode")]  # the ModeCommand
    assert len(mode) == 1 and mode[0]["action"] == "set_move_speed" and mode[0]["move_speed"] == level
    assert not any(k.endswith("cmd/motion/intent") for k in pubs)  # setting a speed moves nothing
    assert spoken == [said]  # the direction, no predicted number


@pytest.mark.parametrize("text, intent", [("快一点", "A13"), ("机器人转速快一点", "A15")])
def test_neighbours_keep_their_phrases(text, intent):  # no capture by A16
    d, pubs, _spoken = _turn(text)  # run the turn
    assert d.intent_id == intent  # A13 / A15 unchanged
    assert not any(p.get("action") == "set_move_speed" for k, p in pubs.items() if k.endswith("cmd/mode"))


def test_missing_level_is_refused_not_guessed():  # no guessing
    """MUTATION: default a missing level to "faster" -> the robot may speed up unasked -> fails."""
    with pytest.raises(ModeRequestError):  # refused
        to_mode_command("set_move_speed", slots={}, cmd_id="c", source="cloud")  # slots without move_speed
