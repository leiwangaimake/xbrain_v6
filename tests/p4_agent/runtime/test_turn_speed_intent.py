"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_turn_speed_intent.py
Brief: "机器人转速快一点 / 慢一点 / 最快 / 最慢" reach A15 and go out as ModeCommand set_turn_speed; bare "转速快一点" stays PTZ E09
Description:
User ruling 2026-10-30 (99 U104): the body turn speed is voice-adjustable;
phrases must name the body, and the bare "转速..." family keeps meaning the PTZ
(E09). These cases run the production orchestrator on the production
configs:
  * each body phrase -> A15, one cmd/mode frame with the right turn_speed and
    no motion frame, and a spoken ack naming the direction;
  * bare "转速快一点" -> E09 on cmd/ptz, never cmd/mode;
  * "转身" alone still turns around (A11) -- A15 took no "转身" + level keyword;
  * the mode request refuses a missing level instead of guessing.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.mode_request import ModeRequestError, to_mode_command  # A15 frame
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # turn path
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # dialog state
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets
from xbrain.p4_agent.slots.motion_slots import parse_turn_speed_level  # the level reader

pytestmark = pytest.mark.no_device  # pure in-process


def _orch():  # production orchestrator
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production registry
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as fh:  # production presets
        cc = yaml.safe_load(fh)  # chitchat
    return build_orchestrator(reg, ChitchatResponder(cc), l2_timeout_ms=8000)  # real presets


def _turn(text):  # one cloud text turn -> (decision, {key: payload}, spoken)
    d = _orch().handle_turn(text, OrchestratorSession(), 1000, channel="cloud", modality="text")  # one turn on the production orchestrator
    pubs = decision_to_publishes(d)  # what would go out
    by_key = {k.split("/", 1)[-1] if k.startswith("xbrain/") else k: p for k, p in pubs}  # bare keys
    spoken = [p["text"] for k, p in pubs if k.endswith("audio/speak")]  # TTS
    return d, by_key, spoken  # decision, frames, speech


@pytest.mark.parametrize("text, level, said", [  # body phrases
    ("机器人转速快一点", "faster", "好的,机器人转速调快一点"),  # step up
    ("机器人转速慢一点", "slower", "好的,机器人转速调慢一点"),  # step down
    ("机器人转速最快", "max", "好的,机器人转速调到最快"),  # to max
    ("机器人转速最慢", "min", "好的,机器人转速调到最慢"),  # to min
    ("转向速度最快", "max", "好的,机器人转速调到最快"),  # 转向速度 form
    ("机器人转慢一点", "slower", "好的,机器人转速调慢一点"),  # 机器人转慢 form
    ("机身转速快一点", "faster", "好的,机器人转速调快一点"),  # 机身 form
])
def test_body_phrases_send_set_turn_speed(text, level, said):  # A15 -> cmd/mode
    """MUTATION: drop "A15" from INTENT_TO_KEY -> no cmd/mode frame -> fails."""
    d, pubs, spoken = _turn(text)  # run the turn
    assert d.intent_id == "A15" and d.kind == "dispatch"  # classified and dispatched
    mode = [p for k, p in pubs.items() if k.endswith("cmd/mode")]  # the ModeCommand
    assert len(mode) == 1 and mode[0]["action"] == "set_turn_speed" and mode[0]["turn_speed"] == level  # one set_turn_speed frame
    assert not any(k.endswith("cmd/motion/intent") for k in pubs)  # setting a speed moves nothing
    assert spoken == [said]  # the direction, no predicted number


def test_bare_rotation_speed_stays_ptz():  # PTZ keeps bare 转速
    """User ruling: "转速快一点" without a body word is the PTZ (E09)."""
    d, pubs, _spoken = _turn("转速快一点")  # bare phrase
    assert d.intent_id == "E09" and not any(k.endswith("cmd/mode") for k in pubs)  # E09, no mode frame


def test_turn_around_still_turns_around():  # A11 untouched
    d, pubs, _spoken = _turn("转身")  # bare 转身
    assert d.intent_id == "A11" and any(k.endswith("cmd/motion/intent") for k in pubs)  # still a 180 degree turn


@pytest.mark.parametrize("text, level", [("调到最快", "max"), ("最慢", "min"), ("快一点", "faster"),  # level words
                                         ("转得慢一点", "slower"), ("机器人转速", None)])  # no level word -> None
def test_level_reader(text, level):  # the reader
    """Longest / most specific first: "调到最快" is max, not faster."""
    assert parse_turn_speed_level(text) == level  # closed-set value


def test_missing_level_is_refused_not_guessed():  # no guessing
    """MUTATION: default a missing level to "faster" -> the robot may speed up unasked -> fails."""
    with pytest.raises(ModeRequestError):  # refused
        to_mode_command("set_turn_speed", slots={}, cmd_id="c", source="cloud")  # slots without turn_speed
