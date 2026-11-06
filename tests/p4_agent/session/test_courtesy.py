"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_courtesy.py
Brief: J04 thanks / J05 farewell answer a courtesy preset; a greeting answers in the time of day the operator said

Description:
Field test 2026-10-29 (P231 / P232, 4.9): "谢谢" / "再见" got "没太听清" on text
and were overheard on voice; "上午好 / 中午好 / 下午好" were overheard. User
ruling 2026-10-30 (99 U101) restored J04 / J05 and asked for the time-of-day
greetings. Production orchestrator, production configs:
  * each phrase answers one of its chitchat.yaml replies and sends no command;
  * "下午好" is answered with a 下午 greeting -- the time the operator SAID,
    not one picked from the clock.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize, marks
import yaml  # production configs

from xbrain.p4_agent.registry.intents import load_intent_registry  # 18 registry
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # turn path
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # dialog state
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets

pytestmark = pytest.mark.no_device  # pure in-process

with open("/opt/xbrain_v6/configs/chitchat.yaml", encoding="utf-8") as _fh:  # production content file
    CC = yaml.safe_load(_fh)  # production presets


def _say(text, channel="cloud", modality="text"):  # one turn -> (decision, spoken, commands)
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production content file
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    o = build_orchestrator(reg, ChitchatResponder(CC))  # real presets
    d = o.handle_turn(text, OrchestratorSession(), 1000, channel=channel, modality=modality)  # local read of the state field
    pubs = decision_to_publishes(d)  # what would go out
    return d, [p["text"] for k, p in pubs if k.endswith("audio/speak")], [k for k, _ in pubs if not k.endswith("audio/speak")]  # result to the caller


@pytest.mark.parametrize("text, block", [("谢谢", "thanks"), ("辛苦了", "thanks"),
                                         ("再见", "farewell"), ("拜拜", "farewell")])
def test_courtesy_phrase_gets_its_preset(text, block):  # one query answer: test courtesy phrase gets its preset
    """MUTATION: drop thanks / farewell from _CHITCHAT_NAMES -> they dispatch
    to cmd/audio/speak and echo the utterance -> this fails."""
    d, spoken, cmds = _say(text)
    assert spoken and spoken[0] in CC[block]["replies"] and cmds == []  # a preset, nothing sent
    assert text not in spoken  # never the user's words


@pytest.mark.parametrize("text, tod", [("下午好", "afternoon"), ("中午好", "noon"), ("上午好", "forenoon")])
def test_greeting_answers_the_time_of_day_said(text, tod):  # one query answer: test greeting answers the time of day said
    """MUTATION: drop the said-time lookup -> the default greeting -> fails."""
    d, spoken, _c = _say(text)
    assert spoken and spoken[0] in CC["greeting"]["time_variant"][tod]  # mirrors what was said


def test_courtesy_on_voice_is_answered_too():  # one query answer: test courtesy on voice is answered too
    """The voice path used to call "谢谢" overheard (P231)."""
    d, spoken, _c = _say("谢谢", channel="mic_local", modality="voice")
    assert d.intent_id == "J04" and spoken  # answered
