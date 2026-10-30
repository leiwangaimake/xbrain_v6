"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_session_intents.py
Brief: I01-I04 session intents answer from session state / presets -- "再说一遍" repeats the robot's last reply, never the user's words

Description:
Field test 2026-10-29 (P101-P106): "再说一遍", "什么", "没听清", "没事了",
"结束对话" answered "收到", often after reading the user's own words back.
The I class dispatched to cmd/audio/speak, i.e. the utterance was spoken.
These cases drive the production orchestrator over the production configs:
  * I03 repeats the last thing the ROBOT said (and twice in a row repeats the
    same answer, not the repeat);
  * I03 with nothing said yet and I04 answer their chitchat.yaml session
    preset (a bare "是" / "不" outside a confirm window is overheard by design,
    so the I01 / I02 no_pending_confirm reply is a backstop, not tested here);
  * I04 drops a half-open confirm;
  * no I01-I04 turn ever speaks the user's words or publishes a command;
  * "最近都发生了什么" reaches G10 instead of being stolen by I03's "什么".
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
SESSION = CC["session"]  # the I-class replies


def _orch():  # production orchestrator, no query_fn needed here
    with open("/opt/xbrain_v6/configs/intents.yaml", encoding="utf-8") as fh:  # production content file
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    return build_orchestrator(reg, ChitchatResponder(CC), l2_timeout_ms=8000)  # real presets


def _turn(o, s, text):  # one cloud text turn -> (decision, spoken, other keys)
    d = o.handle_turn(text, s, 1000, channel="cloud", modality="text")  # the turn
    pubs = decision_to_publishes(d)  # what would go out
    spoken = [p["text"] for k, p in pubs if k.endswith("audio/speak")]  # TTS texts
    return d, spoken, [k for k, _p in pubs if not k.endswith("audio/speak")]  # result to the caller


def test_repeat_says_the_robot_last_reply_not_the_user_words():  # one query answer: test repeat says the robot last reply not the user words
    """MUTATION: drop the last_reply bookkeeping in handle_turn -> I03 answers
    repeat_none -> this fails."""
    o, s = _orch(), OrchestratorSession()  # one dialog
    _d, first, _k = _turn(o, s, "你是谁")  # the robot introduces itself
    _d, again, _k = _turn(o, s, "再说一遍")  # repeat
    assert again == first  # the robot's own words, again
    _d, third, _k = _turn(o, s, "没听清")  # repeat the repeat
    assert third == first  # still the answer, not "the repeat"


def test_repeat_with_nothing_said_yet():  # one query answer: test repeat with nothing said yet
    """Nothing to repeat: the preset, never the utterance."""
    _d, spoken, _k = _turn(_orch(), OrchestratorSession(), "再说一遍")
    assert spoken == [SESSION["repeat_none"]]  # the property this case pins


@pytest.mark.parametrize("text, key", [("没事了", "cancel_session"), ("结束对话", "cancel_session"),
                                       ("不用了", "cancel_session")])
def test_session_intents_answer_their_preset(text, key):  # one query answer: test session intents answer their preset
    """MUTATION: remove the I01-I04 branch -> they dispatch and speak the
    utterance again -> this fails."""
    d, spoken, keys = _turn(_orch(), OrchestratorSession(), text)
    assert spoken == [SESSION[key]] and keys == []  # a reply, no command
    assert text not in spoken  # never the user's words


def test_cancel_drops_a_half_open_confirm():  # one query answer: test cancel drops a half open confirm
    """I04 while a confirm is pending: the pending action must not survive."""
    o, s = _orch(), OrchestratorSession()
    d, _sp, _k = _turn(o, s, "删除路径")  # L2: asks to confirm (重启 is refused while unwired)
    assert d.kind == "await_confirm" and s.pending_confirm is not None  # the property this case pins
    _turn(o, s, "结束对话")
    assert s.pending_confirm is None  # dropped


def test_recent_events_phrase_is_not_stolen_by_repeat():  # one query answer: test recent events phrase is not stolen by repeat
    """P131 "最近都发生了什么" used to hit I03's bare "什么"."""
    d, _sp, _k = _turn(_orch(), OrchestratorSession(), "最近都发生了什么")
    assert d.intent_id == "G10"  # the property this case pins
