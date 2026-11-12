"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_record_waypoint_turn.py
Brief: "记录当前位置为1号测试点" by voice reaches P3 as a mark_once with the first-word pose

Description:
99 U109, the 2026-11-04 F06 audit, end to end through the microphone
handler with the real registry and a scripted tier-2:
  * a voice turn is not addressed (16 S5.2.1), so it reaches tier-2 only
    through a keyword -- before U109 every phrasing but the doc example was
    dropped as overheard;
  * the name the model returns must have been said, travels in one spelling,
    and is 2-12 characters;
  * "unknown / missing_slot" asks for the name instead of "没太听清";
  * the command carries the 18 PS-6 pose_snap taken at the first word, and
    is not sent without one (16 EV-6).
The content files are this tree's own (the registry under test), not the
installed /opt copy. Nothing reaches a robot: the handler only returns frames.
"""
from __future__ import annotations

import json  # frames are JSON bytes
import os  # this tree's configs

import pytest  # parametrize, marker
import yaml  # the content files, as p4 loads them

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,  # as p4's wiring builds it
    make_turn_handler,  # the microphone entry
)
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession,  # a fresh dialog per case
    Tier2Classification,  # what a tier-2 call returns
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

pytestmark = pytest.mark.no_device  # no robot, no Zenoh, no LLM

#: This tree's configs: the F06 keywords under test live here.
_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")
_SNAP = {"lat": 34.697, "lon": 135.505, "fix_type": "rtk_fixed",
         "snap_mono_s": 97.0, "snap_age_ms": 3000}  # PS-6 shape


class _LLM:  # scripted tier-2
    """A tier-2 fake that records the texts it was asked about."""

    def __init__(self, answer):  # the reply to give
        self.answer, self.texts = answer, []  # scripted reply, call log

    def __call__(self, text, _session, _now_ms):  # tier2_fn signature
        self.texts.append(text)  # one more call
        return self.answer  # the scripted classification


class _Snaps:  # scripted pose_snap_fn
    """A pose_snap_fn fake: records the first-word time it was asked for."""

    def __init__(self, snap=_SNAP):  # the snapshot to give
        self.snap, self.asked = snap, []  # reply, call log

    def __call__(self, t_first, now_ms):  # pose_snap_fn signature
        self.asked.append((t_first, now_ms))  # what the orchestrator passed
        return self.snap  # None = no pose near the word


def _turn(text, llm, snaps=None, t_first_word=None):
    """One mic turn -> (teach frame or None, spoken text)."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))  # chitchat presets
    orch = build_orchestrator(reg, cc, tier2_fn=llm,
                              pose_snap_fn=snaps if snaps is not None else _Snaps())
    pairs = make_turn_handler(orch, OrchestratorSession())(
        text, t_first_word=t_first_word)  # frames, nothing sent
    teach = [json.loads(d) for k, d in pairs if k == "cmd/teach"]  # the capture
    spoken = [json.loads(d)["text"] for k, d in pairs
              if k == "cmd/audio/speak"]  # the reply
    return (teach[0] if teach else None), (spoken[0] if spoken else "")


def _named(name):  # one-line LLM fake
    """The model answering record_waypoint with this name."""
    return _LLM(Tier2Classification("record_waypoint", {"name": name}))


@pytest.mark.parametrize("text,answer,stored", [
    ("记录当前位置为1号测试点", "1号测试点", "1号测试点"),  # the audit's phrasing
    ("保存当前位置为1号测试点", "1号测试点", "1号测试点"),  # 保存
    ("保持当前位置为1号测试点", "1号测试点", "1号测试点"),  # the user's 保持
    ("把当前位置保存为1号测试点", "1号测试点", "1号测试点"),  # was a fence save
    ("把这个点存为1号测试点", "1号测试点", "1号测试点"),  # 这个点
    ("把这里记为1号测试点", "1号测试点", "1号测试点"),  # the 18 shape
    ("当前位置记为一号测试点", "一号测试点", "1号测试点"),  # one spelling
    ("记录当前位置为去程起点", "去程起点", "去程起点"),  # B01's 去 not stealing
])
def test_voice_capture_reaches_p3_as_mark_once(text, answer, stored):
    """MUTATION: drop the F06 keywords and each of these is overheard (no
    frame, no reply) -- the state before 99 U109."""
    teach, spoken = _turn(text, _named(answer))  # one voice turn
    assert teach is not None, spoken  # a capture went out
    assert teach["action"] == "mark_once"  # F06, not a goto / fence save
    assert teach["mark_once"]["name"] == stored  # canonical spelling
    assert teach["pose_snap"] == _SNAP  # 18 PS-6 attached
    assert spoken == "好的,记录这个点"  # the result follows from the ack


def test_a_keyword_that_contains_the_name_still_has_a_name():
    """Live 2026-11-04: "把这里记为东门岗亭" is itself an F06 keyword (18's
    example), and the longest-keyword tail was empty -> refused as nameless.
    MUTATION: cut after the longest keyword."""
    teach, spoken = _turn("把这里记为东门岗亭", _named("东门岗亭"))  # the 18 example
    assert teach is not None and teach["mark_once"]["name"] == "东门岗亭"  # captured


def test_the_snapshot_is_taken_at_the_first_word():
    """18 PS-1. MUTATION: ask for the pose at the turn's own clock."""
    snaps = _Snaps()  # records what it was asked
    _turn("记录当前位置为1号测试点", _named("1号测试点"), snaps,
          t_first_word=12345)  # the VAD onset
    assert snaps.asked and snaps.asked[0][0] == 12345  # the word, not "now"


def test_no_pose_near_the_word_sends_nothing():
    """16 EV-6. MUTATION: send without pose_snap and P3 reads "now"."""
    teach, spoken = _turn("记录当前位置为1号测试点", _named("1号测试点"),
                          _Snaps(snap=None))  # no frame near the word
    assert teach is None  # nothing sent
    assert spoken == "现在拿不到定位, 这个点没有记录"  # said why


def test_an_invented_name_is_refused():
    """16 EV-2. MUTATION: trust the model and a point named "东门岗亭"
    appears that nobody named."""
    llm = _named("东门岗亭")  # the model would invent this
    teach, spoken = _turn("记录当前位置为", llm)  # no name said
    assert teach is None  # nothing sent
    assert spoken == "这个点没有记录, 因为没有说名字. 请说: 把这里记为某某点"  # refused, not asked
    assert llm.texts == []  # 99 U110: no name after the keyword -> no LLM


def test_a_name_the_model_did_not_take_from_the_words_is_refused():
    """The keyword is followed by words, but the model's name is not among
    them. MUTATION: trust the model."""
    teach, spoken = _turn("记录当前位置为北边那个", _named("东门岗亭"))
    assert teach is None and spoken == "这个点没有记录, 因为没有说名字. 请说: 把这里记为某某点"  # refused


def test_the_command_words_are_not_taken_as_the_name():
    """Measured on the real model: "记录当前位置" -> name "当前位置".
    MUTATION: grounding without the place words and that point is stored."""
    teach, spoken = _turn("记录当前位置是当前位置", _named("当前位置"))  # copied the place
    assert teach is None and spoken == "这个点没有记录, 因为没有说名字. 请说: 把这里记为某某点"  # refused


def test_unknown_missing_slot_is_refused_not_dropped():
    """99 U109: the grammar can now say unknown. MUTATION: treat it as a
    decline and the operator hears "没太听清" for a clear command."""
    llm = _LLM(Tier2Classification("unknown", {}, reason="missing_slot"))
    teach, spoken = _turn("记录当前位置为那个", llm)  # words, but no name
    assert teach is None and spoken == "这个点没有记录, 因为没有说名字. 请说: 把这里记为某某点"  # the refusal


@pytest.mark.parametrize("text", ["记录当前位置", "把这里记为", "保存当前位置。",
                                  "记住这个位置", "记录当前位置为"])
def test_no_name_is_refused_directly_without_the_llm(text):
    """99 U110 user ruling: no follow-up question, no LLM call. MUTATION:
    send it to tier-2 and the 3B model names the point "当前位置"."""
    llm = _named("当前位置")  # what the real model answered
    teach, spoken = _turn(text, llm)  # no name said
    assert teach is None and spoken == "这个点没有记录, 因为没有说名字. 请说: 把这里记为某某点"  # refused
    assert llm.texts == []  # decided without the model


def test_unknown_for_any_other_reason_declines():
    llm = _LLM(Tier2Classification("unknown", {}, reason="unsupported"))
    teach, spoken = _turn("记录当前位置为1号测试点", llm)  # model declined
    assert teach is None and "没有说名字" not in spoken  # a decline, not the no-name refusal


def test_a_name_over_twelve_characters_is_refused():
    name = "一二三四五六七八九十一二三"  # 13 characters
    teach, spoken = _turn("记录当前位置为" + name, _named(name))  # said, too long
    assert teach is None  # 16 S8.0.4: refused, not cut
    assert spoken == "名字要两到十二个字, 请换个名字"  # said why


def test_a_name_over_twelve_characters_is_a_denied_turn():
    """2026-11-12: the length refusal is "denied" (verdict rejected), not a
    reply that reads accepted. MUTATION: drop _CAPTURE_INTENTS from the
    F-class refusal kind -> "reply" -> red."""
    name = "一二三四五六七八九十一二三"  # 13 characters
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))  # chitchat presets
    orch = build_orchestrator(reg, cc, tier2_fn=_named(name), pose_snap_fn=_Snaps())
    d = orch.handle_turn("记录当前位置为" + name, OrchestratorSession(), 1000,
                         channel="mic_local")  # one mic turn
    assert d.kind == "denied" and d.tts_text == "名字要两到十二个字, 请换个名字"
