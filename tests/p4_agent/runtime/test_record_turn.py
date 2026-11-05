"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_record_turn.py
Brief: 99 U133 -- recording by voice: start with a full name, exit saves, recording mode refuses control

Description:
The user's rulings of 2026-11-05 (99 U133) on the P4 side, through a real
orchestrator built as p4 wires it (no LLM: a call to it fails the case):
  * "现在开始录制1号路径" -> cmd/teach start with start.name; a fence name ->
    a fence recording; no full name / a taken name / a bad length -> a spoken
    refusal and nothing sent, never a question;
  * while state/teach says recording: another start, a goto, a patrol, a
    motion and a payload command are answered "正在录制..." and nothing is
    sent; a query still answers; exit sends finish with the session id
    (F02 for a route, F08 for a fence); status answers from state/teach;
  * exit / status with no recording answer "现在没有在录制";
  * "删除1号路径" / "删除东门围栏" open the L2 confirm of F11 / F13.
The parser cases pin record_slots itself (F06's "记录当前位置为X" must stay
F06's).
"""
from __future__ import annotations  # postponed annotations

import os  # this tree's configs

import pytest  # parametrize, marker
import yaml  # content files

from xbrain.p4_agent.registry.intents import load_intent_registry  # the registry loader
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # as wired
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # a fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets
from xbrain.p4_agent.slots.record_slots import parse_record  # under test

pytestmark = pytest.mark.no_device  # no robot, no LLM

_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree
_ITEMS = [  # catalogue: one route, one fence, one keypoint
    {"geo_id": "r-1", "type": "route", "name": "1号路径", "state": "active", "rev": 1},
    {"geo_id": "f-1", "type": "fence", "name": "东门围栏", "state": "active", "rev": 1},
    {"geo_id": "w-1", "type": "waypoint", "name": "1号测试点", "state": "active", "rev": 1},
]
_RECORDING = {"session": {"session_id": "ts-1", "state": "recording", "kind": "route",
                          "name": "2号路径"},
              "stats": {"point_count": 12, "length_m": 23.6}}  # state/teach while recording


def _no_llm(*_a, **_k):  # tier-2 fake
    raise AssertionError("tier-2 LLM called for a recording turn")  # fail loudly


def _turn(text, teach=None):  # one mic turn
    """(decision, {key: frame}) with state/teach = teach (None = idle)."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:  # registry
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:  # presets
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    state = {"state/geo/manifest": {"items": _ITEMS},  # the catalogue
             "state/teach": teach or {"session": {"state": "idle"}}}  # the session
    orch = build_orchestrator(reg, cc, l2_timeout_ms=8000, tier2_fn=_no_llm,
                              state_fn=lambda: state)  # as p4 wires it
    d = orch.handle_turn(text, OrchestratorSession(), 1000, channel="mic_local")  # a mic turn
    frames = {k: p for k, p in decision_to_publishes(d) if k != "cmd/audio/speak"}  # minus speech
    return d, frames  # what the case checks


def _teach(frames):  # the cmd/teach body
    body = frames["cmd/teach"]  # may be enveloped
    return body.get("data", body) if isinstance(body, dict) else body  # bare body


# -- the parser ---------------------------------------------------------------
@pytest.mark.parametrize("text,kind,name,rtype", [
    ("现在开始录制1号路径", "start", "1号路径", "route"),  # the user's example
    ("开始录制东门围栏", "start", "东门围栏", "fence"),  # a fence
    ("录制一条路径叫3号路线", "start", "3号路线", "route"),  # 叫 introduces the name
    ("开始记录西门环线", "start", "西门环线", "route"),  # 记录 with a full name
    ("开始录制", "start_noname", "", None), ("开始录制路径", "start_noname", "", None),  # no name
    ("开始录制围栏", "start_noname", "", None),  # the bare fence word names nothing either
    ("录制1号", "start_noname", "", None),  # no type word
    ("退出路径录制", "exit", "", None), ("退出录制", "exit", "", None),  # exits
    ("录制完成", "exit", "", None), ("围栏录制完成", "exit", "", None),
    ("记录完成", "exit", "", None), ("结束录制", "exit", "", None), ("录完了", "exit", "", None),
    ("录了几个点", "status", "", None), ("正在录制吗", "status", "", None),  # status
])
def test_parse_record(text, kind, name, rtype):  # the rule
    """MUTATION: accept the bare type word as a name -> 开始录制路径 is a start -> red."""
    got = parse_record(text)  # one parse
    assert got is not None and (got.kind, got.name, got.rtype) == (kind, name, rtype)  # as ruled


@pytest.mark.parametrize("text", ["记录当前位置为1号点", "记住这个位置", "去1号测试点"])
def test_not_a_recording_sentence(text):  # F06 keeps 记录
    """MUTATION: claim every 记录 sentence -> F06 is stolen -> red."""
    assert parse_record(text) is None  # left to the other layers


# -- starting -----------------------------------------------------------------
@pytest.mark.parametrize("text,kind,name", [("现在开始录制2号路径", "route", "2号路径"),
                                            ("开始录制北门围栏", "fence", "北门围栏")])
def test_a_named_start_sends_the_name(text, kind, name):  # start
    """MUTATION: send only name_hint -> no start.name -> red."""
    d, frames = _turn(text)  # idle
    start = _teach(frames)["start"]  # the block
    assert d.kind == "dispatch" and start["kind"] == kind and start["name"] == name  # bound name
    assert name in d.tts_text and "键盘" in d.tts_text  # tells the operator how to drive


@pytest.mark.parametrize("text,said", [
    ("开始录制路径", "录制指令要带上完整名称"),  # bare type word
    ("开始录制", "录制指令要带上完整名称"),  # nothing at all
    ("现在开始录制1号路径", "已经有叫1号路径的路径了"),  # taken route
    ("开始录制东门围栏", "已经有叫东门围栏的围栏了"),  # taken fence
    ("开始录制特别特别特别特别特别长的路径", "名字要两到十二个字"),  # 14 characters
])
def test_bad_starts_are_answered_not_asked(text, said):  # no second question
    """MUTATION: drop the catalogue clash check -> a start is sent -> red."""
    d, frames = _turn(text)  # idle
    assert "cmd/teach" not in frames and said in d.tts_text  # refused with the reason
    assert d.kind != "await_confirm"  # never a question


def test_the_keyword_leg_without_a_name_is_refused_too():  # "画围栏" (F07 keyword)
    """MUTATION: let the keyword leg build a nameless start -> red."""
    d, frames = _turn("画围栏")  # F07 keyword, no name
    assert "cmd/teach" not in frames and "完整名称" in d.tts_text  # refused


# -- recording mode -----------------------------------------------------------
@pytest.mark.parametrize("text,said", [
    ("开始录制3号路径", "不能开始新的录制"),  # another start
    ("去1号测试点", "录制期间只能用键盘"),  # a goto (PTP rule)
    ("沿1号路径巡逻", "录制期间只能用键盘"),  # a patrol (PTP rule)
    ("向前走一米", "录制期间只能用键盘"),  # a motion (A class)
    ("打开照明灯", "录制期间只能用键盘"),  # a payload (D class)
    ("删除1号路径", "录制期间只能用键盘"),  # a delete
    ("下午3点去1号测试点", "录制期间只能用键盘"),  # a timed goto: refused before its clock check
])
def test_recording_mode_refuses_control(text, said):  # U133
    """MUTATION: no recording gate -> the command is sent -> red."""
    d, frames = _turn(text, _RECORDING)  # recording
    assert not frames and said in d.tts_text and "2号路径" in d.tts_text  # refused, names the recording


def test_a_query_still_answers_while_recording():  # G class allowed
    """MUTATION: block every class -> the query is refused -> red."""
    d, _frames = _turn("有哪些路径", _RECORDING)  # G21
    assert d.intent_id == "G21" and "录制期间" not in (d.tts_text or "")  # answered


@pytest.mark.parametrize("teach,fid", [(_RECORDING, "F02"),
                                       ({**_RECORDING, "session": {**_RECORDING["session"],
                                                                   "kind": "fence",
                                                                   "name": "西门围栏"}}, "F08")])
def test_exit_sends_finish_with_the_session(teach, fid):  # exit = save
    """MUTATION: route exit to the keyword layer -> no session id -> red."""
    d, frames = _turn("退出录制", teach)  # exit
    cmd = _teach(frames)  # the frame
    assert d.intent_id == fid and cmd["action"] == "finish" and cmd["session_id"] == "ts-1"  # finish


def test_status_answers_from_state_teach():  # status
    """MUTATION: answer with a constant -> counts missing -> red."""
    d, frames = _turn("录了几个点", _RECORDING)  # status
    assert not frames and d.tts_text == "正在录制2号路径, 已经录了12个点, 长24米"  # live counts


@pytest.mark.parametrize("text", ["退出录制", "录了几个点"])
def test_exit_and_status_without_a_recording(text):  # idle
    d, frames = _turn(text)  # idle
    assert not frames and d.tts_text == "现在没有在录制"  # nothing to end


@pytest.mark.parametrize("text,fid", [("删除1号路径", "F11"), ("删除东门围栏", "F13")])
def test_route_and_fence_deletes_ask_for_confirmation(text, fid):  # F11 / F13 by name
    """MUTATION: dispatch at once (L1a) -> a cmd/geo frame -> red."""
    d, frames = _turn(text)  # idle
    assert d.kind == "await_confirm" and d.intent_id == fid and not frames  # L2


# -- 99 U137: cancel discards at once ------------------------------------------
@pytest.mark.parametrize("text", ["取消录制", "放弃录制", "取消路径录制", "取消围栏录制"])
def test_cancel_discards_without_a_question(text):  # F04 L1a
    """MUTATION: keep F04 at L2 -> await_confirm, nothing sent -> red."""
    d, frames = _turn(text, _RECORDING)  # recording
    cmd = _teach(frames)  # the frame
    assert d.kind == "dispatch" and d.intent_id == "F04" and cmd["action"] == "discard"  # sent now
    assert cmd["session_id"] == "ts-1" and d.tts_text == "好的,取消录制2号路径"  # names it


def test_cancel_without_a_recording():  # idle
    d, frames = _turn("取消录制")  # idle
    assert not frames and d.tts_text == "现在没有在录制"  # nothing to discard


@pytest.mark.parametrize("text", ["不要了", "作废"])
def test_casual_words_no_longer_discard(text):  # narrowed trigger
    """MUTATION: keep 不要了 / 作废 as F04 keywords -> a chat word throws the
    recording away -> red."""
    d, frames = _turn(text, _RECORDING)  # recording
    assert "cmd/teach" not in frames  # nothing discarded

