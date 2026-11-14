"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_one_question_one_answer.py
Brief: 99 U147 -- every voice / text command is answered in one turn: no confirm round, no follow-up question

Description:
The user's ruling of 2026-11-06: "系统所有语音, 文本指令都是一问一答, 不允许有
反问, 追问的情况出现". Two halves are pinned here:
  * the wording: every spoken table P4 answers from (the orchestrator's refusal
    and reply tables, the ack speaker's tables, chitchat.yaml) holds
    statements -- no question mark, no sentence ending in 吗 (a final 呢 is
    kept: "我在呢" is a statement). A
    statement may still say how to phrase the command ("例如: 前进3米"); the
    next utterance is a new command either way, nothing waits for it;
  * the behaviour: a command missing its argument is refused on its own turn
    with that statement and sends nothing, and an L2 command (F11 delete
    route) is sent on its own turn.
Why a wording scan and not only turn cases: the tables hold dozens of lines
reached by paths no single test drives (P3 refusal reasons, teach issues);
a question re-added to any of them is the defect this ruling forbids, and
only a scan over the whole table sees it. Each case names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import os  # this tree's configs
import re  # the question shape

import pytest  # parametrize
import yaml  # content files

from xbrain.p4_agent.registry.intents import load_intent_registry  # registry
from xbrain.p4_agent.runtime import ack_speech, turn_orchestrator  # the tables
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # as wired
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession, Tier2Classification  # a dialog, an LLM answer
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets

pytestmark = pytest.mark.no_device  # pure logic

_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree
_QUESTION = re.compile("[?%s]|%s\\s*$" % (chr(0xFF1F), chr(0x5417)))  # ? / full-width ? / ends in 吗 (a final 呢 stays: "我在呢")
_CJK = re.compile("[%s-%s]" % (chr(0x4E00), chr(0x9FFF)))  # spoken Chinese text


def _strings(obj):  # every string inside a nested table
    """Yield every str leaf of dicts / lists / tuples."""
    if isinstance(obj, str):
        yield obj  # a leaf
    elif isinstance(obj, dict):
        for v in obj.values():  # values only: keys are ids
            yield from _strings(v)
    elif isinstance(obj, (list, tuple)):
        for v in obj:  # each item
            yield from _strings(v)


def _module_tables(mod):  # module-level dict tables of spoken text
    """(name, text) for every Chinese string in a module-level dict."""
    for name, val in vars(mod).items():  # the module's globals
        if isinstance(val, dict) and name.startswith("_"):  # private tables only
            for text in _strings(val):
                if _CJK.search(text):  # spoken Chinese, not ids / keys
                    yield name, text


def _spoken_lines():  # everything P4 can say from a table
    """(where, text) over the orchestrator, ack speaker and chitchat.yaml."""
    out = list(_module_tables(turn_orchestrator)) + list(_module_tables(ack_speech))
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:
        out += [("chitchat.yaml", t) for t in _strings(yaml.safe_load(fh)) if _CJK.search(t)]
    return out


def test_the_scan_surface_is_not_empty():  # guards the scan itself
    """A scan over nothing is always green (CLAUDE.md 3.2). MUTATION: read
    the wrong module -> no motion refusal in the surface -> red."""
    lines = [t for _w, t in _spoken_lines()]  # the surface
    assert any(t.startswith("没有说走多远") for t in lines)  # the motion refusal is in it
    assert any(t.startswith("没有说调音量,灯光还是云台") for t in lines)  # and chitchat.yaml (99 U197 wording)
    assert len(lines) > 100  # the whole tables, not a sample


def test_no_spoken_line_is_a_question():  # the wording half
    """MUTATION: restore "要走多远? 请说一个米数" (or the chitchat.yaml
    "要调音量还是灯光?") -> listed here -> red."""
    asked = [(w, t) for w, t in _spoken_lines() if _QUESTION.search(t)]  # questions
    assert asked == []  # none at all


def _turn(text, items=(), tier2=None):  # one mic turn
    """(spoken, {key: frame} minus speech) for one turn; tier2 None = LLM down."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    state = {"state/geo/manifest": {"items": list(items)}}  # the catalogue
    orch = build_orchestrator(reg, cc, tier2_fn=tier2, state_fn=lambda: state)  # None = null tier-2
    d = orch.handle_turn(text, OrchestratorSession(), 1000, channel="mic_local")  # one turn
    frames = {k: p for k, p in decision_to_publishes(d) if k != "cmd/audio/speak"}  # minus speech
    return d.tts_text, frames  # what the case checks


@pytest.mark.parametrize("text,said", [
    ("前进", "没有说走多远, 没有执行. 请带上米数说, 例如: 前进3米"),  # A05 without metres
    ("左转", "没有说转多少度, 没有执行. 请带上度数说, 例如: 左转90度"),  # A09 without degrees
    ("开始巡逻", "没有说沿哪条路线巡逻, 没有执行. 请带上路线名称说, 例如: 沿1号路径巡逻"),  # B02 without a route
])
def test_a_missing_argument_is_refused_in_one_turn(text, said):  # the behaviour half
    """MUTATION: ask "要走多远?" -> red; send the command without the slot ->
    a frame -> red."""
    got, frames = _turn(text, [{"type": "route", "geo_id": "r-1", "name": "1号路径",
                                "state": "active"}])  # a catalogue, so only the slot is missing
    assert got == said and frames == {}  # one statement, nothing sent


def test_an_l2_command_is_sent_on_its_own_turn():  # F11, no confirm round
    """MUTATION: restore the L2 confirm -> "确认...吗,请说是或否", no frame -> red."""
    got, frames = _turn("删除1号路径", [{"type": "route", "geo_id": "r-1", "name": "1号路径",
                                        "state": "active", "rev": 1}])  # the route exists
    assert "cmd/geo" in frames and got == "好的,删除1号路径"  # sent and said, this turn


def test_an_llm_routed_l2_command_says_what_it_does():  # the label ack
    """An L2 command the LLM classified (no rule-layer ack) answers with the
    action's name, not a bare "收到". MUTATION: drop the F11 label -> "收到"
    -> red."""
    llm = lambda *a, **k: Tier2Classification(name="delete_route", slots={"route": "1号路径"})  # noqa: E731
    got, frames = _turn("帮我处理一下那个事情", [{"type": "route", "geo_id": "r-1", "name": "1号路径",
                                               "state": "active", "rev": 1}], tier2=llm)  # no keyword
    assert "cmd/geo" in frames and got == "好的,删除路径"  # sent and named, this turn
