"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptp_turn.py
Brief: go to X (now or timed), delete X without a confirm, rename X to Y -- whole turns to the frames

Description:
99 U110 end to end through the orchestrator with this tree's registry, a
three-object catalogue and a fixed wall clock. The tier-2 LLM is a fake
that FAILS the test if called: these sentences must never reach it (it
invented the ids). Each case names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import os  # this tree's configs
from datetime import datetime  # expected epochs
from zoneinfo import ZoneInfo  # site zone

import pytest  # parametrize, marker
import yaml  # content files

from xbrain.p4_agent.registry.intents import load_intent_registry  # the registry loader
from xbrain.p4_agent.runtime.orchestrator_turn import (  # p4 builders
    build_orchestrator,  # as p4's wiring builds it
    decision_to_publishes,  # the frames a turn sends
)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # a fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets

pytestmark = pytest.mark.no_device  # no robot, no LLM

_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree
_TZ = "Asia/Shanghai"  # site zone
_NOW = datetime(2026, 11, 4, 10, 0, tzinfo=ZoneInfo(_TZ)).timestamp()  # 10:00
_ITEMS = [  # catalogue
    {"geo_id": "w-1", "type": "waypoint", "name": "1号测试点", "state": "active", "rev": 3},  # a keypoint
    {"geo_id": "w-gate", "type": "waypoint", "name": "东门岗亭", "state": "active", "rev": 1},  # another keypoint
    {"geo_id": "r-east", "type": "route", "name": "东门路线", "state": "active", "rev": 2},  # a route (rename only)
]  # the catalogue the names resolve against


def _no_llm(*_a, **_k):  # tier-2 fake
    """These sentences must never reach tier-2."""
    raise AssertionError("tier-2 LLM called for a rule-parsed PTP command")  # fail loudly


def _turn(text, clock=True):  # one turn helper
    """One mic turn -> (decision, {key: frame}) -- nothing is sent."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:  # registry file
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:  # presets file
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    orch = build_orchestrator(  # as p4 wires it
        reg, cc, l2_timeout_ms=8000, tier2_fn=_no_llm,  # no LLM allowed
        state_fn=lambda: {"state/geo/manifest": {"items": _ITEMS}},  # the catalogue
        wall_now_fn=(lambda: _NOW) if clock else None, site_timezone=_TZ)  # fixed clock, site zone
    d = orch.handle_turn(text, OrchestratorSession(), 1000, channel="mic_local")  # a mic turn
    frames = {k: p for k, p in decision_to_publishes(d) if k != "cmd/audio/speak"}  # frames, minus speech
    return d, frames  # what the case checks


@pytest.mark.parametrize("text,wp", [  # goto now
    ("去1号测试点", "w-1"), ("前往1号测试点", "w-1"), ("到1号测试点去", "w-1"),  # verb forms
    ("导航到东门岗亭", "w-gate"), ("去一号测试点", "w-1"),  # more verbs
    ("现在请立刻去1号测试点", "w-1"), ("马上前往东门岗亭", "w-gate"),  # prefixes
])
def test_goto_now_dispatches_the_catalogue_id(text, wp):  # case body
    """MUTATION: route these to the M3 LLM leg (the fake raises)."""
    d, frames = _turn(text)  # voice, not addressed
    assert d.kind == "dispatch" and d.intent_id == "B01"  # dispatched
    task = frames["cmd/task"]["task"]  # 11 S7.2
    assert task["params"]["waypoint_id"] == wp and "not_before_ts" not in task  # id, untimed


@pytest.mark.parametrize("text,epoch,said", [  # timed gotos
    ("5分钟后去1号测试点", _NOW + 300, "好的,5分钟后去1号测试点"),
    ("2小时后去1号测试点", _NOW + 7200, "好的,2小时后去1号测试点"),
    ("下午3点去1号测试点",
     datetime(2026, 11, 4, 15, 0, tzinfo=ZoneInfo(_TZ)).timestamp(),
     "好的,今天15点去1号测试点"),
    ("明天14点去1号测试点",
     datetime(2026, 11, 5, 14, 0, tzinfo=ZoneInfo(_TZ)).timestamp(),
     "好的,明天14点去1号测试点"),
])
def test_timed_goto_carries_not_before_ts(text, epoch, said):  # case body
    """11 S7.2 task.not_before_ts. MUTATION: drop it and the goto runs now."""
    d, frames = _turn(text)  # one turn
    assert frames["cmd/task"]["task"]["not_before_ts"] == epoch  # scheduled
    assert d.tts_text == said  # restated with the time


@pytest.mark.parametrize("text,said", [  # refusals
    ("去5号测试点", "没找到5号测试点, 可以问我有哪些位置点"),
    ("上午9点去1号测试点", "这个时间已经过了, 请换个时间"),
    ("删除5号测试点", "没找到5号测试点, 可以问我有哪些位置点"),
    ("删除航点", "没有说要删除哪个点, 没有删除"),
    ("下午3点删除1号测试点", "删除和改名不能定时, 请去掉时间再说一遍"),
    ("把1号测试点改名为1号测试点", "新名字和原来一样, 没有改名"),
])
def test_refusals_send_nothing(text, said):  # case body
    d, frames = _turn(text)  # one turn
    assert not frames and d.tts_text == said  # said why, sent nothing


def test_timed_goto_without_a_clock_is_refused():  # no clock
    """MUTATION: schedule against a guessed clock."""
    d, frames = _turn("下午3点去1号测试点", clock=False)  # no wall clock
    assert not frames and d.tts_text == "现在不能定时, 请直接说去哪个点"  # said why


@pytest.mark.parametrize("text,wp", [  # deletes
    ("删除1号测试点", "w-1"), ("立刻删除1号测试点", "w-1"),
    ("把1号测试点删掉", "w-1"), ("删除航点东门岗亭", "w-gate"),
])
def test_delete_runs_without_a_confirm(text, wp):  # case body
    """99 U110 user ruling: F12 is L1a. MUTATION: leave it L2 and the turn
    stops at await_confirm."""
    d, frames = _turn(text)  # one turn
    assert d.kind == "dispatch" and d.auth == "L1a"  # no confirm
    assert frames["cmd/geo"]["action"] == "delete" and frames["cmd/geo"]["geo_id"] == wp  # the delete frame


@pytest.mark.parametrize("text,gid,new", [  # renames
    ("将1号测试点改名为2号测试点", "w-1", "2号测试点"),
    ("现在把东门岗亭重命名为西门岗亭", "w-gate", "西门岗亭"),
    ("把东门路线改名为北门路线", "r-east", "北门路线"),
    ("把1号测试点改名为三号测试点", "w-1", "3号测试点"),
])
def test_rename(text, gid, new):  # case body
    """MUTATION: the M6 LLM leg (it lost the old name)."""
    d, frames = _turn(text)  # one turn
    geo = frames["cmd/geo"]  # 11 S7.9
    assert d.intent_id == "F14" and geo["action"] == "rename"  # F14
    assert geo["geo_id"] == gid and geo["obj"] == {"name": new}  # canonical new name
