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
from xbrain.p4_agent.state.route_membership import make_membership_fn  # U130 answers

pytestmark = pytest.mark.no_device  # no robot, no LLM

_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree
_TZ = "Asia/Shanghai"  # site zone
_NOW = datetime(2026, 11, 4, 10, 0, tzinfo=ZoneInfo(_TZ)).timestamp()  # 10:00
_ITEMS = [  # catalogue
    {"geo_id": "w-1", "type": "waypoint", "name": "1号测试点", "state": "active", "rev": 3},  # a keypoint
    {"geo_id": "w-gate", "type": "waypoint", "name": "东门岗亭", "state": "active", "rev": 1},  # another keypoint
    {"geo_id": "r-east", "type": "route", "name": "东门路线", "state": "active", "rev": 2},  # a route (rename only)
    {"geo_id": "w-dock1", "type": "waypoint", "name": "1号充电站", "state": "active", "rev": 1},  # a dock point (U128)
]  # the catalogue the names resolve against
_HOME = {"geo_id": "w-home", "type": "waypoint", "name": "返航终点站", "state": "active", "rev": 1}  # U128


def _no_llm(*_a, **_k):  # tier-2 fake
    """These sentences must never reach tier-2."""
    raise AssertionError("tier-2 LLM called for a rule-parsed PTP command")  # fail loudly


def _turn(text, clock=True, items=None, objects=None, member=None):  # one turn helper
    """One mic turn -> (decision, {key: frame}) -- nothing is sent."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:  # registry file
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:  # presets file
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    orch = build_orchestrator(  # as p4 wires it
        reg, cc, tier2_fn=_no_llm,  # no LLM allowed
        state_fn=lambda: {"state/geo/manifest": {"items": items if items is not None else _ITEMS},
                          "state/geo/objects": objects},  # the catalogue + the geometry (U130)
        membership_fn=make_membership_fn(member) if member else None,  # common.geo.route_membership_m (U130)
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
    ("删除5号测试点", "没找到叫5号测试点的航点, 路径或围栏, 没有删除"),  # 99 U177 wording
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
    assert not frames and d.tts_text == "现在拿不到系统时间, 不能定时, 没有执行"  # said why (U147: no follow-up)


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


# -- 99 U121: a named patrol --------------------------------------------------
@pytest.mark.parametrize("text", ["沿东门路线巡逻", "巡逻东门路线", "现在去东门路线巡逻"])
def test_named_patrol_dispatches_the_route_id(text):  # case body
    """MUTATION: leave patrol to the keyword layer -> no route slot -> the
    turn ends in "which route?" and no cmd/task is sent (the 11-05 finding)."""
    d, frames = _turn(text)  # one turn
    assert d.kind == "dispatch" and d.intent_id == "B02"  # B02 dispatched
    task = frames["cmd/task"]["task"]  # 11 S7.2
    assert task["type"] == "patrol" and task["route_id"] == "r-east"  # the catalogue id
    assert "not_before_ts" not in task  # untimed


def test_timed_named_patrol_carries_not_before_ts():  # case body
    """B03 through the rule layer. MUTATION: drop the patrol time branch and
    the patrol starts now."""
    d, frames = _turn("下午3点沿东门路线巡逻")  # timed patrol
    epoch = datetime(2026, 11, 4, 15, 0, tzinfo=ZoneInfo(_TZ)).timestamp()  # 15:00 today
    assert frames["cmd/task"]["task"]["not_before_ts"] == epoch  # scheduled
    assert d.tts_text == "好的,今天15点沿东门路线巡逻"  # restated with the time


@pytest.mark.parametrize("text,slots,said", [  # 99 U131 laps reach P3
    ("沿东门路线巡逻3圈", {"route": "东门路线", "loops": 3}, "好的,沿东门路线巡逻3圈"),
    ("沿东门路线来回巡逻2次", {"route": "东门路线", "loops": 2, "round_trip": True},
     "好的,沿东门路线来回巡逻2次"),
    ("沿东门路线巡逻", {"route": "东门路线"}, "好的,沿东门路线巡逻"),  # nothing added
])
def test_patrol_laps_ride_in_the_slots(text, slots, said):  # case body
    """Until U131 a count above one was refused. MUTATION: drop _lap_slots
    -> P3 runs one pass -> red; ack without {how} -> red."""
    d, frames = _turn(text)  # one turn
    task = frames["cmd/task"]["task"]  # 11 S7.2
    assert task["route_id"] == "r-east" and task["params"]["slots"] == slots  # what P3 reads
    assert d.tts_text == said  # restated


def _running_patrol_state(items=None):  # state with a running patrol
    return {"state/geo/manifest": {"items": items or _ITEMS},  # catalogue
            "state/task": {"current": {"task_id": "t-1", "type": "patrol", "state": "running"}}}  # 11 S4.4


@pytest.mark.parametrize("text,slots,said", [  # 99 U131 B04
    ("再巡一圈", {"loops": 1}, "好的,再巡逻1圈"),
    ("来回巡三次", {"loops": 3, "round_trip": True}, "好的,再来回巡逻3次"),
])
def test_repeat_sends_a_routeless_patrol_repeat(text, slots, said):  # case body
    """P3 fills the route. MUTATION: send it as B02 -> no route -> refused
    "没有说沿哪条路线巡逻" -> red."""
    d, frames = _turn(text)  # nothing running
    task = frames["cmd/task"]["task"]  # 11 S7.2
    assert d.intent_id == "B04" and task["type"] == "patrol" and "route_id" not in task  # P3 fills
    assert task["params"]["intent"] == "patrol_repeat" and task["params"]["slots"] == slots  # as said
    assert d.tts_text == said  # restated


@pytest.mark.parametrize("text", ["再巡一圈", "接着巡"])  # rule leg and keyword leg
def test_repeat_while_patrolling_is_answered_not_queued(text):  # case body
    """The user's ruling: "当前巡逻结束后再说". MUTATION: check only in the
    rule branch -> "接着巡" (keyword leg) queues one -> red."""
    from xbrain.p4_agent.runtime import turn_orchestrator as to_mod  # the orchestrator module
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:  # registry file
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:  # presets file
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    orch = build_orchestrator(reg, cc, tier2_fn=_no_llm,  # as p4 wires it
                              state_fn=_running_patrol_state,  # a patrol is running
                              wall_now_fn=lambda: _NOW, site_timezone=_TZ)  # clock
    d = orch.handle_turn(text, OrchestratorSession(), 1000, channel="mic_local")  # a mic turn
    keys = [k for k, _ in decision_to_publishes(d)]  # what went out
    assert "cmd/task" not in keys  # nothing queued
    assert d.tts_text == to_mod._PTP_CN["patrol_running"]  # said why


@pytest.mark.parametrize("text,said", [  # patrol refusals
    ("沿5号路线巡逻", "没找到5号路线, 可以问我有哪些路径"),
])
def test_patrol_refusals_send_nothing(text, said):  # case body
    """MUTATION: say "位置点" for a route."""
    d, frames = _turn(text)  # one turn
    assert not frames and d.tts_text == said  # said why, sent nothing



# -- 99 U128: B09 named dock, B08 home pre-check --------------------------------
def test_named_dock_dispatches_a_charge_task_to_that_point():  # case body
    """MUTATION: drop GEO_REF_SLOT goto_dock -> no waypoint_id -> P3 would
    pick the nearest dock instead of the named one -> red."""
    d, frames = _turn("去1号充电站")  # named station
    assert d.kind == "dispatch" and d.intent_id == "B09"  # B09
    task = frames["cmd/task"]["task"]  # 11 S7.2
    assert task["type"] == "charge" and task["params"]["waypoint_id"] == "w-dock1"  # that dock


def test_bare_dock_sends_a_charge_task_without_a_point():  # case body
    """P3 picks the nearest dock. MUTATION: refuse the bare form -> red."""
    d, frames = _turn("去充电站")  # no name
    task = frames["cmd/task"]["task"]  # 11 S7.2
    assert task["type"] == "charge" and "waypoint_id" not in task["params"]  # nearest, P3's call


def test_return_without_a_home_point_says_so_and_sends_nothing():  # case body
    """MUTATION: drop the pre-check -> a return_home is sent and later fails
    (or, worse, goes to a dock) -> red."""
    d, frames = _turn("返航")  # no 返航终点站 in the catalogue
    assert not frames and "返航终点站" in d.tts_text  # said why, sent nothing


def test_return_with_a_home_point_is_sent():  # case body
    """MUTATION: check the wrong name -> refused -> red."""
    d, frames = _turn("返航", items=_ITEMS + [_HOME])  # home defined
    assert frames["cmd/task"]["task"]["type"] == "return_home"  # sent


# -- 99 U143: timed return / charge ("支持定时", user 2026-11-06) ----------------
_T15 = datetime(2026, 11, 4, 15, 0, tzinfo=ZoneInfo(_TZ)).timestamp()  # 15:00 today


@pytest.mark.parametrize("text,ttype,epoch,said", [
    ("下午3点返航", "return_home", _T15, "好的,今天15点返航"),  # clock time
    ("下午3点回来", "return_home", _T15, "好的,今天15点返航"),  # the other B08 word
    ("10分钟后返航", "return_home", _NOW + 600, "好的,10分钟后返航"),  # delay
    ("下午3点去充电", "charge", _T15, "好的,今天15点回充电桩"),  # nearest dock
    ("10分钟后回充电桩", "charge", _NOW + 600, "好的,10分钟后回充电桩"),
])
def test_timed_return_and_charge_are_scheduled(text, ttype, epoch, said):  # case body
    """Until 99 U143 the time was dropped and the robot left AT ONCE while
    saying "好的,返航". MUTATION: drop the new parse kinds -> not_before_ts
    absent -> red; untimed ack -> red."""
    d, frames = _turn(text, items=_ITEMS + [_HOME])  # home defined
    task = frames["cmd/task"]["task"]  # 11 S7.2
    assert task["type"] == ttype and task["not_before_ts"] == epoch  # scheduled
    assert "waypoint_id" not in task["params"]  # charge: the nearest dock, P3's call at the time
    assert d.tts_text == said  # restated with the time


@pytest.mark.parametrize("text", ["返航", "回充电桩"])
def test_untimed_return_and_charge_still_go_now(text):  # case body
    """MUTATION: route every return / charge through the timed kinds ->
    resolve_epoch on no time -> red."""
    d, frames = _turn(text, items=_ITEMS + [_HOME])  # home defined
    assert "not_before_ts" not in frames["cmd/task"]["task"]  # at once, as before


@pytest.mark.parametrize("text,said", [
    ("凌晨1点返航", "这个时间已经过了, 请换个时间"),  # 01:00 already passed at 10:00
    ("25小时后去充电", "定时只能在一天以内, 或一周内某天的某个时刻"),  # over a day
])
def test_timed_return_refusals(text, said):  # case body
    """The same bounds as a timed goto (16 S8.0.4). MUTATION: skip the
    resolve refusal -> a frame is sent -> red."""
    d, frames = _turn(text, items=_ITEMS + [_HOME])  # one turn
    assert not frames and d.tts_text == said  # said why, sent nothing


def test_timed_return_without_a_home_point_is_refused():  # case body
    """The B08 home check holds for a timed return too. MUTATION: bypass
    _build_task_command -> a return_home is scheduled with nowhere to go."""
    d, frames = _turn("下午3点返航")  # no 返航终点站
    assert not frames and "返航终点站" in d.tts_text  # refused at once, not at 15:00



# -- 99 U130: G48 / G49 answered at once from the geometry ---------------------
_LAT0, _LON0 = 31.0, 121.0  # a local origin
_DLAT = 1.0 / 111195.0  # ~1 m of latitude
_DLON = 1.0 / (111195.0 * 0.8571673)  # ~1 m of longitude at 31 deg N


def _p(x, y):  # local metres -> {lat, lon}
    return {"lat": _LAT0 + y * _DLAT, "lon": _LON0 + x * _DLON}  # small-area approximation


_OBJECTS = {  # state/geo/objects: one 100 m line east, three points
    "routes": [{"geo_id": "r-1", "name": "1号路径", "loop_mode": "oneway", "state": "active",
                "points": [_p(0, 0), _p(100, 0)]}],  # a straight route
    "waypoints": [dict(_p(80, 3), geo_id="w-b", name="B点", state="active"),  # 3 m off, at x=80
                  dict(_p(20, 4.9), geo_id="w-a", name="A点", state="active"),  # 4.9 m off, at x=20
                  dict(_p(50, 9), geo_id="w-c", name="C点", state="active")],  # 9 m off: not on it
}


def test_route_points_lists_members_in_route_order():  # case body
    """A (x=20) before B (x=80); C at 9 m is not on it. MUTATION: no distance
    gate -> C listed; insertion order instead of along-route -> B before A."""
    d, frames = _turn("1号路径上有哪些航点", objects=_OBJECTS, member=5.0)  # G48
    assert not frames and d.intent_id == "G48"  # a reply, nothing sent
    assert d.tts_text == "1号路径上有2个航点: A点,B点"  # members, in order


def test_point_routes_and_none():  # case body
    """MUTATION: compare with the vertices only -> B (between them) is not on
    the route -> red."""
    assert _turn("B点在哪条路径上", objects=_OBJECTS, member=5.0)[0].tts_text == "B点在1号路径上"  # member
    assert _turn("C点在哪条路径上", objects=_OBJECTS, member=5.0)[0].tts_text == "C点不在任何路径上"  # not


def test_membership_without_geometry_says_so():  # case body
    """No state/geo/objects (or no configured distance) -> honest reply, not a
    guess. MUTATION: answer "no points" instead -> red."""
    assert "还没有加载好" in _turn("1号路径上有哪些航点", objects=None, member=5.0)[0].tts_text  # no data
    assert "还没有加载好" in _turn("1号路径上有哪些航点", objects=_OBJECTS, member=None)[0].tts_text  # no distance



# -- 99 U177 (user ruling 2026-11-12): a delete matches the whole name, any type --
_U177 = [
    {"geo_id": "w-4", "type": "waypoint", "name": "4号路径起点", "state": "active", "rev": 1},  # the robot's two
    {"geo_id": "w-4_1", "type": "waypoint", "name": "4号路径1点", "state": "active", "rev": 2},  # route-named points
    {"geo_id": "r-5", "type": "route", "name": "5号路径", "state": "active", "rev": 4},  # a live route
    {"geo_id": "r-4", "type": "route", "name": "4号路径", "state": "deleted", "rev": 9},  # a tombstone
    {"geo_id": "f-1", "type": "fence", "name": "东门围栏", "state": "draft", "rev": 1},  # a fence
    {"geo_id": "w-twin", "type": "waypoint", "name": "西门", "state": "active", "rev": 1},  # one name
    {"geo_id": "r-twin", "type": "route", "name": "西门", "state": "active", "rev": 1},  # in two tables
]


@pytest.mark.parametrize("text,gtype,gid", [
    ("删除4号路径1点", "waypoint", "w-4_1"),  # measured 2026-11-12: "没找到你说的那个对象"
    ("删除4号路径起点", "waypoint", "w-4"),  # the other one
    ("删除四号路径一点", "waypoint", "w-4_1"),  # Chinese numerals == digits, everywhere in the name
    ("删除5号路径", "route", "r-5"),  # a route, by the same rule
    ("把东门围栏删掉", "fence", "f-1"),  # a fence, by the same rule
])
def test_a_delete_matches_the_whole_name_whatever_the_object(text, gtype, gid):
    """MUTATION: guess the type from the name (the pre-U177 _keypoint rule) ->
    the route-named points go to the LLM fake, which raises -> red; compare
    with canonical_name only -> "四号路径一点" not found -> red."""
    d, frames = _turn(text, items=_U177)  # the robot's catalogue shape
    geo = frames["cmd/geo"]  # 11 S7.9 command
    assert d.kind == "dispatch" and geo["action"] == "delete"  # sent at once (U147)
    assert (geo["type"], geo["geo_id"]) == (gtype, gid)  # the object the name names


@pytest.mark.parametrize("text,said", [
    ("删除4号路径", "没找到叫4号路径的航点, 路径或围栏, 没有删除"),  # only a tombstone has it
    ("删除西门", "有2个对象都叫西门: 航点, 路径. 没有删除, 请先把其中一个改名"),  # two tables
])
def test_no_match_or_two_matches_deletes_nothing(text, said):
    """MUTATION: delete the first of two hits -> a cmd/geo frame -> red;
    count tombstones -> 4号路径 found -> red."""
    d, frames = _turn(text, items=_U177)  # one turn
    assert "cmd/geo" not in frames and d.tts_text == said  # said, nothing sent
