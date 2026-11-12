"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_schedule_cancel.py
Brief: 99 U144 -- voice cancel / delete of timed tasks, and one timed task per minute (P4 half)

Description:
The user's rulings of 2026-11-06: a timed task must be cancellable and
deletable by voice ("取消任务" only ever reached the current / latest
suspended task), and a second timed task at the same time is refused with a
spoken reason. The clock is fixed at 10:00 site time; two tasks are
scheduled at 15:00 (goto 1号测试点) and 15:30 (patrol 1号路径). Each case
names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import os  # this tree's configs
from datetime import datetime  # fixed clock
from zoneinfo import ZoneInfo  # site zone

import pytest  # parametrize
import yaml  # content files

from xbrain.p4_agent.registry.intents import load_intent_registry  # registry
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # as wired
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # a dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets
from xbrain.p4_agent.slots.ptp_slots import parse_ptp  # the rule layer
from xbrain.p4_agent.state.schedule_view import clash, scheduled_items, spoken_epoch  # the view

pytestmark = pytest.mark.no_device  # pure logic

_TZ = "Asia/Shanghai"  # site zone
_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree
_NOW = datetime(2026, 11, 6, 10, 0, tzinfo=ZoneInfo(_TZ)).timestamp()  # 10:00
_T15 = datetime(2026, 11, 6, 15, 0, tzinfo=ZoneInfo(_TZ)).timestamp()  # 15:00 today
_ITEMS = [{"type": "waypoint", "geo_id": "w-1", "name": "1号测试点", "state": "active"},  # the catalogue (names P4 speaks)
          {"type": "route", "geo_id": "r-1", "name": "1号路径", "state": "active"},
          {"type": "waypoint", "geo_id": "w-home", "name": "返航终点站", "state": "active"}]
_A = {"task_id": "t-a", "type": "goto", "state": "scheduled", "priority": 5,  # 15:00 goto
      "not_before_ts": _T15, "target_waypoint_id": "w-1"}  # 15:00 goto
_B = {"task_id": "t-b", "type": "patrol", "state": "scheduled", "priority": 5,  # 15:30 patrol
      "not_before_ts": _T15 + 1800, "target_route_id": "r-1"}  # 15:30 patrol


def _orch(queue):  # an orchestrator over one queue
    """An orchestrator over this queue, the catalogue and the fixed clock."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    state = {"state/geo/manifest": {"items": _ITEMS},
             "state/task": {"current": None, "queue": queue, "suspended": []}}  # 11 S4.4
    return build_orchestrator(reg, cc, state_fn=lambda: state,
                              wall_now_fn=lambda: _NOW, site_timezone=_TZ)  # as wired


def _say(orch, session, text):  # one turn, spoken text and frame
    """One mic turn -> (spoken, cmd/task frame or None)."""
    d = orch.handle_turn(text, session, 1000, channel="mic_local")  # a mic turn
    frames = dict(decision_to_publishes(d))  # key -> payload
    return d.tts_text, frames.get("cmd/task")  # what the case checks


# -- the view -------------------------------------------------------------------------
def test_items_are_named_and_timed_earliest_first():  # names and order
    """MUTATION: name by id -> "去w-1" -> red; no sort -> red."""
    items = scheduled_items({"queue": [_B, _A]}, _ITEMS, _NOW, _TZ)  # out of order
    assert [(i["when"], i["label"]) for i in items] == [
        ("今天15点", "去1号测试点"), ("今天15点30分", "沿1号路径巡逻")]  # spoken, sorted


def test_clash_is_by_minute():  # the slot is a minute
    """MUTATION: compare seconds -> 15:00:40 misses -> red."""
    items = scheduled_items({"queue": [_A]}, _ITEMS, _NOW, _TZ)  # one at 15:00
    assert clash(items, _T15 + 40) is not None and clash(items, _T15 + 60) is None


def test_spoken_epoch_days():  # day words
    """Tomorrow is said as such. MUTATION: always 今天 -> red."""
    assert spoken_epoch(_T15 + 86400, _NOW, _TZ) == "明天15点"  # next day


# -- the parse ------------------------------------------------------------------------
@pytest.mark.parametrize("text,when,every", [
    ("取消定时任务", False, False), ("删除定时任务", False, False),  # bare
    ("取消下午3点的定时任务", True, False), ("把下午3点的定时任务删掉", True, False),  # by time
    ("取消下午3点的任务", True, False),  # time without 定时
    ("删除今天下午1点的沿1号路径巡逻任务", True, False),  # 99 U145: time + content
    ("取消所有定时任务", False, True), ("删除全部定时任务", False, True),  # all
])
def test_parse_cancel_scheduled(text, when, every):  # one parse per sentence
    """MUTATION: try the delete rule first -> "删除定时任务" is a keypoint
    delete -> red."""
    got = parse_ptp(text)  # one parse
    assert got is not None and got.kind == "cancel_scheduled"  # ours
    assert (got.when is not None, got.every) == (when, every)  # the selector


@pytest.mark.parametrize("text", ["取消任务", "删除1号测试点", "取消巡逻的任务"])
def test_other_cancels_are_not_ours(text):  # B07 / F12 keep theirs
    """MUTATION: match any 取消 -> plain cancel / waypoint delete hijacked -> red."""
    got = parse_ptp(text)  # one parse
    assert got is None or got.kind != "cancel_scheduled"  # left to B07 / F12


# -- the flows (99 U145: one question, one answer -- never a confirm or a follow-up) --
def test_none_scheduled_says_so():  # nothing scheduled
    """MUTATION: fall through to plain 取消任务 -> red."""
    said, frame = _say(_orch([]), OrchestratorSession(), "取消定时任务")
    assert said == "现在没有定时任务" and frame is None  # nothing sent


def test_the_only_one_is_cancelled_at_once_by_id():  # one scheduled task
    """MUTATION: open a confirm -> await_confirm, no frame -> red; cancel
    the "current" task instead of the picked id -> no task_id t-a -> red."""
    said, frame = _say(_orch([_A]), OrchestratorSession(), "删除定时任务")  # one turn
    assert said == "好的,已取消今天15点去1号测试点的定时任务"  # done, said what
    assert frame["action"] == "cancel" and frame["task_id"] == "t-a"  # by id, this turn


@pytest.mark.parametrize("text,tid,said", [
    ("删除今天下午3点半的沿1号路径巡逻任务", "t-b", "好的,已取消今天15点30分沿1号路径巡逻的定时任务"),  # the user's example form
    ("取消下午3点的定时任务", "t-a", "好的,已取消今天15点去1号测试点的定时任务"),  # time only
    ("取消沿一号路径巡逻的定时任务", "t-b", "好的,已取消今天15点30分沿1号路径巡逻的定时任务"),  # content only, 一号 == 1号
    ("把下午3点的定时任务删掉", "t-a", "好的,已取消今天15点去1号测试点的定时任务"),  # back word order
])
def test_time_or_content_picks_one_and_cancels_it(text, tid, said):  # two scheduled
    """MUTATION: ignore the content -> the patrol sentence without a time is
    ambiguous -> red; ignore the time -> red."""
    got, frame = _say(_orch([_A, _B]), OrchestratorSession(), text)  # one turn
    assert got == said and frame["task_id"] == tid  # that one, at once


def test_several_and_nothing_names_one_cancels_none_and_says_how():  # ambiguous
    """No follow-up question (U145) and no guess: nothing is cancelled and the
    answer states the tasks with an example. MUTATION: cancel the first -> a
    frame -> red; ask "取消几点的" -> red."""
    said, frame = _say(_orch([_A, _B]), OrchestratorSession(), "取消定时任务")
    assert said == ("现在有2个定时任务: 今天15点去1号测试点, 今天15点30分沿1号路径巡逻. 没有取消. "
                    "要取消其中一个, 请带上时间说, 例如: 取消今天15点的定时任务") and frame is None


@pytest.mark.parametrize("text,said", [
    ("取消下午4点的定时任务", "没有找到今天16点的定时任务"),  # nothing at that time
    ("取消下午3点的沿1号路径巡逻任务", "没有找到今天15点沿1号路径巡逻的定时任务"),  # time and content disagree
])
def test_nothing_matching_says_not_found(text, said):  # answered, nothing sent
    """MUTATION: fall back to the first item -> red."""
    got, frame = _say(_orch([_A, _B]), OrchestratorSession(), text)
    assert got == said and frame is None


def test_all_clears_the_scheduled_state_at_once():  # every scheduled task
    """The turn says no count (U147 race fix: the count is P3's ack).
    MUTATION: clear_queue without the state filter -> ready tasks too -> red;
    count from the snapshot -> "全部2个" -> red."""
    said, frame = _say(_orch([_A, _B]), OrchestratorSession(), "取消所有定时任务")
    assert said == "好的,取消全部定时任务"  # no number predicted
    assert frame["action"] == "clear_queue" and frame["filter"] == {"state": "scheduled"}


@pytest.mark.parametrize("queue", [[], [_A]])  # snapshot shows none / only one
def test_all_is_sent_even_on_a_stale_snapshot(queue):  # the race itself
    """A timed task submitted a moment ago may not be in state/task yet; "所有"
    still goes to P3, which clears what is really there. MUTATION: answer
    "现在没有定时任务" from the snapshot / cancel the one by id -> red."""
    said, frame = _say(_orch(queue), OrchestratorSession(), "取消所有定时任务")
    assert frame["action"] == "clear_queue" and frame["filter"] == {"state": "scheduled"}
    assert said == "好的,取消全部定时任务"  # the count follows from P3


def test_plain_cancel_task_is_not_a_scheduled_cancel():  # B07 itself
    """Plain 取消任务 never reaches a scheduled task: with nothing running it
    says so, in one answer (99 U147: no confirm round either). MUTATION: read
    取消任务 as a scheduled cancel -> t-a cancelled -> red."""
    said, frame = _say(_orch([_A]), OrchestratorSession(), "取消任务")  # nothing running
    assert frame is None and said and "确认" not in said  # answered, nothing sent


@pytest.mark.parametrize("text,said", [
    ("下午3点去1号测试点", "今天15点已经有一个定时任务: 去1号测试点. 同一时间只能排一个任务, 请换个时间"),
    ("下午3点返航", "今天15点已经有一个定时任务: 去1号测试点. 同一时间只能排一个任务, 请换个时间"),
    ("下午3点半沿1号路径巡逻", "今天15点30分已经有一个定时任务: 沿1号路径巡逻. 同一时间只能排一个任务, 请换个时间"),
])
def test_a_second_task_in_the_same_minute_is_refused(text, said):  # same minute, any kind
    """MUTATION: drop the pre-check -> a submit goes out and P3 refuses after
    "好的" -> red."""
    got, frame = _say(_orch([_A, _B]), OrchestratorSession(), text)  # same minute
    assert got == said and frame is None  # refused with the reason, nothing sent


def test_the_next_minute_is_free():  # 15:01 is another slot
    """MUTATION: compare hours -> 15:01 refused -> red."""
    _said, frame = _say(_orch([_A]), OrchestratorSession(), "下午3点01分去1号测试点")
    assert frame is not None and frame["task"]["not_before_ts"] == _T15 + 60  # scheduled



# -- 99 U174 (user ruling 2026-11-12): the whole queue by voice / text -----------
@pytest.mark.parametrize("text", ["取消所有任务", "取消全部任务", "删除所有的任务",
                                  "取消排队中的任务", "取消所有排队的任务",
                                  "清空任务队列", "清空所有任务", "把所有任务取消掉"])
def test_parse_cancel_queue(text):
    """MUTATION: drop _QUEUE_WORDS / _CLEAR_QUEUE -> not parsed -> red."""
    got = parse_ptp(text)
    assert got is not None and got.kind == "cancel_queue"


@pytest.mark.parametrize("text", ["取消任务", "取消巡逻的任务", "取消所有定时任务"])
def test_plain_and_scheduled_cancels_are_not_the_queue(text):
    """MUTATION: read every 取消...任务 as the queue -> B07 / U144 hijacked -> red."""
    got = parse_ptp(text)
    assert got is None or got.kind != "cancel_queue"


def test_cancel_all_tasks_clears_the_whole_queue():
    """MUTATION: keep the scheduled filter -> ready tasks stay -> red."""
    ready = dict(_A, task_id="t-r", state="ready", not_before_ts=None)
    said, frame = _say(_orch([_A, ready]), OrchestratorSession(), "取消所有任务")
    assert frame["action"] == "clear_queue" and "filter" not in frame
    assert said == "好的,取消全部排队中的任务"  # the count follows from P3's ack


def test_cancel_all_tasks_says_the_running_one_is_kept():
    """S7.2: clear_queue leaves running alone -- said, not hidden.
    MUTATION: drop the running branch -> the plain sentence -> red."""
    orch = _orch([_A])
    state = orch._state_fn()  # the snapshot the orchestrator reads
    state["state/task"]["current"] = {"task_id": "t-run", "type": "goto", "state": "running"}
    said, frame = _say(orch, OrchestratorSession(), "取消所有任务")
    assert frame["action"] == "clear_queue"
    assert said == "好的,取消全部排队中的任务. 正在执行的任务不受影响, 要停下请说取消任务"
