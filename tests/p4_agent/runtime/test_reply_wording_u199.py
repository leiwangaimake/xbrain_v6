"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_reply_wording_u199.py
Brief: 99 U199 -- a reply never says "好的" and then the opposite; state P4 can see is answered up front
Description:
The commx live run of 2026-11-14 afternoon (stack up, defect list re-sent)
found answers that contradict themselves or the robot:
  * "退出报警" in idle answered "好的,退出报警模式";
  * "报警模式" with the siren down: "好的,进入报警模式", then "没有成功";
  * "好的,前进2米", then "底盘锁定中, 现在不能动" -- two answers that disagree;
  * "别跟了" with no follow on: "好的,停止跟随";
  * "继续" woke a task suspended the day before without saying which;
  * "强行对时" said "not wired", "时间跳过来" said "已同步" -- one request;
  * "到那里要多久": "算不出距离".
Each case runs the real orchestrator (production configs, no LLM) or the real
AckSpeaker / time-sync answer, so a reverted line turns its case red.
"""
from __future__ import annotations  # postponed annotations

import json  # frames are JSON bytes
import os  # this tree's configs

import pytest  # parametrize, marker
import yaml  # content files

from xbrain.p3_task.state.task_state import build_task_state  # P3 state/task body
from xbrain.p3_task.state import task_state  # its SELECT columns
from xbrain.p4_agent.registry.intents import load_intent_registry  # the registry loader
from xbrain.p4_agent.runtime.ack_speech import AckSpeaker  # the executor answers
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator, decision_to_publishes  # as wired
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession  # a fresh dialog
from xbrain.p4_agent.session.chitchat import ChitchatResponder  # presets
from xbrain.p4_agent.state.mode_switch_answer import mode_precheck  # the precheck
from xbrain.p4_agent.state.chassis_lock import chassis_locked  # the lock rule
from xbrain.p4_agent.state import status_answers  # G06
from xbrain.p4_agent.state.cache import StateCache  # the live cache

pytestmark = pytest.mark.no_device  # no robot, no LLM

_CONFIGS = os.path.join(os.path.dirname(__file__), "..", "..", "..", "configs")  # this tree
_OK = {"state": "ok"}  # a healthy item
_HEALTHY = {"items": {"payload_siren": _OK, "payload_strobe": _OK}}  # deter possible
_SIREN_DOWN = {"items": {"payload_siren": {"state": "fail"}, "payload_strobe": _OK}}  # deter fails


def _no_llm(*_a, **_k):  # tier-2 fake
    raise AssertionError("tier-2 LLM called")  # every case here is rule / keyword


def _reg_cc():  # this tree's registry and chitchat presets
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:  # registry
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:  # presets
        return reg, ChitchatResponder(yaml.safe_load(fh))  # both


def _orch(mode=None, health=None, task=None):  # production orchestrator
    """An orchestrator over this tree's configs with the given live state."""
    with open(os.path.join(_CONFIGS, "intents.yaml"), encoding="utf-8") as fh:  # registry
        reg = load_intent_registry(yaml.safe_load(fh))  # this tree's registry
    with open(os.path.join(_CONFIGS, "chitchat.yaml"), encoding="utf-8") as fh:  # presets
        cc = ChitchatResponder(yaml.safe_load(fh))  # presets
    state = {"state/geo/manifest": {"items": []}, "state/teach": {"session": {"state": "idle"}},
             "state/task": task or {"current": None, "queue": [], "suspended": []},
             "state/mode": {"voice_mode": mode} if mode else None,  # P2 mode
             "health/summary": health}  # P2 health
    return build_orchestrator(reg, cc, tier2_fn=_no_llm, state_fn=lambda: state)  # as wired


def _turn(text, orch):  # one turn
    """(decision, frames other than speech) for one sentence."""
    d = orch.handle_turn(text, OrchestratorSession(), 1000, addressed=True,
                         channel="cloud", modality="text")  # one turn
    return d, {k: p for k, p in decision_to_publishes(d) if k != "cmd/audio/speak"}  # minus speech


# -- modes: what state/mode and health/summary already settle ---------------------

@pytest.mark.parametrize("text, said", [("退出报警", "现在不在报警模式, 不需要退出"),
                                        ("退出喊话", "现在不在喊话模式, 不需要退出")])
def test_leaving_a_mode_not_entered_is_answered(text, said):  # live: "好的,退出报警模式"
    """MUTATION: drop the C02 / C04 precheck -> "好的,退出..." and a frame -> red."""
    d, frames = _turn(text, _orch(mode="idle", health=_HEALTHY))  # idle
    assert d.tts_text == said and "cmd/mode" not in frames  # answered, nothing sent


def test_leaving_the_current_mode_is_sent():  # the normal case still works
    d, frames = _turn("退出报警", _orch(mode="alarm", health=_HEALTHY))  # in alarm
    assert d.tts_text == "好的,退出报警模式" and frames["cmd/mode"]["action"] == "exit_alarm"  # sent


def test_alarm_with_the_siren_down_is_refused_up_front():  # live: "好的" then "没有成功"
    """MUTATION: skip the deter check -> "好的,进入报警模式" and a frame -> red."""
    d, frames = _turn("报警模式", _orch(mode="idle", health=_SIREN_DOWN))  # siren failed
    assert d.tts_text == "现在不能进入报警模式: 警笛状态故障, 报警需要警笛和警示灯"  # why
    assert "cmd/mode" not in frames  # nothing sent


@pytest.mark.parametrize("text, mode, said", [("开始喊话", "broadcast", "现在已经是喊话模式了"),
                                              ("进入巡逻模式", "dialog_a", "现在已经是巡逻模式了")])
def test_entering_the_current_mode_is_answered(text, mode, said):
    d, frames = _turn(text, _orch(mode=mode, health=_HEALTHY))  # already there
    assert d.tts_text == said and "cmd/mode" not in frames  # answered


def test_unreadable_mode_sends_and_lets_p2_decide():
    """MUTATION: treat a missing state/mode as idle -> "现在不在报警模式" -> red."""
    assert mode_precheck("exit_alarm", {}) is None  # no guess


def test_stop_follow_wording_is_true_either_way():  # live: "好的,停止跟随" with nothing on
    d, frames = _turn("别跟了", _orch(mode="idle", health=_HEALTHY))  # one turn
    assert d.tts_text == "好的,不再跟随" and frames["cmd/mode"]["behavior"] == "normal"  # still sent


# -- motion refusal after "好的,前进2米" --------------------------------------------

def test_a_locked_chassis_refusal_says_not_done_first():  # live: two answers that disagree
    """MUTATION: drop _MOTION_NOT_DONE -> "底盘锁定中, ..." alone -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn([("cmd/motion/intent", json.dumps(
        {"v": 1, "data": {"cmd_id": "mi-1", "intent": "move_forward"}}).encode()),
        ("cmd/audio/speak", json.dumps({"schema": "p4_speak_v1", "text": "好的,前进2米",
                                         "source": "tts_cloud"}).encode())])  # the turn
    out = sp.on_ack(json.dumps({"cmd_id": "mi-1", "result": "rejected", "code": "E_LOCKED",
                                "detail": {"reason": "cmd_timeout"}}).encode())  # P1 refusal
    assert json.loads(out[1])["text"] == "没有执行: 底盘锁定中, 请先解锁"  # not done, why, what to do


# -- which task "继续 / 取消任务" acted on -----------------------------------------

def test_resume_names_the_task_it_wakes():  # live: a day-old task woke unnamed
    """MUTATION: drop _task_words -> "好的,继续最近挂起的任务" alone -> red."""
    task = {"current": None, "queue": [], "suspended": [
        {"task_id": "t-old", "type": "goto", "state": "suspended", "suspend_kind": "passive",
         "suspended_ts": 1.0, "command_text": "去10号测试点"}]}  # what P3 now publishes
    d, frames = _turn("继续", _orch(mode="idle", health=_HEALTHY, task=task))  # one turn
    assert d.tts_text == "好的,继续最近挂起的任务:去10号测试点"  # named
    assert frames["cmd/task"]["task_id"] == "t-old"  # and that is what is sent


def test_p3_publishes_the_command_text():  # the field the ack reads
    """MUTATION: drop command_text from the P3 item -> the ack has nothing to say -> red."""
    rows = [{"task_id": "t-1", "task_type": "goto", "state": "suspended", "suspend_kind": "passive",
             "command_text": "去10号测试点", "priority": 1, "submit_seq": 1}]  # one row
    # live run: the builder carried it, but read_task_state never SELECTed it
    assert "command_text" in task_state._STATE_COLUMNS  # read from task.db
    body = build_task_state(rows)  # P3 builder
    items = (body.get("suspended") or []) + [body.get("current") or {}]  # wherever it lands
    assert any(i.get("command_text") == "去10号测试点" for i in items if i)  # carried


# -- distance with no place, asked as time ------------------------------------------

def test_how_long_with_no_place_says_both():  # live: "算不出距离" to "要多久"
    d, _ = _turn("到那里要多久", _orch(mode="idle", health=_HEALTHY))  # one turn
    assert d.tts_text.startswith("没有说去哪里, 算不出距离和时间")  # both


# -- devices health/summary reports down (three-in-one unplugged) -------------------

_SPEAKER_DOWN = {"items": {"payload_speaker": {"state": "fail"}, "payload_light": {"state": "fail"},
                           "payload_siren": {"state": "fail"}, "payload_strobe": {"state": "fail"}}}


@pytest.mark.parametrize("text, said", [("音量调到50", "喇叭现在故障, 这次没有调音量"),
                                        ("开到最亮", "照明灯现在故障, 这次没有调亮度"),
                                        ("用第三种", "警示灯现在故障, 这次没有换闪法"),
                                        ("喊第二条", "喇叭现在故障, 这次没有喊话")])
def test_a_device_reported_down_is_not_told_ok(text, said):  # live: "好的,音量调到50" unplugged
    """MUTATION: drop payload_precheck -> "好的,..." and a frame -> red."""
    d, frames = _turn(text, _orch(mode="idle", health=_SPEAKER_DOWN))  # all down
    assert d.tts_text == said and frames == {}  # refused, nothing sent


def test_a_device_not_reported_down_is_sent():  # unknown is not "down"
    """MUTATION: refuse on a missing item -> a healthy-but-unreported device refused -> red."""
    d, frames = _turn("音量调到50", _orch(mode="idle", health={"items": {}}))  # no item
    assert d.tts_text == "好的,音量调到50" and "cmd/payload" in frames  # sent


def test_alarm_with_no_health_yet_is_sent():  # health/summary not received
    assert mode_precheck("enter_alarm", {"state/mode": {"voice_mode": "idle"}}) is None  # P2 decides


# -- "还有几个任务排队" with nothing running ------------------------------------------

def _task_text(task):  # G06 from a state/task body
    with open(os.path.join(_CONFIGS, "query_templates.yaml"), encoding="utf-8") as fh:  # templates
        tpl = yaml.safe_load(fh)  # production wording
    c = StateCache()  # the live cache
    c.update("state/task", task, 1000)  # sent on change: the last value
    return status_answers.task_answer(c, tpl, 1000, max_age_ms=3000).text  # the reply


def test_nothing_running_also_says_the_empty_queue():  # live: queue question, no queue answer
    """MUTATION: drop the empty-queue clause -> "当前没有执行中的任务" alone -> red."""
    assert _task_text({"current": None, "queue": [], "suspended": []}) == "当前没有执行中的任务,也没有排队的任务"


def test_nothing_running_names_what_continue_would_wake():  # "继续" acts on these
    task = {"current": None, "queue": [], "suspended": [
        {"task_id": "t-1", "suspend_kind": "passive"}, {"task_id": "t-2", "suspend_kind": "passive"},
        {"task_id": "t-3", "suspend_kind": "yielding"}]}  # yielding resumes by itself
    assert _task_text(task) == ("当前没有执行中的任务,也没有排队的任务;"
                                "有2个挂起的任务,说继续会恢复最近挂起的那个")  # passive only


# -- chassis lock and payload queries (state P4 now reads) ---------------------------

_LOCKED = {"hes_lock": False, "timeout_lock": True}  # never unlocked after boot


def test_a_locked_chassis_is_answered_up_front():  # live: "好的,前进2米" then "底盘锁定中"
    """MUTATION: drop the _MOVES lock check -> "好的,前进2米" and a frame -> red."""
    orch = _orch(mode="idle", health=_HEALTHY)  # base state
    orch._state_fn().update({"state/robot": _LOCKED})  # locked
    d, frames = _turn("前进2米", orch)  # one turn
    assert d.tts_text == "底盘锁定中, 请先解锁, 这个动作没有执行" and frames == {}  # once, nothing sent


def test_hold_is_still_sent_while_locked():  # P1 accepts hold when locked
    orch = _orch(mode="idle", health=_HEALTHY)  # base state
    orch._state_fn().update({"state/robot": _LOCKED})  # locked
    d, frames = _turn("原地待命", orch)  # one turn
    assert d.tts_text == "好的,原地待命" and "cmd/motion/intent" in frames  # sent


@pytest.mark.parametrize("robot", [None, {"hes_lock": False}, {"data": {"timeout_lock": True}}])
def test_an_unknown_lock_sends(robot):  # never default a lock
    """MUTATION: treat a missing lock field as locked -> refused -> red."""
    assert chassis_locked(robot) is None  # unknown


def test_health_answer_says_the_lock_first():  # live: "系统允许运动" while locked
    """MUTATION: drop the lock clause -> "系统允许运动..." alone -> red."""
    with open(os.path.join(_CONFIGS, "query_templates.yaml"), encoding="utf-8") as fh:  # templates
        tpl = yaml.safe_load(fh)  # production wording
    c = StateCache()  # the live cache
    c.update("health/summary", {"allow_motion": True, "speed_factor": 1.0, "items": {"mic": _OK}}, 1000)
    c.update("state/robot", _LOCKED, 1000)  # locked
    text = status_answers.health_answer(c, tpl, 1000, max_age_ms=3000).text  # G16
    assert text == "底盘锁定中,解锁前不能走;系统正常,健康检查允许运动,不限速"  # lock first


def test_payload_query_says_a_failed_device():  # live: "载荷状态暂时读不到,请稍后再问"
    """MUTATION: keep _unreadable -> "请稍后再问" -> red."""
    with open(os.path.join(_CONFIGS, "query_templates.yaml"), encoding="utf-8") as fh:  # templates
        tpl = yaml.safe_load(fh)  # production wording
    c = StateCache()  # the live cache
    c.update("health/summary", _SPEAKER_DOWN, 1000)  # all four parts failed
    assert status_answers.payload_all_answer(c, tpl, 1000, max_age_ms=3000).text == "三合一设备现在故障,读不到载荷状态"
    assert status_answers.siren_state_answer(c, tpl, 1000, max_age_ms=3000).text == "警笛现在故障,读不到警笛状态"


@pytest.mark.parametrize("text", ["云台", "电池", "云台转到30度"])
def test_typed_miss_is_not_called_out_of_scope(text):  # "云台" IS a patrol word
    """MUTATION: answer with the out_of_scope line -> "我只能处理巡逻相关的事情" -> red."""
    orch = build_orchestrator(*_reg_cc(), tier2_fn=lambda *_a, **_k: None,
                              state_fn=lambda: {})  # LLM finds nothing
    d, frames = _turn(text, orch)  # typed
    assert d.tts_text.startswith("这句话我没有理解,没有执行") and frames == {}  # true for any miss


def test_keep_patrolling_restates_the_lap():  # "接着巡" answered "收到"
    """MUTATION: drop the B04 restatement -> "收到" -> red."""
    task = {"current": None, "queue": [], "suspended": []}  # nothing running
    d, frames = _turn("接着巡", _orch(mode="idle", health=_HEALTHY, task=task))  # keyword leg
    assert d.tts_text == "好的,再巡逻1圈" and "cmd/task" in frames  # P3 takes one lap


def test_a_refused_submit_says_not_done_first():  # live: "好的,再巡逻1圈" then "定位失效..."
    """MUTATION: drop the submit prefix -> "定位失效, 现在不接受新任务" alone -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn([("cmd/task", json.dumps({"cmd_id": "c-1", "action": "submit"}).encode()),
                  ("cmd/audio/speak", json.dumps({"schema": "p4_speak_v1", "text": "好的,再巡逻1圈",
                                                   "source": "tts_cloud"}).encode())])  # the turn
    out = sp.on_ack(json.dumps({"cmd_id": "c-1", "result": "rejected", "code": "E_DEGRADED",
                                "detail": {"reason": "rtk_lost"}}).encode())  # P3 refusal
    assert json.loads(out[1])["text"] == "没有执行: 定位失效, 现在不接受新任务"  # not done, then why
