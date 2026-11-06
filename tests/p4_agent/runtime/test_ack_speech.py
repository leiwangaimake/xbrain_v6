"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ack_speech.py
Brief: executors' answers spoken -- capture results, and why a task / geo / motion command was refused (11 CMD-30 / AP-1)

Description:
99 U109. Before, the operator heard "收到" and nothing else, whether P3 stored
the point or refused it. The speaker remembers this process's mark_once by
cmd_id (only when the turn spoke) and voices the ack once, on the same TTS
source. Each case names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import json  # frames are JSON bytes

import pytest  # marker

from xbrain.p4_agent.runtime.ack_speech import (  # unit under test
    ACK_KEYS,  # the answer keys
    TASK_EVENT_KEY,  # 16 S8.3C
    AckSpeaker,  # unit under test
)

pytestmark = pytest.mark.no_device  # pure


def _b(obj):  # bytes helper
    """One frame's bytes."""
    return json.dumps(obj, ensure_ascii=False).encode("utf-8")  # wire form


def _turn(cmd_id="c-1", name="1号测试点", source="tts_local", spoke=True):  # one F06 turn
    """The frames one F06 turn publishes: the command, then the reply."""
    pairs = [("cmd/teach", _b({"cmd_id": cmd_id, "action": "mark_once",
                               "mark_once": {"kind": "waypoint",
                                             "name": name}}))]  # the capture
    if spoke:
        pairs.append(("cmd/audio/speak", _b({"schema": "p4_speak_v1",
                                             "text": "好的,记录这个点",
                                             "source": source})))  # the reply
    return pairs  # in publish order


def _said(out):  # unpack a speak
    """(key, text, source) of what the speaker would publish."""
    key, data = out  # one pair
    body = json.loads(data)  # p4_speak_v1
    return key, body["text"], body["source"]  # what the case checks


def test_accepted_is_restated_from_applied():  # accepted capture
    """AP-1: the applied name, on the turn's own source. MUTATION: say the
    requested name only, or speak on a fixed source."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn(source="tts_cloud"))  # a cloud turn
    out = sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted", "code": "OK",
                        "detail": {"applied": {"name": "1号测试点"},
                                   "warn": []}}))  # P3 stored it
    assert _said(out) == ("cmd/audio/speak", "已记录1号测试点", "tts_cloud")  # restated


def test_no_heading_is_said():  # warn
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn())  # one capture
    out = sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted", "code": "OK",
                        "detail": {"applied": {"name": "1号测试点"},
                                   "warn": ["no_heading"]}}))  # heading invalid
    assert _said(out)[1] == "已记录1号测试点, 没有记下朝向"  # told


@pytest.mark.parametrize("code,detail,text", [
    ("E_TEACH_QUALITY", {"reason": "rtk_float"},
     "1号测试点没有记录, 定位精度不够, 需要RTK固定解"),  # indoors
    ("E_TEACH_QUALITY", {"item": "snap_age"},
     "1号测试点没有记录, 位置快照已过期, 请再说一遍"),  # 18 PS-6
    ("E_NAME_CONFLICT", {"name": "1号测试点"},
     "1号测试点这个名字已经有了, 请换个名字"),  # duplicate
    ("E_TEACH_STATE", {"reason": "x"}, "1号测试点没有记录成功"),  # anything else
])
def test_a_refusal_says_why(code, detail, text):  # case body
    """MUTATION: one generic sentence -- the operator cannot act on it."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn())  # one capture
    out = sp.on_ack(_b({"cmd_id": "c-1", "result": "rejected", "code": code,
                        "detail": detail}))  # P3 refused
    assert _said(out)[1] == text  # the reason


def test_once_only_and_only_our_own():  # match rules
    """MUTATION: keep the entry and a duplicate ack speaks twice; answer every
    ack and another initiator's capture is voiced here."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn())  # c-1 only
    ok = _b({"cmd_id": "c-1", "result": "accepted", "code": "OK",
             "detail": {"applied": {"name": "1号测试点"}}})  # the ack
    assert sp.on_ack(ok) is not None  # first: spoken
    assert sp.on_ack(ok) is None  # again: already answered
    assert sp.on_ack(_b({"cmd_id": "c-9", "result": "accepted"})) is None  # not ours
    sp.note_turn(_turn(cmd_id="c-2"))  # another capture
    assert sp.on_ack(_b({"cmd_id": "c-2", "result": "duplicate"})) is None  # S12A.4 replay


def test_a_silent_turn_stays_silent():  # silent turn
    """11 S8.7.5 require_tts_reply=false: no reply, so no result either.
    MUTATION: remember every capture and a typed silent command speaks."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn(spoke=False))  # no speak frame
    assert sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted"})) is None  # nothing


def _sent(key, body, source="tts_local"):  # one command turn
    """One turn: a command frame on key, then the turn's reply."""
    return [(key, _b(body)),
            ("cmd/audio/speak", _b({"schema": "p4_speak_v1", "text": "好的",
                                    "source": source}))]  # in publish order


@pytest.mark.parametrize("action,code,text", [
    ("resume", "E_TASK_STATE", "这个任务现在的状态不能继续"),  # the live 11-04 case
    ("pause", "E_NOT_FOUND", "没找到这个任务, 没有暂停"),  # gone meanwhile
    ("cancel", "E_SCHEMA", "任务没有取消成功"),  # anything else
])
def test_a_refused_task_command_says_why(action, code, text):  # case body
    """99 U111: "继续" was refused by P3 and the robot said "收到".
    MUTATION: stop listening on cmd/task/ack."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/task", {"cmd_id": "c-1", "action": action}))
    out = sp.on_ack(_b({"cmd_id": "c-1", "result": "rejected", "code": code}))
    assert _said(out)[1] == text  # the reason


def test_no_completed_patrol_has_its_own_sentence():  # 99 U131 B04
    """P3 refuses "再巡一圈" with E_NOT_FOUND + reason no_completed_patrol.
    MUTATION: key on the code only -> "没找到这个任务, 没有执行" -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/task", {"cmd_id": "c-1", "action": "submit"}))  # a B04 submit
    out = sp.on_ack(_b({"cmd_id": "c-1", "result": "rejected", "code": "E_NOT_FOUND",  # refused
                        "detail": {"reason": "no_completed_patrol"}}))  # by P3
    assert _said(out)[1] == "还没有巡逻完成过的路径, 请说沿哪条路径巡逻"  # the reason


def test_an_accepted_task_command_adds_nothing():  # accepted task
    """The turn already said what it does. MUTATION: speak every ack."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/task", {"cmd_id": "c-1", "action": "pause"}))
    assert sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted"})) is None  # silent


@pytest.mark.parametrize("code,reason,text", [
    ("E_BUSY", "voice_hold", "现在处于原地待命, 请先说继续"),  # 99 U111
    ("E_BUSY", "mode_hold", "喊话或报警进行中, 现在不能动"),  # B / D
    ("E_LOCKED", None, "底盘锁定中, 现在不能动"),  # G-9
    ("E_NO_HEADING", None, "现在没有可靠的航向, 不能转向"),  # G-6
])
def test_a_refused_motion_says_why(code, reason, text):  # case body
    """The motion frame is enveloped; its cmd_id is inside data."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/motion/intent",
                       {"v": 1, "data": {"cmd_id": "mi-1", "intent": "move_forward"}}))
    detail = {"reason": reason} if reason else {}  # E_BUSY carries a reason
    out = sp.on_ack(_b({"cmd_id": "mi-1", "result": "rejected", "code": code,
                        "detail": detail}))
    assert _said(out)[1] == text  # the reason


def test_a_refused_geo_command_says_why():  # geo
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/geo", {"cmd_id": "g-1", "action": "delete"}))
    out = sp.on_ack(_b({"cmd_id": "g-1", "result": "rejected",
                        "code": "E_GEO_CONFLICT"}))
    assert _said(out)[1] == "这个对象刚被别处修改过, 请再说一遍"  # the reason


@pytest.mark.parametrize("code,detail,text", [  # 99 U133 start refusals
    ("E_NAME_CONFLICT", {}, "1号路径这个名字已经有了, 请换个名字再录制"),  # taken meanwhile
    ("E_TEACH_QUALITY", {"reason": "fix_quality"}, "现在不能开始录制: 定位不是RTK固定解"),  # check 3
    ("E_UNHEALTHY", {"reason": "no_nonvoice_estop"}, "现在不能开始录制: 没有可用的非语音急停通道"),
    ("E_BUSY", {"reason": "task_running"}, "有任务正在执行, 现在不能开始录制"),  # check 2
])
def test_a_refused_recording_start_says_why(code, detail, text):  # start
    """Until U133 a refused start was silent after "好的,开始录制X".
    MUTATION: skip start in note_turn -> nothing said -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/teach", {"cmd_id": "t-1", "action": "start",
                                     "start": {"kind": "route", "name": "1号路径"}}))
    out = sp.on_ack(_b({"cmd_id": "t-1", "result": "rejected", "code": code, "detail": detail}))
    assert _said(out)[1] == text  # the reason


def test_an_accepted_recording_start_adds_nothing():  # start accepted
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/teach", {"cmd_id": "t-1", "action": "start",
                                     "start": {"kind": "route", "name": "1号路径"}}))
    assert sp.on_ack(_b({"cmd_id": "t-1", "result": "accepted", "code": "OK"})) is None  # turn said it


@pytest.mark.parametrize("ack,text", [  # 99 U133 finish = save
    ({"result": "accepted", "code": "OK", "detail": {"saved": True, "kind": "route",
      "applied": {"name": "1号路径", "point_count": 47, "total_len_m": 320.4}}},
     "已保存1号路径, 共47个点, 长320米"),
    ({"result": "accepted", "code": "OK", "detail": {"saved": True, "kind": "fence",
      "applied": {"name": "东门围栏", "point_count": 6, "total_len_m": 80.0}}},
     "已保存东门围栏, 共6个顶点. 围栏已保存但未启用, 需要时请说启用东门围栏"),
    ({"result": "rejected", "code": "E_TEACH_GEOMETRY", "detail": {
      "saved": False, "close_reason": "invalid_geometry", "name": "2号路径",
      "issues": [{"code": "too_few_points"}]}},
     "录制已退出, 2号路径没有保存: 录到的点不足两个"),
])
def test_a_recording_exit_says_what_was_saved(ack, text):  # finish
    """MUTATION: say nothing for finish -> the operator never hears the result -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/teach", {"cmd_id": "t-2", "action": "finish", "session_id": "ts-1"}))
    out = sp.on_ack(_b({"cmd_id": "t-2", **ack}))
    assert _said(out)[1] == text  # the result


def test_a_discard_says_what_was_thrown_away():  # 99 U137
    """No confirmation turn any more, so the result must be said.
    MUTATION: skip discard in note_turn -> silent -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/teach", {"cmd_id": "t-3", "action": "discard", "session_id": "ts-1"}))
    out = sp.on_ack(_b({"cmd_id": "t-3", "result": "accepted", "code": "OK",
                        "detail": {"name": "2号路径", "discarded_points": 12}}))
    assert _said(out)[1] == "已取消录制2号路径, 录到的12个点已丢弃"  # the result


def test_session_actions_are_not_voiced_here():  # sessions
    sp = AckSpeaker()  # fresh
    sp.note_turn([("cmd/teach", _b({"cmd_id": "c-1", "action": "start"})),
                  ("cmd/audio/speak", _b({"source": "tts_local"}))])  # a start
    assert sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted"})) is None  # state/teach's job


def test_outstanding_captures_are_bounded():  # bound
    """MUTATION: an unbounded dict grows forever when P3 is down."""
    sp = AckSpeaker()  # fresh
    for i in range(40):  # more than the bound
        sp.note_turn(_turn(cmd_id="c-%d" % i))  # one each
    assert sp.on_ack(_b({"cmd_id": "c-0", "result": "accepted"})) is None  # evicted
    assert sp.on_ack(_b({"cmd_id": "c-39", "result": "accepted"})) is not None  # kept
    assert "cmd/teach/ack" in ACK_KEYS  # 11 S2.2.4 key


# -- 16 S8.3C task failure speech (99 U140) -----------------------------------------
def _submitted(sp, task, task_id="t-1", source="tts_local", spoke=True):  # one accepted submit
    """A voice submit of task, accepted by P3 under task_id."""
    pairs = _sent("cmd/task", {"cmd_id": "c-1", "action": "submit", "task": task}, source)
    sp.note_turn(pairs if spoke else pairs[:1])  # the turn
    assert sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted", "code": "OK",
                         "detail": {"task_id": task_id}})) is None  # accepted: silent


def _event(kind, task_id="t-1", error=None):  # one 11 S6.2 task event
    """event/{sev}/task as P3 publishes it."""
    detail = {"kind": kind, "task_id": task_id, "state": "failed"}
    if error is not None:
        detail["error"] = error  # 99 U140 terminal facts
    return _b({"eid": "task-x-1", "title": "task", "detail": detail, "src": "p3_task"})


_GOTO = {"type": "goto", "params": {"slots": {"waypoint": "1号测试点"}, "waypoint_id": "w-a"}}


@pytest.mark.parametrize("task,kind,error,text", [
    (_GOTO, "failed", {"code": "E_GEO_INCOMPLETE", "cause": "no_position"},
     "去1号测试点没有完成: 现在拿不到定位"),  # the 11-05 runtime test case
    ({"type": "patrol", "route_id": "r-1", "params": {"slots": {"route": "1号路径"}}},
     "failed", {"code": "E_GEO_INCOMPLETE", "cause": "route_unconfirmed"},
     "沿1号路径巡逻没有完成: 运动模块没有确认路线"),  # RA-3
    ({"type": "patrol", "params": {"slots": {}}}, "failed",
     {"code": "E_NOT_FOUND", "cause": "target_missing"}, "再巡逻没有完成: 找不到目标点或路径"),  # B04
    ({"type": "charge", "params": {"slots": {}}}, "failed",
     {"code": "E_SCHEMA", "cause": "plan_refused"}, "去充电站没有完成: 路线生成失败"),  # B09
    ({"type": "return_home", "params": {"slots": {}}}, "failed",
     {"fail_reason": "max_deviation"}, "返航没有完成: 途中导航失败"),  # P1 on the way
    (_GOTO, "rejected", None, "去1号测试点没有完成: 任务检查没有通过"),  # admission
    (_GOTO, "failed", {"code": "E_INTERNAL", "detail": "x"}, "去1号测试点没有完成"),  # unknown
])
def test_a_failed_task_is_said_with_why(task, kind, error, text):  # FS-2..FS-4
    """The 2026-11-05 runtime test: a goto failed with E_GEO_INCOMPLETE and the
    robot had only said "好的,去1号测试点". MUTATION: ignore task events ->
    None -> red; MUTATION: drop error.cause -> no reason -> red."""
    sp = AckSpeaker()  # fresh
    _submitted(sp, task)  # accepted
    key, said, source = _said(sp.on_task_event(_event(kind, error=error)))  # the outcome
    assert (key, said, source) == ("cmd/audio/speak", text, "tts_local")  # on the turn's source


def test_completed_and_cancelled_are_silent_and_forgotten():  # FS-2
    """MUTATION: voice every terminal kind -> a sentence for completed -> red;
    MUTATION: keep the entry after completed -> a later failed is voiced -> red."""
    sp = AckSpeaker()  # fresh
    _submitted(sp, _GOTO)  # accepted
    assert sp.on_task_event(_event("completed")) is None  # nothing to say
    assert sp.on_task_event(_event("failed", error={"cause": "no_position"})) is None  # forgotten


def test_only_our_own_voiced_submits_are_said():  # FS-1
    """MUTATION: voice any task's failure -> the HMI's task speaks -> red;
    MUTATION: remember a submit the turn did not speak for -> red."""
    sp = AckSpeaker()  # fresh
    assert sp.on_task_event(_event("failed", "t-hmi", {"cause": "no_position"})) is None  # not ours
    _submitted(sp, _GOTO, task_id="t-2", spoke=False)  # silent turn
    assert sp.on_task_event(_event("failed", "t-2", {"cause": "no_position"})) is None  # stays silent
    sp.note_turn(_sent("cmd/task", {"cmd_id": "c-9", "action": "submit", "task": _GOTO}))
    sp.on_ack(_b({"cmd_id": "c-9", "result": "rejected", "code": "E_DEGRADED",
                  "detail": {"task_id": "t-3"}}))  # refused: said by on_ack, not remembered
    assert sp.on_task_event(_event("failed", "t-3", {"cause": "no_position"})) is None


def test_started_is_not_an_outcome_and_the_key_covers_every_severity():  # FS-2
    """MUTATION: pop on started -> the later failure is lost -> red;
    MUTATION: subscribe event/warn/task only -> completed (info) never forgets."""
    sp = AckSpeaker()  # fresh
    _submitted(sp, _GOTO)  # accepted
    assert sp.on_task_event(_event("started")) is None  # not an outcome
    assert _said(sp.on_task_event(_event("failed", error={"cause": "no_position"})))[1].startswith("去1号测试点")
    assert TASK_EVENT_KEY == "event/*/task"  # info and warn



# -- 99 U142: the RL-3 voice of a positioning loss ------------------------------------
def _rtk(kind, **detail):  # one 11 S3.3.4 rtk event as P3 publishes it
    """event/{alarm|info}/rtk bytes with this detail.type."""
    return _b({"sev": "alarm" if kind == "rtk_lost" else "info", "cat": "rtk",
               "detail": dict(detail, type=kind)})  # wire form


@pytest.mark.parametrize("kind,detail,text", [
    ("rtk_lost", {"action_taken": "stop_and_suspend"}, "定位失效, 已停车, 任务已暂停"),  # a task waits
    ("rtk_lost", {"action_taken": "stop_and_hold"}, "定位失效, 已停车"),  # 11 S3.3.4 RL-3 wording
    ("heading_recovered", {"task_id": "t-1"}, "定位已恢复, 要接着执行任务请说继续"),  # RL-7 hint
    ("heading_recovered", {"task_id": None}, "定位已恢复"),  # nothing to resume
])
def test_positioning_loss_and_recovery_are_spoken_locally(kind, detail, text):
    """RL-3, whoever started the task. MUTATION: say "请说继续" with no task
    waiting -> red; speak on a turn's source -> red (local alarm voice)."""
    key, said, source = _said(AckSpeaker().on_rtk_event(_rtk(kind, **detail)))  # one event
    assert (key, said, source) == ("cmd/audio/speak", text, "tts_local")  # the line, locally


def test_heading_degraded_and_junk_are_silent_and_the_key_covers_both_severities():
    """heading_degraded is rtk_driver's L1 -> L2 note, not an alarm to voice.
    MUTATION: subscribe event/alarm/rtk only -> the recovery (info) is never
    heard -> red."""
    from xbrain.p4_agent.runtime.ack_speech import RTK_EVENT_KEY  # the subscription
    sp = AckSpeaker()  # fresh
    assert sp.on_rtk_event(_rtk("heading_degraded")) is None  # not voiced
    assert sp.on_rtk_event(b"not json") is None  # malformed ignored
    assert RTK_EVENT_KEY == "event/*/rtk"  # alarm and info
