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
    assert _said(out)[1] == "还没有巡逻完成过的路径, 没有执行. 请带上路径名说, 例如: 沿1号路径巡逻"  # the reason (U147: a statement)


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


@pytest.mark.parametrize("detail,text", [  # 99 U184 enabling is told, never refused
    ({"state": "active", "warnings": [{"code": "outside_activity_area",  # outside the area
                                       "activity_area": "营区活动区域围栏"}]},  # named
     "注意: 它不在活动区域营区活动区域围栏内, 机器人到不了, 不会生效"),  # said
    ({"state": "active", "unreachable": [{"fence": "营区活动区域围栏", "role": "allow",  # left out
                                          "reason": "outside_activity_area",  # by the area
                                          "objects": ["1号路径", "返航终点站"]}],  # two names
      "robot": {"note": "outside_activity_area", "fence": "营区活动区域围栏"}},  # robot outside too
     "注意: 1号路径和返航终点站在活动区域外面, 机器人到不了. 机器人现在在活动区域外, 只能往区域里面开"),
    ({"state": "active", "unreachable": [{"fence": "北门禁止区域围栏", "role": "forbid",  # covered
                                          "reason": "inside_forbid",  # by a forbid zone
                                          "objects": ["a", "b", "c", "d", "e", "f"]}],  # six names
      "robot": {"note": "inside_forbid", "fence": "北门禁止区域围栏"}},  # robot inside it
     "注意: a, b, c, d, e等6个在禁止区域北门禁止区域围栏里面, 机器人到不了. "  # capped at five
     "机器人现在在禁止区域北门禁止区域围栏里面, 只能往外开"),  # where it may drive
])
def test_switching_a_fence_on_says_what_it_leaves_out_of_reach(detail, text):  # F15
    """The turn said "好的,启用X"; the notes follow it.
    MUTATION: accepted geo acks say nothing again -> red; the list not
    capped -> the six-name case reads all six -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/geo", {"cmd_id": "g-2", "action": "set_state"}))  # F15 turn
    out = sp.on_ack(_b({"cmd_id": "g-2", "result": "accepted", "code": "OK", "detail": detail}))  # P3 ack
    assert _said(out)[1] == text  # the notes


def test_a_fence_switched_on_with_nothing_to_tell_adds_nothing():  # F15, quiet
    """MUTATION: always speak on an accepted geo ack -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/geo", {"cmd_id": "g-3", "action": "set_state"}))  # F15 turn
    assert sp.on_ack(_b({"cmd_id": "g-3", "result": "accepted", "code": "OK",  # nothing out of reach
                         "detail": {"state": "active"}})) is None  # nothing more said


def test_a_waypoint_saved_out_of_reach_is_told():  # F06 + 99 U184
    """MUTATION: drop the notes from the stored sentence -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/teach", {"cmd_id": "t-9", "action": "mark_once",  # F06 turn
                                     "mark_once": {"name": "3号测试点"}}))  # the name said
    out = sp.on_ack(_b({"cmd_id": "t-9", "result": "accepted", "code": "OK", "detail": {  # stored
        "applied": {"name": "3号测试点"}, "warn": [],  # heading kept
        "unreachable": [{"fence": "北门禁止区域围栏", "role": "forbid",  # but in a forbid zone
                         "reason": "inside_forbid", "objects": ["3号测试点"]}]}}))  # in force
    assert _said(out)[1] == "已记录3号测试点. 注意: 3号测试点在禁止区域北门禁止区域围栏里面, 机器人到不了"  # both


@pytest.mark.parametrize("code,detail,text", [  # 99 U133 start refusals
    ("E_NAME_CONFLICT", {}, "1号路径这个名字已经有了, 请换个名字再录制"),  # taken meanwhile
    ("E_TEACH_QUALITY", {"reason": "fix_quality"}, "现在不能开始录制: 定位不是RTK固定解"),  # check 3
    ("E_UNHEALTHY", {"reason": "no_nonvoice_estop"}, "现在不能开始录制: 没有可用的非语音急停通道"),
    ("E_BUSY", {"reason": "task_running"}, "有任务正在执行, 现在不能开始录制"),  # check 2
    # 99 U176: one activity area (~~activity_area_missing~~ gone with U184)
    ("E_GEO_INVALID", {"reason": "activity_area_exists", "existing": "营区活动区域围栏"},
     "已经有活动区域围栏营区活动区域围栏了, 只能有一个活动区域, 没有开始录制"),
    ("E_TEACH_STATE", {"reason": "recording_config_missing",
                       "missing": ["fence.speed_limit_default_mps"]},  # 99 U185 key
     "现在不能录制限速区域围栏: 限速值没有配置"),  # MUTATION: generic config sentence -> red
    ("E_TEACH_STATE", {"reason": "recording_config_missing", "missing": ["teach.sample_hz"]},
     "现在不能开始录制: 录制参数没有配置"),  # the U132 keys keep theirs
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
    ({"result": "rejected", "code": "E_TEACH_GEOMETRY", "detail": {  # 99 U184 overlap
      "saved": False, "close_reason": "invalid_geometry", "name": "北门禁止区域围栏",
      "issues": [{"code": "close_gap_large"},
                 {"code": "forbid_overlap", "fences": ["东门报警区域围栏"]}]}},
     "录制已退出, 北门禁止区域围栏没有保存: 和东门报警区域围栏重叠了, "
     "禁止区域不能和报警区域, 限速区域或其它禁止区域重叠"),
    ({"result": "accepted", "code": "OK", "detail": {  # 99 U184: saved, and told
      "saved": True, "kind": "fence",
      "applied": {"name": "北门禁止区域围栏", "point_count": 4, "total_len_m": 80.0},
      "validation": {"ok": True, "issues": [
          {"code": "outside_activity_area", "activity_area": "营区活动区域围栏"}]},
      "unreachable": [{"fence": "北门禁止区域围栏", "role": "forbid",
                       "reason": "inside_forbid", "objects": ["1号测试点"]}]}},
     "已保存北门禁止区域围栏, 共4个顶点. 围栏已保存但未启用, 需要时请说启用北门禁止区域围栏. "
     "注意: 它不在活动区域营区活动区域围栏内, 机器人到不了, 不会生效. "
     "注意: 1号测试点在禁止区域北门禁止区域围栏里面, 机器人到不了"),
    ({"result": "accepted", "code": "OK", "detail": {  # an area leaving older fences out
      "saved": True, "kind": "fence",
      "applied": {"name": "营区活动区域围栏", "point_count": 4, "total_len_m": 400.0},
      "validation": {"ok": True, "issues": [
          {"code": "activity_area_excludes", "fences": ["东门报警区域围栏"]}]}}},
     "已保存营区活动区域围栏, 共4个顶点. 围栏已保存但未启用, 需要时请说启用营区活动区域围栏. "
     "注意: 东门报警区域围栏不在这个活动区域里, 机器人到不了"),
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


# -- 99 U146: the single low-battery voice ---------------------------------------------
def _lb(kind, first=True, **detail):  # one P3 low-battery report
    """event/alarm/charging bytes with this detail.type."""
    return _b({"sev": "alarm", "cat": "charging",
               "detail": dict(detail, type=kind, first=first)})  # wire form


@pytest.mark.parametrize("kind,detail,text", [
    ("low_battery", {"soc_pct": 18.4, "target_name": "1号充电站"}, "电量剩余18%, 正在返回1号充电站"),
    ("low_battery_unreachable", {"soc_pct": 12, "target_name": "返航终点站"},
     "电量剩余12%, 正在返回返航终点站, 剩余电量可能不够走到"),
    ("low_battery_no_target", {"soc_pct": 19}, "电量剩余19%, 没有定义充电桩和返航终点站, 无法自动返回"),
    ("low_battery_stop", {"soc_pct": 5}, "电量剩余5%, 已停车趴下"),
])
def test_low_battery_is_spoken_once_locally(kind, detail, text):
    """MUTATION: wrong sentence / turn source -> red."""
    from xbrain.p4_agent.runtime.ack_speech import low_battery_speech  # under test
    assert _said(low_battery_speech(_lb(kind, **detail))) == ("cmd/audio/speak", text, "tts_local")


def test_low_battery_repeats_and_other_reports_are_silent():
    """The cloud gets a report every 2 s; the voice is single. MUTATION:
    ignore `first` -> every repeat spoken -> red; voice the boot report -> red."""
    from xbrain.p4_agent.runtime.ack_speech import CHARGING_EVENT_KEY, low_battery_speech
    assert low_battery_speech(_lb("low_battery", first=False, soc_pct=18)) is None  # a repeat
    assert low_battery_speech(_lb("low_battery_cleared", soc_pct=30)) is None  # closed
    assert low_battery_speech(_lb("return_target_missing")) is None  # boot report: cloud only
    assert CHARGING_EVENT_KEY == "event/*/charging"  # alarm and info


@pytest.mark.parametrize("cleared,text", [(["t-1"], "已取消1个定时任务"),
                                          (["t-1", "t-2"], "已取消2个定时任务"),
                                          ([], "现在没有定时任务")])
def test_clear_scheduled_speaks_p3s_count(cleared, text):  # 99 U147 race fix
    """The count comes from P3's cleared_ids, not P4's snapshot. MUTATION:
    stay silent on an accepted clear_queue -> no sentence -> red; count the
    wrong field -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/task", {"cmd_id": "c-1", "action": "clear_queue",
                                    "filter": {"state": "scheduled"}}))  # the "所有" frame
    out = sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted", "code": "OK",
                        "detail": {"cleared_ids": cleared, "applied": {"cleared": len(cleared)}}}))
    assert _said(out)[1] == text  # P3's number


def test_an_unfiltered_clear_says_the_queue_count():  # 99 U174
    """Until 2026-11-12 no P4 turn sent an unfiltered clear_queue and this case
    asserted silence; "取消所有任务" (U174) sends one and its count is P3's.
    MUTATION: speak the scheduled sentence for it -> "定时任务" -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_sent("cmd/task", {"cmd_id": "c-2", "action": "clear_queue"}))  # no filter
    out = sp.on_ack(_b({"cmd_id": "c-2", "result": "accepted", "code": "OK",  # P3 swept the queue
                        "detail": {"cleared_ids": ["t-1"]}}))  # S7.2: cleared_ids is mandatory
    assert _said(out)[1] == "已取消1个排队中的任务"  # not the scheduled wording


# -- 99 U162: a recording's result as a second state/voice_turn ----------------
# The exit / cancel turn itself only says "the command went out" (its verdict is
# accepted as soon as cmd/teach is published). Whether the recording was SAVED
# arrives later, on cmd/teach/ack. Before U162 that result reached only the
# speaker and cmd/teach/ack, so the cloud -- which reads state/voice_turn --
# never learned it. These tests pin the second record that now carries it.
_VT_KEY = "xbrain/gj-001/state/voice_turn"  # the full key, as the wiring publishes it


def _rec_turn(action="finish", cmd_id="c-f", spoke=True, intent="record_route_stop"):
    """The frames of an exit / cancel turn: cmd/teach, (speech), voice_turn.

    spoke=False is a turn under require_tts_reply=false: no cmd/audio/speak,
    but the voice_turn record is still published -- note_turn must pair the
    result with it all the same.
    """
    pairs = [("cmd/teach", _b({"cmd_id": cmd_id, "action": action,  # finish | discard
                               "session_id": "ts-1"}))]  # a session_id, as a voice exit sends
    if spoke:  # the turn's own reply ("going to save") -- not the result
        pairs.append(("cmd/audio/speak", _b({"schema": "p4_speak_v1",  # the turn's own reply
                                             "text": "好的,退出录制, 正在保存1号路径",
                                             "source": "tts_cloud"})))
    pairs.append((_VT_KEY, _b({"turn_id": "vt-1", "session": "cloud", "channel": "cloud",  # the turn's record
                               "asr_raw": "1号路径录制完成", "asr_text": "1号路径录制完成",  # an utterance with a name
                               "corrections": [], "bypass": None, "intent": intent,  # slots stay empty for record turns
                               "slots": {}, "auth_level": "L1a", "verdict": "accepted",  # accepted = sent, not saved
                               "reject_code": None,
                               "reply_text": "好的,退出录制, 正在保存1号路径",
                               "reply_kind": "stream", "layer": "record_rule",  # the rule layer made it
                               "ts_mono": 1.0,  # copied fields come from here
                               "latency_ms": {"llm": 5}})))  # must NOT be copied
    return pairs  # in publish order


# P3's ack for a named finish that saved (11 S12A.4 detail.applied).
_SAVED = {"cmd_id": "c-f", "result": "accepted", "code": "OK",  # P3 answered OK
          "detail": {"saved": True, "session_id": "ts-1", "kind": "route",  # saved, as a route
                     "applied": {"name": "1号路径", "point_count": 64,  # what the sentence reads back
                                 "total_len_m": 320.4}}}


def _turns(out):
    """The decoded voice_turn records among on_ack_frames' (key, data) pairs."""
    return [json.loads(d) for k, d in out if k == _VT_KEY]  # (key, json) -> dict


def test_a_saved_recording_is_published_as_a_second_voice_turn():
    """MUTATION: publish only the speech -> no voice_turn result -> red."""
    sp = AckSpeaker()  # a fresh speaker per test
    sp.note_turn(_rec_turn())  # the turn is noted first
    out = sp.on_ack_frames(_b(_SAVED))  # then P3's ack arrives
    assert [k for k, _ in out] == ["cmd/audio/speak", _VT_KEY]  # speech, then record
    (rec,) = _turns(out)  # exactly one result record
    # accepted = saved; layer tells it apart from the turn's own record
    assert rec["verdict"] == "accepted" and rec["layer"] == "teach_result"
    # the reply is the RESULT sentence (what was saved), not "going to save"
    assert rec["reply_text"].startswith("已保存1号路径") and "64" in rec["reply_text"]
    # a record of its own (new turn_id) about the same utterance (same text)
    assert rec["turn_id"] != "vt-1" and rec["asr_text"] == "1号路径录制完成"
    assert rec["channel"] == "cloud" and rec["intent"] == "record_route_stop"  # copied
    # spoken -> stream; the turn's latency belongs to that turn, not this one
    assert rec["reply_kind"] == "stream" and "latency_ms" not in rec


def test_a_silent_turn_still_gets_its_result_on_voice_turn():
    """require_tts_reply=false: nothing is spoken, but voice_turn is then the
    ONLY place the result goes. MUTATION: remember only turns that spoke -> red."""
    sp = AckSpeaker()  # same as above, silent
    sp.note_turn(_rec_turn(spoke=False))  # no cmd/audio/speak in the turn
    out = sp.on_ack_frames(_b(_SAVED))  # the same saved ack
    assert [k for k, _ in out] == [_VT_KEY]  # no speech, the record only
    (rec,) = _turns(out)
    assert rec["verdict"] == "accepted" and rec["reply_kind"] == "none"  # not spoken


def test_a_recording_not_saved_is_published_as_rejected():
    """MUTATION: always verdict accepted -> red."""
    sp = AckSpeaker()  # the exit turn
    sp.note_turn(_rec_turn())  # registers c-f
    out = sp.on_ack_frames(_b({"cmd_id": "c-f", "result": "rejected",  # geometry failed
                               "code": "E_TEACH_GEOMETRY",  # an invalid track
                               "detail": {"saved": False, "close_reason": "invalid_geometry",
                                          "name": "1号路径", "issues": []}}))
    (rec,) = _turns(out)  # one record even when not saved
    # the cloud must be able to tell "not saved" from "saved" without parsing text
    assert rec["verdict"] == "rejected" and rec["reject_code"] == "E_TEACH_GEOMETRY"  # not saved, by code
    assert "1号路径" in rec["reply_text"]  # the sentence names what was not saved


def test_a_discard_result_is_published_too():
    """A cancel ("取消1号路径录制") is a result as well: the points are gone."""
    sp = AckSpeaker()  # the cancel turn
    sp.note_turn(_rec_turn(action="discard", intent="record_route_cancel"))  # the cancel intent is copied
    out = sp.on_ack_frames(_b({"cmd_id": "c-f", "result": "accepted", "code": "OK",  # accepted = discarded
                               "detail": {"name": "1号路径", "discarded_points": 12}}))  # N points thrown away
    (rec,) = _turns(out)  # one record
    assert rec["verdict"] == "accepted" and "12" in rec["reply_text"]  # N points dropped


def test_result_record_once_and_only_for_our_commands():
    """One result per command: a repeated ack (zenoh may redeliver) is not a
    second result, and an ack for a cmd_id this P4 did not send is not ours."""
    sp = AckSpeaker()  # the same exit turn
    sp.note_turn(_rec_turn())  # c-f is ours now
    assert _turns(sp.on_ack_frames(_b(_SAVED)))  # first: published
    assert sp.on_ack_frames(_b(_SAVED)) == []  # again: already answered
    assert sp.on_ack_frames(_b({**_SAVED, "cmd_id": "other"})) == []  # not ours


# -- 99 U163: recording reports with no command behind them -----------------------------
def _teach(result, first=True, **detail):  # one P3 recording report
    """event/{sev}/teach bytes (11 S12A.12): "warning" -> teach.time_warning,
    anything else -> teach.expired with that detail.result."""
    ev = ({"kind": "teach.time_warning"} if result == "warning"  # the 10 / 1 min warning
          else {"kind": "teach.expired", "result": result})  # or the auto-finish
    return _b({"sev": "warn", "cat": "teach", "detail": dict(detail, first=first, **ev)})  # an event/*/teach body


_BRIEF = {"name": "1号路径", "session_kind": "route", "channel": "local_voice", "duration_s": 10800}  # the common fields of a report


@pytest.mark.parametrize("kind,detail,text", [
    ("warning", {"remaining_s": 600}, "路径1号路径的录制还剩10分钟, 到时会自动结束并保存"),  # 10 min left
    ("warning", {"remaining_s": 59.5}, "路径1号路径的录制还剩1分钟, 到时会自动结束并保存"),  # rounded up to 1 min
    ("saved", {"point_count": 10800, "length_m": 5400.4},  # a full 180 min recording
     "录制已到180分钟上限, 已自动保存为路径1号路径, 共10800个点, 长5400米"),
    ("save_failed", {"code": "E_NAME_CONFLICT"},  # write failed, data kept
     "录制已到180分钟上限, 路径1号路径自动保存失败, 名字已被占用. 录制数据还在, 请说保存为新的名字, 或者放弃录制"),
    ("discarded", {}, "录制已到180分钟上限, 路径1号路径的轨迹无效, 没有保存"),  # invalid track
    ("awaiting_name", {}, "录制已到180分钟上限, 已停止录制. 请说保存为某个名字, 或者放弃录制"),  # unnamed session
])
def test_each_recording_report_is_spoken(kind, detail, text):
    """99 U163. MUTATION: drop a result from _teach_event_text -> no speech -> red."""
    from xbrain.p4_agent.runtime.ack_speech import teach_event_frames
    out = teach_event_frames(_teach(kind, **dict(_BRIEF, **detail)), None)  # no record key: speech only
    assert len(out) == 1 and _said(out[0]) == ("cmd/audio/speak", text, "tts_local")  # one sentence, local voice


def test_a_fence_save_says_it_is_not_enabled():
    from xbrain.p4_agent.runtime.ack_speech import teach_event_frames
    out = teach_event_frames(_teach("saved", **dict(
        _BRIEF, session_kind="fence", name="仓库", point_count=40, length_m=120.0)), None)
    assert "围栏没有启用, 需要时请说启用围栏仓库" in _said(out[0])[1]  # a draft fence is said to be off


def test_a_recording_report_goes_on_voice_turn_under_its_channel():
    """The record the HMI and the cloud read: no ASR, layer teach_result, filed
    under the channel that started the recording, verdict by outcome.
    MUTATION: hard-code mic_local -> the cloud row -> red; verdict accepted on
    a failure -> red; first=False still spoken -> red."""
    from xbrain.p4_agent.runtime.ack_speech import teach_event_frames
    key = "xbrain/gj-001/state/voice_turn"  # the record key
    out = teach_event_frames(_teach("saved", **dict(
        _BRIEF, channel="cloud_voice", point_count=10, length_m=5.0)), key)
    assert [k for k, _ in out] == ["cmd/audio/speak", key]  # speech, then the record
    rec = json.loads(out[1][1])  # the record
    assert (rec["channel"], rec["session"], rec["layer"], rec["verdict"]) == (
        "cloud", "cloud", "teach_result", "accepted")
    assert rec["asr_raw"] == "" and rec["reply_text"].startswith("录制已到180分钟上限")  # nobody spoke: no ASR text
    bad = teach_event_frames(_teach("save_failed", **dict(
        _BRIEF, code="E_NAME_CONFLICT")), key)
    rec2 = json.loads(bad[1][1])  # the failed save
    assert rec2["verdict"] == "rejected" and rec2["reject_code"] == "E_NAME_CONFLICT"  # rejected with the real code
    assert rec2["channel"] == "mic_local"  # unknown channel files under the local mic
    assert teach_event_frames(_teach("saved", first=False, **_BRIEF), key) == []  # a repeat report is not spoken again


def test_other_teach_events_are_not_spoken():
    """Only teach.time_warning and teach.expired are spoken; the rest of the
    11 S12A.12 rows (an operator's own start / save) are answered by their ack.
    MUTATION: speak any teach event -> red."""
    from xbrain.p4_agent.runtime.ack_speech import teach_event_frames
    for kind in ("teach.started", "teach.saved", "teach.discarded"):
        ev = _b({"sev": "info", "cat": "teach", "detail": dict(_BRIEF, kind=kind, first=True)})  # a teach event outside the spoken set
        assert teach_event_frames(ev, "k") == []  # nothing published


# -- 99 U165: the start result, and a finish that kept its data ------------------
def _start_turn(spoke=True):
    """A start turn: cmd/teach start, (speech), the voice_turn record."""
    start = ("cmd/teach", _b({"cmd_id": "c-s", "action": "start",  # the start command
                              "start": {"kind": "route", "name": "1号路径"}}))
    return [start] + _rec_turn(spoke=spoke, intent="record_route_start")[1:]  # speech?, record


def test_an_accepted_start_is_published_but_not_spoken():
    """The client opens its keyboard on this record, never on the turn's own
    "好的,开始录制X". MUTATION: leave start out of the result set -> no record -> red."""
    sp = AckSpeaker()  # fresh speaker
    sp.note_turn(_start_turn())  # the start turn is noted
    out = sp.on_ack_frames(_b({"cmd_id": "c-s", "result": "accepted", "code": "OK",  # P3 accepted
                               "detail": {"session_id": "ts-1", "state": "recording"}}))
    assert [k for k, _ in out] == [_VT_KEY]  # nothing spoken: the turn already said it
    (rec,) = _turns(out)  # the start result
    assert rec["layer"] == "teach_result" and rec["verdict"] == "accepted"  # started
    assert rec["intent"] == "record_route_start" and rec["reply_text"] == "已开始录制1号路径"
    assert rec["reply_kind"] == "none"  # not spoken


def test_a_refused_start_is_published_as_rejected_with_its_reason():
    """Measured 2026-11-11: P3 refused (no rtk_fixed) after the turn said
    "好的,开始录制X", and only cmd/teach/ack carried it.
    MUTATION: verdict always accepted -> red."""
    sp = AckSpeaker()  # fresh speaker
    sp.note_turn(_start_turn(spoke=False))  # a silent cloud text turn
    out = sp.on_ack_frames(_b({"cmd_id": "c-s", "result": "rejected",  # P3 refused
                               "code": "E_TEACH_QUALITY", "detail": {"fix_type": "no_fix"}}))
    (rec,) = _turns(out)  # the only place the refusal goes
    assert rec["verdict"] == "rejected" and rec["reject_code"] == "E_TEACH_QUALITY"  # rejected with P3's code
    assert rec["reply_text"] == "现在不能开始录制: 定位不是RTK固定解"  # the spoken reason


def test_a_finish_that_kept_its_data_says_so():
    """99 U165: the write failed, the session waits in finalizing; "录制已退出"
    would be false. MUTATION: drop the retained branch -> generic sentence -> red."""
    sp = AckSpeaker()  # fresh speaker
    sp.note_turn(_rec_turn())  # a finish turn
    out = sp.on_ack_frames(_b({"cmd_id": "c-f", "result": "rejected", "code": "E_NAME_CONFLICT",  # write failed
                               "detail": {"saved": False, "retained": True, "name": "1号路径",
                                          "state": "finalizing"}}))
    (rec,) = _turns(out)  # one record
    assert rec["verdict"] == "rejected" and rec["reject_code"] == "E_NAME_CONFLICT"  # the real code, not E_TEACH_GEOMETRY
    assert rec["reply_text"] == ("1号路径没有保存: 名字已被占用. 录制数据还在, "
                                 "请说保存为新的名字, 或者放弃录制")


# -- 2026-11-12: a point capture's result as a second state/voice_turn --------
# Measured 2026-11-11 22:29: "保存当前位置为2号充电站" read accepted in
# voice_turn while P3 refused it (dock 13.5 m off every route) and the speaker
# was dead. The capture now publishes its result like a recording does.
def _cap_turn(spoke=True):
    """The frames of one F06 turn with its voice_turn record."""
    pairs = _turn(cmd_id="c-m", name="2号充电站", spoke=spoke)  # cmd/teach (+ speech)
    pairs.append((_VT_KEY, _b({"turn_id": "vt-2", "session": "cloud", "channel": "cloud",  # a cloud text turn
                               "asr_raw": "保存当前位置为2号充电站",  # what the operator sent
                               "asr_text": "保存当前位置为2号充电站", "corrections": [],  # the only link to the request
                               "bypass": None, "intent": "record_waypoint", "slots": {},  # F06
                               "auth_level": "L1a", "verdict": "accepted",  # sent, not stored
                               "reject_code": None, "reply_text": "好的,记录这个点",  # the turn's own reply
                               "reply_kind": "stream", "layer": "long_phrase",  # as the 11-11 record read
                               "ts_mono": 1.0})))  # copied fields come from here
    return pairs  # in publish order


def test_a_stored_capture_is_published_as_a_second_voice_turn():
    """MUTATION: drop mark_once from _RESULT_TO_TURN -> no record -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_cap_turn(spoke=False))  # a silent cloud text turn
    out = sp.on_ack_frames(_b({"cmd_id": "c-m", "result": "accepted", "code": "OK",  # P3's teach ack
                               "detail": {"applied": {"geo_id": "w-7", "name": "2号充电站"},  # AP-1 values
                                          "warn": []}}))  # P3 stored it
    (rec,) = _turns(out)  # the only place a silent turn's result goes
    assert rec["verdict"] == "accepted" and rec["layer"] == "teach_result"  # stored, as a result record
    assert rec["reply_text"] == "已记录2号充电站" and rec["intent"] == "record_waypoint"  # said from applied
    assert rec["turn_id"] != "vt-2" and rec["reply_kind"] == "none"  # its own record


def test_a_refused_capture_is_published_as_rejected():
    """The 2026-11-11 case. MUTATION: verdict always accepted -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_cap_turn())  # spoke "好的,记录这个点"
    out = sp.on_ack_frames(_b({"cmd_id": "c-m", "result": "rejected", "code": "E_GEO_INVALID",  # U146 refusal
                               "detail": {"reason": "dock_off_route", "distance_m": 13.5}}))  # the measured gap
    assert [k for k, _ in out] == ["cmd/audio/speak", _VT_KEY]  # speech, then record
    (rec,) = _turns(out)  # exactly one result record
    assert rec["verdict"] == "rejected" and rec["reject_code"] == "E_GEO_INVALID"  # by code, not by text
    # 99 U169 wording (distance from P3's detail; no limit_m in this ack)
    assert rec["reply_text"] == ("2号充电站没有保存: 这里离最近的路径有13.5米, "
                                 "充电站和充电桩必须建在路径旁边. 请走到路径旁边再说一次")



# -- 99 U174: the count of a whole-queue cancel is P3's -------------------------
@pytest.mark.parametrize("ids,said", [(["t-1", "t-2"], "已取消2个排队中的任务"),
                                      ([], "现在没有排队中的任务")])
def test_a_whole_queue_cancel_says_how_many_p3_cleared(ids, said):
    """MUTATION: no unfiltered branch -> accepted says nothing -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn([("cmd/task", _b({"cmd_id": "c-q", "action": "clear_queue",  # the U174 frame:
                                    "source": "local"})),  # no filter at all
                  ("cmd/audio/speak", _b({"schema": "p4_speak_v1",  # the turn spoke, so the
                                          "text": "好的,取消全部排队中的任务",  # answer is spoken
                                          "source": "tts_local"}))])  # on the same source
    out = sp.on_ack(_b({"cmd_id": "c-q", "result": "accepted", "code": "OK",  # P3 answered
                        "detail": {"cleared_ids": ids, "applied": {"cleared": len(ids)}}}))  # its own count
    assert _said(out)[1] == said  # P3's number, never the snapshot's
