"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_command_result_turn.py
Brief: 99 U169 -- every worded command result also goes out as a voice_turn record

Description:
User ruling 2026-11-12: every reply P4 gives after a voice / text command must
reach the cloud for echo. The cloud and commx read state/voice_turn only, and
until U169 only recording / capture results (teach_result, U162 / U165 / U167)
got a second record; an executor refusal on cmd/task / cmd/geo /
cmd/motion/intent, a clear_queue count and the later failure of an accepted
task were spoken only (update_cloud.md S8.7), and a silent text turn
(require_tts_reply false) got no answer anywhere.

Pins, per case: the record exists, carries layer command_result, the right
verdict / reject_code, the same sentence that is spoken, the turn's channel /
asr_text, and a new turn_id; a silent turn gets the record without speech; an
accepted command with nothing more to say gets nothing; a capture still gets
exactly one record (teach_result), not two.
"""

from __future__ import annotations  # postponed annotations

import json  # wire frames

from xbrain.p4_agent.runtime.ack_speech import (  # the unit under test
    COMMAND_RESULT_LAYER,  # the new layer
    AckSpeaker,  # remembers turns, answers acks
)

_VT = "xbrain/gj-001/state/voice_turn"  # the full key, as the wiring publishes it
_SPEAK = "cmd/audio/speak"  # the TTS key


def _b(obj) -> bytes:
    """One frame's wire bytes."""
    return json.dumps(obj, ensure_ascii=False).encode("utf-8")  # as published


def _record(text: str, intent: str, channel: str = "cloud") -> dict:
    """The turn's own voice_turn record (11 S8.7.6 shape, the fields copied)."""
    return {"turn_id": "vt-1", "session": channel, "channel": channel,  # where it came from
            "asr_raw": text, "asr_text": text, "corrections": [], "bypass": None,  # the request
            "intent": intent, "slots": {}, "auth_level": "L1a", "verdict": "accepted",  # sent
            "reject_code": None, "reply_text": "好的", "reply_kind": "stream",  # the turn's reply
            "layer": "long_phrase", "ts_mono": 1.0, "latency_ms": {"asr": 10}}  # measured


def _turn(key: str, cmd: dict, text: str, intent: str, spoke: bool = True) -> list:
    """One turn's frames: the command, (the reply speech), the voice_turn record."""
    pairs = [(key, _b(cmd))]  # the command
    if spoke:  # require_tts_reply true
        pairs.append((_SPEAK, _b({"schema": "p4_speak_v1", "text": "好的",  # the reply
                                  "source": "tts_cloud"})))  # on this source
    pairs.append((_VT, _b(_record(text, intent))))  # the record, published last
    return pairs  # in publish order


def _split(out):
    """(spoken texts, decoded voice_turn records) of one answer."""
    said = [json.loads(d)["text"] for k, d in out if k == _SPEAK]  # speech
    recs = [json.loads(d) for k, d in out if k == _VT]  # records
    return said, recs  # both lists


def test_an_executor_refusal_is_spoken_and_recorded():
    """The 11-04 case "继续" refused by P3. MUTATION: no command_result branch
    -> speech only, the cloud sees "好的" -> red; reply_text not the spoken
    sentence -> red; latency copied / turn_id reused -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn("cmd/task", {"cmd_id": "c-1", "action": "resume"}, "继续任务", "resume_task"))
    said, recs = _split(sp.on_ack_frames(_b({"cmd_id": "c-1", "result": "rejected",  # P3 refused
                                             "code": "E_TASK_STATE"})))  # wrong state
    assert len(said) == 1 and len(recs) == 1  # spoken, and recorded once
    rec = recs[0]  # the result record
    assert (rec["layer"], rec["verdict"], rec["reject_code"]) == (
        COMMAND_RESULT_LAYER, "rejected", "E_TASK_STATE")  # what the cloud shows
    assert rec["reply_text"] == said[0] and rec["reply_kind"] == "stream"  # same sentence, aloud
    assert (rec["asr_text"], rec["channel"], rec["intent"]) == ("继续任务", "cloud", "resume_task")
    assert rec["turn_id"] != "vt-1" and "latency_ms" not in rec  # a record of its own


def test_a_silent_turn_gets_the_record_without_speech():
    """require_tts_reply false (commx --no-tts, cloud text): until U169 the
    refusal went nowhere. MUTATION: skip silent turns in note_turn -> [] -> red;  # see the case docstring
    speak anyway -> red."""  # see the case docstring
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn("cmd/task", {"cmd_id": "c-2", "action": "pause"}, "暂停任务",  # the turn is noted first
                       "pause_task", spoke=False))  # no speech in the turn
    said, recs = _split(sp.on_ack_frames(_b({"cmd_id": "c-2", "result": "rejected",  # speech and records of the answer
                                             "code": "E_TASK_STATE"})))  # the ack body
    assert said == []  # silent stays silent
    assert len(recs) == 1 and recs[0]["reply_kind"] == "none"  # but recorded
    assert recs[0]["verdict"] == "rejected"  # the refusal reaches the cloud


def test_an_accepted_command_with_nothing_more_to_say_adds_nothing():
    """The turn already said what it does. MUTATION: record every ack -> a
    duplicate "accepted" record per command -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn("cmd/task", {"cmd_id": "c-3", "action": "pause"}, "暂停任务", "pause_task"))
    assert sp.on_ack_frames(_b({"cmd_id": "c-3", "result": "accepted", "code": "OK"})) == []


def test_a_worded_accepted_result_is_recorded_as_accepted():
    """U174 "取消所有任务": the count is a result. MUTATION: verdict from the
    presence of a sentence (rejected) -> red."""  # see the case docstring
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn("cmd/task", {"cmd_id": "c-4", "action": "clear_queue"},  # the turn is noted first
                       "取消所有任务", "clear_queue"))  # see the case docstring
    said, recs = _split(sp.on_ack_frames(_b({"cmd_id": "c-4", "result": "accepted", "code": "OK",  # speech and records of the answer
                                             "detail": {"cleared_ids": ["t-1", "t-2"]}})))  # the ack body
    assert recs[0]["verdict"] == "accepted" and recs[0]["reject_code"] is None  # done
    assert recs[0]["reply_text"] == said[0] == "已取消2个排队中的任务"  # the count


def test_a_motion_refusal_is_recorded():
    """cmd/motion/intent (enveloped ack). MUTATION: leave motion out -> red."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn("cmd/motion/intent", {"cmd_id": "c-5", "intent": "rotate"},  # the turn is noted first
                       "左转90度", "rotate_left"))  # see the case docstring
    out = sp.on_ack_frames(_b({"data": {"cmd_id": "c-5", "result": "rejected",  # S3.0 envelope
                                        "code": "E_BUSY", "detail": {"reason": "rotation_blocked"}}}))  # the ack body
    _said, recs = _split(out)  # the record
    assert len(recs) == 1 and recs[0]["reject_code"] == "E_BUSY"  # refused, by code


def _submit_then_accept(sp, spoke=True):
    """A goto turn P3 accepted as t-9."""
    sp.note_turn(_turn("cmd/task", {"cmd_id": "c-6", "action": "submit",  # the submit
                                    "task": {"type": "goto", "params": {"slots": {"target": "1号测试点"}}}},  # a goto by name
                       "去1号测试点", "goto", spoke=spoke))  # see the case docstring
    sp.on_ack_frames(_b({"cmd_id": "c-6", "result": "accepted", "code": "OK",  # P3 took it
                         "detail": {"task_id": "t-9"}}))  # the ack body


def test_a_task_that_fails_later_is_recorded():
    """FS-2 (U140): "X没有完成: why" was spoken only. MUTATION: no record in
    on_task_event_frames -> red; the error code dropped -> red."""
    sp = AckSpeaker()  # fresh
    _submit_then_accept(sp)  # accepted
    out = sp.on_task_event_frames(_b({"detail": {"kind": "failed", "task_id": "t-9",  # later
                                                  "error": {"code": "E_NAV_FAILED",
                                                            "fail_reason": "stalled"}}}))
    said, recs = _split(out)  # speech + record
    assert len(said) == 1 and len(recs) == 1  # both
    assert (recs[0]["verdict"], recs[0]["reject_code"], recs[0]["layer"]) == (
        "rejected", "E_NAV_FAILED", COMMAND_RESULT_LAYER)  # failed, by code
    assert recs[0]["reply_text"] == said[0] and recs[0]["asr_text"] == "去1号测试点"  # tied to the turn


def test_a_silent_submit_that_fails_later_is_recorded_without_speech():
    """MUTATION: tasks of silent turns not remembered -> [] -> red."""
    sp = AckSpeaker()  # fresh
    _submit_then_accept(sp, spoke=False)  # a cloud text turn, no TTS
    said, recs = _split(sp.on_task_event_frames(_b({"detail": {"kind": "failed", "task_id": "t-9"}})))
    assert said == [] and len(recs) == 1  # the record only
    assert sp.on_task_event(_b({"detail": {"kind": "failed", "task_id": "t-9"}})) is None  # forgotten


def test_a_completed_task_adds_nothing():
    """MUTATION: record every outcome -> a "completed" record nobody asked for -> red."""
    sp = AckSpeaker()  # fresh
    _submit_then_accept(sp)  # accepted
    assert sp.on_task_event_frames(_b({"detail": {"kind": "completed", "task_id": "t-9"}})) == []


def test_a_capture_still_gets_exactly_one_result_record():
    """The teach_result path stays as it was. MUTATION: command_result added
    beside teach_result -> two records for one capture -> red."""  # see the case docstring
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn("cmd/teach", {"cmd_id": "c-7", "action": "mark_once",  # F06
                                     "mark_once": {"kind": "waypoint", "name": "2号测试点",  # what F06 sends
                                                   "overwrite": False, "capture_heading": True}},  # see the case docstring
                       "保存当前位置为2号测试点", "record_waypoint"))  # see the case docstring
    _said, recs = _split(sp.on_ack_frames(_b({"cmd_id": "c-7", "result": "rejected",  # speech and records of the answer
                                              "code": "E_TEACH_QUALITY"})))  # the ack body
    assert [r["layer"] for r in recs] == ["teach_result"]  # one, the existing kind


def _dock_refusal(detail: dict):
    """A 2号充电站 capture refused by P3 with this detail: (spoken, record)."""
    sp = AckSpeaker()  # fresh
    sp.note_turn(_turn("cmd/teach", {"cmd_id": "c-8", "action": "mark_once",  # F06 / F10
                                     "mark_once": {"kind": "waypoint", "name": "2号充电站",  # what F06 sends
                                                   "overwrite": False, "capture_heading": True}},  # see the case docstring
                       "保存当前位置为2号充电站", "record_waypoint"))  # see the case docstring
    said, recs = _split(sp.on_ack_frames(_b({"cmd_id": "c-8", "result": "rejected",  # P3
                                             "code": "E_GEO_INVALID", "detail": detail})))  # the ack body
    return said[0], recs[0]  # one spoken sentence, one teach_result record


def test_a_dock_off_route_says_the_distance_the_limit_and_what_to_do():
    """User 2026-11-12: compose the failure reply. The measured 11-11 case was
    13.5 m off. MUTATION: drop the distance / the limit / the advice -> red;
    the limit typed in instead of read -> 8 m case reads 5 -> red."""
    said, rec = _dock_refusal({"reason": "dock_off_route", "distance_m": 13.5, "limit_m": 5.0})
    assert said == ("2号充电站没有保存: 这里离最近的路径有13.5米, 充电站和充电桩必须在路径5米以内. "
                    "请走到路径旁边再说一次")  # how far, the rule, what to do
    assert rec["reply_text"] == said and rec["verdict"] == "rejected"  # the cloud gets the same
    other, _r = _dock_refusal({"reason": "dock_off_route", "distance_m": 9.0, "limit_m": 8.0})
    assert "路径有9米" in other and "路径8米以内" in other  # read, not typed in


def test_a_dock_with_no_route_at_all_says_record_a_route_first():
    """distance_m null = no route exists. MUTATION: "null米" / the distance
    sentence -> red; a missing limit_m invents a number -> red."""  # see the case docstring
    said, _rec = _dock_refusal({"reason": "dock_off_route", "distance_m": None, "limit_m": 5.0})  # see the case docstring
    assert said == "2号充电站没有保存: 还没有任何路径, 充电站和充电桩必须建在路径旁边. 请先录制路径"  # the property
    nolimit, _r = _dock_refusal({"reason": "dock_off_route", "distance_m": 7.0})  # older P3
    assert "必须建在路径旁边" in nolimit and "5米" not in nolimit  # no invented number
