"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_teach_ack_speech.py
Brief: a F06 / F10 capture's result is spoken from cmd/teach/ack (11 CMD-30 / AP-1)

Description:
99 U109. Before, the operator heard "收到" and nothing else, whether P3 stored
the point or refused it. The speaker remembers this process's mark_once by
cmd_id (only when the turn spoke) and voices the ack once, on the same TTS
source. Each case names the mutant it kills.
"""
from __future__ import annotations

import json  # frames are JSON bytes

import pytest  # marker

from xbrain.p4_agent.runtime.teach_ack_speech import (
    CMD_TEACH_ACK,  # the answer key
    TeachAckSpeaker,  # unit under test
)

pytestmark = pytest.mark.no_device  # pure


def _b(obj):
    """One frame's bytes."""
    return json.dumps(obj, ensure_ascii=False).encode("utf-8")  # wire form


def _turn(cmd_id="c-1", name="1号测试点", source="tts_local", spoke=True):
    """The frames one F06 turn publishes: the command, then the reply."""
    pairs = [("cmd/teach", _b({"cmd_id": cmd_id, "action": "mark_once",
                               "mark_once": {"kind": "waypoint",
                                             "name": name}}))]  # the capture
    if spoke:
        pairs.append(("cmd/audio/speak", _b({"schema": "p4_speak_v1",
                                             "text": "好的,记录这个点",
                                             "source": source})))  # the reply
    return pairs  # in publish order


def _said(out):
    """(key, text, source) of what the speaker would publish."""
    key, data = out  # one pair
    body = json.loads(data)  # p4_speak_v1
    return key, body["text"], body["source"]  # what the case checks


def test_accepted_is_restated_from_applied():
    """AP-1: the applied name, on the turn's own source. MUTATION: say the
    requested name only, or speak on a fixed source."""
    sp = TeachAckSpeaker()  # fresh
    sp.note_turn(_turn(source="tts_cloud"))  # a cloud turn
    out = sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted", "code": "OK",
                        "detail": {"applied": {"name": "1号测试点"},
                                   "warn": []}}))  # P3 stored it
    assert _said(out) == ("cmd/audio/speak", "已记录1号测试点", "tts_cloud")  # restated


def test_no_heading_is_said():
    sp = TeachAckSpeaker()  # fresh
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
def test_a_refusal_says_why(code, detail, text):
    """MUTATION: one generic sentence -- the operator cannot act on it."""
    sp = TeachAckSpeaker()  # fresh
    sp.note_turn(_turn())  # one capture
    out = sp.on_ack(_b({"cmd_id": "c-1", "result": "rejected", "code": code,
                        "detail": detail}))  # P3 refused
    assert _said(out)[1] == text  # the reason


def test_once_only_and_only_our_own():
    """MUTATION: keep the entry and a duplicate ack speaks twice; answer every
    ack and another initiator's capture is voiced here."""
    sp = TeachAckSpeaker()  # fresh
    sp.note_turn(_turn())  # c-1 only
    ok = _b({"cmd_id": "c-1", "result": "accepted", "code": "OK",
             "detail": {"applied": {"name": "1号测试点"}}})  # the ack
    assert sp.on_ack(ok) is not None  # first: spoken
    assert sp.on_ack(ok) is None  # again: already answered
    assert sp.on_ack(_b({"cmd_id": "c-9", "result": "accepted"})) is None  # not ours
    sp.note_turn(_turn(cmd_id="c-2"))  # another capture
    assert sp.on_ack(_b({"cmd_id": "c-2", "result": "duplicate"})) is None  # S12A.4 replay


def test_a_silent_turn_stays_silent():
    """11 S8.7.5 require_tts_reply=false: no reply, so no result either.
    MUTATION: remember every capture and a typed silent command speaks."""
    sp = TeachAckSpeaker()  # fresh
    sp.note_turn(_turn(spoke=False))  # no speak frame
    assert sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted"})) is None  # nothing


def test_session_actions_are_not_voiced_here():
    sp = TeachAckSpeaker()  # fresh
    sp.note_turn([("cmd/teach", _b({"cmd_id": "c-1", "action": "start"})),
                  ("cmd/audio/speak", _b({"source": "tts_local"}))])  # a start
    assert sp.on_ack(_b({"cmd_id": "c-1", "result": "accepted"})) is None  # state/teach's job


def test_outstanding_captures_are_bounded():
    """MUTATION: an unbounded dict grows forever when P3 is down."""
    sp = TeachAckSpeaker()  # fresh
    for i in range(40):  # more than the bound
        sp.note_turn(_turn(cmd_id="c-%d" % i))  # one each
    assert sp.on_ack(_b({"cmd_id": "c-0", "result": "accepted"})) is None  # evicted
    assert sp.on_ack(_b({"cmd_id": "c-39", "result": "accepted"})) is not None  # kept
    assert CMD_TEACH_ACK == "cmd/teach/ack"  # 11 S2.2.4 key
