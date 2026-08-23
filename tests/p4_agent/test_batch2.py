"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch2.py
Brief: GWY-P4-03/04/06 ASR three-layer post + correction log + priority chain

Description:
*** Brief 由占位串改写(2026-08-23). 原值是按路径自动生成的 "p4_agent tests -- batch2" --
既没说清本文件测什么, 也无法据以索引任务号(CLAUDE.md 2.5). 同一处理已对
P2 做过; 补 Brief 而不是绕过去, 是因为绕过去只会让下一个人再读一遍.
GWY-P4-03/04/05/06 batch 2 tests.
"""


import pytest

from xbrain.p4_agent.asr_post.correction_log import (
    CorrectionLayer, build_row, negative_samples, reject_row,
)
from xbrain.p4_agent.asr_post.three_layer import (
    L1Dict, L3ClosedSet, post_process,
)
from xbrain.p4_agent.classifier.priority_chain import (
    ChainLayer, classify_after_bypass, is_directed_at_robot,
    is_semantically_broken,
)
from xbrain.p4_agent.safety_bypass.recording_gate import (
    RecordingState, SuppressionRecord, evaluate,
)


pytestmark = pytest.mark.no_device


# --- P4-03 L1 dict longest-match-first ---

def test_l1_longest_match_first():
    d = L1Dict(entries={"三号": "3号", "三号路劲": "三号路径"})
    # Long match wins: "三号路劲" -> "三号路径", NOT "3号路劲".
    assert d.apply("走三号路劲") == "走三号路径"


def test_l1_empty_leaves_text():
    d = L1Dict()
    assert d.apply("hello") == "hello"


def test_l3_snaps_when_score_high_and_unique():
    """A candidate with similar chars but NOT containing any member
    snaps to the best-scoring member."""
    l3 = L3ClosedSet(members=("东门", "西门"), snap_threshold=0.3)
    # Fuzzy candidate not containing any member literally.
    assert l3.apply("东们") == "东门"


def test_l3_no_op_when_member_already_in_candidate():
    """If a legal member is ALREADY in the text, no rewrite needed."""
    l3 = L3ClosedSet(members=("东门", "西门"), snap_threshold=0.5)
    assert l3.apply("去东门口") == "去东门口"


def test_l3_does_not_snap_below_threshold():
    """Q-P4-9: 宁可不吸附, 也不吸附错. Below threshold keeps raw."""
    l3 = L3ClosedSet(members=("东门", "西门"), snap_threshold=0.95)
    # Very different candidate; no member wins strongly.
    assert l3.apply("北墙路径") == "北墙路径"


def test_post_process_end_to_end():
    l1 = L1Dict(entries={"东门儿": "东门"})
    l3 = L3ClosedSet(members=("东门", "西门"), snap_threshold=0.5)
    assert post_process("去东门儿看看", l1, l3) == "去东门看看"


def test_post_process_empty_stays_empty():
    assert post_process("", L1Dict(), L3ClosedSet()) == ""


# --- P4-04 correction log ---

def test_reject_row_flips_accepted_to_false():
    row = build_row("2026-08-09", "东门儿", "东门",
                     CorrectionLayer.L1, 1.0, "s1")
    rejected = reject_row(row)
    assert rejected.accepted is False
    # Original row still accepted (frozen dataclass immutability).
    assert row.accepted is True


def test_negative_samples_filters():
    rows = [
        build_row("t1", "a", "A", CorrectionLayer.L1, 1.0, "s1"),
        reject_row(build_row("t2", "b", "B", CorrectionLayer.L1, 1.0, "s1")),
    ]
    neg = negative_samples(rows)
    assert len(neg) == 1
    assert neg[0].raw == "b"


# --- P4-05 U45 recording-state suppression ---

def test_not_in_recording_no_suppression():
    st = RecordingState(in_recording=False)
    assert evaluate(st, "estop") is None


def test_in_recording_estop_suppressed():
    st = RecordingState(in_recording=True)
    r = evaluate(st, "estop")
    assert r is not None
    assert r.route == "suppressed"
    assert r.accepted == 0
    assert "手柄急停" in r.tts_advice
    assert st.voice_estop_suppress_count == 1


def test_in_recording_prone_and_stand_not_suppressed():
    """U45 suppresses ONLY estop. prone/stand remain executable
    (they are movement, not safety-stops)."""
    st = RecordingState(in_recording=True)
    assert evaluate(st, "prone") is None
    assert evaluate(st, "stand") is None


# --- P4-06 priority chain ---

def test_long_phrase_wins_layer_2():
    r = classify_after_bypass(
        "停止喊话",
        long_phrase_match="stop_announce",
        session_state_match=None,
        large_class_match=None,
    )
    assert r.layer == ChainLayer.LONG_PHRASE.value
    assert r.intent == "stop_announce"


def test_session_state_wins_layer_3():
    r = classify_after_bypass(
        "结束录制",
        long_phrase_match=None,
        session_state_match="record_route_stop",
        large_class_match="unrelated",
    )
    # Layer 3 fires before layer 4.
    assert r.layer == ChainLayer.SESSION_STATE.value


def test_large_class_wins_layer_4():
    r = classify_after_bypass(
        "向前走",
        long_phrase_match=None,
        session_state_match=None,
        large_class_match="move_forward",
    )
    assert r.layer == ChainLayer.LARGE_CLASS.value


def test_overheard_wins_layer_5_when_no_match_and_not_directed():
    r = classify_after_bypass(
        "他今天买了新车",
        long_phrase_match=None,
        session_state_match=None,
        large_class_match=None,
    )
    assert r.layer == ChainLayer.OVERHEARD.value
    assert r.fires_llm is False


def test_unknown_falls_to_llm_when_directed_but_no_match():
    r = classify_after_bypass(
        "机器人,你好啊",     # wake-word triggers directed
        long_phrase_match=None,
        session_state_match=None,
        large_class_match=None,
    )
    assert r.layer == ChainLayer.UNKNOWN.value
    assert r.fires_llm is True


def test_is_directed_at_robot_wake_word():
    assert is_directed_at_robot("机器人前进", matched_intent_kw=False)
    assert is_directed_at_robot("嘿, 停一下", matched_intent_kw=False)


def test_is_directed_at_robot_matched_kw_alone():
    """Rule 3: matched intent kw alone = directed."""
    assert is_directed_at_robot("blah", matched_intent_kw=True)


def test_is_semantically_broken_low_confidence():
    assert is_semantically_broken("hi", asr_confidence=0.1)


def test_asymmetric_default_overheard_on_uncertainty():
    """16 S5.2.1: 拿不准时判 overheard."""
    r = classify_after_bypass(
        "嗯", asr_confidence=0.3,
        long_phrase_match=None,
        session_state_match=None,
        large_class_match=None,
    )
    assert r.layer == ChainLayer.OVERHEARD.value
