"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: speak_stop.py
Brief: CHK-1-32 D12 speak_stop -- four-branch reply + clear pending + idempotent

Description:
D12 speak_stop stops an in-progress broadcast. The reply MUST
narrate the ACTUAL state of the queue, not just say 'stopped':

  case 1 -- >1 utterance queued and current mid-play:
    'later utterances won't play; the current one will finish'
  case 2 -- only current utterance left:
    'this one won't cut off' (can't-stop-mid-utterance semantics
    of the underlying TTS device)
  case 3 -- nothing playing:
    'no broadcast active right now' (idempotent -- still emit ONE
    cmd/payload stop to keep the audit trail complete)
  case 4 -- repeat-N cycle with n remaining:
    same as case 1 plus 'the remaining N replays are cancelled'

All four reply strings MUST be pairwise distinct AND none equal
the bare 'stopped' string (that string would mask cases 1/2/4).

Route-2 discipline (spec: 'P2 sends each utterance individually
using est_ms'): D08 downstream must not use a [32]-count loop
endpoint. Static asserted by the meta-check.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List


REPLY_STOPPED_BARE = "已停止喊话"      # forbidden as a sole reply
REPLY_CASE_1 = "后面几遍不再播了, 当前这一句会播完"
REPLY_CASE_2 = "这一句停不下来, 说完就结束"
REPLY_CASE_3 = "当前没有在喊话, 已按幂等发出停止"
REPLY_CASE_4 = ("后面几遍不再播了, 当前这一句会播完; "
                  "剩余重复遍数已清零")


@dataclass(frozen=True)
class SpeakStopDecision:
    """What the D12 handler emits after inspecting state/audio."""
    reply: str
    case: str                # 'case_1' / 'case_2' / 'case_3' / 'case_4'
    emit_payload_stop: bool
    clear_pending: bool      # for cases 1 and 4


def decide(pending_count: int,
             is_current_mid_play: bool,
             repeat_remaining: int) -> SpeakStopDecision:
    """Given queue depth + current-play state, pick the reply."""
    if not is_current_mid_play and pending_count == 0 and repeat_remaining == 0:
        return SpeakStopDecision(
            reply=REPLY_CASE_3, case="case_3",
            emit_payload_stop=True, clear_pending=False)
    if repeat_remaining > 0:
        return SpeakStopDecision(
            reply=REPLY_CASE_4, case="case_4",
            emit_payload_stop=True, clear_pending=True)
    if pending_count > 0:
        return SpeakStopDecision(
            reply=REPLY_CASE_1, case="case_1",
            emit_payload_stop=True, clear_pending=True)
    return SpeakStopDecision(
        reply=REPLY_CASE_2, case="case_2",
        emit_payload_stop=True, clear_pending=False)


def replies_pairwise_distinct() -> bool:
    """CHK-1-32 (ii) meta-check: none of the four is bare 'stopped',
    and all four differ from each other."""
    all_replies = [REPLY_CASE_1, REPLY_CASE_2, REPLY_CASE_3, REPLY_CASE_4]
    if any(r == REPLY_STOPPED_BARE for r in all_replies):
        return False
    return len(set(all_replies)) == len(all_replies)


def assert_route2_no_bracket_loop(source_text: str) -> None:
    """CHK-1-32 (iv) static guard: D08 route-2 spec forbids a
    literal '[32]' loop-endpoint pattern. Caller passes concatenated
    dialog code."""
    if "[32]" in source_text:
        raise AssertionError(
            "route-2 must not use [32] loop endpoint; P2 sends each "
            "utterance individually via est_ms (D08 spec)")
