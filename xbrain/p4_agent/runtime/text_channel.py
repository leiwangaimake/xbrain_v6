"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: text_channel.py
Brief: GWY-P4-42 (32.J) -- cmd/voice_text -> orchestrator (skips ASR)

Description:
11 S8.7.5: text commands arrive on cmd/voice_text from the cloud (T), WeChat
(E), and HMI channels. The entry point is TEXT: the orchestrator's
handle_turn already takes text, so a text command runs the SAME six-step
classify + confirm + reply as a voice turn, minus the ASR front-end (11
S9178: 'T text ... does NOT enter ASR'). Voice adds VAD + ASR before this;
text goes straight in.

Two things this enforces, each with a mutation test:
  * text skips ASR (criterion 1). This module imports no ASR client and
    calls no transcribe -- the text is already text. A path that re-ran ASR
    on a typed command would be nonsense (there is no audio) and slower.
  * channel admission (criterion 2). Before the turn runs, the H03f
    override (16 S3233) is checked: H03 set_time_sync with force_step==true
    is cloud-only; on HMI/WeChat it is E_CHANNEL_DENIED. The check is a
    cheap keyword pre-classify (no LLM) so a restricted intent is refused
    before any dispatch.

Why the channel gate is a pre-check and not inside the orchestrator: the
orchestrator is channel-agnostic (voice is always local). The text entry is
the gateway point where 18's channel column applies, so the gate lives
here, next to the channel token.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import time
from typing import Any, List, Mapping, Optional, Tuple

from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher
from xbrain.p4_agent.registry.channel_permission import (
    ChannelAdmissionError, channel_admission,
)
from xbrain.p4_agent.runtime.intent_dispatch import CMD_AUDIO_SPEAK
from xbrain.p4_agent.runtime.orchestrator_turn import decision_to_publishes
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession, TurnDecision, TurnOrchestrator,
)


# cmd/voice_text channel closed set (11 S8.7.5).
_TEXT_CHANNELS = frozenset({"cloud", "wecom", "hmi"})

_CHANNEL_DENIED_REPLY = "该指令不支持当前通道"


class TextChannelError(RuntimeError):
    """The cmd/voice_text message is malformed (missing/typed field)."""


@dataclass(frozen=True)
class TextTurnResult:
    """Outcome of one text command."""
    denied: bool
    code: str
    decision: Optional[TurnDecision]
    publishes: List[Tuple[str, bytes]] = field(default_factory=list)


def _speak_bytes(text: str) -> Tuple[str, bytes]:
    payload = {"schema": "p4_speak_v1", "text": text,
               "mono_ms": int(time.monotonic() * 1000)}
    return (CMD_AUDIO_SPEAK,
            json.dumps(payload, ensure_ascii=False,
                       separators=(",", ":")).encode("utf-8"))


def handle_text_command(
    msg: Mapping[str, Any],
    orchestrator: TurnOrchestrator,
    session: OrchestratorSession,
    matcher: KeywordMatcher,
    now_mono_ms: int,
) -> TextTurnResult:
    """Process one cmd/voice_text message. NO ASR is invoked.

    msg fields (11 S8.7.5): channel (cloud/wecom/hmi), text, optional slots
    (structured fields the gateway parsed, e.g. force_step), optional
    require_tts_reply (default true).

    Runs the channel admission gate first (H03f cloud-only), then the
    orchestrator on the TEXT directly. Returns the decision + the (key,
    payload_bytes) pairs to publish.
    """
    channel = msg.get("channel")
    if channel not in _TEXT_CHANNELS:
        raise TextChannelError(
            "channel %r not in %s" % (channel, sorted(_TEXT_CHANNELS)))
    text = (msg.get("text") or "").strip()
    slots = msg.get("slots") or {}
    want_tts = msg.get("require_tts_reply", True)

    # Channel admission (H03f cloud-only). Pre-classify with the keyword
    # matcher only (no LLM, no ASR) to learn the intent id for the gate; an
    # unclassified phrase carries no channel restriction and passes to the
    # orchestrator, which handles unknown via tier-2.
    intent_id = matcher.longest_match(text)
    if intent_id is not None:
        allowed, code = channel_admission(intent_id, slots, channel)
        if not allowed:
            pubs = [_speak_bytes(_CHANNEL_DENIED_REPLY)] if want_tts else []
            return TextTurnResult(denied=True, code=code, decision=None,
                                  publishes=pubs)

    # Allowed -> run the orchestrator on the TEXT (skips ASR).
    decision = orchestrator.handle_turn(text, session, now_mono_ms)
    pubs: List[Tuple[str, bytes]] = []
    for key, payload in decision_to_publishes(decision):
        # require_tts_reply=false: text/HMI/WeChat callers can suppress the
        # spoken reply (11 S8.7.5) so a typed query does not seize domain 2.
        if key == CMD_AUDIO_SPEAK and not want_tts:
            continue
        data = json.dumps(payload, ensure_ascii=False,
                          separators=(",", ":")).encode("utf-8")
        pubs.append((key, data))
    return TextTurnResult(denied=False, code="", decision=decision,
                          publishes=pubs)
