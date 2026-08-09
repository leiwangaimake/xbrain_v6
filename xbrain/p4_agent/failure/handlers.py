"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: handlers.py
Brief: GWY-P4-19 -- failure handling (GATE-*/ESTOP-*/DIRFREE-1/SPEAK-1/LIMITER-1)

Description:
16 S13 named failure handlers. Each is a specific fallback for a
specific failure mode.

  GATE-1  rt/audio/gate publisher silent > 1s -> assume closed, drop ASR
  GATE-2  gate reason=hes -> refuse ALL voice commands (even estop-echo)
  ESTOP-1 safety-bypass hit but arbiter down -> emit local WAV siren
  ESTOP-2 estop dispatch failed to reach chassis -> retry x3 + fault
  DIRFREE-1 directional-check failed but user IS in earshot ->
            still no-reply (asymmetric cost per 16 S5.2.1)
  SPEAK-1 payload TTS failed -> attempt local WAV fallback
  LIMITER-1 per-min TTS count exceeded -> defer excess to warn+drop
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class FailureCode(str, Enum):
    GATE_1_HEARTBEAT_LOST = "GATE-1"
    GATE_2_HES_ASSERTED = "GATE-2"
    ESTOP_1_ARBITER_DOWN = "ESTOP-1"
    ESTOP_2_DISPATCH_FAILED = "ESTOP-2"
    DIRFREE_1_UNDIRECTED = "DIRFREE-1"
    SPEAK_1_TTS_FAILED = "SPEAK-1"
    LIMITER_1_TTS_QUOTA_EXCEEDED = "LIMITER-1"


@dataclass(frozen=True)
class HandlerResult:
    action: str          # e.g., 'drop', 'local_wav', 'retry', 'defer'
    detail: str = ""


def handle_gate1_heartbeat_lost() -> HandlerResult:
    """GATE-1: assume mic closed, drop the ASR result for this turn."""
    return HandlerResult(
        action="drop", detail="rt/audio/gate silent > 1s; mic assumed closed")


def handle_gate2_hes() -> HandlerResult:
    """GATE-2: HES asserted; ALL voice commands refused, including
    the estop-echo path (already-stopped robot doesn't need another
    stop, and voice-estop while HES is dangerous confusion)."""
    return HandlerResult(
        action="drop", detail="HES asserted; voice channel disabled")


def handle_estop1_arbiter_down() -> HandlerResult:
    """ESTOP-1: bypass matched but arbiter is unreachable.
    Emit local WAV siren as audible confirmation the robot heard,
    even though it can't send arbitration commands."""
    return HandlerResult(
        action="local_wav", detail="siren.wav (arbiter down)")


def handle_estop2_dispatch_failed(retry_count: int) -> HandlerResult:
    """ESTOP-2: retry cmd/estop up to 3 times before falling to fault."""
    if retry_count < 3:
        return HandlerResult(action="retry",
                              detail="retry #%d" % (retry_count + 1))
    return HandlerResult(
        action="fault", detail="3 retries exhausted; escalate to operator")


def handle_speak1_tts_failed() -> HandlerResult:
    """SPEAK-1: payload TTS failed; local WAV fallback if the
    canned message exists for this intent."""
    return HandlerResult(
        action="local_wav", detail="fallback tone (TTS unavailable)")


@dataclass
class TtsQuotaLimiter:
    """LIMITER-1: per-minute TTS count cap."""
    per_minute_cap: int
    _bucket: list = None   # type: ignore[assignment]

    def __post_init__(self) -> None:
        self._bucket = []

    def try_emit(self, now_mono_ms: int) -> HandlerResult:
        # Drop timestamps > 60_000 ms old.
        cutoff = now_mono_ms - 60_000
        self._bucket = [t for t in self._bucket if t >= cutoff]
        if len(self._bucket) >= self.per_minute_cap:
            return HandlerResult(
                action="defer",
                detail="LIMITER-1: per-min quota %d reached; TTS deferred"
                       % self.per_minute_cap)
        self._bucket.append(now_mono_ms)
        return HandlerResult(action="emit")
