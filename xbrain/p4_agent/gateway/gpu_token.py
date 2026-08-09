"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: gpu_token.py
Brief: GWY-P4-02 -- GPU admission token (domain 6) + circuit breaker

Description:
16 S9: LLM invocations compete for a single GPU token (domain 6 in
the arbiter). Only ONE outstanding LLM call at a time; a second
request must WAIT or be denied.

Circuit breaker (16 S9): 3 consecutive LLM timeouts -> circuit
opens for 60 s. In open state EVERY LLM request is denied
immediately with E_UNAVAILABLE + TTS explicitly tells the user.

* 'silent circuit break' is a variant explicitly banned by the
spec: the operator MUST hear a message. So the return from an open
circuit has 'must_tts' true and a canned text.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


class CircuitState(str):
    CLOSED = "closed"       # normal; requests admitted
    OPEN = "open"           # tripped; all requests denied
    HALF_OPEN = "half_open" # trial: single request will be admitted


@dataclass
class GpuTokenState:
    """Domain 6 single-slot admission + circuit breaker."""
    slot_taken: bool = False
    consecutive_timeouts: int = 0
    timeouts_before_open: int = 3
    open_since_millis: Optional[int] = None
    open_duration_millis: int = 60_000

    def circuit_state(self, now_mono_ms: int) -> str:
        if self.open_since_millis is None:
            return CircuitState.CLOSED
        elapsed = now_mono_ms - self.open_since_millis
        if elapsed >= self.open_duration_millis:
            return CircuitState.HALF_OPEN
        return CircuitState.OPEN


@dataclass(frozen=True)
class AdmissionResult:
    admitted: bool
    reason: str = ""
    must_tts: bool = False
    tts_text: str = ""


def try_admit(state: GpuTokenState, now_mono_ms: int) -> AdmissionResult:
    """Try to acquire the GPU slot.

      * Circuit OPEN -> denied + must_tts
      * Slot taken -> denied (busy)
      * Otherwise -> take slot, return admitted
    """
    cs = state.circuit_state(now_mono_ms)
    if cs == CircuitState.OPEN:
        return AdmissionResult(
            admitted=False,
            reason="circuit_open",
            must_tts=True,
            tts_text="AI service is temporarily unavailable, please retry",
        )
    if state.slot_taken:
        return AdmissionResult(
            admitted=False, reason="slot_taken",
        )
    state.slot_taken = True
    return AdmissionResult(admitted=True)


def release(state: GpuTokenState, success: bool,
            now_mono_ms: int) -> None:
    """Called after LLM call completes (or errors). success=True
    resets consecutive_timeouts; success=False (timeout) increments.
    On timeouts >= threshold: circuit opens."""
    state.slot_taken = False
    if success:
        # Success closes half-open circuit; success in closed does nothing.
        state.consecutive_timeouts = 0
        state.open_since_millis = None
    else:
        state.consecutive_timeouts += 1
        if state.consecutive_timeouts >= state.timeouts_before_open \
                and state.open_since_millis is None:
            state.open_since_millis = now_mono_ms
