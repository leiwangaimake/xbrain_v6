"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: gate.py
Brief: BIZ-P2-4 -- AsrGate reason evaluator + rt/audio/gate publisher

Description:
11 S8.9.2 AsrGate carries mic_open (bool) + reason (closed 7-value)
+ reopen_eta_ms (optional).

Reason evaluation is a PRIORITY CHAIN (spec: '7 reasons top-to-
bottom, first hit wins'). Order is:
  1. hes             (hardware estop asserted)
  2. device_fault    (mic hardware / driver failed)
  3. not_configured  (no audio device configured)
  4. b_mode          (in B broadcast mode -- explicit muting)
  5. speaker_active  (holder present in domain 2 -> half-duplex)
  6. tail_hold       (in T_tail window after speaker released)
  7. unknown         (nothing else fired; mic OPEN)

reopen_eta_ms:
  * MUST be present when reason in {speaker_active, tail_hold}
  * MUST be OMITTED (not None, not 0) for the other five reasons

gate_seq (11 S8.9.2 GS-1..GS-3):
  * increments ONLY on state change; a 1 Hz heartbeat republishes
    the same seq (GS-1: heartbeats don't bump seq)
  * starts at 0 at process boot; NOT persisted (GS-3: downstream
    MUST NOT compare across process restart)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


# 11 S8.9.2 reasons in priority order. Priority CANNOT be reordered
# without changing observed behavior; the "device_fault before
# speaker_active" order is what makes "mic fail while speaker
# talking" report device_fault instead of masking it as speaker_active.
_REASON_PRIORITY = (
    "hes",
    "device_fault",
    "not_configured",
    "b_mode",
    "speaker_active",
    "tail_hold",
    "unknown",
)

# Which reasons require reopen_eta_ms in the published message.
_REQUIRES_REOPEN_ETA = frozenset({"speaker_active", "tail_hold"})


@dataclass
class GateInputs:
    """The seven booleans + one integer that the priority chain
    reduces to a single AsrGate reason."""
    hes_asserted: bool
    mic_device_fault: bool
    mic_not_configured: bool
    b_mode_active: bool
    speaker_holder_present: bool
    in_tail_hold: bool
    # For the two "reopen_eta_ms required" reasons, this is the ms
    # from now until mic can reopen (max(0, t_end_est + T_tail - now)).
    reopen_eta_ms: Optional[int] = None


@dataclass(frozen=True)
class AsrGateMessage:
    """The message published on rt/audio/gate (11 S8.9.2 shape)."""
    mic_open: bool
    reason: str
    gate_seq: int
    # reopen_eta_ms is omitted (None) except for {speaker_active, tail_hold}
    reopen_eta_ms: Optional[int] = None


def evaluate_reason(inputs: GateInputs) -> str:
    """Priority chain: return the FIRST reason whose input is truthy.
    Falls to 'unknown' when nothing else fires (mic OPEN case)."""
    checks = [
        ("hes",            inputs.hes_asserted),
        ("device_fault",   inputs.mic_device_fault),
        ("not_configured", inputs.mic_not_configured),
        ("b_mode",         inputs.b_mode_active),
        ("speaker_active", inputs.speaker_holder_present),
        ("tail_hold",      inputs.in_tail_hold),
    ]
    for reason, hit in checks:
        if hit:
            return reason
    return "unknown"


class GatePublisher:
    """Owns the gate_seq counter (per-process) and emits AsrGateMessage.

    Publishing itself is delegated to an injected callable so tests
    do not need a Zenoh session."""

    def __init__(self) -> None:
        # GS-3: seq starts at 0 each boot; NEVER persisted.
        self._gate_seq = 0
        # Last (mic_open, reason) pair so heartbeats can be detected
        # (no state change -> no seq bump).
        self._last_state: Optional[tuple] = None

    def compose(self, inputs: GateInputs) -> AsrGateMessage:
        """Compute the AsrGate message for the current inputs.

        Bumps gate_seq iff (mic_open, reason) changed from last call.
        For reasons other than {speaker_active, tail_hold}, reopen_eta_ms
        is FORCED to None (spec: 'MUST be omitted for the other five')."""
        reason = evaluate_reason(inputs)
        mic_open = (reason == "unknown")
        cur = (mic_open, reason)
        if cur != self._last_state:
            self._gate_seq += 1
            self._last_state = cur
        eta = inputs.reopen_eta_ms if reason in _REQUIRES_REOPEN_ETA else None
        return AsrGateMessage(
            mic_open=mic_open,
            reason=reason,
            gate_seq=self._gate_seq,
            reopen_eta_ms=eta,
        )

    def heartbeat(self) -> AsrGateMessage:
        """Republish the current state without bumping gate_seq.

        Callers on the tx thread call this at 1 Hz (spec: heartbeat
        rate). If no state has been composed yet, returns a default
        'not_configured' message (safe default: closed mic)."""
        if self._last_state is None:
            # Never composed -> default closed with not_configured.
            self._last_state = (False, "not_configured")
            self._gate_seq += 1
        mic_open, reason = self._last_state
        return AsrGateMessage(
            mic_open=mic_open,
            reason=reason,
            gate_seq=self._gate_seq,
        )

    @property
    def gate_seq(self) -> int:
        return self._gate_seq
