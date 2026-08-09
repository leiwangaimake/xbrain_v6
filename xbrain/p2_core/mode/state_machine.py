"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: state_machine.py
Brief: BIZ-P2-11 -- 3-top-state mode SM + DIALOG A/C/E slots

Description:
14 S5 mode state machine. Three top-level states:
  IDLE / PATROL     (default; mode.device -> func1)
  DIALOG            (has slot: A local | C cloud | E wecom; all -> func1)
  BROADCAST         (B mode; device -> func2)
  ALARM             (D mode; device -> deter)

Transition invariants (14 S5, 11 S7A):
  * P-1' preflight: check all 4 target domains free / preemptible
    BEFORE any transition begins. Failure -> report ALL blocked
    domains in blocked[] (NOT short-circuit).
  * Atomicity: switch acquires target domains in switch_order sequence
    (device_mode, payload_light, ptz, motion, audio); any failure
    triggers full rollback of everything already acquired.
  * D <-> B: only domain 2 (speaker) changes holder. domains 3, 4, 5
    gen must NOT advance through this transition.
  * min_dwell_s: applies to `autonomous` triggers only; cmd / timeout
    / safety triggers are exempt (MD-1).

★ This module owns the STATE MACHINE (states, transitions, preflight
result). It does NOT own the actual acquire/release calls (those are
BIZ-P2-12 two-layer mapper + BIZ-P2-16 D-mode sequencer).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import FrozenSet, List, Optional


class ModeState(str, Enum):
    """3 top-level states + 3 DIALOG dialects."""
    IDLE = "idle"
    DIALOG_A = "dialog_a"       # local mic ASR
    DIALOG_C = "dialog_c"       # cloud ASR
    DIALOG_E = "dialog_e"       # wecom ASR
    BROADCAST = "broadcast"
    ALARM = "alarm"


class TriggerKind(str, Enum):
    """MD-1: only autonomous triggers respect min_dwell_s."""
    AUTONOMOUS = "autonomous"     # rule engine self-initiated
    CMD = "cmd"                   # explicit cmd/mode
    TIMEOUT = "timeout"           # b_cast_max_duration_s expiry
    SAFETY = "safety"             # health downgrade / hes / soft_estop


@dataclass(frozen=True)
class TransitionRequest:
    """One requested mode transition."""
    to_state: ModeState
    trigger: TriggerKind
    cmd_id: str = ""              # for cmd triggers; idempotency key


@dataclass(frozen=True)
class TransitionResult:
    """The outcome of preflight + attempted transition."""
    accepted: bool
    from_state: ModeState
    to_state: ModeState
    # Blocked / self_held domains (14 S5.5 P-1' + BIZ-P2-11 spec).
    # Both are ALWAYS populated (empty tuple on success). NEVER
    # short-circuited: if two domains are blocked, both are named.
    blocked: FrozenSet[str] = field(default_factory=frozenset)
    self_held: FrozenSet[str] = field(default_factory=frozenset)
    # Reason for rejection (empty on accept). Closed-set values.
    reason: str = ""
    # Idempotency: if a duplicate cmd_id arrived, this is the same
    # result the first call returned.
    is_replay: bool = False


class ModeStateMachine:
    """The mode SM. Consumed by P2's main loop; not thread-safe by
    design (main-thread only)."""

    def __init__(self, initial: ModeState = ModeState.IDLE) -> None:
        self._state = initial
        # cmd_id -> TransitionResult, for idempotency (BIZ-P2-11 spec:
        # duplicate cmd_id returns first result, does NOT re-execute).
        self._cmd_history: dict = {}

    @property
    def state(self) -> ModeState:
        return self._state

    def request(self, req: TransitionRequest,
                blocked_domains: Optional[FrozenSet[str]] = None,
                self_held_domains: Optional[FrozenSet[str]] = None,
                dwell_ok: bool = True) -> TransitionResult:
        """Attempt the transition.

        Args:
            req: the requested transition.
            blocked_domains: domains held by non-preemptible sources
                (from arbiter snapshot). If non-empty, transition
                fails and blocked[] is populated.
            self_held_domains: domains our own process already holds
                that need to release_for_switch first (14 CR-1).
            dwell_ok: whether min_dwell_s permits this transition
                (caller computes; SM only checks trigger exemption).
        """
        # Idempotency: replay if this cmd_id already ran.
        if req.cmd_id and req.cmd_id in self._cmd_history:
            replay = self._cmd_history[req.cmd_id]
            # Mark the replay flag on the returned object.
            return TransitionResult(
                accepted=replay.accepted,
                from_state=replay.from_state,
                to_state=replay.to_state,
                blocked=replay.blocked,
                self_held=replay.self_held,
                reason=replay.reason,
                is_replay=True,
            )

        blocked = blocked_domains or frozenset()
        self_held = self_held_domains or frozenset()

        # Target == current state: accepted BUT no work done, no event.
        if req.to_state == self._state:
            result = TransitionResult(
                accepted=True,
                from_state=self._state,
                to_state=self._state,
                reason="already_in_target_state",
            )
            if req.cmd_id:
                self._cmd_history[req.cmd_id] = result
            return result

        # min_dwell_s check: only autonomous triggers respect it (MD-1).
        if req.trigger == TriggerKind.AUTONOMOUS and not dwell_ok:
            result = TransitionResult(
                accepted=False, from_state=self._state,
                to_state=req.to_state,
                reason="min_dwell_not_met",
            )
            if req.cmd_id:
                self._cmd_history[req.cmd_id] = result
            return result

        # P-1' preflight: blocked domains report ALL of them.
        # Do NOT short-circuit on the first blocked; the spec is
        # explicit that blocked[] must name every conflict.
        if blocked:
            result = TransitionResult(
                accepted=False, from_state=self._state,
                to_state=req.to_state,
                blocked=blocked,
                self_held=self_held,
                reason="preflight_blocked",
            )
            if req.cmd_id:
                self._cmd_history[req.cmd_id] = result
            return result

        # OK path: commit the transition.
        old = self._state
        self._state = req.to_state
        result = TransitionResult(
            accepted=True, from_state=old, to_state=req.to_state,
            self_held=self_held,
            reason="committed",
        )
        if req.cmd_id:
            self._cmd_history[req.cmd_id] = result
        return result

    def rollback(self, from_state: ModeState) -> None:
        """Set state back after a partial commit failure. Called by
        BIZ-P2-12 mapper when an acquire in switch_order failed
        after previous ones succeeded."""
        self._state = from_state
