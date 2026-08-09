"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: machine.py
Brief: BIZ-P3-7 task state machine (11 §4.4 12-value + suspend_kind/reason)

Description:
Task lifecycle uses the 12-value lowercase closed set from 11 §4.4.
The transition table below is the SOURCE OF TRUTH; every arrow is
listed. Anything not listed is REJECTED (returns None), matching
the CLAUDE.md 3.5 rule that closed-set outputs must not be silently
transited.

suspend_kind and suspend_reason are ORTHOGONAL fields:
  suspend_kind:   one of {'estop', 'user', 'system', 'low_battery',
                          'network_lost', 'unknown'}
  suspend_reason: free text within a 512-byte cap
Only 'suspended' state permits non-empty suspend_kind. Every other
state must have suspend_kind == '' or the transition is rejected
(T-3 idempotency has a separate check below).
"""

from __future__ import annotations

from dataclasses import dataclass


TASK_STATES = frozenset({
    "pending", "queued", "starting", "running", "suspended",
    "resuming", "completing", "completed", "cancelling", "cancelled",
    "failed", "aborted",
})

SUSPEND_KINDS = frozenset({
    "estop", "user", "system", "low_battery", "network_lost", "unknown",
})


# Transition table: (from_state, event) -> to_state
# Events are the exhaustive set (T-1..T-12 in 15 S7).
TRANSITIONS: dict = {
    ("pending",    "admit"):     "queued",
    ("pending",    "cancel"):    "cancelled",
    ("queued",     "dispatch"):  "starting",
    ("queued",     "cancel"):    "cancelled",
    ("starting",   "started"):   "running",
    ("starting",   "abort"):     "aborted",
    ("running",    "suspend"):   "suspended",
    ("running",    "complete"):  "completing",
    ("running",    "fail"):      "failed",
    ("running",    "abort"):     "aborted",
    ("running",    "cancel"):    "cancelling",
    ("suspended",  "resume"):    "resuming",
    ("suspended",  "cancel"):    "cancelled",
    ("suspended",  "abort"):     "aborted",
    ("resuming",   "resumed"):   "running",
    ("resuming",   "abort"):     "aborted",
    ("completing", "committed"): "completed",
    ("completing", "abort"):     "aborted",
    ("cancelling", "cancelled_ack"): "cancelled",
}


TERMINAL_STATES = frozenset({"completed", "cancelled", "failed", "aborted"})


class InvalidTransition(Exception):
    """Rejected state transition (not in the TRANSITIONS table)."""


@dataclass(frozen=True)
class TransitionResult:
    from_state: str
    to_state: str
    idempotent: bool


def apply_transition(from_state: str, event: str) -> TransitionResult:
    """Look up the next state. Raises InvalidTransition on miss.
    Idempotency: applying an event whose target equals the current
    state is legal (T-3) and reported as idempotent=True.
    Rationale: retry-on-crash needs to be safe."""
    if from_state not in TASK_STATES:
        raise InvalidTransition(f"unknown from_state {from_state!r}")
    to = TRANSITIONS.get((from_state, event))
    if to is None:
        # T-3 idempotency: some events name the terminal state
        # they lead to; if we're already there, no-op.
        if event in TERMINAL_STATES and from_state == event:
            return TransitionResult(from_state, from_state, idempotent=True)
        raise InvalidTransition(
            f"no transition from {from_state!r} on {event!r}")
    return TransitionResult(from_state, to, idempotent=False)


def validate_suspend_fields(state: str, suspend_kind: str) -> None:
    """Only 'suspended' state may carry a non-empty suspend_kind."""
    if suspend_kind and suspend_kind not in SUSPEND_KINDS:
        raise InvalidTransition(f"unknown suspend_kind {suspend_kind!r}")
    if state == "suspended" and not suspend_kind:
        raise InvalidTransition(
            "state='suspended' requires non-empty suspend_kind")
    if state != "suspended" and suspend_kind:
        raise InvalidTransition(
            f"state={state!r} must have empty suspend_kind")
