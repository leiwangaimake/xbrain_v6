"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: machine.py
Brief: BIZ-P3-7 task state machine (11 S4.4 12-value + suspend_kind/reason)

Description:
Task lifecycle over the 12-value lowercase task_state closed set from
11 S4.4. This module owns the TRANSITION GRAPH; the VALUE set itself is
NOT redefined here -- it is imported from common.enums (the single frozen
source loaded from common/enums/sets.yaml). Duplicating the value list as a
local literal is exactly the drift this file used to carry: an earlier
version hard-coded a DIFFERENT 12 values (queued/starting/completing/...),
matching neither 11 S4.4 nor sets.yaml, with no mapping -- so the cloud/HMI
that speak 11 S4.4 could not reconstruct the state (TSK-11).

States (11 S4.4, grouped 15 S3.2):
  waiting  : pending  scheduled  blocked
  active   : ready    running    suspended
  terminal : done  failed  cancelled  needs_review  interrupted  wait_for_power_off

The graph is the SOURCE OF TRUTH -- every legal arrow is listed; anything not
listed is REJECTED (raises InvalidTransition), matching the CLAUDE.md 3.5 rule
that closed-set outputs must not be silently transited. Shape (15 S3.2):

  pending --validate_ok--------> ready
          --validate_defer_time-> scheduled     (timed task, not_before_ts)
          --validate_defer_dep--> blocked       (dependency unmet)
          --validate_fail-------> failed        (rejected at admission; the
                                                  15 S3.2 '拒绝' sink, persisted
                                                  as a terminal row for audit +
                                                  90-day dedup, not left pending)
  scheduled --due-------------->  ready
  blocked   --unblock--------->   ready
  ready     --dispatch-------->   running
  running   --complete-------->   done
            --fail------------->   failed
            --suspend---------->   suspended     (kind passive|yielding)
  suspended --resume---------->   ready          (passive: cond met / operator;
                                                  yielding: yielded-to task done)
            --review----------->   needs_review  (suspend timeout / resume
                                                  re-validate failed; 15 S3.4 teach)
  <any active/waiting> --cancel-> cancelled
  <terminal> --await_power_off-> wait_for_power_off   (11 S7.15.7 shutdown of a
                                                  terminal-but-not-uplinked task)

interrupted has NO producer this period (15 S3.2 v0.4): it stays a member of
the closed set (so a historical row still decodes) but no arrow emits it.

suspend_kind / suspend_reason (11 S4.4, both from common.enums):
  * required exactly when state == 'suspended', forbidden otherwise;
  * kind in {passive, yielding};
  * reason PRODUCER set = suspend_reason closed set MINUS 'energy_unreachable'
    (CR-6, 15 S9.5): energy_unreachable is a closed-set MEMBER with no producer
    and is rejected at the tasks CHECK, so the machine must never emit it either
    (else a machine-accepted value fails the DDL INSERT = fail-silent);
  * CR-8 pairing: kind=='yielding' IFF reason in {preempted, mode_takeover};
    every other reason is passive. The two CHECKs individually admit the
    fail-silent combo passive+preempted (a preempted task written passive never
    re-enters the yielding auto-resume scan), so the pairing is enforced here.
"""

from __future__ import annotations

from dataclasses import dataclass

from xbrain.common.enums import SUSPEND_KIND, SUSPEND_REASON, TASK_STATE


# The 12-value closed set, taken from the single source (NOT re-listed). A
# frozenset for O(1) membership; the graph below is what constrains order.
TASK_STATES = frozenset(TASK_STATE.values)

# suspend_kind closed set (passive, yielding) from the same source.
SUSPEND_KINDS = frozenset(SUSPEND_KIND.values)

# suspend_reason values a transition may PRODUCE. This is the sets.yaml closed
# set minus 'energy_unreachable' -- see the module docstring (CR-6). Deriving it
# by subtraction (not a second literal) keeps it honest if 11 S4.4 grows a
# reason: the new value flows in automatically and only the one deliberate hole
# is spelled out here.
_ENERGY_UNREACHABLE = "energy_unreachable"
SUSPEND_REASONS = frozenset(SUSPEND_REASON.values) - {_ENERGY_UNREACHABLE}

# CR-8: the two reasons that pair with kind 'yielding'. Every other reason is
# 'passive'. (11 S4.4 suspend_reason table, suspend_kind column.)
_YIELDING_REASONS = frozenset({"preempted", "mode_takeover"})


# Grouping (15 S3.2). 'active' = the states a scheduler/geo-linkage treats as
# occupying the robot; used by callers instead of a NOT-IN-terminal test so a
# future state is never silently counted active.
WAITING_STATES = frozenset({"pending", "scheduled", "blocked"})
ACTIVE_STATES = frozenset({"ready", "running", "suspended"})
TERMINAL_STATES = frozenset({
    "done", "failed", "cancelled", "needs_review", "interrupted",
    "wait_for_power_off",
})


# Transition graph: (from_state, event) -> to_state. Exhaustive; a miss raises.
TRANSITIONS: dict = {
    # -- admission / validation (from pending, 15 S3.3 insert-then-validate) --
    ("pending",   "validate_ok"):         "ready",
    ("pending",   "validate_defer_time"): "scheduled",
    ("pending",   "validate_defer_dep"):  "blocked",
    ("pending",   "validate_fail"):       "failed",
    # -- waiting -> ready --
    ("scheduled", "due"):     "ready",
    ("scheduled", "block"):   "blocked",       # a dependency appears before due
    ("blocked",   "unblock"): "ready",
    # -- ready -> running --
    ("ready",     "dispatch"): "running",
    ("ready",     "suspend"):  "suspended",     # estop while queued (active set)
    # -- running --
    ("running",   "complete"): "done",
    ("running",   "fail"):     "failed",
    ("running",   "suspend"):  "suspended",
    # -- suspended --
    ("suspended", "resume"):   "ready",
    ("suspended", "review"):   "needs_review",
    ("suspended", "fail"):     "failed",         # resume gave up (non-review)
    # -- cancel from any waiting/active state --
    ("pending",   "cancel"):   "cancelled",
    ("scheduled", "cancel"):   "cancelled",
    ("blocked",   "cancel"):   "cancelled",
    ("ready",     "cancel"):   "cancelled",
    ("running",   "cancel"):   "cancelled",
    ("suspended", "cancel"):   "cancelled",
    # -- shutdown of a terminal-but-not-uplinked task (11 S7.15.7) --
    ("done",      "await_power_off"):      "wait_for_power_off",
    ("failed",    "await_power_off"):      "wait_for_power_off",
    ("cancelled", "await_power_off"):      "wait_for_power_off",
    ("needs_review", "await_power_off"):   "wait_for_power_off",
}


class InvalidTransition(Exception):
    """Rejected state transition (not in the TRANSITIONS graph)."""


@dataclass(frozen=True)
class TransitionResult:
    from_state: str
    to_state: str
    idempotent: bool


def apply_transition(from_state: str, event: str) -> TransitionResult:
    """Look up the next state. Raises InvalidTransition on an unknown
    from_state or an arrow not in the graph.

    Idempotency (T-3, 15 S7 / TSK-12): re-applying an event whose target is the
    state we are already in is a legal no-op (retry-on-crash safety), reported
    idempotent=True. This is keyed on the TARGET, so e.g. a second 'complete'
    on a task already 'done' is absorbed, while 'complete' on 'running' the
    first time is a real transition."""
    if from_state not in TASK_STATES:
        raise InvalidTransition(f"unknown from_state {from_state!r}")
    to = TRANSITIONS.get((from_state, event))
    if to is None:
        # The event may name (or map to) the state we are already in; if so it
        # is an idempotent replay, not an error. We check by scanning for any
        # arrow on this event whose target == from_state.
        for (fs, ev), ts in TRANSITIONS.items():
            if ev == event and ts == from_state:
                return TransitionResult(from_state, from_state, idempotent=True)
        raise InvalidTransition(
            f"no transition from {from_state!r} on {event!r}")
    return TransitionResult(from_state, to, idempotent=False)


def validate_suspend_fields(state: str,
                            suspend_kind: str | None,
                            suspend_reason: str | None) -> None:
    """Enforce the 11 S4.4 suspend field rules (matches the tasks DDL CHECKs).

    * suspend_kind / suspend_reason are non-null IFF state == 'suspended';
    * kind in {passive, yielding}; reason in the PRODUCER set (no
      energy_unreachable, CR-6);
    * CR-8 pairing: kind == 'yielding' IFF reason in {preempted, mode_takeover}.
    Raises InvalidTransition on any violation (never silently coerces)."""
    is_suspended = state == "suspended"
    has_kind = bool(suspend_kind)
    has_reason = bool(suspend_reason)
    # Paired with the suspended state (both present, or both absent).
    if is_suspended != has_kind:
        raise InvalidTransition(
            f"state={state!r} requires suspend_kind present iff suspended")
    if is_suspended != has_reason:
        raise InvalidTransition(
            f"state={state!r} requires suspend_reason present iff suspended")
    if not is_suspended:
        return
    if suspend_kind not in SUSPEND_KINDS:
        raise InvalidTransition(f"unknown suspend_kind {suspend_kind!r}")
    if suspend_reason not in SUSPEND_REASONS:
        raise InvalidTransition(f"unknown suspend_reason {suspend_reason!r}")
    # CR-8: the kind and reason must agree on yielding-vs-passive.
    reason_is_yielding = suspend_reason in _YIELDING_REASONS
    kind_is_yielding = suspend_kind == "yielding"
    if kind_is_yielding != reason_is_yielding:
        raise InvalidTransition(
            f"suspend_kind {suspend_kind!r} does not pair with "
            f"suspend_reason {suspend_reason!r} (CR-8)")
