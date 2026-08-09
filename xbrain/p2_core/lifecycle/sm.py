"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: sm.py
Brief: BIZ-P2-22 -- P2 lifecycle state machine (INIT->...->RUNNING)

Description:
14 S9 P2 lifecycle state machine (6 states):

  INIT       process just started; config not yet validated
  WAIT_BIT   quick BIT scheduled but not complete
  EVALUATE   quick BIT complete; deciding factor granting policy
  BLOCKED    quick BIT had fatal-fail OR main-loop over-budget 3x
  GRANT      publishing cmd/motion/factor{allow_motion=true, ...}
  RUNNING    steady state; main loop ticks + audits

Transitions (14 S9 verbatim):
  INIT -> WAIT_BIT     : config OK, BIT scheduled
  WAIT_BIT -> EVALUATE : quick BIT reported (any result)
  EVALUATE -> GRANT    : no fatal-fail
  EVALUATE -> BLOCKED  : fatal-fail present (BIT-33)
  GRANT -> RUNNING     : first factor publish successful
  RUNNING -> BLOCKED   : main-loop 3x over budget (BIZ-P2-1)
  BLOCKED -> EVALUATE  : recovery signal (operator ack + BIT re-run)

Stage 4 release (10 S3.3) = the GRANT publish. p2_core is the sole
executor of Stage 4 (spec: NOT systemd).
"""

from __future__ import annotations

from enum import Enum
from typing import Optional


class LifecycleState(str, Enum):
    INIT = "init"
    WAIT_BIT = "wait_bit"
    EVALUATE = "evaluate"
    BLOCKED = "blocked"
    GRANT = "grant"
    RUNNING = "running"


# Legal transitions. Enforced by transition().
_LEGAL_TRANSITIONS = {
    LifecycleState.INIT: {LifecycleState.WAIT_BIT},
    LifecycleState.WAIT_BIT: {LifecycleState.EVALUATE},
    LifecycleState.EVALUATE: {LifecycleState.GRANT, LifecycleState.BLOCKED},
    LifecycleState.GRANT: {LifecycleState.RUNNING},
    LifecycleState.RUNNING: {LifecycleState.BLOCKED},
    LifecycleState.BLOCKED: {LifecycleState.EVALUATE},
}


class IllegalTransition(RuntimeError):
    pass


class LifecycleSM:
    """P2 lifecycle. Single instance per process."""

    def __init__(self, initial: LifecycleState = LifecycleState.INIT) -> None:
        self._state = initial

    @property
    def state(self) -> LifecycleState:
        return self._state

    def transition(self, to: LifecycleState) -> None:
        """Move to `to`. Raise on illegal transition. There is
        deliberately NO 'force' escape hatch (CLAUDE.md 3.6)."""
        allowed = _LEGAL_TRANSITIONS.get(self._state, set())
        if to not in allowed:
            raise IllegalTransition(
                "cannot transition %s -> %s (legal from %s: %s)"
                % (self._state.value, to.value, self._state.value,
                   sorted(s.value for s in allowed)))
        self._state = to

    def can_release_stage_4(self) -> bool:
        """Stage 4 release only from GRANT (which then transitions to
        RUNNING). NOT from RUNNING (which means we're past release)."""
        return self._state == LifecycleState.GRANT

    def is_operational(self) -> bool:
        """RUNNING is the only truly-operational state; GRANT is the
        moment of publishing the first factor."""
        return self._state == LifecycleState.RUNNING
