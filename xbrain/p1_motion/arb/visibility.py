"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: visibility.py
Brief: CHK-1-17 域① 对外可见性 (state/arb/motion + event/arbitration + dedup_key)

Description:
11 §7A.8 domain-1 (p1_motion) arbitration MUST publish TWO keys:
  * P1-22  state/arb/motion              (1 Hz + change-triggered)
  * P1-23  event/{severity}/arbitration  (dedup_key = arb:motion:{action},
                                            10s coalesce window)

Discipline (§7A.8 table body verbatim):
  * publish req/grant   -- NO  (domain-1 does not accept external req)
  * publish state       -- MUST
  * publish audit event -- MUST

  ** BOTH must ship together, not just one. Only shipping state
     without the audit event is half-done -- the intent of §7A.8
     is that a winner change is BOTH visible in state AND named
     in the audit log.

Semantics:
  * gen++ only on winner change (winner_task_id or winner_source
    changes). A stable holder for 10s must NOT bump gen every
    tick; gen incrementing every 20 Hz control-loop tick would
    make downstream diff-based observers churn.
  * change-triggered publish fires on: winner change OR gen bump.
    A pure heartbeat at 1 Hz is the fallback so a subscriber knows
    the process is alive.
  * dedup_key = "arb:motion:{action}" so repeat events in a
    10-second window coalesce into a single audit entry with a
    counter.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


ARB_MOTION_STATE_KEY = "state/arb/motion"
ARB_MOTION_EVENT_KEY_TEMPLATE = "event/{severity}/arbitration"
DEDUP_WINDOW_MS = 10_000
HEARTBEAT_PERIOD_MS = 1000


@dataclass(frozen=True)
class ArbState:
    """Field set aligned with 11 §7A.5 state/arb/{domain} row.

    Kept a small dataclass so a schema-diff meta-test can enumerate
    fields via dataclass introspection rather than trusting a
    hand-written second list."""
    domain: str            # 'motion'
    winner_source: str
    winner_task_id: str
    gen: int
    since_mono_ms: int


@dataclass(frozen=True)
class ArbEvent:
    """audit event for a domain-1 winner change (§7A.7)."""
    severity: str          # 'info' / 'warn' / 'error'
    dedup_key: str         # 'arb:motion:{action}'
    action: str            # e.g. 'winner_change' / 'preempted'
    from_source: str
    to_source: str
    from_task_id: str
    to_task_id: str
    mono_ms: int


def dedup_key_for(action: str) -> str:
    """Compose the 10s coalesce dedup_key. Only ONE composition
    site -- so a change in the shape happens in ONE place."""
    return f"arb:motion:{action}"


class ArbPublisher:
    """State machine that decides whether to publish state and/or
    event this tick. Callers hand it the CURRENT winner + last
    publish timestamps; it returns tuples describing what to send."""

    def __init__(self) -> None:
        self._last_winner_source: str = ""
        self._last_winner_task_id: str = ""
        self._gen: int = 0
        self._last_state_publish_ms: int = 0

    def observe(self, winner_source: str, winner_task_id: str,
                now_mono_ms: int) -> tuple:
        """Return (state_to_publish_or_None, event_to_publish_or_None).

        state fires on: winner change OR >= HEARTBEAT_PERIOD_MS since
                        last publish
        event fires on: winner change only (heartbeat produces no
                        audit event -- would flood the log)"""
        winner_changed = (
            winner_source != self._last_winner_source
            or winner_task_id != self._last_winner_task_id
        )
        heartbeat_due = (
            now_mono_ms - self._last_state_publish_ms
            >= HEARTBEAT_PERIOD_MS
        )
        if winner_changed:
            # gen bumps ONLY on winner change (never on heartbeat).
            self._gen += 1
            event = ArbEvent(
                severity="info",
                dedup_key=dedup_key_for("winner_change"),
                action="winner_change",
                from_source=self._last_winner_source,
                to_source=winner_source,
                from_task_id=self._last_winner_task_id,
                to_task_id=winner_task_id,
                mono_ms=now_mono_ms)
            self._last_winner_source = winner_source
            self._last_winner_task_id = winner_task_id
        else:
            event = None
        state = None
        if winner_changed or heartbeat_due:
            state = ArbState(
                domain="motion",
                winner_source=self._last_winner_source,
                winner_task_id=self._last_winner_task_id,
                gen=self._gen,
                since_mono_ms=now_mono_ms)
            self._last_state_publish_ms = now_mono_ms
        return state, event

    @property
    def gen(self) -> int:
        return self._gen
