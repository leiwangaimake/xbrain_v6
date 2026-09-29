"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: arb_state.py
Brief: domain-1 arbitration visibility -- state/arb/motion (11 S7A.5.1) and event/{sev}/arbitration (11 S7A.7)

Description:
11 S7A.8 makes domain 1 (p1_motion's behaviour-source arbiter) a process-
internal special case with ONE non-negotiable: it publishes no req/grant, but
it MUST publish the state and the audit event, in the SAME outward schema as
the six other domains (ARB-0) -- P1-22 state/arb/motion, P1-23
event/{severity}/arbitration. This module builds both from what the P1
arbiter already knows each tick; it is pure (no clock, no session) so the
gen / heartbeat / edge rules are unit-testable.

Mapping of the 7A.5.1 fields onto domain 1 (7A.8 row by row):
  holder        the winner of this tick's re-selection ({source_id, priority,
                since_mono_ms, held_ms, req_id, note}); req_id is null -- there
                are no requests in domain 1, the source is re-selected every
                tick, and a fabricated id would be exactly the kind of "looks
                populated" value 11 S3.1 warns about. Never null itself: hold
                (100) is always alive (12 S4.2), so the domain is never idle.
  gen           +1 ONLY when the winner changes (7A.8 verbatim), never per tick
  suspended     null | "soft_estop" from the P1-21 latch (7A.6.5; hes /
                cmd_timeout come from RobotState, not wired -- E-2 gap noted
                in NEXT.md)
  waiting[]     always empty: highest active priority wins at once
  last_change   {action, from, to, reason, forced=false, mono_ms}: preempt
                (higher priority took over), release (the holder went inactive
                and a lower source won), suspend / rearm (estop edge)
  sources[]     every 12 S4.2 source with priority, policy "immediate" (the
                re-selection has no wait_atomic form) and alive == active
Cadence: state on every change plus a 1 Hz heartbeat (11 S1.1.6 P1-22 "1 Hz
+ 变更即发"); events on changes only (a heartbeat event would flood the
audit log), dedup_key arb:motion:{action} composed in ONE place
(arb/visibility.dedup_key_for).

Dedup: 7A.7 gives the key a 10 s window and lists five actions that are
NEVER merged; two of them, suspend and rearm, are actions domain 1 emits.
Both halves come from xbrain/common/arbiter/audit.py -- the same module the
p5 side renders audit events with -- rather than being spelled again here,
because a second copy of a contract table is a second thing to keep in sync.

*** Why the window has to be ON THE EVENT and not left to the consumer.
p5's record_dao._try_merge takes the latest still-open row with this
dedup_key and, when NEITHER the event NOR that row carries a window, merges
unconditionally. So an audit event published with a key and no window means
the first arbitration of the process life opens a row and every later one --
every rns_avoid <-> path_follow handover, which 7A.7 itself calls the
largest event source in the system -- folds into it silently. The stream
looks implemented and carries one row.

What it does NOT do: no publishing (runtime/nav_wiring.py), no arbitration
(sources/arbiter_p1.py decides; this only reports), no ArbSummary (that is
p5's aggregation, 7A.5.2).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from xbrain.common.arbiter.audit import DEDUP_EXEMPT, DEDUP_WINDOW_S, severity_of
from xbrain.common.arbiter.model import ArbAction
from xbrain.p1_motion.arb.visibility import HEARTBEAT_PERIOD_MS, dedup_key_for
from xbrain.p1_motion.sources.arbiter_p1 import BehaviorSource, SourceState, priority_of

DOMAIN = "motion"
POLICY = "immediate"                 # 7A.5.1 sources[].policy for a re-selected domain
ACTION_GRANT = "grant"
ACTION_PREEMPT = "preempt"
ACTION_RELEASE = "release"
ACTION_SUSPEND = "suspend"
ACTION_REARM = "rearm"
SUSPENDED_SOFT_ESTOP = "soft_estop"  # 7A.6.5 suspended value for the P1-21 latch


def _dedup_window_for(action: str) -> int:
    """7A.7 window for one action, in seconds.

    0 for an action on the never-merge list (of the five, domain 1 emits
    suspend and rearm), DEDUP_WINDOW_S otherwise. 0 rather than "no key":
    the shared renderer keeps the key on every event and sets the window to
    0, so the two producers of this stream agree, and a reader can still
    group by key. With a rising ts the merge test (ts - last_ts) > 0 then
    passes for every later event, i.e. nothing is ever collapsed -- which is
    what 不去重 asks for. Collapsing two soft-estops into "count: 2" would
    hide one of them.

    ArbAction(action) raises on an action outside the contract's ten, which
    is the same loudness severity_of() gives.
    """
    return 0 if ArbAction(action) in DEDUP_EXEMPT else DEDUP_WINDOW_S


@dataclass(frozen=True)
class ArbEmit:
    """One audit event: severity + the 7A.7 fixed detail + dedup key/window."""
    severity: str
    action: str
    dedup_key: str
    dedup_window_s: int
    detail: Dict[str, Any]


class ArbVisibility:
    """gen / holder / last_change bookkeeping for domain 1."""

    def __init__(self) -> None:
        self._gen = 0
        self._winner: Optional[BehaviorSource] = None
        self._since_ms = 0
        self._last_change: Optional[Dict[str, Any]] = None
        self._suspended: Optional[str] = None
        self._last_state_ms: Optional[int] = None

    @property
    def gen(self) -> int:
        return self._gen

    def _state_body(self, snapshot: Dict[BehaviorSource, SourceState],
                    now_ms: int) -> Dict[str, Any]:
        w = self._winner
        holder = None
        if w is not None:
            holder = {"source_id": w.value, "req_id": None, "priority": priority_of(w),
                      "since_mono_ms": self._since_ms,
                      "held_ms": max(0, now_ms - self._since_ms),
                      "note": "domain-1 re-selected every tick (11 S7A.8)"}
        sources = [{"source_id": s.value, "priority": priority_of(s), "policy": POLICY,
                    "alive": bool(snapshot[s].active) if s in snapshot else False}
                   for s in BehaviorSource]
        return {"domain": DOMAIN, "gen": self._gen, "suspended": self._suspended,
                "holder": holder, "waiting": [], "last_change": self._last_change,
                "sources": sources}

    def _event(self, action: str, frm: Optional[BehaviorSource],
               to: Optional[BehaviorSource], reason: str, held_ms: int) -> ArbEmit:
        detail = {"domain": DOMAIN, "action": action,
                  "from": frm.value if frm else None, "to": to.value if to else None,
                  "reason": reason, "policy": POLICY, "held_ms": held_ms,
                  "wait_ms": 0, "overdue_ms": 0, "forced": False, "gen": self._gen,
                  "code": "OK", "count": 1}
        # severity comes from the shared 7A.7 map, not from a second table
        # here: two copies of one contract row drift, and the drift shows up
        # as an event the cloud's alarm subscription never matches (V-2).
        return ArbEmit(severity_of(action), action, dedup_key_for(action),
                       _dedup_window_for(action), detail)

    def observe(self, *, holder: Optional[BehaviorSource],
                snapshot: Dict[BehaviorSource, SourceState],
                suspended: Optional[str], now_mono_ms: int
                ) -> Tuple[Optional[Dict[str, Any]], List[ArbEmit]]:
        """One tick. Returns (state body or None, audit events). gen moves
        only with the winner (7A.8); the state body is emitted on any change
        and on the 1 Hz heartbeat.
        mutant: bump gen every tick -> test_gen_moves_only_with_the_winner
        red."""
        events: List[ArbEmit] = []
        changed = False
        if holder is not self._winner:
            old = self._winner
            held = max(0, now_mono_ms - self._since_ms) if old is not None else 0
            if old is None:
                action, reason = ACTION_GRANT, "first_holder"
            elif holder is not None and priority_of(holder) > priority_of(old):
                action, reason = ACTION_PREEMPT, "higher_priority"
            else:
                action, reason = ACTION_RELEASE, "source_deactivated"
            self._gen += 1
            self._winner = holder
            self._since_ms = now_mono_ms
            self._last_change = {"action": action,
                                 "from": old.value if old else None,
                                 "to": holder.value if holder else None,
                                 "reason": reason, "forced": False,
                                 "mono_ms": now_mono_ms}
            events.append(self._event(action, old, holder, reason, held))
            changed = True
        if suspended != self._suspended:
            action = ACTION_SUSPEND if suspended else ACTION_REARM
            self._suspended = suspended
            events.append(self._event(action, self._winner, self._winner,
                                      suspended or "cleared", 0))
            changed = True
        due = (self._last_state_ms is None
               or now_mono_ms - self._last_state_ms >= HEARTBEAT_PERIOD_MS)
        if changed or due:
            self._last_state_ms = now_mono_ms
            return self._state_body(snapshot, now_mono_ms), events
        return None, events
