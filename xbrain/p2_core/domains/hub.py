"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: hub.py
Brief: P2's four domain arbiters behind one lock -- request/release/ack, tick, state/arb frames, P-1' preflight

Description:
What problem this solves
------------------------
xbrain/common/arbiter/core.py is the one arbitration implementation (11 S7A)
and domains/factory.py builds P2's four domains from p2_core.yaml (speaker,
asr, payload_light, ptz). Until 2026-10-28 (99 U99, second batch C1) none of it
ran: the speaker was a plain non-blocking lock ("MVP is FIFO" in its own
docstring), the lights had no arbitration at all, and ModeFace's P-1'
preflight was fed two empty sets -- every mode switch passed. This hub is the
one place the live arbiters are owned.

Why a hub and not four free arbiters
------------------------------------
The arbiters are touched from two kinds of thread: the P2 main loop (mode
transitions, tick, state/arb publishing) and the speaker / payload worker
threads (a TTS request, a light command). Arbiter is not thread-safe by design
(it reads no clock and holds plain dicts), so every call goes through ONE
re-entrant lock here. One lock for all four domains is deliberate: a mode
switch touches several domains in one transaction (14 S5.5.2 step 5) and must
not interleave with a TTS request half-way.

What it does
------------
  * request / release / ack_preempt / cancel -- thin locked wrappers that also
    allocate req_ids (process-unique, "<domain>-<source>-<n>").
  * renew / renew_all -- the 11 S7A.4 keep-alive: every speaker source has a
    1 s lease, so the TTS thread renews while it plays and the main loop
    renews the mode's holds each pass (missed in the C1 first cut, U99).
  * being_preempted(domain, source, req_id) -- "I still hold it and a higher
    source is waiting": the TTS thread polls this while it plays and acks, so a
    broadcast takes the speaker at once instead of after the 3 s forced
    deadline (11 S7A.3 P-1..P-3).
  * tick(now) -- lease / forced-preempt reclamation; returns the drained audit
    events of every domain (core.tick returns its own, drain_events the rest).
  * states(now) -- render_domain_state for state/arb/{domain} (11 S2.2.2: all
    seven domains must publish; these are P2's four).
  * preflight(target, current_self) -- 14 S5.5.3 P-1' on the live holders:
    for each domain the target mode needs, the holder must be absent, one of
    the current mode's own sources (S_self, rule 2), or strictly lower and not
    protected (rule 3); otherwise the domain is blocked. ALL blocked domains
    are reported, never short-circuited.

What it does NOT do: it does not decide which sources a mode needs (the
caller's MODE_SOURCES table, from 14 S5.5.1 table A), it does not publish
anything, and it does not touch the payload device.
"""

from __future__ import annotations  # postponed annotations

import threading  # the one lock
from typing import Dict, FrozenSet, List, Mapping, Optional, Set, Tuple  # type hints only

from xbrain.common.arbiter.core import Arbiter  # the 11 S7A implementation
from xbrain.common.arbiter.model import ArbEvent, Grant, Holder, Request  # its value types
from xbrain.common.arbiter.state import render_domain_state  # state/arb/{domain} body
from xbrain.p2_core.domains.factory import build_all  # the four domains from config


class ArbHub:  # owned by p2 main_wiring
    """P2's four live arbiters behind one re-entrant lock."""

    def __init__(self, arbiter_yaml: Mapping) -> None:  # built once at startup
        """arbiter_yaml is p2_core.yaml `arbiter` from the resolved snapshot;
        build_all raises DomainConfigError on any missing block, which must
        stop P2 at startup rather than run with a partial set."""
        self._arbs: Dict[str, Arbiter] = build_all(arbiter_yaml)  # raises on a bad block
        self._lock = threading.RLock()      # one lock, all four domains
        self._n = 0                         # req_id sequence (process-unique)

    @property  # read-only view
    def domains(self) -> Tuple[str, ...]:  # for the state/arb publishers
        """The domain names, in factory order (speaker, asr, payload_light, ptz)."""
        return tuple(self._arbs)  # factory order

    def priority_of(self, domain: str, source: str) -> Optional[int]:  # rule 3 input
        """Registered priority of a source, None when not registered."""
        for s in self._arbs[domain].sources():  # the registered sources
            if s.source_id == source:  # found
                return s.priority  # its 14 S4 priority
        return None  # not registered

    def new_req_id(self, domain: str, source: str) -> str:  # 11 S7A.1 req_id
        """A process-unique request id (11 S7A.1 req_id)."""
        with self._lock:  # every arbiter call is locked
            self._n += 1  # monotonic within the process
            return "%s-%s-%d" % (domain, source, self._n)  # readable in audit

    def request(self, domain: str, source: str, now_mono_ms: int, *,
                req_id: Optional[str] = None, atomic: bool = False,
                est_ms: Optional[int] = None) -> Grant:
        """Acquire `domain` for `source`; returns the core's Grant unchanged."""
        rid = req_id or self.new_req_id(domain, source)  # caller may pre-allocate
        with self._lock:  # every arbiter call is locked
            return self._arbs[domain].request(source, Request(  # core decides
                req_id=rid, mono_ms=now_mono_ms, atomic=atomic, est_ms=est_ms))

    def renew(self, domain: str, source: str, req_id: str,  # 11 S7A.4 keep-alive
              now_mono_ms: int) -> bool:
        """Push the holder's lease out; False when (source, req_id) no longer
        holds `domain` (revoked, released, or still queued). Every speaker
        source has the 1 s default lease (14 S11), so a holder that does not
        renew at >= 2 Hz is reclaimed by the next tick -- the C1 first cut
        forgot this and a 3.4 s TTS was revoked at 1 s, leaving the mic muted."""
        with self._lock:  # every arbiter call is locked
            return self._arbs[domain].renew(source, req_id, now_mono_ms)  # gen untouched (G-1)

    def renew_all(self, holds: Mapping[str, Tuple[str, str]],  # mode holds
                  now_mono_ms: int) -> None:
        """Renew every (domain -> (source, req_id)) a mode holds; the P2 main
        loop calls it each pass (10 Hz) before tick. A queued entry simply
        does not renew (it holds nothing yet)."""
        for domain, (source, req_id) in list(holds.items()):  # copy: callers mutate
            self.renew(domain, source, req_id, now_mono_ms)  # queued -> False, harmless

    def handover(self, domain: str, old: Tuple[str, str], new_source: str,  # B <-> D
                 new_req_id: str, now_mono_ms: int) -> Grant:
        """Swap the mode's own hold on `domain` from old=(source, req_id) to
        new_source under ONE lock acquisition (14 S5.5.3 rule 2, self-held).

        B -> D moves the speaker from broadcast_b to alarm_d, D -> B back. Two
        separate calls would leave the speaker idle for an instant and a TTS
        thread could take it in between. old is released (if it holds) or
        cancelled (if still queued) -- both are no-ops on a mismatch -- with
        reason mode_exit (core.release: mode_switch is not in the closed set
        until 14 CR-1 lands), then new_source requests the now-idle domain.
        """
        with self._lock:  # one atomic swap
            arb = self._arbs[domain]  # that domain
            arb.release(old[0], old[1], "mode_exit", now_mono_ms)  # no-op unless it holds
            arb.cancel(old[0], old[1], now_mono_ms)  # no-op unless it is queued
            return arb.request(new_source, Request(  # the domain is idle now
                req_id=new_req_id, mono_ms=now_mono_ms, atomic=False, est_ms=None))

    def release(self, domain: str, source: str, req_id: str, reason: str,  # voluntary
                now_mono_ms: int) -> None:
        """Voluntary release (reason in done | mode_exit | user_cancel | error)."""
        with self._lock:  # every arbiter call is locked
            self._arbs[domain].release(source, req_id, reason, now_mono_ms)  # one holder change

    def ack_preempt(self, domain: str, source: str, req_id: str,  # P-2 ack
                    now_mono_ms: int) -> None:
        """The holder confirms it stopped; the waiter is promoted at once."""
        with self._lock:  # every arbiter call is locked
            self._arbs[domain].ack_preempt(source, req_id, now_mono_ms)  # waiter promoted

    def cancel(self, domain: str, source: str, req_id: str,  # queued withdraw
               now_mono_ms: int) -> None:
        """Withdraw a still-queued request (it never got the domain)."""
        with self._lock:  # every arbiter call is locked
            self._arbs[domain].cancel(source, req_id, now_mono_ms)  # never held

    def holder(self, domain: str) -> Optional[Holder]:  # snapshot read
        """Frozen snapshot of the current holder, None when idle."""
        with self._lock:  # every arbiter call is locked
            return self._arbs[domain].holder()  # frozen copy

    def holder_gen(self, domain: str) -> Tuple[Optional[Holder], int]:  # E_BUSY detail read
        """Holder snapshot AND the domain gen (11 S7A.2) under ONE lock: an
        E_BUSY detail (11 S7A.0 {domain, holder, holder_priority, gen}) must
        never pair a holder with the gen of the holder after it."""
        with self._lock:  # every arbiter call is locked
            arb = self._arbs[domain]  # that domain
            return arb.holder(), arb.gen()  # read together

    def holds(self, domain: str, source: str, req_id: str) -> bool:  # am I the holder
        """True while (source, req_id) is the current holder of `domain`."""
        h = self.holder(domain)  # one snapshot
        return h is not None and h.source_id == source and h.req_id == req_id  # same attempt

    def being_preempted(self, domain: str, source: str, req_id: str) -> bool:  # P-1 notice
        """Still the holder AND someone is queued to take over (P-1 notice)."""
        with self._lock:  # every arbiter call is locked
            arb = self._arbs[domain]  # that domain
            h = arb.holder()  # who holds
            return (h is not None and h.source_id == source
                    and h.req_id == req_id and bool(arb.waiters()))

    def tick(self, now_mono_ms: int) -> List[ArbEvent]:  # called every loop pass
        """Lease / forced-preempt reclamation, then every pending audit event."""
        out: List[ArbEvent] = []  # all domains
        with self._lock:  # every arbiter call is locked
            for arb in self._arbs.values():  # four domains
                out.extend(arb.tick(now_mono_ms))      # time-driven reclaims
                out.extend(arb.drain_events())         # grants / releases / preempts
        return out  # for the audit publisher

    def states(self, now_mono_ms: int) -> Dict[str, dict]:  # state/arb bodies
        """state/arb/{domain} bodies (11 S7A.5.1) for all four domains."""
        with self._lock:  # every arbiter call is locked
            return {d: render_domain_state(a, now_mono_ms)
                    for d, a in self._arbs.items()}

    def preflight(self, target: Mapping[str, str],
                  self_sources: Set[str]) -> Tuple[FrozenSet[str], FrozenSet[str]]:
        """14 S5.5.3 P-1' for the domains the target mode needs.

        target maps domain -> the source the target mode will hold there;
        self_sources are the source ids the CURRENT mode holds (S_self, 14
        S5.5.1). Returns (blocked, self_held) as domain-name frozensets, the
        shape ModeStateMachine.request takes. Order of the rules is fixed: rule
        2 (self-held) before rule 3 (strictly lower) -- reversed, alarm_d(900)
        over broadcast_b(800) would land in rule 4 and D -> B would be refused,
        the exact gap 14 S5.5.3 was written to close.
        """
        blocked: Set[str] = set()  # rule 4 domains
        self_held: Set[str] = set()  # rule 2 domains
        with self._lock:  # every arbiter call is locked
            for domain, source in target.items():  # each domain the target needs
                h = self._arbs[domain].holder()  # current holder
                if h is None:  # rule 1
                    continue                                 # rule 1: free
                if h.source_id in self_sources:  # rule 2 first
                    self_held.add(domain)                    # rule 2: ours
                    continue
                want = self.priority_of(domain, source)  # the target source's priority
                if want is not None and h.priority < want:  # rule 3
                    continue                                 # rule 3: preemptible
                blocked.add(domain)                          # rule 4: blocked
        return frozenset(blocked), frozenset(self_held)  # ModeStateMachine shape


__all__ = ["ArbHub"]  # public surface
