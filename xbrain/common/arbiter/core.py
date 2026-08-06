"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: core.py
Brief: The per-domain arbitration state machine (11 S7A, framework core)

Description:
What problem this solves. Every one of the seven resource domains (motion,
speaker, asr, payload_light, ptz, gpu, dock) needs the SAME arbitration rules --
one holder at a time, higher priority wins, a lease so a stuck holder is
reclaimed, a preemption protocol so a higher source can take over, and a
generation counter so a just-preempted holder cannot slip out one more frame.
Writing that seven times is how 14 S3.1 says V5 rotted (仲裁逻辑散落各模块). This
class is the one implementation; each owning process instantiates one Arbiter per
domain it owns (14 S3.3, 11 S7A.0). BIZ-CM-2 assembles state/arb and the audit
stream from what this emits; BIZ-CM-3 adds the disarm (缴械) semantics; BIZ-CM-5
adds the T-2/T-3/T-4 refinements of the preempt protocol. This file is the core
the three build on: register / request / renew / release / ack_preempt / cancel /
tick / holder, plus SourceSpec, the four gen rules, and the lease.

Which sections this implements (NUM-4 anchors, grep the doc):
  * 11 S7A.1  request/renew/release/ack_preempt/cancel, mono_ms is CLOCK_MONOTONIC
  * 11 S7A.2  the four gen rules G-1..G-4 (grep "gen 的四条规则")
  * 11 S7A.3  preempt policies and T-1 forced preempt (grep "deadline_mono_ms")
  * 11 S7A.4  lease, >= 2 Hz renew, source death (grep "租约")
  * 11 S7A.10 the invariants ARB-1..ARB-8 this core must not violate

The clock discipline, stated because it is the whole point of AB-6. This class
reads NO clock. Every timestamp -- request mono_ms, tick's now_mono_ms -- is
measured by the CALLER from CLOCK_MONOTONIC and passed in. All lease and deadline
tests are integer subtractions of those passed values. That is precisely why a
wall-clock forward step (chronyd stepping the clock seconds-to-years when RTK
first locks, 11 S7A.1) cannot judge the seven holders dead: the step never enters,
because there is no wall clock here to read. The mutation that proves this is real
makes tick() read time.time() instead of its argument; the seven-domain test then
goes red. If you are tempted to add a time.monotonic() call here "for
convenience", that is the defect -- keep time a parameter.

What this file deliberately does NOT do, so BIZ-CM-2/3/5 have room and nothing is
half-built here (a half-built guard is CLAUDE.md 3.2 form 1, an assertion an empty
shell passes):
  * No disarm / suspend / rearm and no E_ARB_DISARMED. That is BIZ-CM-3. There is
    no suspended field and no soft-estop branch here.
  * No stuck-source ban (T-3) and no E_ARB_DISABLED. Banning a source after three
    forced preempts in 60 s, AND restoring it on process restart, is BIZ-CM-5.
    Implementing only the ban half here would pass a "gets disabled" test while a
    single jitter permanently kills a voice source -- exactly the one-sided
    assertion 14 warns about, so it is left out entirely rather than stubbed.
  * No severity assignment and no dedup. tick() stamps an ArbAction; BIZ-CM-2 maps
    it to info/warn/fault and merges duplicates. Deciding severity here would put
    the mapping in two places.
  * No message serialisation. Grant/Holder are returned as objects; turning them
    into cmd/arb/{domain}/grant and state/arb/{domain} is BIZ-CM-2.

Traps -- things that look right and are not:
  1. renew() must NOT advance gen (G-1: 续租不增). gen is the "holder changed"
     counter; bumping it on renew makes every executor discard the holder's own
     in-flight output as stale. The mutation for this is one line in renew.
  2. A resident source (lease_timeout_s None) has holder.lease_ms None and is
     skipped by the lease check. Feeding None through the lease arithmetic as 0
     would expire mode_driver on the next tick (14 S4.3 bug). None means skip.
  3. The holder swap on ack/forced/release is ONE holder change: the old holder
     is cleared and the waiter promoted with a SINGLE gen bump. Doing it as
     clear-then-grant with two bumps is the BIZ-CM-2 mutation that turns one
     audit event into two; keep it atomic (14 S5.5.2 H-1).
"""

from typing import Dict, List, Optional

# The shared error library and the shared closed sets. Bare names, never string
# literals (CLAUDE.md 3.5): errors.E_BUSY is a name, "E_BUSY" would be a second
# spelling that only disagrees during integration.
from xbrain.common import errors                    # E_BUSY / E_ARB_NO_SOURCE / OK
from xbrain.common.enums import (                   # closed-set validators
    ARB_SUSPENDED, DOMAIN, RELEASE_REASON,
)

from .model import (                                # the frozen value layer
    ArbAction, ArbEvent, Grant, GrantResult, Holder, IMMEDIATE_GRACE_MS,
    LastChange, Preempt, PreemptPolicy, Request, SourceSnapshot, SourceSpec,
    WaiterSnapshot,
)

__all__ = ["Arbiter"]                              # the class is the whole surface


# --- internal mutable state -------------------------------------------------
# These three are private and mutable; the public surface hands out only frozen
# snapshots (Holder, Grant). __slots__ so a mistyped attribute assignment raises
# instead of quietly creating a field the state machine never reads.

class _Holding:
    """The live holder record. Mutable because renew, preempt-marking and the
    ack deadline all update it in place between holder changes."""

    # __slots__ turns a typo like h.since_mono = ... into an AttributeError
    # rather than a silently-ignored new field on the object the state reads.
    __slots__ = ("source_id", "req_id", "priority", "since_mono_ms",
                 "last_renew_mono_ms", "lease_ms", "being_preempted",
                 "preempt_by", "preempt_deadline_mono_ms")

    def __init__(self, source_id, req_id, priority, since_mono_ms,
                 last_renew_mono_ms, lease_ms):
        self.source_id = source_id                 # who holds it now
        self.req_id = req_id                       # the hold attempt id (11 S7A.1)
        self.priority = priority                   # cached so requests compare fast
        self.since_mono_ms = since_mono_ms         # for held_ms, computed at publish
        # Advanced by renew(); the lease check measures now - this. Starts equal
        # to since so a holder that never renews still gets one full lease.
        self.last_renew_mono_ms = last_renew_mono_ms
        # ms, or None for a resident holder that is never lease-reclaimed.
        self.lease_ms = lease_ms
        # Set when a higher request has started preempting this holder. Until it
        # acks or the deadline passes, this holder is still the holder.
        self.being_preempted = False               # no preemption in flight yet
        self.preempt_by = None                     # the preemptor's id, informational
        self.preempt_deadline_mono_ms = None       # set when a preemption starts


class _Waiter:
    """A queued acquire that could not be granted immediately."""

    # Same __slots__ reasoning as _Holding: catch a mistyped field at write time.
    __slots__ = ("source_id", "req_id", "priority", "urgency", "since_mono_ms",
                 "lease_ms")

    def __init__(self, source_id, req_id, priority, urgency, since_mono_ms,
                 lease_ms):
        self.source_id = source_id                 # who is waiting
        self.req_id = req_id                        # echoed on grant when promoted
        self.priority = priority                    # primary selection key
        self.urgency = urgency                      # tie-break at EQUAL priority only
        self.since_mono_ms = since_mono_ms          # final tie-break: earliest wins
        # The request's lease override, preserved across the wait so the promoted
        # holder gets the lease it asked for rather than only the source default.
        self.lease_ms = lease_ms


class _SourceEntry:
    """One registered source: its spec plus whether its process is alive."""

    __slots__ = ("spec", "alive")                  # spec is frozen; alive flips

    def __init__(self, spec):
        self.spec = spec                            # the registered SourceSpec
        # alive starts True and is flipped by reap_dead_source(); a live request
        # flips it back, because a request is proof the process is running.
        self.alive = True


class Arbiter:
    """One resource domain's arbiter (11 S7A). Not thread-safe by design.

    The owning process serialises all calls onto its own arbitration thread
    (P2's fast 50 Hz thread, P1's 20 Hz loop): the state machine is a plain
    single-threaded object so that "one holder at a time" (ARB-1) is a property
    of the code and not of a lock nobody can see. Zenoh subscriber callbacks that
    arrive on a Rust thread must hand the request across with
    publish_threadsafe / call_soon_threadsafe (CLAUDE.md 4.2), never call these
    methods directly.
    """

    # domain and wait_atomic_timeout are injected; nothing is defaulted here. The
    # domain is validated against the closed set at construction so an
    # instantiation with a bad domain fails at startup with the key in the
    # traceback, not at the first request. wait_atomic_timeout_ms is the domain's
    # ceiling for how long a wait_atomic preemptor waits (11 S7A.3); it has no
    # code default so a caller states it (DEFAULT_WAIT_ATOMIC_TIMEOUT_MS in the
    # model is the contract value for a bring-up caller with no domain config).
    def __init__(self, domain: str, wait_atomic_timeout_ms: int) -> None:
        # DOMAIN.parse raises ClosedSetViolation on an off-contract domain -- the
        # same boundary discipline every other decode uses, no silent accept.
        self._domain = DOMAIN.parse(domain)             # validated once, at startup
        self._wait_atomic_timeout_ms = wait_atomic_timeout_ms   # the domain ceiling
        self._registry: Dict[str, _SourceEntry] = {}    # source_id -> entry
        self._holder: Optional[_Holding] = None         # None == idle domain
        self._waiting: List[_Waiter] = []               # queued higher requests
        # G-4: gen starts at 0 and only ever increases within this process; it is
        # never persisted and never compared across a restart. The first grant
        # makes it 1.
        self._gen = 0                                   # the arbitration generation
        self._last_change: Optional[LastChange] = None  # mirrors the last audit event
        # The audit buffer for source-initiated events (request/release/ack/
        # cancel). Driver-initiated events (tick, reap) are RETURNED, not buffered
        # here, so a consumer merging `tick() + drain_events()` never double
        # counts. drain_events() empties this.
        self._events: List[ArbEvent] = []               # buffered audit records
        # BIZ-CM-3 disarm (缴械) state. None == armed (normal). A non-null reason
        # (soft_estop | hes | cmd_timeout) means every request() is denied
        # E_ARB_DISARMED until arb_rearm(). _suspend_cmd_id is the idempotency key:
        # the same soft-estop arrives on cmd/estop AND state/robot (two paths, one
        # cmd_id, 11 S7A.6), and must produce exactly ONE suspend event.
        self._suspended: Optional[str] = None           # ARB_SUSPENDED value or None
        self._suspend_cmd_id: Optional[str] = None      # dedupes the two arrival paths

    # -- registration --------------------------------------------------------

    def register(self, spec: SourceSpec) -> None:
        """Register a competing source. Done at startup, fixed at run time.

        Idempotent on the id: re-registering the same source_id replaces its
        entry and marks it alive again, which is how a restarted process rejoins
        (11 S7A.3 T-3 recovery is 'its process restarts'). It does not touch the
        holder or the gen.
        """
        # A fresh entry (alive True) each time; replacing an existing id is how a
        # revived process comes back with a clean alive flag.
        self._registry[spec.source_id] = _SourceEntry(spec)

    # -- the acquire path ----------------------------------------------------

    def request(self, source_id: str, req: Request) -> Grant:
        """Acquire the domain. Always returns exactly one Grant (11 S7A.1).

        The result is granted (free, or you already hold it), denied (E_ARB_NO_
        SOURCE / E_BUSY), or queued (you are higher and are preempting a holder
        that has an atomic action to finish). It is never a dangling queued that
        resolves to nothing (ARB-3): a queued requester is either promoted on the
        holder's ack or force-granted on the deadline in tick().
        """
        # BIZ-CM-3: while disarmed, EVERY request is denied E_ARB_DISARMED -- this
        # is checked before the registry lookup, so even a registered source gets
        # the disarmed code, not E_ARB_NO_SOURCE (11 S7A.6.2: "此后任何 request()
        # 一律 denied(E_ARB_DISARMED)"). re-arming is an explicit arb_rearm() the
        # consumer calls on a genuinely new command (S7A.6.3), never implicit here.
        if self._suspended is not None:
            return self._denied(req, source_id, errors.E_ARB_DISARMED)

        entry = self._registry.get(source_id)           # None if never registered
        if entry is None:
            # Unregistered: denied, no holder change, no audit event (denials are
            # not in the 14 S3.6 audited-action set).
            return self._denied(req, source_id, errors.E_ARB_NO_SOURCE)
        entry.alive = True                              # a request proves the process is up

        h = self._holder                                # the current holder, or None
        if h is None:
            # Free domain: grant it outright, first holder change of this hold.
            return self._grant_free(source_id, req, entry, self._events)
        if h.source_id == source_id:
            # The current holder re-acquiring (new req_id). Treat as a renew: no
            # holder change, so no gen bump (G-1) and no event.
            h.req_id = req.req_id                        # adopt the new attempt id
            h.last_renew_mono_ms = req.mono_ms           # and treat it as a renew
            return self._granted_now(req, source_id, forced=False)

        # Held by someone else. ARB-2: only a STRICTLY higher priority may take
        # it; equal or lower is denied E_BUSY (equal is where urgency would order
        # the queue, but it still cannot preempt).
        if entry.spec.priority <= h.priority:
            return self._denied(req, source_id, errors.E_BUSY)
        # A reject source never waits and never preempts-and-waits: it takes the
        # domain only when free, and the domain is busy, so deny (14 S4.3).
        if entry.spec.preempt_policy is PreemptPolicy.REJECT:
            return self._denied(req, source_id, errors.E_BUSY)
        holder_spec = self._registry[h.source_id].spec   # the holder's registration
        # A protected holder (preemptible False) is not taken even by a higher
        # priority. No domain config sets this today; honoured for completeness.
        if not holder_spec.preemptible:
            return self._denied(req, source_id, errors.E_BUSY)
        # Strictly higher, holder is preemptible: start the preemption protocol.
        return self._begin_preempt(source_id, req, entry, h, holder_spec)

    # -- renew / release / ack / cancel -------------------------------------

    def renew(self, source_id: str, req_id: str, now_mono_ms: int) -> bool:
        """Extend the holder's lease. Returns True if it was the holder.

        *** G-1: this does NOT advance gen. A renew is not a holder change; the
        holder is the same, its output is still current, and bumping gen here
        would make the executor discard the holder's own frames as stale. This is
        the single line BIZ-CM-1 mutation (2) adds a gen bump to, turning the G-1
        assertion red.
        """
        h = self._holder                                # current holder or None
        if h is None or h.source_id != source_id or h.req_id != req_id:
            # Not the holder (or wrong attempt id): nothing to renew.
            return False
        h.last_renew_mono_ms = now_mono_ms              # push the lease deadline out
        return True                                     # and NOTE: gen untouched (G-1)

    def release(self, source_id: str, req_id: str, release_reason: str,
                now_mono_ms: int) -> None:
        """Voluntarily give up the domain (11 S7A.1 op=release).

        release_reason is validated against the closed set (done | mode_exit |
        user_cancel | error); an off-contract reason raises rather than passing
        through, the same boundary rule as every other decode. If a higher
        request was already queued, releasing completes that handover as one
        holder change (H -> W, single gen bump), not H -> idle -> W.
        """
        # RELEASE_REASON.parse raises ClosedSetViolation on an unknown reason.
        # mode_switch is intentionally NOT a member yet (14 CR-1 pending in 11);
        # the self-held handover emits mode_exit until it lands.
        RELEASE_REASON.parse(release_reason)            # validate or raise
        h = self._holder                                # current holder or None
        if h is None or h.source_id != source_id or h.req_id != req_id:
            return                                      # not the holder: ignore
        gone = self._end_holder(now_mono_ms, notify_lost=False)   # voluntary: no on_lost
        # A waiter present means a preemption was in flight; label the change a
        # grant to the new holder. Otherwise the domain goes idle.
        action = ArbAction.GRANT if self._waiting else ArbAction.RELEASE
        # One holder change (bump + event) whether it idles or hands over.
        self._promote_or_idle(now_mono_ms, gone, action, release_reason,
                              forced=False, detail={}, sink=self._events)

    def ack_preempt(self, source_id: str, req_id: str,
                    now_mono_ms: int) -> Optional[Grant]:
        """The preempted holder confirms it has stopped (11 S7A.1 op=ack_preempt).

        P-2 requires the holder to ack BEFORE finishing cleanup; this method just
        completes the handover -- the highest waiter becomes holder in one change.
        Returns the promoted holder's Grant, or None if there was nothing to
        promote (e.g. a stray ack from a source that is not the holder).
        """
        h = self._holder                                # current holder or None
        if (h is None or h.source_id != source_id or h.req_id != req_id
                or not h.being_preempted):
            # Stray ack: not the holder, or the holder is not being preempted.
            return None
        gone = self._end_holder(now_mono_ms, notify_lost=False)   # graceful: no on_lost
        # Promote the highest waiter as the new holder in one holder change.
        return self._promote_or_idle(
            now_mono_ms, gone, ArbAction.GRANT, "higher_priority",
            forced=False, detail={}, sink=self._events)

    def cancel(self, source_id: str, req_id: str, now_mono_ms: int) -> None:
        """Withdraw a still-queued acquire (11 S7A.1 op=cancel).

        Only a queued request can be cancelled; a granted holder uses release().
        If the cancelled request was the reason a holder was being preempted and
        no other waiter remains, the preemption is called off so the holder is no
        longer under an ack deadline it need not meet.
        """
        before = len(self._waiting)                     # to detect whether we removed one
        self._waiting = [w for w in self._waiting       # drop the matching waiter
                         if not (w.source_id == source_id and w.req_id == req_id)]
        if len(self._waiting) == before:
            return                                      # nothing matched: no-op
        # No one is waiting any more: stand the holder back down from preemption.
        if not self._waiting and self._holder is not None \
                and self._holder.being_preempted:
            self._holder.being_preempted = False        # cancel the in-flight preempt
            self._holder.preempt_by = None              # clear the informational id
            self._holder.preempt_deadline_mono_ms = None    # and drop the ack deadline

    # -- the periodic driver -------------------------------------------------

    def tick(self, now_mono_ms: int) -> List[ArbEvent]:
        """Reclaim on the two time-driven paths and return what happened.

        *** now_mono_ms is CLOCK_MONOTONIC milliseconds the CALLER measured. This
        method reads no clock (AB-6). BIZ-CM-1 mutation (1) makes it read
        time.time() instead; the seven-domain test then judges every holder dead.

        Order: the forced-preempt deadline (T-1) first, then the lease. A forced
        preempt installs a fresh holder whose lease has not started, so the lease
        check that follows cannot also fire on it in the same tick.
        """
        out: List[ArbEvent] = []                        # this tick's events, returned
        h = self._holder                                # current holder or None
        # T-1: a preemption whose ack deadline has passed with no ack. The
        # deadline is None only under mutation (4); the reference always sets one,
        # so this branch really does fire in the test.
        if (h is not None and h.being_preempted
                and h.preempt_deadline_mono_ms is not None
                and now_mono_ms > h.preempt_deadline_mono_ms):
            overdue = now_mono_ms - h.preempt_deadline_mono_ms   # T-2 detail.overdue_ms
            gone = self._end_holder(now_mono_ms, notify_lost=True)   # involuntary: on_lost
            # Force-grant the waiter; forced=True marks the granted as revoked-then-taken.
            self._promote_or_idle(
                now_mono_ms, gone, ArbAction.FORCED_PREEMPT, "preempt_timeout",
                forced=True, detail={"overdue_ms": overdue}, sink=out)
        # Lease timeout on the current (possibly just-installed) holder. A
        # resident holder has lease_ms None and is skipped -- mutation (3) gives
        # it a lease and this reclaims mode_driver by mistake.
        h = self._holder                                # re-read: forced preempt may have changed it
        if (h is not None and h.lease_ms is not None
                and now_mono_ms - h.last_renew_mono_ms > h.lease_ms):
            gone = self._end_holder(now_mono_ms, notify_lost=True)   # involuntary: on_lost
            # Reclaim (and promote a waiter if any) as one holder change.
            self._promote_or_idle(
                now_mono_ms, gone, ArbAction.LEASE_TIMEOUT, "lease_timeout",
                forced=False, detail={}, sink=out)
        return out                                      # events for this tick

    def reap_dead_source(self, source_id: str,
                         now_mono_ms: int) -> List[ArbEvent]:
        """Process-death reclamation (11 S7A.4), the second detection path.

        This is the heartbeat half of the double detection: P5 loses a process's
        heartbeat and tells each arbiter to drop that process's sources. It is
        independent of the lease -- it reclaims even a resident holder that the
        lease path would never touch -- which is exactly why 14 C-2 requires both:
        the lease catches 'alive but stuck', this catches 'the process is gone'.
        """
        entry = self._registry.get(source_id)           # None if unknown source
        if entry is None:
            return []                                    # nothing registered under that id
        entry.alive = False                             # mark it dead in the registry
        # A dead source cannot sit in the queue either.
        self._waiting = [w for w in self._waiting if w.source_id != source_id]
        out: List[ArbEvent] = []                        # this reap's events, returned
        if self._holder is not None and self._holder.source_id == source_id:
            gone = self._end_holder(now_mono_ms, notify_lost=True)   # involuntary: on_lost
            # Reclaim the domain (promote a waiter if one is queued).
            self._promote_or_idle(
                now_mono_ms, gone, ArbAction.SOURCE_DEATH, "source_death",
                forced=False, detail={}, sink=out)
        return out                                      # source_death event, if any

    # -- disarm (BIZ-CM-3, 缴械, 11 S7A.6) -----------------------------------

    def arb_suspend(self, reason: str, cmd_id: str,
                    now_mono_ms: int) -> Optional[ArbEvent]:
        """Disarm the whole domain: a soft-estop / lock takes every source's
        holding eligibility (11 S7A.6.1 "锁的是源的持有资格, 不是机器人状态").

        reason is validated against ARB_SUSPENDED (soft_estop | hes | cmd_timeout).
        The current holder is revoked (on_lost fires so it stops producing), the
        queue is cleared, and gen bumps once. After this every request() is denied
        E_ARB_DISARMED until arb_rearm().

        Idempotent on cmd_id: the same soft-estop reaches P1 on cmd/estop AND on
        state/robot (11 S7A.6, two paths, one cmd_id). The SECOND arrival, already
        suspended under the same cmd_id, is a no-op returning None -- so exactly
        ONE suspend audit event is produced (the BIZ-CM-3 idempotency criterion).
        A DIFFERENT cmd_id while already suspended re-stamps and re-events (a fresh
        estop epoch).

        Returns the suspend ArbEvent to publish, or None on the idempotent hit.
        There is deliberately NO "hold" holder installed here: the framework leaves
        the domain holder-less (armed sources all revoked), and the consumer's
        zero-speed 'hold' behaviour (11 S7A.6.2, p1_motion) is layered on top by
        reading suspended(), keeping this class free of motion-domain specifics.
        """
        # Boundary: an off-contract reason raises here, never reaches the wire.
        reason = ARB_SUSPENDED.parse(reason)
        # Idempotent second path: already disarmed under this cmd_id -> no event.
        if self._suspended is not None and self._suspend_cmd_id == cmd_id:
            return None
        # Revoke the current holder (involuntary: on_lost tells it to stop) and
        # clear the queue; no source may hold across a disarm.
        gone = self._end_holder(now_mono_ms, notify_lost=True)   # from_source, or None
        self._waiting = []                              # every queued request is dropped
        self._suspended = reason                        # the domain is now disarmed
        self._suspend_cmd_id = cmd_id                   # remember, to dedupe the 2nd path
        self._gen += 1                                  # G-1: a holder change (-> none)
        self._last_change = LastChange(ArbAction.SUSPEND.value, gone, None,
                                       reason, False, now_mono_ms)   # mirror the event
        # detail carries cmd_id so a consumer can correlate the two arrival paths.
        return self._make_event(ArbAction.SUSPEND, gone, None, reason, False,
                                {"cmd_id": cmd_id}, now_mono_ms)

    def arb_rearm(self, now_mono_ms: int) -> Optional[ArbEvent]:
        """Re-arm the domain on a new command (11 S7A.6.2 / S7A.6.3).

        The consumer calls this when it sees a genuinely NEW motion command (a new
        cmd_id/req_id after the suspend, RE-1), never for a source continuing on
        its own. Clears the disarm and bumps gen; the domain comes back idle, so
        the new command's own request() (issued next by the consumer) grants
        normally. Idempotent: re-arming an already-armed domain is a no-op (None),
        so a second new command does not emit a spurious rearm.
        """
        if self._suspended is None:
            return None                                 # already armed: nothing to do
        self._suspended = None                          # armed again
        self._suspend_cmd_id = None                     # clear the idempotency key
        self._gen += 1                                  # G-1: the disarm state changed
        self._last_change = LastChange(ArbAction.REARM.value, None, None,
                                       "rearm", False, now_mono_ms)
        return self._make_event(ArbAction.REARM, None, None, "rearm", False,
                                {}, now_mono_ms)

    # -- read-only views -----------------------------------------------------

    def holder(self) -> Optional[Holder]:
        """The current holder as a frozen snapshot, or None if idle."""
        return self._holder_snapshot()                  # never the live object

    def gen(self) -> int:
        """The current arbitration generation (G-1..G-4)."""
        return self._gen                                # monotonic within this process

    def domain(self) -> str:
        """This arbiter's domain (a validated closed-set value, 11 S7A.0)."""
        return self._domain                             # parsed once at construction

    def suspended(self) -> Optional[str]:
        """The disarm reason (soft_estop | hes | cmd_timeout), or None if armed.

        This is ArbDomainState.suspended (11 S7A.5.1). None means normal; a
        non-null value means every request() is denied E_ARB_DISARMED (BIZ-CM-3).
        """
        return self._suspended

    def last_change(self) -> Optional[LastChange]:
        """The most recent holder change (11 S7A.5.1 last_change)."""
        return self._last_change                        # same source as the audit event

    def waiters(self) -> List[WaiterSnapshot]:
        """The queued requests as frozen snapshots, highest priority first.

        For ArbDomainState.waiting[] (11 S7A.5.1). Frozen copies, never the live
        _Waiter objects, so a consumer serialising the state cannot mutate the
        queue the state machine is still selecting the next holder from. Sorted so
        the HMI shows the next-in-line at the top; the arbiter's own promotion
        uses the same (priority, urgency, since) key, but this view exposes only
        what the wire shape carries.
        """
        ordered = sorted(self._waiting,
                         key=lambda w: (-w.priority, -w.urgency, w.since_mono_ms))
        return [WaiterSnapshot(w.source_id, w.req_id, w.priority, w.since_mono_ms)
                for w in ordered]

    def sources(self) -> List[SourceSnapshot]:
        """The whole registry as frozen snapshots, alive=false included.

        For ArbDomainState.sources[] (11 S7A.5.1): the HMI shows every registered
        source, including one whose process died, so "谁本可以抢但进程没了" is
        answerable. Order is registration order (dict insertion), which is stable.
        """
        return [SourceSnapshot(e.spec.source_id, e.spec.priority,
                               e.spec.preempt_policy.value, e.alive)
                for e in self._registry.values()]

    def drain_events(self) -> List[ArbEvent]:
        """Return and clear the buffered source-initiated audit events.

        The authoritative audit stream for a cycle is `tick() + drain_events()`:
        tick() and reap_dead_source() RETURN their events (not buffered here), and
        request/release/ack/cancel buffer theirs here. Merging the two covers
        every event exactly once.
        """
        out = self._events                              # hand back what accumulated
        self._events = []                               # and reset the buffer
        return out

    # -- seams the mutations target -----------------------------------------
    # These two are the single points the BIZ-CM-1 mutations override, so the
    # tests inject each defect by subclassing and overriding ONE method rather
    # than editing shipped source. They are real factorings, not test hooks bolted
    # on: the lease rule and the deadline rule each genuinely belong in one place.

    def _effective_lease_ms(self, spec: SourceSpec,
                            req_lease_ms: Optional[int]) -> Optional[int]:
        """The lease actually in force for a grant, ms, or None for resident.

        None in, None out: a resident source (lease_timeout_s None, e.g.
        mode_driver) is never lease-reclaimed. Any real value is clamped down to
        the source ceiling (11 S7A.1: lease_ms 不得超过域配置上限). Mutation (3)
        overrides this to return a number for the None case, which makes the
        resident holder expire.
        """
        if spec.lease_timeout_s is None:
            return None                                 # resident: no lease at all
        base = int(spec.lease_timeout_s * 1000)         # seconds -> ms, the ceiling
        if req_lease_ms is not None:
            return min(req_lease_ms, base)              # clamp a request override down
        return base                                     # otherwise the source default

    def _preempt_deadline_ms(self, holder_spec: SourceSpec,
                             now_mono_ms: int) -> Optional[int]:
        """When the preempted holder must have acked by (11 S7A.3).

        immediate (and reject, which does no atomic wait) yields within
        IMMEDIATE_GRACE_MS; wait_atomic gets the domain's wait_atomic ceiling.
        Mutation (4) overrides this to return None for the immediate case, so the
        ack deadline never passes and the T-1 forced-preempt path never fires.
        """
        if holder_spec.preempt_policy is PreemptPolicy.WAIT_ATOMIC:
            return now_mono_ms + self._wait_atomic_timeout_ms   # atomic-action grace
        return now_mono_ms + IMMEDIATE_GRACE_MS         # immediate/reject: 100 ms

    # -- private helpers -----------------------------------------------------

    def _grant_free(self, source_id, req, entry, sink) -> Grant:
        """Grant an idle domain to a requester. One holder change (None -> N)."""
        lease = self._effective_lease_ms(entry.spec, req.lease_ms)   # None if resident
        self._holder = _Holding(source_id, req.req_id, entry.spec.priority,
                                req.mono_ms, req.mono_ms, lease)   # install as holder
        self._gen += 1                                  # G-1: holder changed
        self._last_change = LastChange(ArbAction.GRANT.value, None, source_id,
                                       req.reason, False, req.mono_ms)   # None -> N
        sink.append(self._make_event(ArbAction.GRANT, None, source_id,
                                     req.reason, False, {}, req.mono_ms))   # audit
        return self._granted_now(req, source_id, forced=False)   # granted to requester

    def _begin_preempt(self, source_id, req, entry, h, holder_spec) -> Grant:
        """Queue a higher requester and start preempting the current holder.

        No holder change happens here, so gen does NOT move: the holder is still
        the holder until it acks or the deadline passes. The holder is notified
        (P-1: stop producing new output) and a preempt event is audited. If a
        preemption is already in flight, the new requester just joins the queue;
        the highest waiter is the one promoted when the holder yields.
        """
        self._waiting.append(_Waiter(source_id, req.req_id, entry.spec.priority,
                                     req.urgency, req.mono_ms, req.lease_ms))   # queue it
        if not h.being_preempted:                       # only start once per preemption
            h.being_preempted = True                    # mark the holder under preempt
            h.preempt_by = source_id                    # record who started it (info)
            h.preempt_deadline_mono_ms = self._preempt_deadline_ms(holder_spec,
                                                                   req.mono_ms)   # the seam
            # P-1 notification. on_preempt takes the preemptor's id; guard None.
            if holder_spec.on_preempt is not None:
                holder_spec.on_preempt(source_id)       # tell the holder to stop
            self._events.append(self._make_event(       # audit the preempt start
                ArbAction.PREEMPT, h.source_id, source_id, "higher_priority",
                False, {}, req.mono_ms))
        preempt = Preempt(by=source_id, by_priority=entry.spec.priority,
                          policy=holder_spec.preempt_policy.value,
                          deadline_mono_ms=h.preempt_deadline_mono_ms,
                          reason="higher_priority")      # describe the notice to the requester
        return Grant(self._domain, GrantResult.QUEUED, req.req_id, source_id,
                     errors.OK, self._gen, self._holder_snapshot(), preempt,
                     False, req.mono_ms)                 # queued, holder unchanged

    def _end_holder(self, now_mono_ms, notify_lost) -> Optional[str]:
        """Clear the current holder. Returns its source_id (or None).

        notify_lost fires on_lost for the INVOLUNTARY paths (lease, death, forced)
        so the source learns it lost the resource; a voluntary release or ack does
        not, because the source is the one that acted. Does not touch gen -- the
        single bump belongs to _promote_or_idle so a clear-then-promote is one
        holder change, not two.
        """
        old = self._holder                              # remember who is leaving
        self._holder = None                             # domain is momentarily idle
        if old is None:
            return None                                 # nothing was held
        if notify_lost:                                 # involuntary loss only
            spec = self._registry[old.source_id].spec   # find its on_lost hook
            if spec.on_lost is not None:
                spec.on_lost()                          # tell it the resource is gone
        return old.source_id                            # the from_source of the change

    def _promote_or_idle(self, now_mono_ms, from_source, action, reason, forced,
                         detail, sink) -> Optional[Grant]:
        """Install the highest waiter as holder, or go idle. One gen bump.

        This is the single place a holder change is committed after the old holder
        was cleared, so exactly one gen increment and one audit event correspond
        to the change (ARB-5 / G-1). Returns the new holder's Grant, or None when
        the domain went idle.
        """
        w = self._pop_highest_waiter()                  # the winning waiter, or None
        if w is None:
            to = None                                   # no one waiting: stay idle
        else:
            spec = self._registry[w.source_id].spec     # the promoted source's spec
            lease = self._effective_lease_ms(spec, w.lease_ms)   # its effective lease
            self._holder = _Holding(w.source_id, w.req_id, w.priority,
                                    now_mono_ms, now_mono_ms, lease)   # install fresh
            to = w.source_id                            # the to_source of the change
        self._gen += 1                                  # G-1: exactly one holder change
        self._last_change = LastChange(action.value, from_source, to, reason,
                                       forced, now_mono_ms)   # mirror the event
        sink.append(self._make_event(action, from_source, to, reason, forced,
                                     detail, now_mono_ms))   # one audit record
        if to is None:
            return None                                 # went idle: no Grant to return
        return Grant(self._domain, GrantResult.GRANTED, self._holder.req_id, to,
                     errors.OK, self._gen, self._holder_snapshot(), None, forced,
                     now_mono_ms)                        # the promoted holder's grant

    def _pop_highest_waiter(self) -> Optional[_Waiter]:
        """Remove and return the winning waiter: highest priority, then highest
        urgency, then earliest arrival. urgency breaks ties only at equal
        priority (11 S7A.1), and it is compared only inside this selection."""
        if not self._waiting:
            return None                                 # empty queue
        # max() keeps the first maximal element on ties; ordering the key so a
        # LATER arrival sorts lower (negated since_mono_ms) makes the earliest
        # arrival win an otherwise exact tie.
        best = max(self._waiting,
                   key=lambda w: (w.priority, w.urgency, -w.since_mono_ms))
        self._waiting.remove(best)                      # take it out of the queue
        return best                                     # the one to promote

    def _make_event(self, action, from_source, to_source, reason, forced,
                    detail, now_mono_ms) -> ArbEvent:
        """Build one audit record. gen is read AFTER the change so the event
        carries the generation the change produced."""
        # dict(detail) copies so two events cannot alias one detail dict.
        return ArbEvent(action.value, self._domain, from_source, to_source,
                        reason, forced, self._gen, now_mono_ms, dict(detail))

    def _holder_snapshot(self) -> Optional[Holder]:
        """A frozen copy of the live holder, so no caller can reach in and
        mutate the state machine's own record."""
        h = self._holder                                # the live record or None
        if h is None:
            return None                                 # idle domain
        return Holder(h.source_id, h.req_id, h.priority, h.since_mono_ms,
                      h.lease_ms)                        # immutable snapshot

    def _denied(self, req, source_id, code) -> Grant:
        """A denial Grant. No holder, no change, carries the current gen so the
        requester sees the generation it lost to."""
        return Grant(self._domain, GrantResult.DENIED, req.req_id, source_id,
                     code, self._gen, self._holder_snapshot(), None, False,
                     req.mono_ms)                        # holder unchanged, code set

    def _granted_now(self, req, source_id, forced) -> Grant:
        """A granted Grant for the current holder (free grant or holder
        re-acquire). Built from live state after the change was committed."""
        return Grant(self._domain, GrantResult.GRANTED, req.req_id, source_id,
                     errors.OK, self._gen, self._holder_snapshot(), None, forced,
                     req.mono_ms)                        # granted to the requester
