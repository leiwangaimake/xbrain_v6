"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_arbiter.py
Brief: Behaviour and mutation tests for the arbitration framework core (11 S7A)

Description:
What problem this solves. BIZ-CM-1 delivers the arbitration core, and CLAUDE.md
3.3 is explicit that an assertion is not written until a mutation makes it go red
-- only-positive assertions pass an empty shell. This file therefore does two
things for every guarantee the core makes: it asserts the reference behaviour is
green, and it builds a mutant that reintroduces the specific defect and asserts
the SAME guard would go red against it. The four mutations named in the BIZ-CM-1
done-criterion each get a subclass and a confirmed-red test:

  (1) tick() reads time.time() instead of its monotonic argument
      => the seven-domain holders are all judged dead (AB-6).
  (2) renew() also does gen += 1
      => the G-1 "renew does not advance gen" assertion goes red.
  (3) a resident source (lease_timeout_s None) is given a real lease
      => "mode_driver held 60 s without renew still holds" goes red.
  (4) the immediate preempt deadline is unbounded
      => the T-1 forced-preempt assertion goes red.

Plus the double detection (14 C-2): the lease path and the process-heartbeat path
each reclaim a held resource on their own, shown by a case the lease cannot cover
(a resident holder) that the heartbeat still reaps.

Each mutant is a subclass overriding ONE method, not an edit to shipped source,
so the reference stays exactly as deployed while the defect is exercised against a
real object through the real public API. That the mutant is a subclass rather than
a monkeypatch is deliberate: it is visible at its definition and cannot leak into
another test.
"""

import os                            # for the ROOT path derivation below
import sys                           # to put the repo root on sys.path
import time                          # only the AB-6 mutant reads the wall clock

import pytest                        # raises-assertions and the runner

# ROOT is three levels up from tests/common/test_arbiter.py, same derivation the
# other common tests use, so `from xbrain...` resolves whether pytest is invoked
# from the repo root or from tests/.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)             # make the xbrain package importable

from xbrain.common.arbiter import (  # noqa: E402  -- import after sys.path edit
    Arbiter, ArbAction, DEFAULT_LEASE_MS, GrantResult, IMMEDIATE_GRACE_MS,
    PreemptPolicy, Request, SourceSpec,
)
from xbrain.common.enums import DOMAIN  # noqa: E402  -- the seven-domain closed set
from xbrain.common import errors  # noqa: E402  -- E_BUSY / E_ARB_NO_SOURCE checks


# -- builders ------------------------------------------------------------------
# Defaults live on these TEST helpers, never on SourceSpec itself (SourceSpec
# requires every field, by design -- see model.py trap 1). A helper default is
# fine: it is test convenience, not a production fallback.

def _spec(source_id, priority, *, policy=PreemptPolicy.IMMEDIATE, lease_s=1.0,
          preemptible=True, on_preempt=None, on_lost=None):
    """A SourceSpec with test-friendly defaults."""
    # Positional order matches the frozen dataclass: id, priority, preemptible,
    # policy, lease, then the two callbacks.
    return SourceSpec(source_id, priority, preemptible, policy, lease_s,
                      on_preempt, on_lost)


def _req(req_id, mono_ms, **kw):
    """A Request; optional fields carry the contract's own defaults."""
    # urgency/lease_ms/atomic/reason/est_ms default on Request itself (11 S7A.1).
    return Request(req_id, mono_ms, **kw)


# -- the four named mutants ----------------------------------------------------

class _WallClockTickArbiter(Arbiter):
    """Mutation (1), AB-6: tick reads the wall clock instead of its argument.

    This is the exact defect the monotonic rule exists to prevent: the moment
    chronyd steps the wall clock forward on RTK lock, a wall-clock lease check
    sees now jump seconds-to-years ahead of the last renew and reclaims every
    domain at once.
    """

    def tick(self, now_mono_ms):
        # int(time.time() * 1000) is wall-clock milliseconds, ~1.7e12, which is
        # more than 3600 s -- far more -- ahead of any monotonic value the test
        # feeds the reference. That gap is the injected forward step.
        return super().tick(int(time.time() * 1000))   # the defect: wall, not arg


class _RenewBumpsGenArbiter(Arbiter):
    """Mutation (2), G-1: renew advances gen, which it must never do."""

    def renew(self, source_id, req_id, now_mono_ms):
        ok = super().renew(source_id, req_id, now_mono_ms)   # do the real renew
        # The defect: a renew is not a holder change, so this bump is wrong and
        # would make the executor discard the holder's own live output.
        self._gen += 1                                 # the mutation
        return ok                                      # preserve the return type


class _LeaseKillsResidentArbiter(Arbiter):
    """Mutation (3): a resident source (lease None) is given a real lease."""

    def _effective_lease_ms(self, spec, req_lease_ms):
        if spec.lease_timeout_s is None:               # the resident case
            # The defect: None means resident and must stay None. Handing back a
            # number puts mode_driver on a lease it will never renew.
            return DEFAULT_LEASE_MS                     # the mutation
        return super()._effective_lease_ms(spec, req_lease_ms)   # else unchanged


class _UnboundedImmediateDeadlineArbiter(Arbiter):
    """Mutation (4), T-1: the immediate preempt deadline is unbounded."""

    def _preempt_deadline_ms(self, holder_spec, now_mono_ms):
        if holder_spec.preempt_policy is PreemptPolicy.IMMEDIATE:   # the immediate case
            # The defect: no deadline means tick never forces the preemption, so
            # a holder that refuses to ack blocks the higher source forever.
            return None                                 # the mutation
        return super()._preempt_deadline_ms(holder_spec, now_mono_ms)   # else unchanged


# -- (1) AB-6: monotonic time base ---------------------------------------------

def _seven_domains_each_holding(cls, t0):
    """Build one arbiter per contract domain, each with a leased holder acquired
    at t0. Returns the list of arbiters. Used by both the reference assertion and
    the mutant confirmation so the two run the identical setup."""
    arbiters = []                                       # one Arbiter per domain
    for domain in DOMAIN.values:                        # the seven closed-set names
        a = cls(domain, 3000)                           # class is real or mutant
        # 1 s lease, the contract default; the point of the test is that a lease
        # measured in monotonic ms is untouched by a wall-clock step.
        a.register(_spec("holder", 500, lease_s=1.0))   # a single leased source
        a.request("holder", _req("r", t0))              # acquire at t0
        arbiters.append(a)                              # collect it
    return arbiters                                     # seven arbiters, each held


def test_ab6_wall_clock_forward_step_does_not_kill_seven_domain_holders():
    """AB-6 (11 S7A.1 / 11 S7A.10). The reference reads no clock: tick is fed a
    monotonic value 200 ms after the grant, well inside the 1 s lease, so all
    seven holders survive.

    The wall clock, meanwhile, sits ~1.7e9 s away from these monotonic values --
    a forward step far larger than the seconds-to-years RTK step the rule guards
    against. Because the reference never reads it, the step cannot enter, and the
    holders are alive. Mutation (1) is confirmed red below.
    """
    t0 = 5_000_000            # a plausible monotonic ms reading (~5000 s uptime)
    arbiters = _seven_domains_each_holding(Arbiter, t0)   # the reference class
    for a in arbiters:
        # 200 ms of MONOTONIC time; the wall clock may have jumped arbitrarily.
        a.tick(t0 + 200)                                # inside the 1 s lease
    survivors = [a.holder().source_id                   # collect the survivors
                 for a in arbiters if a.holder() is not None]
    assert len(survivors) == len(DOMAIN.values), (      # all seven must survive
        "a monotonic-based arbiter must keep all seven holders across a "
        "wall-clock step; survivors=%r" % survivors)


def test_ab6_mutation_confirmed_red_wall_clock_kills_all_holders():
    """Confirms mutation (1) turns the assertion above red: with tick reading the
    wall clock, the same 200 ms monotonic tick finds now ~1.7e12 ms against a last
    renew of 5e6 ms and reclaims every domain."""
    t0 = 5_000_000                                      # same setup as the guard
    arbiters = _seven_domains_each_holding(_WallClockTickArbiter, t0)   # mutant
    for a in arbiters:
        a.tick(t0 + 200)                                # mutant reads wall here
    survivors = [a for a in arbiters if a.holder() is not None]   # who is left
    assert survivors == [], (                           # none: the guard goes red
        "the wall-clock mutant should have judged every holder dead, so the "
        "AB-6 guard would go red")


# -- (2) G-1: renew does not advance gen ---------------------------------------

def test_g1_renew_does_not_advance_gen():
    """G-1 (11 S7A.2): gen advances on a holder change and NOT on a renew. The
    holder acquires (gen -> 1), renews three times, and gen stays 1 -- the
    holder never changed."""
    a = Arbiter("speaker", 3000)                        # a real arbiter
    a.register(_spec("tts", 600, policy=PreemptPolicy.WAIT_ATOMIC, lease_s=1.0))
    a.request("tts", _req("r1", 1000))                  # acquire: gen becomes 1
    g0 = a.gen()                                        # capture the post-grant gen
    assert g0 == 1                     # first grant is the first holder change
    for t in (1300, 1600, 1900):                        # three renews within lease
        assert a.renew("tts", "r1", t) is True          # each is accepted
    assert a.gen() == g0, "renew must not advance gen (G-1)"   # gen unchanged


def test_g1_mutation_confirmed_red_renew_bumps_gen():
    """Confirms mutation (2): with renew bumping gen, three renews move gen off
    its post-grant value, which is what the G-1 assertion forbids."""
    a = _RenewBumpsGenArbiter("speaker", 3000)          # the mutant
    a.register(_spec("tts", 600, policy=PreemptPolicy.WAIT_ATOMIC, lease_s=1.0))
    a.request("tts", _req("r1", 1000))                  # acquire
    g0 = a.gen()                                        # gen after the grant
    for t in (1300, 1600, 1900):                        # three renews
        a.renew("tts", "r1", t)                         # each bumps gen (the defect)
    assert a.gen() != g0, (                             # gen moved: guard goes red
        "the renew-bumps-gen mutant should have advanced gen, turning the G-1 "
        "guard red")


# -- (3) resident source has no lease ------------------------------------------

def test_resident_source_holds_60s_without_renew():
    """11 S7A.4: a source registered with lease_timeout_s None is resident and is
    never lease-reclaimed. mode_driver holds domain 4 for P2's whole life and
    renewing it 2 Hz would be pure spin (14 S4.3); here it holds 60 s with no
    renew and no tick reclaims it."""
    a = Arbiter("payload_light", 3000)                  # a real arbiter
    a.register(_spec("mode_driver", 900, lease_s=None))   # None == resident
    a.request("mode_driver", _req("d1", 1000))          # acquire at t=1000
    a.tick(1000 + 60_000)              # 60 s later, never renewed
    assert a.holder() is not None and a.holder().source_id == "mode_driver", (
        "a resident (lease None) source must not be lease-reclaimed")   # still held


def test_resident_lease_mutation_confirmed_red_kills_mode_driver():
    """Confirms mutation (3): giving the resident source a real lease reclaims it
    the moment the lease elapses, so the '60 s still holds' assertion goes red."""
    a = _LeaseKillsResidentArbiter("payload_light", 3000)   # the mutant
    a.register(_spec("mode_driver", 900, lease_s=None))   # resident registration
    a.request("mode_driver", _req("d1", 1000))          # acquire
    a.tick(1000 + 60_000)                               # long past the injected lease
    assert a.holder() is None, (                        # reclaimed: guard goes red
        "the resident-gets-a-lease mutant should have reclaimed mode_driver, "
        "turning the resident guard red")


# -- (4) T-1 forced preempt on a missed immediate deadline ---------------------

def _preempt_setup(cls):
    """auto (500, immediate) holds; manual (700, immediate) requests and is
    queued with an ack deadline. Returns (arbiter, notified, grant) where
    notified records the preempt notification, so the P-1 call is observable."""
    notified = []                                       # records on_preempt(by)
    a = cls("ptz", 3000)                                # real or mutant arbiter
    a.register(_spec("auto", 500, policy=PreemptPolicy.IMMEDIATE, lease_s=1.0,
                     on_preempt=lambda by: notified.append(by)))   # the holder
    a.register(_spec("manual", 700, policy=PreemptPolicy.IMMEDIATE, lease_s=30.0))
    a.request("auto", _req("a1", 1000))                 # auto takes the domain
    g = a.request("manual", _req("m1", 1000))           # manual preempts, queued
    return a, notified, g                               # hand back for assertions


def test_t1_forced_preempt_when_holder_misses_immediate_deadline():
    """T-1 (11 S7A.3): the immediate deadline is now + 100 ms; a holder that does
    not ack by then is force-revoked and the waiter is granted forced. The queued
    grant carries that deadline, the holder is notified (P-1), and a tick past the
    deadline installs manual with forced=True and a warn event carrying
    overdue_ms."""
    a, notified, g = _preempt_setup(Arbiter)            # the reference class
    assert g.result is GrantResult.QUEUED               # manual is queued, not granted
    assert g.preempt.deadline_mono_ms == 1000 + IMMEDIATE_GRACE_MS   # now + 100
    assert notified == ["manual"], "the holder must get the P-1 preempt notice"
    # Still the holder before the deadline; no premature handover.
    assert a.tick(1000 + 50) == []                      # 50 ms in: nothing fires
    assert a.holder().source_id == "auto"               # auto still holds
    # Past the deadline (no ack): forced preempt.
    out = a.tick(1000 + 200)                            # 200 ms in: deadline passed
    assert a.holder().source_id == "manual"             # manual is now the holder
    assert a.last_change().forced is True               # marked as a forced change
    assert a.last_change().to_source == "manual"        # to the preemptor
    forced = [e for e in out if e.action == ArbAction.FORCED_PREEMPT.value]
    assert len(forced) == 1 and forced[0].detail["overdue_ms"] == 100   # T-2 detail


def test_t1_mutation_confirmed_red_unbounded_deadline_never_forces():
    """Confirms mutation (4): with the immediate deadline unbounded, the same
    tick past 100 ms never forces the handover, so auto keeps the domain and the
    T-1 assertion goes red."""
    a, _notified, g = _preempt_setup(_UnboundedImmediateDeadlineArbiter)   # mutant
    assert g.preempt.deadline_mono_ms is None      # the defect is visible here
    out = a.tick(1000 + 200)                            # would-be past the deadline
    assert out == [] and a.holder().source_id == "auto", (   # nothing forced: red
        "the unbounded-deadline mutant should never force the preempt, turning "
        "the T-1 guard red")


# -- double detection: lease AND heartbeat, each on its own --------------------

def test_lease_path_reclaims_a_stuck_but_live_holder():
    """Detection path 1 (11 S7A.4, 14 C-2): a holder that is alive but has stopped
    renewing is reclaimed by the lease in tick(). on_lost fires and a warn
    lease_timeout event is produced. This path alone catches 'alive but stuck'."""
    lost = []                                           # records the on_lost call
    a = Arbiter("speaker", 3000)                        # a real arbiter
    a.register(_spec("tts", 600, lease_s=1.0, on_lost=lambda: lost.append(1)))
    a.request("tts", _req("r1", 1000))                  # acquire at t=1000
    out = a.tick(1000 + 1001)          # one ms past the 1 s lease
    assert a.holder() is None                           # reclaimed
    assert lost == [1], "on_lost must fire on an involuntary lease reclaim"
    assert [e.action for e in out] == [ArbAction.LEASE_TIMEOUT.value]   # warn event


def test_heartbeat_path_reclaims_independently_of_the_lease():
    """Detection path 2 (11 S7A.4, 14 C-2): process death reclaims a holder even
    when the lease never would. The holder here is RESIDENT (lease None), so the
    lease path is structurally incapable of reclaiming it -- ticking 100 s leaves
    it in place -- and only the heartbeat reap frees it. That the two paths are
    independent is the whole point of having both: lease covers 'stuck', heartbeat
    covers 'gone'."""
    lost = []                                           # records the on_lost call
    a = Arbiter("payload_light", 3000)                  # a real arbiter
    a.register(_spec("mode_driver", 900, lease_s=None,   # resident source
                     on_lost=lambda: lost.append(1)))
    a.request("mode_driver", _req("d1", 1000))          # acquire
    # The lease path cannot touch a resident holder, no matter how long.
    assert a.tick(1000 + 100_000) == []                 # 100 s: lease never fires
    assert a.holder().source_id == "mode_driver"        # still held
    # The heartbeat path reclaims it regardless.
    out = a.reap_dead_source("mode_driver", 1000 + 100_001)   # process died
    assert a.holder() is None                           # now reclaimed
    assert lost == [1]                                  # on_lost fired
    assert [e.action for e in out] == [ArbAction.SOURCE_DEATH.value]   # fault event


def test_heartbeat_reaps_a_leased_holder_before_its_lease_expires():
    """The heartbeat is independent of lease TIMING too: a leased holder whose
    process dies is reaped at once, not after waiting out its lease. Reaping at
    t0 + 100 ms, well inside a 1 s lease, still frees the domain."""
    a = Arbiter("asr", 3000)                            # a real arbiter
    a.register(_spec("asr_cloud", 600, lease_s=1.0))    # a 1 s lease
    a.request("asr_cloud", _req("r1", 1000))            # acquire
    a.reap_dead_source("asr_cloud", 1100)      # inside the lease
    assert a.holder() is None                           # reaped anyway


# -- the rest of the public surface --------------------------------------------

def test_gen_starts_at_zero_and_only_advances_on_holder_change():
    """G-4: gen starts at 0 (idle domain) and the first grant makes it 1. Each
    later holder change adds one; a denial does not."""
    a = Arbiter("gpu", 3000)                            # a real arbiter
    assert a.gen() == 0                                 # idle domain starts at 0
    a.register(_spec("llm", 0, lease_s=10.0))           # two equal-priority sources
    a.register(_spec("other", 0, lease_s=10.0))
    a.request("llm", _req("r1", 1000))                  # first grant
    assert a.gen() == 1                                 # gen advanced once
    # A denied equal-priority request changes nothing.
    d = a.request("other", _req("r2", 1000))            # equal priority: denied
    assert d.result is GrantResult.DENIED and a.gen() == 1   # no change on denial


def test_unregistered_source_is_denied_no_source():
    """11 S7A.1: a source that never registered is denied E_ARB_NO_SOURCE, and
    the constant comes from the shared error library, never a literal."""
    a = Arbiter("ptz", 3000)                            # no sources registered
    g = a.request("ghost", _req("r1", 1000))            # an unknown source_id
    assert g.result is GrantResult.DENIED               # denied
    assert g.code == errors.E_ARB_NO_SOURCE             # with the shared code


def test_lower_priority_is_denied_busy_and_holder_is_unchanged():
    """ARB-2: a lower priority request cannot take a held domain; it is denied
    E_BUSY and the holder and gen are untouched."""
    a = Arbiter("speaker", 3000)                        # a real arbiter
    a.register(_spec("alarm", 900))                     # high-priority holder
    a.register(_spec("tts", 600))                       # lower-priority requester
    a.request("alarm", _req("a1", 1000))                # alarm takes it
    g = a.request("tts", _req("t1", 1000))              # tts tries, lower
    assert g.result is GrantResult.DENIED and g.code == errors.E_BUSY   # E_BUSY
    assert a.holder().source_id == "alarm" and a.gen() == 1   # holder unchanged


def test_equal_priority_cannot_preempt():
    """ARB-2 at the boundary: equal priority does not preempt. urgency orders the
    queue among equals but never unseats a holder."""
    a = Arbiter("gpu", 3000)                            # a real arbiter
    a.register(_spec("a", 500))                         # two equal-priority sources
    a.register(_spec("b", 500))
    a.request("a", _req("a1", 1000))                    # a holds
    g = a.request("b", _req("b1", 1000, urgency=9))     # b, equal, high urgency
    assert g.result is GrantResult.DENIED and a.holder().source_id == "a"   # a keeps it


def test_reject_source_is_granted_when_free_and_denied_when_busy():
    """14 S4.3: a reject source (the manual light source) takes the domain only
    when it is free; while a holder is present it is denied E_BUSY rather than
    queued, even though here it is the higher priority. It does not fight."""
    a = Arbiter("payload_light", 3000)                  # a real arbiter
    a.register(_spec("manual", 700, policy=PreemptPolicy.REJECT, lease_s=30.0))
    a.register(_spec("mode_driver", 900, lease_s=None))   # the resident driver
    # Free: granted.
    assert a.request("manual", _req("m1", 1000)).result is GrantResult.GRANTED
    a.release("manual", "m1", "user_cancel", 1100)      # manual lets go
    # mode_driver takes it; now manual (reject) is denied rather than queued.
    a.request("mode_driver", _req("d1", 1200))          # driver takes the domain
    g = a.request("manual", _req("m2", 1300))           # manual tries again
    assert g.result is GrantResult.DENIED and g.code == errors.E_BUSY   # denied, not queued


def test_ack_preempt_completes_the_handover_in_one_gen_bump():
    """P-2 (11 S7A.3): the preempted holder acks and the waiter is installed as
    holder. It is one holder change -- gen goes from the grant value straight to
    the next, not two -- and the preempted holder's on_lost is NOT called,
    because acking is a voluntary, graceful exit."""
    lost = []                                           # would record on_lost
    a = Arbiter("ptz", 3000)                            # a real arbiter
    a.register(_spec("auto", 500, on_lost=lambda: lost.append(1)))   # the holder
    a.register(_spec("manual", 700, lease_s=30.0))      # the preemptor
    a.request("auto", _req("a1", 1000))                 # auto holds
    g_before = a.gen()                                  # gen after auto's grant
    a.request("manual", _req("m1", 1000))      # queued, auto notified
    assert a.gen() == g_before                 # queuing is not a holder change
    a.ack_preempt("auto", "a1", 1050)                   # auto acks the preempt
    assert a.holder().source_id == "manual"             # manual promoted
    assert a.gen() == g_before + 1             # exactly one change
    assert lost == [], "a graceful ack is not an involuntary loss"   # no on_lost


def test_cancel_removes_a_queued_request_and_stands_the_holder_down():
    """11 S7A.1: cancel withdraws a still-queued acquire. When the sole preemptor
    cancels, the holder is no longer under an ack deadline, so a later tick does
    not force anything."""
    a = Arbiter("ptz", 3000)                            # a real arbiter
    a.register(_spec("auto", 500))                      # the holder
    a.register(_spec("manual", 700, lease_s=30.0))      # the preemptor
    a.request("auto", _req("a1", 1000))                 # auto holds
    a.request("manual", _req("m1", 1000))      # queued, deadline now+100
    a.cancel("manual", "m1", 1050)                      # manual withdraws
    # Past where the deadline would have been: no forced preempt, auto holds.
    assert a.tick(1000 + 500) == []                     # nothing to force
    assert a.holder().source_id == "auto"               # auto still holds


def test_release_frees_the_domain_and_emits_one_release_event():
    """11 S7A.1: a voluntary release with no waiter idles the domain, bumps gen
    once, and buffers exactly one release event for the audit drain."""
    a = Arbiter("speaker", 3000)                        # a real arbiter
    a.register(_spec("tts", 600, lease_s=1.0))          # the holder
    a.request("tts", _req("r1", 1000))                  # tts holds
    g_before = a.gen()                                  # gen after the grant
    a.release("tts", "r1", "done", 1200)                # release, no waiter
    assert a.holder() is None and a.gen() == g_before + 1   # idle, one bump
    events = a.drain_events()                           # pull the buffered audit
    releases = [e for e in events if e.action == ArbAction.RELEASE.value]
    assert len(releases) == 1 and releases[0].from_source == "tts"   # one release


def test_release_with_an_off_contract_reason_raises():
    """The release_reason is validated against the closed set, not passed through
    (11 S13.6). An unknown reason raises rather than being silently accepted."""
    from xbrain.common.errors import ClosedSetViolation   # the boundary exception
    a = Arbiter("speaker", 3000)                        # a real arbiter
    a.register(_spec("tts", 600, lease_s=1.0))          # the holder
    a.request("tts", _req("r1", 1000))                  # tts holds
    with pytest.raises(ClosedSetViolation):             # bad reason must raise
        a.release("tts", "r1", "not_a_reason", 1200)


def test_protected_holder_is_not_taken_even_by_higher_priority():
    """preemptible False protects a holder: a strictly higher request is denied
    E_BUSY rather than preempting. No domain config uses this, but the field is
    honoured."""
    a = Arbiter("gpu", 3000)                            # a real arbiter
    a.register(_spec("guarded", 100, preemptible=False, lease_s=10.0))   # protected
    a.register(_spec("big", 900, lease_s=10.0))         # a much higher source
    a.request("guarded", _req("g1", 1000))              # guarded holds
    g = a.request("big", _req("b1", 1000))              # big tries to preempt
    assert g.result is GrantResult.DENIED and a.holder().source_id == "guarded"


def test_bad_domain_at_construction_raises():
    """The domain is validated against the closed set at construction, so a typo
    fails at startup with the value in the traceback, not at the first request."""
    from xbrain.common.errors import ClosedSetViolation   # the boundary exception
    with pytest.raises(ClosedSetViolation):             # a misspelt domain raises
        Arbiter("speakerr", 3000)                       # note the extra r


def test_higher_priority_preempts_lower_and_holder_is_notified():
    """The plain preempt path (not the timeout): a higher source queues, the
    lower holder is notified to stop (P-1), and holder does not change until the
    ack. gen does not move while the request is merely queued."""
    notified = []                                       # records on_preempt(by)
    a = Arbiter("speaker", 3000)                        # a real arbiter
    a.register(_spec("tts", 600, policy=PreemptPolicy.WAIT_ATOMIC, lease_s=1.0,
                     on_preempt=lambda by: notified.append(by)))   # the holder
    a.register(_spec("alarm", 900))                     # the higher source
    a.request("tts", _req("t1", 1000))                  # tts holds
    g_before = a.gen()                                  # gen after tts' grant
    g = a.request("alarm", _req("al1", 1000))           # alarm preempts
    assert g.result is GrantResult.QUEUED               # alarm is queued
    assert notified == ["alarm"]                        # tts got the P-1 notice
    assert a.holder().source_id == "tts" and a.gen() == g_before   # unchanged, no bump


def test_wait_atomic_deadline_is_the_domain_ceiling_not_the_immediate_grace():
    """11 S7A.3: preempting a wait_atomic holder gives it the domain's wait_atomic
    ceiling, not the 100 ms immediate grace. The queued grant's deadline reflects
    the HOLDER's policy."""
    a = Arbiter("speaker", 3000)            # wait_atomic ceiling 3000 ms
    a.register(_spec("tts", 600, policy=PreemptPolicy.WAIT_ATOMIC, lease_s=1.0))
    a.register(_spec("alarm", 900))                     # the preemptor
    a.request("tts", _req("t1", 1000))                  # tts (wait_atomic) holds
    g = a.request("alarm", _req("al1", 1000))           # alarm preempts
    assert g.preempt.deadline_mono_ms == 1000 + 3000    # ceiling, not 100 ms
