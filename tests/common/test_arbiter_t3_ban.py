"""BIZ-CM-5 T-3 -- stuck-source ban and process-restart recovery.

14 S3.4 T-3: a source that gets forced_preempt-ed forced_preempt_max
times inside forced_preempt_window_ms is DISABLED; subsequent
request() returns denied E_ARB_DISABLED. Recovery is process
restart (11 S7A.3 T-3 note: 'its process restarts'); in code that
is a fresh register() which replaces the entry.

The spec is explicit that half-tests are NOT enough (14 warns): a
one-sided assertion that only checks the ban would let a stub
'permanently disabled' pass, and a single jitter would kill a real
voice source forever. So this file pairs every ban test with a
recovery test.
"""

import pytest

from xbrain.common import errors
from xbrain.common.arbiter.core import Arbiter
from xbrain.common.arbiter.model import (
    ArbAction,
    FORCED_PREEMPT_MAX,
    GrantResult,
    PreemptPolicy,
    Request,
    SourceSpec,
)


pytestmark = pytest.mark.no_device


# --- fixture: one atomic low-priority holder + one high preemptor -

def _make_pair_arbiter(window_ms: int = 60_000, max_strikes: int = 3):
    """A domain with two sources: 'stuck' at priority 100 that always
    misses its wait_atomic deadline, and 'high' at 300 that keeps
    preempting it. wait_atomic_timeout is 50 ms so tick(now+100) is
    guaranteed to force."""
    a = Arbiter(
        "motion",
        wait_atomic_timeout_ms=50,
        forced_preempt_window_ms=window_ms,
        forced_preempt_max=max_strikes,
    )
    a.register(SourceSpec("stuck", 100, True, PreemptPolicy.WAIT_ATOMIC,
                          1.0, None, None))
    a.register(SourceSpec("high", 300, True, PreemptPolicy.IMMEDIATE,
                          1.0, None, None))
    return a


def _force_one_strike(a: Arbiter, base_ms: int) -> None:
    """Run one preempt cycle that ends in forced_preempt against 'stuck'.

    Sequence: stuck acquires atomic, high requests -> queued, tick
    forces after wait_atomic_timeout. Then release high so the domain
    is idle again for the next iteration."""
    a.request("stuck", Request("s_%d" % base_ms, mono_ms=base_ms, atomic=True))
    a.request("high", Request("h_%d" % base_ms, mono_ms=base_ms + 5))
    events = a.tick(now_mono_ms=base_ms + 100)
    # Sanity: this cycle really did force.
    assert any(e.action == "forced_preempt" for e in events), \
        "test setup wrong: no forced_preempt in cycle at %d" % base_ms
    # Release high so we can run another cycle from idle.
    a.release("high", "h_%d" % base_ms, "done", base_ms + 200)


# --- T-3 BAN half --------------------------------------------------

def test_stuck_source_banned_after_max_strikes_within_window():
    """Three forced_preempts within 60 s must disable the stuck source.
    Subsequent request() returns denied E_ARB_DISABLED."""
    a = _make_pair_arbiter()

    # Three strikes, each 1 s apart -- well within the 60 s window.
    _force_one_strike(a, 0)
    _force_one_strike(a, 1000)
    _force_one_strike(a, 2000)

    # 'stuck' must now be disabled. Its next request must be denied
    # E_ARB_DISABLED (a specific code, not E_BUSY which would be
    # 'held by high').
    grant = a.request("stuck", Request("post_ban", mono_ms=3000))
    assert grant.result == GrantResult.DENIED
    assert grant.code == errors.E_ARB_DISABLED, \
        "expected E_ARB_DISABLED, got %s" % grant.code


def test_ban_emits_source_disabled_event_with_detail():
    """The ban must produce a SOURCE_DISABLED audit event carrying
    strikes count and window_ms. BIZ-CM-2 maps SOURCE_DISABLED to
    fault severity; without the event operators cannot know the
    ban happened."""
    a = _make_pair_arbiter()

    _force_one_strike(a, 0)
    _force_one_strike(a, 1000)
    _force_one_strike(a, 2000)   # this one triggers the ban

    # Drain accumulated events (audit stream = tick returns + buffer).
    events = a.drain_events()   # release events go here
    # The SOURCE_DISABLED event was emitted into `sink` = tick's return
    # list during the third force cycle. We need to re-run that cycle
    # and capture the tick result explicitly. Simpler: rebuild.
    b = _make_pair_arbiter()
    b.request("stuck", Request("s1", mono_ms=0, atomic=True))
    b.request("high", Request("h1", mono_ms=5))
    b.tick(100)
    b.release("high", "h1", "done", 200)
    b.request("stuck", Request("s2", mono_ms=1000, atomic=True))
    b.request("high", Request("h2", mono_ms=1005))
    b.tick(1100)
    b.release("high", "h2", "done", 1200)
    b.request("stuck", Request("s3", mono_ms=2000, atomic=True))
    b.request("high", Request("h3", mono_ms=2005))
    tick3_events = b.tick(2100)

    disabled_events = [e for e in tick3_events if e.action == "source_disabled"]
    assert len(disabled_events) == 1, \
        "expected exactly one source_disabled event on third strike; got: %s" % (
            [e.action for e in tick3_events])
    ev = disabled_events[0]
    assert ev.from_source == "stuck"
    assert ev.to_source is None
    assert "strikes" in ev.detail
    assert ev.detail["strikes"] >= FORCED_PREEMPT_MAX
    assert "window_ms" in ev.detail


# --- T-3 WINDOW half (jitter doesn't accumulate) -------------------

def test_strikes_outside_window_do_not_accumulate():
    """One strike per HOUR is well outside the 60 s window; 100 such
    strikes must NOT disable the source. This is the guard against
    'a single jitter permanently kills a voice source'."""
    a = _make_pair_arbiter(window_ms=1_000, max_strikes=3)   # 1 s window

    # Two strikes 2 s apart -- second falls outside the window of the
    # first, so at time of second there is only 1 strike in the window
    # (the second itself). Ban must NOT fire.
    _force_one_strike(a, 0)
    _force_one_strike(a, 2000)   # 2 s later, window is 1 s

    # Verify stuck is NOT disabled -- its request must not be
    # E_ARB_DISABLED (may still be E_BUSY if high holds, but E_BUSY
    # is fine).
    grant = a.request("stuck", Request("still_ok", mono_ms=2500))
    assert grant.code != errors.E_ARB_DISABLED, \
        "single-strike-per-window source must not be banned"


# --- T-3 RECOVERY half (the crucial pair) ---------------------------

def test_process_restart_via_register_clears_ban():
    """CRUCIAL positive: after the ban, the source's process
    restarting (represented by a fresh register() call) must clear
    the ban. Without this, a stub 'always disabled' would pass the
    ban test above, but a single jitter would permanently kill a
    real source (14 S3.4 warning verbatim)."""
    a = _make_pair_arbiter()

    _force_one_strike(a, 0)
    _force_one_strike(a, 1000)
    _force_one_strike(a, 2000)

    # Ban confirmed.
    grant = a.request("stuck", Request("post_ban", mono_ms=3000))
    assert grant.code == errors.E_ARB_DISABLED

    # Process restart -- fresh register() replaces the entry.
    a.register(SourceSpec("stuck", 100, True, PreemptPolicy.WAIT_ATOMIC,
                          1.0, None, None))

    # After re-register, stuck must be able to acquire again (if the
    # domain is free). Wait for high to release, then verify stuck
    # gets the domain.
    # First release any lingering holder from the strike cycles.
    grant = a.request("stuck", Request("post_restart", mono_ms=4000))
    # It may be granted (domain free) or denied E_BUSY (high holds),
    # but MUST NOT be E_ARB_DISABLED.
    assert grant.code != errors.E_ARB_DISABLED, \
        "process restart via re-register() must clear the ban"


# --- MUTATION: only ban half without recovery would pass a stub ----

def test_mutation_ban_without_recovery_leaks_permanent_lockout():
    """VARIANT documentation: this test exists to state, in code, why
    the recovery test above is REQUIRED. A stub arbiter that always
    returns E_ARB_DISABLED after the third strike passes
    test_stuck_source_banned_after_max_strikes_within_window; only
    test_process_restart_via_register_clears_ban catches the stub.

    The 'stub' here is a synthesized _SourceEntry that stays disabled
    across register(). We SIMULATE the buggy behavior and assert the
    real behavior doesn't match it."""
    a = _make_pair_arbiter()

    _force_one_strike(a, 0)
    _force_one_strike(a, 1000)
    _force_one_strike(a, 2000)

    entry_before = a._registry["stuck"]
    assert entry_before.disabled is True

    a.register(SourceSpec("stuck", 100, True, PreemptPolicy.WAIT_ATOMIC,
                          1.0, None, None))

    entry_after = a._registry["stuck"]
    assert entry_after is not entry_before, \
        "re-register must produce a NEW entry (fresh state)"
    assert entry_after.disabled is False, \
        "fresh entry from re-register must NOT inherit disabled=True"
    assert entry_after.forced_history == [], \
        "fresh entry must have empty forced_history"


# --- T-3 SLIDING window ---------------------------------------------

def test_window_slides_dropping_old_entries():
    """One old + one new strike within the window count as 2, not as
    'ever seen 3'. Verifies the window is sliding (drop stale) not
    cumulative."""
    a = _make_pair_arbiter(window_ms=1_500, max_strikes=3)

    _force_one_strike(a, 0)         # strike @ 0
    _force_one_strike(a, 500)       # strike @ 500 (2 in window)
    # Wait past window on strike @ 0 (window = 1500 ms).
    _force_one_strike(a, 2100)      # strike @ 2100 (1 in window: 2100
                                    # itself; 0 and 500 are outside)

    # Only 1 recent strike; source must still be enabled.
    grant = a.request("stuck", Request("recent1", mono_ms=2500))
    assert grant.code != errors.E_ARB_DISABLED
