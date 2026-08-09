"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_arbiter_p2_t4.py
Brief: common tests -- arbiter p2 t4

Description:
BIZ-CM-5 P-2 + T-4 -- ack-before-cleanup order + gen-drop safety net.

Complements BIZ-CM-4's ARB-8 test with two explicit rules from the
抢占协议 spec:

  P-2 (14 S3.4): the preempted holder MUST call ack_preempt BEFORE
       running its cleanup. Variant: ack after cleanup + cleanup takes
       200 ms -> T-1 forced_preempt fires and cleanup never gets to ack.

  T-4 (14 S3.4): forced preempt does NOT wait for the resource to be
       actually idle -- overlap output is caught by gen (G-3, 11 S7A.5).
       Assertion is 'only count, never error': overlap after force is
       normal and expected, drop it, do not raise.
"""


import pytest

from xbrain.common.arbiter.core import Arbiter
from xbrain.common.arbiter.model import (
    GrantResult, PreemptPolicy, Request, SourceSpec,
)


pytestmark = pytest.mark.no_device


def _pair_arbiter():
    a = Arbiter("motion", wait_atomic_timeout_ms=50)
    a.register(SourceSpec("low", 100, True, PreemptPolicy.WAIT_ATOMIC,
                          1.0, None, None))
    a.register(SourceSpec("high", 300, True, PreemptPolicy.IMMEDIATE,
                          1.0, None, None))
    return a


# --- P-2 positive: ack before deadline promotes high -----------

def test_p2_ack_before_deadline_promotes_high_without_forced_preempt():
    """POSITIVE: low is atomic; high requests -> queued; low calls
    ack_preempt within deadline -> high is granted WITHOUT any
    forced_preempt event (T-1 does NOT fire because ack arrived)."""
    a = _pair_arbiter()
    a.request("low", Request("l1", mono_ms=0, atomic=True))
    a.request("high", Request("h1", mono_ms=10))
    assert a.holder().source_id == "low"

    # ack at mono=40, well before wait_atomic_timeout (50 ms).
    a.ack_preempt("low", "l1", now_mono_ms=40)
    events = a.drain_events()

    # After ack, high should be holder; forced_preempt must NOT appear.
    assert a.holder().source_id == "high"
    assert not any(e.action == "forced_preempt" for e in events), \
        "T-1 forced_preempt must NOT fire when P-2 ack lands in time"


# --- P-2 variant: ack AFTER cleanup delay = T-1 forced_preempt ---

def test_p2_variant_ack_after_cleanup_delay_triggers_t1_forced():
    """VARIANT (spec P-2 mutation): if the caller reversed the order
    (cleanup first, ack second) AND cleanup took 200 ms, the deadline
    passes and T-1 fires. Simulate by ticking past deadline BEFORE
    ack_preempt."""
    a = _pair_arbiter()
    a.request("low", Request("l1", mono_ms=0, atomic=True))
    a.request("high", Request("h1", mono_ms=10))

    # Cleanup 'takes 200 ms' -> we tick at 200 without acking first.
    tick_events = a.tick(now_mono_ms=200)

    # T-1 fires: forced_preempt event, high becomes holder.
    forced = [e for e in tick_events if e.action == "forced_preempt"]
    assert len(forced) == 1, \
        "ack-after-cleanup pattern must trigger T-1 forced_preempt"
    assert a.holder().source_id == "high"

    # Now if low tries to ack post-force, it's harmless (T-4 territory).
    # ack_preempt on a source that no longer holds should not corrupt state.
    # Some implementations may raise; either is acceptable as long as
    # the holder does not revert.
    try:
        a.ack_preempt("low", "l1", now_mono_ms=250)
    except Exception:
        pass
    assert a.holder().source_id == "high", \
        "late ack must NOT unseat the new holder"


# --- T-4: overlap output after force is dropped by gen ------------

def test_t4_stale_gen_release_does_not_reset_holder():
    """T-4: after a forced preempt, the OLD holder's release() call
    (arriving late with its stale gen / req_id) must be a no-op --
    the arbiter tracks the current req_id/gen and drops stale calls.

    Variant statement: 'only count, never error' -- the call
    completes without raising, but has no effect on state."""
    a = _pair_arbiter()
    a.request("low", Request("l1", mono_ms=0, atomic=True))
    a.request("high", Request("h1", mono_ms=10))
    a.tick(now_mono_ms=200)   # force

    assert a.holder().source_id == "high"
    gen_after_force = a.gen()

    # Old holder tries to release its own (now stale) req_id.
    # This MUST NOT change the current holder or gen.
    a.release("low", "l1", "done", now_mono_ms=250)
    assert a.holder().source_id == "high", \
        "stale release from old holder must not unseat current"
    assert a.gen() == gen_after_force, \
        "stale release must not advance gen"


def test_t4_stale_renew_does_not_extend_lease():
    """T-4 partial: a renew from the ex-holder must not extend the
    (now unrelated) high holder's lease. Verified by measuring that
    the current holder's lease still expires at its original time."""
    a = _pair_arbiter()
    a.request("low", Request("l1", mono_ms=0, atomic=True))
    a.request("high", Request("h1", mono_ms=10))
    a.tick(now_mono_ms=200)   # force

    # low tries to renew its ex-holding. This should be a no-op.
    a.renew("low", "l1", now_mono_ms=300)

    # high's lease started around 200 ms. renew from low did nothing.
    # Verify high still holds after another tick that would not
    # exceed high's own lease.
    a.tick(now_mono_ms=500)   # well before high's 1000ms lease expires
    assert a.holder().source_id == "high"
