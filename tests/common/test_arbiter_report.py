"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_arbiter_report.py
Brief: BIZ-CM-2 -- severity map, the 10 s dedup-window merge, and the
       state/arb/{domain} assembly, each with the mutant that would defeat it

Description:
BIZ-CM-1 (test_arbiter.py) covers the state machine. This file covers the
serialisation BIZ-CM-2 adds on top: turning ArbEvents into the audit stream and
the arbiter snapshot into ArbDomainState (11 S6.1 / S7A.5.1 / S7A.7, 14 S3.6).

Each behaviour is paired with a mutant per CLAUDE.md 3.3:
  * severity map: a completeness check + the mutant of dropping an action.
  * dedup: a burst within the window collapses to count N; a burst spread beyond
    the window does not -- the two together pin the window. And the exempt set is
    made load-bearing by monkeypatching an action OUT of it and watching its
    events start to merge, which they must never do (trap 1 in audit.py).
  * state: the off-contract suspended reason must raise, not be shown verbatim.
"""

import pytest

from xbrain.common.arbiter import (
    Arbiter, ArbAction, PreemptPolicy, Request, SourceSpec,
    DEDUP_EXEMPT, DEDUP_WINDOW_S, SEVERITY_BY_ACTION,
    merge_audit_window, render_audit_event, render_domain_state, severity_of,
)
from xbrain.common.arbiter.model import ArbEvent
from xbrain.common.arbiter import audit as audit_mod


def _event(action, domain="speaker", mono_ms=1000, gen=1, detail=None):
    """A bare ArbEvent for the serialiser tests, no state machine involved."""
    return ArbEvent(action=action.value, domain=domain, from_source="x",
                    to_source="y", reason="r", forced=False, gen=gen,
                    mono_ms=mono_ms, detail=detail or {})


# --------------------------------------------------------------------------
# severity map (11 S7A.7 / 14 S3.6)
# --------------------------------------------------------------------------

def test_severity_map_matches_the_contract():
    """The three severities of 11 S7A.7, for the actions ArbAction defines."""
    assert severity_of("grant") == "info"
    assert severity_of("release") == "info"
    assert severity_of("preempt") == "info"
    assert severity_of("lease_timeout") == "warn"
    assert severity_of("forced_preempt") == "warn"
    assert severity_of("source_death") == "fault"


def test_severity_map_covers_every_action():
    """*** Completeness. Every ArbAction must have a severity.

    Mutation: add an ArbAction member without extending SEVERITY_BY_ACTION and
    this reports the gap -- which is what stops a new action from KeyError-ing
    only the first time it fires in production.
    """
    assert audit_mod.audit_severity_covers_every_action() is None
    # The check is real: emptying one row makes it name the gap.
    saved = dict(SEVERITY_BY_ACTION)
    try:
        del audit_mod.SEVERITY_BY_ACTION[ArbAction.GRANT]
        assert audit_mod.audit_severity_covers_every_action() is not None
    finally:
        audit_mod.SEVERITY_BY_ACTION.clear()
        audit_mod.SEVERITY_BY_ACTION.update(saved)


def test_severity_values_are_closed_set_members():
    """severity is not spelled as a literal; every mapped value is in SEVERITY."""
    from xbrain.common.enums import SEVERITY
    for sev in SEVERITY_BY_ACTION.values():
        assert sev in SEVERITY.values


# --------------------------------------------------------------------------
# audit event payload (11 S6.1)
# --------------------------------------------------------------------------

def test_render_audit_event_fields():
    """The Event payload an arbitration audit record carries."""
    payload = render_audit_event(_event(ArbAction.GRANT, domain="ptz"))
    assert payload["sev"] == "info"
    assert payload["cat"] == "arbitration"          # 11 S6.2 category, closed set
    assert payload["channel"] == "normal"           # S2.2.11 fixed
    assert payload["dedup_key"] == "arb:ptz:grant"
    assert payload["dedup_window_s"] == DEDUP_WINDOW_S
    assert payload["detail"]["count"] == 1
    assert payload["detail"]["domain"] == "ptz"


def test_exempt_action_is_stamped_window_zero():
    """*** forced_preempt / source_death must be marked never-merge (window 0).

    Mutation: make dedup_window_s a constant 10 for every action and this goes
    red, which is what stops a burst of source deaths from ever collapsing.
    """
    assert render_audit_event(_event(ArbAction.SOURCE_DEATH))["dedup_window_s"] == 0
    assert render_audit_event(_event(ArbAction.FORCED_PREEMPT))["dedup_window_s"] == 0
    # a dedupable one, by contrast, carries the real window
    assert render_audit_event(_event(ArbAction.GRANT))["dedup_window_s"] == DEDUP_WINDOW_S


# --------------------------------------------------------------------------
# the 10 s dedup-window merge (11 S7A.7)
# --------------------------------------------------------------------------

def test_a_burst_within_the_window_collapses_to_one_with_count():
    """Three grants inside 10 s -> one record, count 3."""
    events = [_event(ArbAction.GRANT, mono_ms=1000),
              _event(ArbAction.GRANT, mono_ms=1100),
              _event(ArbAction.GRANT, mono_ms=9000)]      # still within 10 s of 1000
    merged = merge_audit_window(events)
    assert len(merged) == 1
    assert merged[0]["detail"]["count"] == 3


def test_events_more_than_the_window_apart_do_not_merge():
    """*** The window is real. 1000 ms and 12000 ms are 11 s apart -> two records.

    Paired with the test above: together they pin dedup_window_s. A mutant that
    merged everything regardless of time would fail here; one that merged nothing
    would fail above.
    """
    events = [_event(ArbAction.GRANT, mono_ms=1000),
              _event(ArbAction.GRANT, mono_ms=12000)]
    merged = merge_audit_window(events)
    assert len(merged) == 2
    assert all(m["detail"]["count"] == 1 for m in merged)


def test_two_different_keys_do_not_cross_merge():
    """A grant and a release in the same window are different dedup keys."""
    events = [_event(ArbAction.GRANT, mono_ms=1000),
              _event(ArbAction.RELEASE, mono_ms=1050)]
    merged = merge_audit_window(events)
    assert len(merged) == 2
    assert {m["dedup_key"] for m in merged} == {"arb:speaker:grant",
                                                "arb:speaker:release"}


def test_exempt_actions_are_never_merged():
    """*** source_death is exempt: three in one window stay three records.

    This is trap 1 in audit.py -- collapsing 'three source deaths' to 'count 3'
    would hide two. The mutation is made executable in the next test.
    """
    events = [_event(ArbAction.SOURCE_DEATH, mono_ms=1000),
              _event(ArbAction.SOURCE_DEATH, mono_ms=1100),
              _event(ArbAction.SOURCE_DEATH, mono_ms=1200)]
    merged = merge_audit_window(events)
    assert len(merged) == 3
    assert all(m["detail"]["count"] == 1 for m in merged)


def test_mutation_source_death_removed_from_exempt_would_merge():
    """*** The exempt set is load-bearing, shown by removing an action from it.

    With source_death no longer exempt, the three above collapse to one -- which
    is exactly the loss the exemption prevents. Restores the set afterwards.
    """
    saved = audit_mod.DEDUP_EXEMPT
    try:
        audit_mod.DEDUP_EXEMPT = frozenset({ArbAction.FORCED_PREEMPT})  # drop SOURCE_DEATH
        events = [_event(ArbAction.SOURCE_DEATH, mono_ms=1000),
                  _event(ArbAction.SOURCE_DEATH, mono_ms=1100)]
        merged = merge_audit_window(events)
        assert len(merged) == 1                          # now wrongly collapsed
        assert merged[0]["detail"]["count"] == 2
    finally:
        audit_mod.DEDUP_EXEMPT = saved


# --------------------------------------------------------------------------
# ArbDomainState (11 S7A.5.1)
# --------------------------------------------------------------------------

def _arb_with_holder_and_waiter():
    """A speaker arbiter holding by alarm_d with tts queued behind it."""
    a = Arbiter("speaker", 3000)
    a.register(SourceSpec("alarm_d", 900, True, PreemptPolicy.IMMEDIATE, None,
                          None, None))
    a.register(SourceSpec("tts", 600, True, PreemptPolicy.WAIT_ATOMIC, 1.0,
                          None, None))
    a.request("alarm_d", Request("a-1", 1000))           # alarm_d holds
    a.drain_events()
    return a


def test_domain_state_shape_and_derived_times():
    """The full ArbDomainState, with held_ms derived at publish time."""
    a = _arb_with_holder_and_waiter()
    st = render_domain_state(a, now_mono_ms=3140)
    assert st["domain"] == "speaker"
    assert st["gen"] == 1
    assert st["suspended"] is None                       # normal, BIZ-CM-3 fills this
    assert st["holder"]["source_id"] == "alarm_d"
    assert st["holder"]["held_ms"] == 3140 - 1000        # derived, not stored
    assert st["last_change"]["action"] == "grant"
    # sources[] is the whole registry (both), so the HMI sees who could preempt
    assert {s["source_id"] for s in st["sources"]} == {"alarm_d", "tts"}


def test_idle_domain_has_null_holder():
    """An arbiter with no holder yet renders holder null, not a stub object."""
    a = Arbiter("dock", 3000)
    st = render_domain_state(a, now_mono_ms=500)
    assert st["holder"] is None
    assert st["last_change"] is None
    assert st["waiting"] == []


def test_dead_source_is_still_listed_with_alive_false():
    """11 S7A.5.1: the registry is published whole, alive=false included."""
    a = _arb_with_holder_and_waiter()
    a.reap_dead_source("tts", now_mono_ms=2000)          # tts process died
    st = render_domain_state(a, now_mono_ms=3000)
    tts = [s for s in st["sources"] if s["source_id"] == "tts"][0]
    assert tts["alive"] is False


def test_state_reflects_the_arbiter_suspended_reason():
    """suspended in the state is read from the arbiter (BIZ-CM-3 sets it).

    A normal domain renders null; after arb_suspend it renders the reason. The
    off-contract-reason rejection is tested at the arb_suspend boundary in
    test_arbiter_disarm.py, which is where the validation now lives.
    """
    a = _arb_with_holder_and_waiter()
    assert render_domain_state(a, now_mono_ms=3000)["suspended"] is None
    a.arb_suspend("soft_estop", cmd_id="c-1", now_mono_ms=3000)
    assert render_domain_state(a, now_mono_ms=3100)["suspended"] == "soft_estop"


def test_waiting_carries_waited_ms():
    """A queued higher requester shows in waiting[] with waited_ms derived.

    A holder with an ATOMIC action under wait_atomic is what makes a higher
    requester queue rather than preempt outright (11 S7A.3), so this builds that
    scenario: tts holds atomically, alarm_d (higher) queues behind it.
    """
    b = Arbiter("speaker", 3000)
    b.register(SourceSpec("tts", 600, True, PreemptPolicy.WAIT_ATOMIC, 1.0, None, None))
    b.register(SourceSpec("alarm_d", 900, True, PreemptPolicy.IMMEDIATE, None, None, None))
    b.request("tts", Request("t-1", 1000, atomic=True))      # tts holds, atomic
    b.request("alarm_d", Request("a-1", 1200))               # higher, must queue
    b.drain_events()
    st = render_domain_state(b, now_mono_ms=1500)
    waiting = st["waiting"]
    assert len(waiting) == 1
    assert waiting[0]["source_id"] == "alarm_d"
    assert waiting[0]["waited_ms"] == 1500 - 1200
