"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_arbiter_disarm.py
Brief: BIZ-CM-3 -- arb_suspend / arb_rearm, the E_ARB_DISARMED denial, and the
       one-event idempotency, each with the mutant that would defeat it

Description:
11 S7A.6 makes a soft-estop a one-shot operation on the arbiter (arb_suspend),
NOT a high-priority source, because U35 forbids a state lock. This file pins the
disarm semantics BIZ-CM-3 adds on top of the BIZ-CM-1 core:

  * while disarmed, EVERY request() is denied E_ARB_DISARMED (11 S7A.6.2);
  * the same soft-estop on two paths (one cmd_id) produces ONE suspend event
    (the idempotency criterion), and the mutant of dropping the cmd_id key makes
    it two;
  * arb_rearm restores the domain and a new request grants again;
  * an off-contract suspend reason raises at the boundary.
"""

import pytest

from xbrain.common import errors
from xbrain.common.enums import ClosedSetViolation
from xbrain.common.arbiter import (
    Arbiter, ArbAction, GrantResult, PreemptPolicy, Request, SourceSpec,
    DEDUP_EXEMPT, merge_audit_window, severity_of,
)
from xbrain.common.arbiter.model import ArbEvent


def _motion_arb():
    """A motion arbiter holding by path_follow, with an on_lost tripwire."""
    lost = {"path_follow": False}
    a = Arbiter("motion", 3000)
    a.register(SourceSpec("path_follow", 500, True, PreemptPolicy.WAIT_ATOMIC,
                          None, None, lambda: lost.__setitem__("path_follow", True)))
    a.request("path_follow", Request("p-1", 1000))       # it holds
    a.drain_events()
    return a, lost


# --------------------------------------------------------------------------
# the denial
# --------------------------------------------------------------------------

def test_request_is_denied_disarmed_while_suspended():
    """*** After arb_suspend, any request() is denied E_ARB_DISARMED.

    Mutation: the suspended check at the top of request() -- remove it and the
    request is granted (or E_BUSY), so this pins that the disarm actually blocks.
    """
    a, _lost = _motion_arb()
    a.arb_suspend("soft_estop", cmd_id="e-1", now_mono_ms=2000)
    g = a.request("path_follow", Request("p-2", 2100))   # even the ex-holder
    assert g.result is GrantResult.DENIED
    assert g.code == errors.E_ARB_DISARMED


def test_disarm_denies_even_an_unregistered_source_with_disarmed_not_no_source():
    """The suspended check runs BEFORE the registry lookup, so a stranger also
    gets E_ARB_DISARMED, not E_ARB_NO_SOURCE (11 S7A.6.2 'any request')."""
    a, _lost = _motion_arb()
    a.arb_suspend("soft_estop", cmd_id="e-1", now_mono_ms=2000)
    g = a.request("never_registered", Request("x-1", 2100))
    assert g.code == errors.E_ARB_DISARMED


# --------------------------------------------------------------------------
# suspend effects
# --------------------------------------------------------------------------

def test_suspend_revokes_holder_clears_queue_and_bumps_gen():
    """arb_suspend fires the holder's on_lost, empties the queue, gen += 1."""
    a, lost = _motion_arb()
    gen_before = a.gen()
    ev = a.arb_suspend("soft_estop", cmd_id="e-1", now_mono_ms=2000)
    assert lost["path_follow"] is True                   # on_lost fired: stop producing
    assert a.holder() is None                            # no source holds while disarmed
    assert a.waiters() == []                             # queue cleared
    assert a.gen() == gen_before + 1                     # one holder change
    assert a.suspended() == "soft_estop"
    assert ev is not None and ev.action == ArbAction.SUSPEND.value
    assert ev.detail["cmd_id"] == "e-1"                  # correlates the two paths


def test_off_contract_suspend_reason_raises():
    """*** suspended is ARB_SUSPENDED (soft_estop | hes | cmd_timeout).

    Mutation: 'paused' is off-contract and must raise at the boundary, never be
    stored and shown. A legal reason is accepted.
    """
    a, _lost = _motion_arb()
    with pytest.raises(ClosedSetViolation):
        a.arb_suspend("paused", cmd_id="e-1", now_mono_ms=2000)
    assert a.suspended() is None                          # nothing was stored
    a.arb_suspend("hes", cmd_id="e-1", now_mono_ms=2000)
    assert a.suspended() == "hes"


# --------------------------------------------------------------------------
# idempotency -- the criterion's core
# --------------------------------------------------------------------------

def test_same_cmd_id_two_paths_produce_one_suspend_event():
    """*** One soft-estop, two arrival paths (cmd/estop and state/robot), one
    event (11 S7A.6). The second call with the same cmd_id is a no-op -> None.

    Mutation: drop the cmd_id idempotency key (always emit) and the second call
    returns a second event -> two suspend events for one estop -> red.
    """
    a, _lost = _motion_arb()
    first = a.arb_suspend("soft_estop", cmd_id="e-1", now_mono_ms=2000)
    second = a.arb_suspend("soft_estop", cmd_id="e-1", now_mono_ms=2005)  # other path
    assert first is not None
    assert second is None                                # idempotent: no 2nd event
    assert a.gen() == 1 + 1                              # gen bumped once, not twice


def test_a_different_cmd_id_while_suspended_re_events():
    """A NEW estop epoch (different cmd_id) is a fresh suspend, not the same one."""
    a, _lost = _motion_arb()
    a.arb_suspend("soft_estop", cmd_id="e-1", now_mono_ms=2000)
    ev2 = a.arb_suspend("soft_estop", cmd_id="e-2", now_mono_ms=3000)
    assert ev2 is not None
    assert ev2.detail["cmd_id"] == "e-2"


# --------------------------------------------------------------------------
# rearm
# --------------------------------------------------------------------------

def test_rearm_restores_the_domain_and_a_new_request_grants():
    """arb_rearm clears the disarm, gen += 1, and a new request grants again."""
    a, _lost = _motion_arb()
    a.arb_suspend("soft_estop", cmd_id="e-1", now_mono_ms=2000)
    gen_suspended = a.gen()
    ev = a.arb_rearm(now_mono_ms=4000)
    assert ev is not None and ev.action == ArbAction.REARM.value
    assert a.suspended() is None
    assert a.gen() == gen_suspended + 1
    # the consumer's next (new) command now grants normally
    g = a.request("path_follow", Request("p-3", 4100))
    assert g.result is GrantResult.GRANTED


def test_rearm_when_armed_is_a_noop():
    """Re-arming an already-armed domain emits nothing (idempotent), so a second
    new command does not produce a spurious rearm event."""
    a, _lost = _motion_arb()
    assert a.arb_rearm(now_mono_ms=4000) is None
    assert a.gen() == 1                                   # unchanged from the initial grant


# --------------------------------------------------------------------------
# audit properties of suspend / rearm
# --------------------------------------------------------------------------

def test_suspend_and_rearm_are_info_severity():
    """11 S7A.7: suspend/rearm -> info."""
    assert severity_of("suspend") == "info"
    assert severity_of("rearm") == "info"


def test_suspend_and_rearm_are_dedup_exempt():
    """*** A soft-estop must never be merged into a count (trap 1 in audit.py).

    Two suspends in one window stay two records, unlike two grants.
    """
    assert ArbAction.SUSPEND in DEDUP_EXEMPT
    assert ArbAction.REARM in DEDUP_EXEMPT
    events = [
        ArbEvent("suspend", "motion", "path_follow", None, "soft_estop", False,
                 2, 1000, {"cmd_id": "e-1"}),
        ArbEvent("suspend", "motion", "path_follow", None, "soft_estop", False,
                 4, 1100, {"cmd_id": "e-2"}),
    ]
    merged = merge_audit_window(events)
    assert len(merged) == 2                              # never collapsed
