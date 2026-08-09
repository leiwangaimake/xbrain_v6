"""BIZ-P2-22 -- lifecycle SM tests."""

import pytest

from xbrain.p2_core.lifecycle.sm import (
    IllegalTransition, LifecycleSM, LifecycleState,
)


pytestmark = pytest.mark.no_device


def test_full_happy_path():
    sm = LifecycleSM()
    assert sm.state == LifecycleState.INIT
    sm.transition(LifecycleState.WAIT_BIT)
    sm.transition(LifecycleState.EVALUATE)
    sm.transition(LifecycleState.GRANT)
    assert sm.can_release_stage_4()
    sm.transition(LifecycleState.RUNNING)
    assert sm.is_operational()


def test_evaluate_to_blocked_on_fatal_fail():
    """BIT-33: fatal-fail -> BLOCKED, not GRANT."""
    sm = LifecycleSM(initial=LifecycleState.EVALUATE)
    sm.transition(LifecycleState.BLOCKED)
    assert sm.state == LifecycleState.BLOCKED
    assert not sm.can_release_stage_4()


def test_running_to_blocked_on_over_budget():
    """BIZ-P2-1 supplement: 3x over-budget -> RUNNING -> BLOCKED."""
    sm = LifecycleSM(initial=LifecycleState.RUNNING)
    sm.transition(LifecycleState.BLOCKED)


def test_illegal_transition_raises():
    """INIT -> GRANT is illegal (must go through WAIT_BIT + EVALUATE)."""
    sm = LifecycleSM(initial=LifecycleState.INIT)
    with pytest.raises(IllegalTransition):
        sm.transition(LifecycleState.GRANT)


def test_blocked_recovery_via_evaluate():
    sm = LifecycleSM(initial=LifecycleState.BLOCKED)
    sm.transition(LifecycleState.EVALUATE)
    sm.transition(LifecycleState.GRANT)


def test_no_force_escape_hatch():
    """CLAUDE.md 3.6: no way to force-transition past the SM. Verify
    by grep of the module surface -- transition() is the only mutator."""
    from xbrain.p2_core.lifecycle import sm
    api = set(dir(sm.LifecycleSM))
    # Only 'transition' should be public-mutator-shaped.
    for banned in ("force_transition", "set_state", "_reset", "override"):
        assert banned not in api, \
            "found force-escape method: %s" % banned
