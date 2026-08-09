"""BIZ-P2-17 -- authz level check + L3 pending-approval flow tests."""

import pytest

from xbrain.p2_core.authz.levels import (
    AuthLevel, PendingApproval, approve, check,
)


pytestmark = pytest.mark.no_device


# --- non-L3 actions accept L0..L2 ------------------------------

@pytest.mark.parametrize("level", [
    AuthLevel.L0, AuthLevel.L1a, AuthLevel.L1b, AuthLevel.L2,
])
def test_non_l3_action_accepts_lower_levels(level):
    d = check("goto_waypoint", level)
    assert d.accepted


# --- L3 actions REQUIRE L3 -------------------------------------

@pytest.mark.parametrize("action", ["shutdown", "reboot"])
@pytest.mark.parametrize("level", [
    AuthLevel.L0, AuthLevel.L1a, AuthLevel.L1b, AuthLevel.L2,
])
def test_l3_action_rejects_lower_levels(action, level):
    d = check(action, level)
    assert not d.accepted
    assert "requires L3" in d.reason


def test_l3_action_with_l3_enters_pending_approval():
    """CMD-33: L3 action + L3 level -> NOT immediately accepted;
    enters pending_approval requiring confirm_token."""
    d = check("shutdown", AuthLevel.L3)
    assert not d.accepted
    assert d.needs_pending_approval is True
    assert d.reason == "L3_pending_approval"


# --- L3 approve: verify token + freshness ---------------------

def test_approve_valid_token_within_timeout_accepts():
    pending = PendingApproval(
        action="shutdown", started_mono_ms=0, timeout_ms=60_000)
    d = approve(pending, "goodtoken",
                token_verifier=lambda t: t == "goodtoken",
                now_mono_ms=30_000)
    assert d.accepted


def test_approve_stale_token_rejected():
    pending = PendingApproval(
        action="shutdown", started_mono_ms=0, timeout_ms=1000)
    d = approve(pending, "any",
                token_verifier=lambda t: True,
                now_mono_ms=5000)
    assert not d.accepted
    assert "stale" in d.reason


def test_approve_bad_token_rejected():
    pending = PendingApproval(
        action="shutdown", started_mono_ms=0, timeout_ms=60_000)
    d = approve(pending, "badtoken",
                token_verifier=lambda t: t == "goodtoken",
                now_mono_ms=100)
    assert not d.accepted
    assert "invalid" in d.reason


def test_approved_pending_carries_token_and_verified_flag():
    pending = PendingApproval(
        action="reboot", started_mono_ms=0, timeout_ms=60_000)
    approve(pending, "t", token_verifier=lambda t: True, now_mono_ms=1)
    assert pending.verified is True
    assert pending.token_received == "t"
