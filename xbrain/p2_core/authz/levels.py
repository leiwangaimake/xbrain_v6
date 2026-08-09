"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: levels.py
Brief: BIZ-P2-17 -- payload authorization L1/L2/L3 + L3 pending-approval flow

Description:
14 S4.3 / 11 S8.13.1 authz levels for payload operations:

  L1a  local mic ASR (default; requires no confirmation)
  L1b  cloud ASR (requires cloud ack for TTS length ceiling)
  L2   HMI / operator button (explicit UI acknowledge)
  L3   cloud-signed confirm_token (only for shutdown / reboot;
       CMD-33: 'switch to pending-approval' 5-step flow)

The L3 5-step flow (14 CMD-33) -- this is what governs H08 shutdown:
  1. P2 sees cmd/system{action:shutdown}, LEVEL=L3
  2. Set pending_approval state, publish state/system{approval=needed}
  3. Wait for cmd/system/confirm{token: <cloud-signed>}
  4. Verify token (cloud pubkey; not implemented in this module --
     that's SEC-4/SEC-10 territory, out of P2 authz layer)
  5. If token valid AND fresh (< 60s old), commit; else reject
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import FrozenSet, Optional


class AuthLevel(str, Enum):
    """14 S8.3A.2 closed set."""
    L0 = "L0"    # no authz (background system messages, e.g. events)
    L1a = "L1a"  # local ASR
    L1b = "L1b"  # cloud ASR
    L2 = "L2"    # HMI operator button
    L3 = "L3"    # cloud-signed confirm_token


# Actions that REQUIRE L3 (shutdown / reboot per 14 CMD-33 + H08).
L3_REQUIRED_ACTIONS: FrozenSet[str] = frozenset({
    "shutdown", "reboot",
})


class AuthzError(RuntimeError):
    """Raised when an action's authz level is inadequate for its
    required level."""


@dataclass
class AuthDecision:
    """The outcome of an authz check."""
    accepted: bool
    reason: str = ""
    # For L3 flow: does the caller need to enter pending_approval?
    needs_pending_approval: bool = False


def check(action: str, requested_level: AuthLevel) -> AuthDecision:
    """Test whether an action is permitted at the requested level.

    Returns AuthDecision with needs_pending_approval=True for L3
    actions that arrived with L3 (the flow needs a confirm_token
    round-trip BEFORE the action commits)."""
    if action in L3_REQUIRED_ACTIONS:
        if requested_level != AuthLevel.L3:
            return AuthDecision(
                accepted=False,
                reason="action %r requires L3, got %s"
                       % (action, requested_level.value),
            )
        # L3 action + L3 level -> enter pending_approval flow.
        return AuthDecision(
            accepted=False,
            reason="L3_pending_approval",
            needs_pending_approval=True,
        )
    # Non-L3 action: any of L0..L2 accepted (14 S8.3A.2 open policy
    # for non-L3 actions). L1a/L1b/L2 all permit run-of-mill commands.
    return AuthDecision(accepted=True)


@dataclass
class PendingApproval:
    """State machine for one L3 action awaiting confirm_token."""
    action: str
    started_mono_ms: int
    timeout_ms: int = 60_000    # 60 s per U62 (fresh token requirement)
    token_received: Optional[str] = None
    verified: bool = False


def approve(pending: PendingApproval, token: str,
            token_verifier, now_mono_ms: int) -> AuthDecision:
    """Second step: caller received the cmd/system/confirm{token}
    message and wants to promote pending to executable.

    token_verifier is an injected callable(str) -> bool that checks
    the cloud signature; if this module verified it, this file would
    have to know cloud pubkey management -- out of scope."""
    if now_mono_ms - pending.started_mono_ms > pending.timeout_ms:
        return AuthDecision(
            accepted=False,
            reason="confirm_token_stale",
        )
    if not token_verifier(token):
        return AuthDecision(
            accepted=False,
            reason="confirm_token_invalid",
        )
    pending.token_received = token
    pending.verified = True
    return AuthDecision(accepted=True, reason="confirm_token_ok")
