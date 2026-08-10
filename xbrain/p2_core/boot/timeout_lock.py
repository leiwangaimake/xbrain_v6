"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: timeout_lock.py
Brief: INF-BT-3 timeout_lock unlock flow (BOOT-L1..L3 + readback gate)

Description:
When state/robot.timeout_lock == True, P2 sits in BLOCKED and
MUST NOT publish cmd/motion/factor{allow_motion:true}. The unlock
path has three enforced rules:

  BOOT-L1  operator sends cmd/chassis/ctrl{action:"enable"} with
           L2 authorisation + confirm_token; missing either -> refuse
  BOOT-L2  DO NOT auto-clear the lock. Timeout / cmd_age recovery /
           heartbeat resume: NONE of these clear it.
  BOOT-L3  readback discipline: the chassis MAY ack accepted but
           state/robot.timeout_lock may still be True; ONLY the
           readback transitioning to False lifts the block.

There is exactly ONE unlock channel (CR-11 / CR-12 in the whitelist).
Any other unlock path is a static-guard violation.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from xbrain.common.errors import (
    E_CHANNEL_DENIED, E_CONFIRM_REQUIRED,
)


class TimeoutLockAction(str, Enum):
    ENABLE = "enable"       # operator-triggered unlock request


ALLOWED_UNLOCK_CHANNELS = frozenset({"CR-11", "CR-12"})


@dataclass(frozen=True)
class UnlockVerdict:
    accepted: bool
    code: str = ""
    reason: str = ""


def validate_unlock_request(cmd: dict, channel: str) -> UnlockVerdict:
    """BOOT-L1 gate: L2 authorisation + confirm_token + allowed channel."""
    if channel not in ALLOWED_UNLOCK_CHANNELS:
        return UnlockVerdict(
            accepted=False, code=E_CHANNEL_DENIED,
            reason=f"unlock channel {channel!r} not in {sorted(ALLOWED_UNLOCK_CHANNELS)}")
    if cmd.get("action") != TimeoutLockAction.ENABLE.value:
        return UnlockVerdict(
            accepted=False, code=E_CONFIRM_REQUIRED,
            reason=f"unlock cmd.action must be {TimeoutLockAction.ENABLE.value!r}")
    if not cmd.get("confirm_token"):
        return UnlockVerdict(
            accepted=False, code=E_CONFIRM_REQUIRED,
            reason="unlock requires L2 confirm_token")
    return UnlockVerdict(accepted=True)


@dataclass
class TimeoutLockGate:
    """Gate over cmd/motion/factor publication driven by the
    state/robot.timeout_lock readback. Ack-only paths are ignored
    (BOOT-L3)."""
    timeout_lock: bool = True    # start locked (safe default)

    def note_readback(self, readback_lock: bool) -> None:
        """The ONLY thing that clears the block: readback transitioning
        to False."""
        self.timeout_lock = readback_lock

    def note_ack_only(self, ack_accepted: bool) -> None:
        """Ack-only signal MUST NOT change the gate. This method
        exists as an explicit no-op so a caller trying to unlock
        on ack finds it here and gets the docstring."""
        _ = ack_accepted   # deliberate no-op; readback is authority

    def note_heartbeat_resumed(self) -> None:
        """BOOT-L2: heartbeat resume is NOT unlock."""

    def note_cmd_age_ok(self) -> None:
        """BOOT-L2: cmd_age recovery is NOT unlock."""

    def note_handshake_completed(self) -> None:
        """BOOT-L2: handshake completion is NOT unlock."""

    def may_publish_factor(self) -> bool:
        """Only when the readback says False."""
        return not self.timeout_lock


class HesLockConflation(Exception):
    """A test/code tried to fold hes_lock + timeout_lock into one bit."""


def assert_locks_are_separate(lock_names: tuple) -> None:
    """HES lock (physical E-stop) and timeout lock (chassis-side
    deadman) are DIFFERENT. Combining them into one bit erases the
    'HES still down but timeout ok' case where the chassis accepts
    commands but the unlock must NOT clear."""
    if len(set(lock_names)) < 2:
        raise HesLockConflation(
            f"hes_lock and timeout_lock must be tracked separately; "
            f"lock_names={lock_names!r} shows only {len(set(lock_names))} "
            f"distinct name(s)")
    for required in ("hes_lock", "timeout_lock"):
        if required not in lock_names:
            raise HesLockConflation(
                f"required lock {required!r} missing from {lock_names!r}")
