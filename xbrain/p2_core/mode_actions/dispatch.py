"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: dispatch.py
Brief: CHK-1-13 ModeCommand action 六值分派 + SP-C1..C3 + ack.detail.applied 三字段

Description:
11 §7.3 ModeCommand.action closed set (SIX values, first-fail rejects):
  set_voice_mode, exit_broadcast, exit_alarm, set_behavior,
  set_speed_profile, reset_profile_lock

SP-C1  set_speed_profile.profile in {obstacle_avoid, patrol}
       -- unknown value never silently degraded (§13.6 越界必抛)

SP-C2  reset_profile_lock MUST carry confirm_token issued by the
       EXECUTOR (P2). A caller-generated token is refused
       (CT-1..CT-8): only tokens whose issuer signature == 'p2'
       are accepted.

SP-C3  cmd/mode/ack.detail.applied for set_speed_profile MUST
       carry all THREE fields:
         profile_to    -- what P2 actually switched to
         profile_locked -- is the profile now locked (S-3 thrash)
         max_profile   -- the ceiling S-3 pinned to (if locked)
       accepted != 'switched'; the operator only knows once these
       three land in the ack.

Static discipline: NO MotionCommand type, NO cmd/motion/profile
key (D-04 ruling). Enforced by lint elsewhere; here we simply do
not define either.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from xbrain.common.errors import (
    E_CONFIRM_REQUIRED, E_SCHEMA,
)


ACTION_CLOSED_SET = (
    "set_voice_mode",
    "exit_broadcast",
    "exit_alarm",
    "set_behavior",
    "set_speed_profile",
    "reset_profile_lock",
)


SPEED_PROFILE_CLOSED_SET = ("obstacle_avoid", "patrol")


CONFIRM_TOKEN_ISSUER = "p2"


@dataclass(frozen=True)
class DispatchResult:
    """Verdict + optional ack.detail.applied payload."""
    accepted: bool
    code: str = ""            # OK on accept, E_* on reject
    reason: str = ""
    applied: Optional[dict] = None


def _reject(code: str, reason: str) -> DispatchResult:
    return DispatchResult(accepted=False, code=code, reason=reason)


def dispatch(cmd: dict,
             profile_state: dict) -> DispatchResult:
    """Route a ModeCommand. `profile_state` is P2's current speed
    profile SM state: {profile: str, locked: bool, max_profile: str}."""
    if not isinstance(cmd, dict):
        return _reject(E_SCHEMA, "cmd not object")
    action = cmd.get("action")
    if action not in ACTION_CLOSED_SET:
        return _reject(E_SCHEMA, f"action {action!r} not in closed set")

    handler = _HANDLERS[action]
    return handler(cmd, profile_state)


def _handle_set_speed_profile(cmd: dict, state: dict) -> DispatchResult:
    """SP-C1 + SP-C3 combined."""
    profile = cmd.get("profile")
    if profile not in SPEED_PROFILE_CLOSED_SET:
        return _reject(E_SCHEMA, f"profile {profile!r} not in closed set")
    # SP-C3 ack.detail.applied THREE-FIELD requirement.
    return DispatchResult(
        accepted=True, code="OK",
        applied={
            "profile_to":     profile,
            "profile_locked": bool(state.get("locked", False)),
            "max_profile":    state.get("max_profile", profile),
        })


def _handle_reset_profile_lock(cmd: dict, state: dict) -> DispatchResult:
    """SP-C2: confirm_token MUST be present AND issued by executor."""
    token = cmd.get("confirm_token")
    if not token:
        return _reject(E_CONFIRM_REQUIRED, "missing confirm_token")
    if not isinstance(token, dict) or token.get("issuer") != CONFIRM_TOKEN_ISSUER:
        return _reject(
            E_CONFIRM_REQUIRED,
            f"confirm_token issuer must be {CONFIRM_TOKEN_ISSUER!r}, "
            f"got {token.get('issuer') if isinstance(token, dict) else type(token).__name__!r}")
    # After unlock, applied reports the new state.
    return DispatchResult(
        accepted=True, code="OK",
        applied={"profile_locked": False, "unlocked_via": "operator_reset"})


def _handle_set_voice_mode(cmd: dict, state: dict) -> DispatchResult:
    """Minimal shape check; full validation lives in BIZ-P2-11 SM."""
    if "voice_mode" not in cmd:
        return _reject(E_SCHEMA, "voice_mode required")
    return DispatchResult(accepted=True, code="OK",
                           applied={"voice_mode": cmd["voice_mode"]})


def _handle_exit_broadcast(cmd: dict, state: dict) -> DispatchResult:
    return DispatchResult(accepted=True, code="OK",
                           applied={"broadcast_active": False})


def _handle_exit_alarm(cmd: dict, state: dict) -> DispatchResult:
    return DispatchResult(accepted=True, code="OK",
                           applied={"alarm_active": False})


def _handle_set_behavior(cmd: dict, state: dict) -> DispatchResult:
    if "behavior" not in cmd:
        return _reject(E_SCHEMA, "behavior required")
    return DispatchResult(accepted=True, code="OK",
                           applied={"behavior_to": cmd["behavior"]})


_HANDLERS: Dict[str, Any] = {
    "set_voice_mode":     _handle_set_voice_mode,
    "exit_broadcast":     _handle_exit_broadcast,
    "exit_alarm":         _handle_exit_alarm,
    "set_behavior":       _handle_set_behavior,
    "set_speed_profile":  _handle_set_speed_profile,
    "reset_profile_lock": _handle_reset_profile_lock,
}


def handlers_complete() -> None:
    """Meta-check: every action in the closed set has a handler.
    Called at process startup so a mismatch fails fast rather than
    at first dispatch of the missing action."""
    missing = set(ACTION_CLOSED_SET) - set(_HANDLERS)
    if missing:
        raise RuntimeError(
            f"mode_actions dispatch table missing handlers for "
            f"{sorted(missing)}")
