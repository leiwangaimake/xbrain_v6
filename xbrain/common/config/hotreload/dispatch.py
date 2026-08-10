"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: dispatch.py
Brief: CHK-1-58 cmd/config scope dispatcher (log_level / debug_flags / asr_dictionary)

Description:
11 §7.6 defines cmd/config as a bounded hot-reload channel. Three
scopes today; a fourth added to the spec table without matching
code is exactly the drift this dispatcher's meta-test catches.

Rules (variant coverage per CHK-1-58 spec):
  * scope closed set: {log_level, debug_flags, asr_dictionary}
    -- anything else -> E_CONFIG_LOCKED (safety params must NEVER
    be hot-reloadable)
  * origin must be 'cloud' (P5 upstream); HMI / voice /
    other origins -> E_CHANNEL_DENIED
  * confirm_token required; a self-issued token (token.issuer
    != executor) is rejected; token has expires_mono_ms measured
    on the MONOTONIC clock (never wall)
  * ack echoes detail.scope == the requested scope
  * responding processes: ALL of P1..P5 (fanout ack)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from xbrain.common.errors import (
    E_CHANNEL_DENIED, E_CONFIG_LOCKED, E_CONFIRM_REQUIRED,
)


WHITELIST_SCOPES = ("log_level", "debug_flags", "asr_dictionary")
ALLOWED_ORIGINS = ("cloud",)
RESPONDING_PROCESSES = ("p1_motion", "p2_core", "p3_task",
                          "p4_agent", "p5_gateway")
TOKEN_ISSUER = "executor"


@dataclass(frozen=True)
class HotReloadVerdict:
    accepted: bool
    code: str = ""
    reason: str = ""
    scope: str = ""


def check_request(cmd: dict, now_mono_ms: int) -> HotReloadVerdict:
    """Validate + admit a cmd/config request. On accept, caller
    invokes the per-scope apply function and emits ack from EVERY
    responding process."""
    if not isinstance(cmd, dict):
        return HotReloadVerdict(False, E_CONFIG_LOCKED, "cmd not object")
    scope = cmd.get("scope")
    if scope not in WHITELIST_SCOPES:
        return HotReloadVerdict(
            False, E_CONFIG_LOCKED,
            f"scope {scope!r} outside whitelist (safety param?)")
    origin = cmd.get("origin")
    if origin not in ALLOWED_ORIGINS:
        return HotReloadVerdict(
            False, E_CHANNEL_DENIED,
            f"origin {origin!r} not in {ALLOWED_ORIGINS}",
            scope=scope)
    token = cmd.get("confirm_token")
    if not token:
        return HotReloadVerdict(
            False, E_CONFIRM_REQUIRED,
            "missing confirm_token", scope=scope)
    if not isinstance(token, dict):
        return HotReloadVerdict(
            False, E_CONFIRM_REQUIRED,
            "confirm_token must be object with issuer + expires_mono_ms",
            scope=scope)
    if token.get("issuer") != TOKEN_ISSUER:
        return HotReloadVerdict(
            False, E_CONFIRM_REQUIRED,
            f"confirm_token issuer must be {TOKEN_ISSUER!r}",
            scope=scope)
    exp = token.get("expires_mono_ms")
    if not isinstance(exp, int):
        return HotReloadVerdict(
            False, E_CONFIRM_REQUIRED,
            "confirm_token.expires_mono_ms must be integer monotonic ms",
            scope=scope)
    if exp <= now_mono_ms:
        return HotReloadVerdict(
            False, E_CONFIRM_REQUIRED,
            f"confirm_token expired at mono_ms={exp} (now={now_mono_ms})",
            scope=scope)
    return HotReloadVerdict(True, "OK", "", scope=scope)


def build_ack(process_name: str, scope: str,
                verdict: HotReloadVerdict) -> dict:
    """Every responding process emits an ack of this shape."""
    if process_name not in RESPONDING_PROCESSES:
        raise ValueError(f"unknown process {process_name!r}")
    return {
        "process": process_name,
        "rejected": not verdict.accepted,
        "code": verdict.code,
        "detail": {"scope": scope, "reason": verdict.reason},
    }


def fanout_ack(scope: str,
                 verdict: HotReloadVerdict) -> tuple:
    """Return ONE ack per responding process."""
    return tuple(build_ack(p, scope, verdict) for p in RESPONDING_PROCESSES)
