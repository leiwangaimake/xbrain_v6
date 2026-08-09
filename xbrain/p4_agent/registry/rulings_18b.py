"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rulings_18b.py
Brief: GWY-P4-31 -- 18-B seven rulings (E09 session-level tier, min() semantics,
       cumulative drift warn, forbidden words)

Description:
18-B (voice extension) sets 7 rulings for PTZ / new intents. This
module encodes the ones with runtime-checkable behavior:

  R-1  E09 (ptz_move_by) is a SESSION-level tier (not one-shot);
       each subsequent E09 in the session USES the same tier
  R-2  E10 (ptz_move_deg) has T-PTZ-3 short-circuit -> rejected
       until real omega measured
  R-3  overflow: 云台转 400 度 -> rejected, NOT silently clipped
       to upper limit
  R-4  cumulative drift warn: if operator stacks E09s that would
       exceed pan/tilt range, warn AND stop
  R-5  forbidden words (E10 restate must NOT say '已转')
  R-6  intent unsupported different from intent out-of-range
       (unsupported = wrong wording; out-of-range = right wording,
       bad value)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Optional


# Forbidden words in PTZ restate (R-5).
_FORBIDDEN_IN_RESTATE = frozenset({"已转", "已完成移动"})


class E09TierError(RuntimeError):
    """R-1 violation: E09 without a session tier established."""


@dataclass
class E09SessionTier:
    """R-1: PTZ speed tier is per-session, persisted across E09s."""
    tier: Optional[str] = None    # 'low' | 'mid' | 'high' | None


def resolve_e09_tier(session_tier: E09SessionTier,
                     requested_tier: Optional[str]) -> str:
    """R-1: if session has no tier, requested is required. If session
    has a tier and requested is None, use session's. If requested
    disagrees with session, use requested AND update session."""
    if requested_tier:
        session_tier.tier = requested_tier
        return requested_tier
    if session_tier.tier is None:
        raise E09TierError(
            "R-1: E09 requires tier; session has none and no tier requested")
    return session_tier.tier


def check_r3_within_range(value_deg: float,
                           lower: float, upper: float) -> None:
    """R-3: reject overflow, do NOT silently clip."""
    if value_deg < lower or value_deg > upper:
        raise ValueError(
            "R-3: %f out of range [%f, %f]; refused (not clipped)"
            % (value_deg, lower, upper))


def check_r5_restate_no_forbidden(text: str) -> None:
    """R-5: PTZ restate MUST NOT include '已转' etc."""
    for w in _FORBIDDEN_IN_RESTATE:
        if w in text:
            raise ValueError(
                "R-5: PTZ restate %r contains forbidden word %r "
                "(overpromising completion)" % (text, w))


@dataclass
class CumulativeDrift:
    """R-4: track cumulative deltas so operator stacking a series of
    tiny E09s doesn't drift past the pan/tilt limits without warning."""
    total_pan_deg: float = 0.0
    total_tilt_deg: float = 0.0

    def add(self, dpan: float, dtilt: float,
            pan_limit: float, tilt_limit: float) -> Optional[str]:
        """Return None if within limits; return a warning string on
        exceeding either limit (caller then STOPS + warn TTS)."""
        self.total_pan_deg += dpan
        self.total_tilt_deg += dtilt
        if abs(self.total_pan_deg) > pan_limit:
            return "R-4: cumulative pan %.1f exceeds limit %.1f" % (
                self.total_pan_deg, pan_limit)
        if abs(self.total_tilt_deg) > tilt_limit:
            return "R-4: cumulative tilt %.1f exceeds limit %.1f" % (
                self.total_tilt_deg, tilt_limit)
        return None
