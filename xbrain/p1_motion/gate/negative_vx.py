"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: negative_vx.py
Brief: CHK-1-45 R2.3-b sign-aware first-stage limiter (negative vx <= 0.5 m/s)

Description:
11 §15.6 D-33: the rear corridor perception is not part of the
current build. Because RNS has no rearward field of view, any
rearward motion must be conservatively capped -- a fast reverse
into an unmapped obstacle is silent (no sensor to warn).

Rule R2.3-b (source-agnostic first-stage cap):
  if vx < 0:   vx := max(vx, -abs_max_reverse_mps)
  if vx >= 0:  unchanged  (positive direction has rns_avoid + fence)

The 0.5 m/s cap value is INJECTED at construction time -- no
dataclass default, no dict.get(k, v), no `or v` (CLAUDE.md 3.1).
The lint scripts/lint/no_safety_default.py covers this file.

Sources it applies to (ALL of them; the whole point is
"regardless of source"):
  teleop (600), teleop_cloud (550), relative_move (500),
  nav2_proxy (backup), rns_avoid (any)

Attribution: `gate.limiter` is set to the enum value
NEGATIVE_VX_CAP (imported from the closed-set enum in
common.enums) NEVER a bare string literal.
"""

from __future__ import annotations


NEGATIVE_VX_CAP_LIMITER = "negative_vx_cap"     # closed-set enum value


class NegativeVxConfigError(Exception):
    """Constructor received an invalid cap value."""


class NegativeVxCap:
    """First-stage sign-aware clamp. Positive direction untouched;
    negative direction clamped to -abs_max_reverse_mps.

    Construction requires an explicit positive cap; zero or negative
    values raise (a 0 cap would silently kill all reverse motion,
    which is the CLAUDE.md 3.1 fail-silent form)."""

    def __init__(self, abs_max_reverse_mps: float) -> None:
        if abs_max_reverse_mps <= 0:
            raise NegativeVxConfigError(
                f"abs_max_reverse_mps must be > 0, got "
                f"{abs_max_reverse_mps!r} (fail-silent form of no cap)")
        self._cap = float(abs_max_reverse_mps)

    def apply(self, vx: float) -> tuple:
        """Return (clamped_vx, limiter_hit).
        limiter_hit is either '' (no clamp) or NEGATIVE_VX_CAP_LIMITER."""
        if vx >= 0:
            return vx, ""
        min_allowed = -self._cap
        if vx < min_allowed:
            return min_allowed, NEGATIVE_VX_CAP_LIMITER
        return vx, ""

    @property
    def cap(self) -> float:
        return self._cap
