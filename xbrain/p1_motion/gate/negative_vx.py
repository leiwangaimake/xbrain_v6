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

*** ATTRIBUTION IS UNDECIDED, and this module deliberately names
nothing. Until 2026-09-30 it exported

  NEGATIVE_VX_CAP_LIMITER = "negative_vx_cap"   # closed-set enum value

and the paragraph here said the value was "imported from the
closed-set enum in common.enums, NEVER a bare string literal".
Three things were wrong with that at once: there is no such
member (11 S9.6.5's gate.limiter is closed at estop / mode /
health / rtk / heading / clock / fence / free_space / target /
brake / gait / profile / spec / none, mirrored in
xbrain/common/enums/sets.yaml), it was not imported from
anywhere, and it WAS a bare string literal. Wiring it would have
put an out-of-set gate.limiter on the wire, which CLAUDE.md 3.5
says must raise rather than pass.

Why the fix is "name nothing" and not "pick a value":
  * 12 S6A.8 OB-1 verbatim: 不擅自往 gate.limiter 里加值. Adding
    a member is a change to 11's F-19 frozen surface and goes
    through review.
  * No existing member fits by inspection either. S9.6.5's whole
    machinery is v_max = min(V_min x C, H_min) plus
    limiter = argmax(delta) over UNSIGNED magnitude caps; this cap
    exists only when vx < 0, so it never enters that formula. Its
    landing point is 12 S8.1 stage 1, DOWNSTREAM of the gate, and
    S9.6.5 step 4 (downstream clipping) was narrowed in v0.7 to
    exactly two members, brake and fence, with the note that both
    are "真的发生在速度门之后" ones. A third has no slot.
  * That leaves the same shape as OB-4 records for wz: no field in
    11 S3.4's gate block answers this, and the choice between a
    new gate field and an event is 11's to make. Registered in
    docs/NEXT.md S8.12 R-D2.
So apply() reports WHETHER it clamped, and the caller that wires
this will have to carry the ruling with it.
"""

from __future__ import annotations


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
        """Return (clamped_vx, clamped).

        `clamped` is a bool, not an attribution string: see the module
        docstring -- naming a gate.limiter value here would either invent a
        closed-set member (11 S9.6.5 / CLAUDE.md 3.5) or make a choice among
        the existing ones that nobody has ruled on (12 S6A.8 OB-1). A bool
        says exactly what this module knows, which is whether it clipped.
        """
        if vx >= 0:
            return vx, False
        min_allowed = -self._cap
        if vx < min_allowed:
            return min_allowed, True
        return vx, False

    @property
    def cap(self) -> float:
        return self._cap
