"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: tilt_limit.py
Brief: CHK-1-07 PTZ tilt soft-limit (ptz.tilt_limit_deg -90..+30)

Description:
PAY-11b: tilt is conservatively bounded to [-90, +30] degrees.
The upper bound at +30 (rather than the hardware +90 ceiling)
prevents the camera from tipping past its cable-strain point
before T-PTZ-3 field measurement proves the hardware envelope.

Discipline:
  * range values come from constructor injection (no
    dataclass default; CLAUDE.md 3.1). Fixture / configs supply
    them per deploy.
  * out-of-range tilt does NOT clamp -- it REJECTS with a code
    from the closed E_* set AND detail.actual + detail.limits.
    Clamping to a nearby value would silently mask an operator
    error (the "clamp to 0 instead of reject" defect the CHK-1-07
    variant 3 test guards).
  * outward expansion (say +30 -> +60) is a config change that
    must be paired with a T-PTZ-3 completion marker; the freeze
    assertion G row rejects an outward expansion without evidence
    (CHK-1-07 variant 1 test). This module does not enforce that
    coupling itself -- it's a freeze-side assertion row -- but it
    documents the coupling in its module doc for the freeze
    row's author to reference.
"""

from __future__ import annotations

from dataclasses import dataclass

from xbrain.common.errors import E_CONFIG_INVALID


@dataclass(frozen=True)
class TiltLimits:
    """Injected at construction. min must be < max; both required."""
    min_deg: float
    max_deg: float

    def __post_init__(self) -> None:
        if self.min_deg >= self.max_deg:
            raise ValueError(
                "TiltLimits: min_deg (%r) must be < max_deg (%r); "
                "an inverted or equal range would refuse every "
                "command silently" % (self.min_deg, self.max_deg))


class TiltOutOfLimits(Exception):
    """Command's tilt value falls outside the injected limits."""

    def __init__(self, actual_deg: float, limits: TiltLimits) -> None:
        self.actual_deg = actual_deg
        self.limits = limits
        self.code = E_CONFIG_INVALID
        self.detail = {
            "kind": "ptz_tilt_out_of_range",
            "actual_deg": actual_deg,
            "min_deg": limits.min_deg,
            "max_deg": limits.max_deg,
        }
        super().__init__(
            "ptz tilt %.2f deg outside limits [%.2f, %.2f]"
            % (actual_deg, limits.min_deg, limits.max_deg))


def check_tilt(tilt_deg: float, limits: TiltLimits) -> None:
    """Raise TiltOutOfLimits if tilt_deg is outside the injected
    range. Boundary values (== min or == max) are ACCEPTED --
    the spec's -90 and +30 are inclusive."""
    if tilt_deg < limits.min_deg or tilt_deg > limits.max_deg:
        raise TiltOutOfLimits(tilt_deg, limits)
