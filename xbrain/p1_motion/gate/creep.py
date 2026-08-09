"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: creep.py
Brief: CHK-1-21 creep-zero clamp + startup assertion v_creep_mps < f(d_free) min

Description:
12 §9.6.5 creep semantics: the speed gate emits a small positive
v_max when d_free is at the edge of the second-lowest band. To
avoid a jittery 'crawl-forward-then-stop' cycle, the gate applies
a CREEP CLAMP:

  if 0 < v_max < v_creep_mps: v_max := 0

Rationale: robot moving < v_creep_mps burns energy without
covering meaningful ground; better to stop and let a fresh d_free
sample decide.

Startup assertion: v_creep_mps MUST be strictly less than the
minimum NON-ZERO band-value returned by f(d_free). Otherwise the
creep clamp would eat every band and the robot would never move.

CHK-1-21 is a cross-file startup check: reads configs, computes
the minimum non-zero band, compares. Any violation refuses to
start (no runtime workaround, per CLAUDE.md 3.1 / 3.6).
"""

from __future__ import annotations


class CreepConfigError(Exception):
    """Startup assertion tripped: v_creep_mps too large."""


def apply_creep_clamp(v_max_mps: float, v_creep_mps: float) -> float:
    """Clamp small positive velocities to zero. v_max <= 0 is
    unchanged. v_creep_mps must be > 0 (from configs)."""
    if v_creep_mps <= 0:
        raise CreepConfigError(
            f"v_creep_mps must be > 0, got {v_creep_mps}")
    if 0 < v_max_mps < v_creep_mps:
        return 0.0
    return v_max_mps


def assert_creep_below_gate_min(v_creep_mps: float,
                                  band_values: list) -> None:
    """Startup gate: v_creep_mps < min(non-zero band values).
    If any non-zero band is <= v_creep_mps, refuse to start --
    at that band, the creep clamp would eat the entire allowance."""
    non_zero = [v for v in band_values if v > 0]
    if not non_zero:
        raise CreepConfigError(
            "no non-zero gate bands; cannot validate creep")
    band_min = min(non_zero)
    if v_creep_mps >= band_min:
        raise CreepConfigError(
            f"v_creep_mps={v_creep_mps} >= gate min={band_min}; "
            f"would clamp every allowance to zero")
