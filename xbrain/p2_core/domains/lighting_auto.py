"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: lighting_auto.py
Brief: BIZ-P2-8 -- auto lighting judgment chain (A/B/C sources + rules A-1..A-7)

Description:
Three-level judgment chain (14 S4.3.2):
  A photocell   -- direct photodiode reading
  B image       -- camera-derived brightness estimate
  C almanac     -- solar elevation (fail-safe fallback, zero-dep)

Rules (14 S4.3.2 A-1..A-7):
  A-1 src_chain in yaml order; downgrade if source unavailable
  A-2 selected source must be fresh (max_age_s per source)
  A-3 hysteresis: on_lux_equiv / off_lux_equiv separate thresholds
  A-4 off-direction dwell = min_dwell_s * off_dwell_mult
  A-5 (subsumed by A-4 dwell math)
  A-6 fail-safe: if ALL sources fail the judgment, light ON
  A-7 red/blue strobe ON AND judged dark -> illumination ON

This module owns the LOGIC. Reading photodiode / running image
brightness / computing solar elevation are separate sensor
adapters. This function takes the three inputs already normalised
and returns the light_effective decision.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class LightingInputs:
    """Normalised inputs from the three sources plus context.

    Each source's value is Optional[float]: None = source unavailable
    for THIS evaluation (stale, disabled, or fails 14 S4.3.2 A-2 age
    check). Rules A-1 walks the chain top-down; first available
    source's value wins."""
    photocell_lux: Optional[float]
    image_lux_equiv: Optional[float]
    almanac_sun_elev_deg: Optional[float]
    # Threshold pair for A / B (lux equivalent) -- from p2_core.yaml.
    on_lux_equiv:  Optional[float]
    off_lux_equiv: Optional[float]
    # Threshold pair for C (degrees). Both must have concrete values
    # or almanac itself is unavailable.
    night_on_sun_elev_deg:  Optional[float]
    night_off_sun_elev_deg: Optional[float]
    # A-7: red/blue strobe currently ON.
    redblue_strobe_active: bool
    # Currently-light-on state (for hysteresis; A-3).
    currently_on: bool


def decide_light_effective(inp: LightingInputs) -> bool:
    """Return the recommended light-on state.

    Rules order:
      1. A-7 short-circuit: strobe active AND judged dark by any
         source -> ON.
      2. Walk src_chain: first available source's judgment wins.
      3. A-6 fail-safe: nobody available -> ON (dark-side safe).
    """
    # Compute "any judged dark" for A-7 short-circuit.
    any_dark = _any_source_says_dark(inp)

    if inp.redblue_strobe_active and any_dark:
        return True   # A-7: strobe ON + dark -> illumination ON

    # Walk chain photocell -> image -> almanac.
    verdict = _judge_photocell(inp)
    if verdict is not None:
        return verdict
    verdict = _judge_image(inp)
    if verdict is not None:
        return verdict
    verdict = _judge_almanac(inp)
    if verdict is not None:
        return verdict

    # A-6 fail-safe: no source available.
    return True


def _judge_photocell(inp: LightingInputs) -> Optional[bool]:
    """Photocell judgment with hysteresis (A-3). None if unavailable
    or thresholds not calibrated (both must be non-None)."""
    if inp.photocell_lux is None:
        return None
    if inp.on_lux_equiv is None or inp.off_lux_equiv is None:
        return None
    return _apply_hysteresis(
        inp.photocell_lux, inp.on_lux_equiv, inp.off_lux_equiv,
        inp.currently_on,
        lower_is_dark=True,
    )


def _judge_image(inp: LightingInputs) -> Optional[bool]:
    if inp.image_lux_equiv is None:
        return None
    if inp.on_lux_equiv is None or inp.off_lux_equiv is None:
        return None
    return _apply_hysteresis(
        inp.image_lux_equiv, inp.on_lux_equiv, inp.off_lux_equiv,
        inp.currently_on,
        lower_is_dark=True,
    )


def _judge_almanac(inp: LightingInputs) -> Optional[bool]:
    """Almanac uses sun elevation (deg). Below night_on -> dark,
    above night_off -> light. Thresholds MUST be non-None or the
    source is unavailable."""
    if inp.almanac_sun_elev_deg is None:
        return None
    if (inp.night_on_sun_elev_deg is None
            or inp.night_off_sun_elev_deg is None):
        return None
    # Lower elevation = darker. on/off thresholds sit at NEGATIVE deg
    # (below horizon). Use hysteresis with lower_is_dark=True.
    return _apply_hysteresis(
        inp.almanac_sun_elev_deg,
        inp.night_on_sun_elev_deg,
        inp.night_off_sun_elev_deg,
        inp.currently_on,
        lower_is_dark=True,
    )


def _apply_hysteresis(value: float, on_thresh: float, off_thresh: float,
                       currently_on: bool, lower_is_dark: bool) -> bool:
    """Two-threshold hysteresis. lower_is_dark: value < on_thresh
    means dark -> light ON. value > off_thresh means light -> light OFF."""
    if currently_on:
        # Stay on until value CROSSES off_thresh in the light direction.
        if lower_is_dark:
            return value <= off_thresh
        return value >= off_thresh
    # Not currently on -> only turn on if value CROSSES on_thresh.
    if lower_is_dark:
        return value <= on_thresh
    return value >= on_thresh


def _any_source_says_dark(inp: LightingInputs) -> bool:
    """A-7 helper: any of the 3 sources judges 'dark'."""
    for judged in (_judge_photocell(inp),
                    _judge_image(inp),
                    _judge_almanac(inp)):
        if judged is True:
            return True
    return False
