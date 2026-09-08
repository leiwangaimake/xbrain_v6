"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: classify.py
Brief: RTK tiers + perception health + obstacle dispatch (P2.6/P3 -- 20 S3.2.1/S5)

Description:
This slice (P2.6/P2.4): the RTK quality tiers (RNS-N-6) and perception-health
speed limiting. The obstacle-class dispatch (RNS-N-7, class_map, static/dynamic,
velocity_frame==raw refusal) lands in P3 -- it shares this file because both are
"decide what the world means before acting", but they are separable and the
dispatch depends on classify config not present this slice.

RTK tiers (RNS-N-6, S3.2.1) -- the three-way, failure directions written in:
  FIXED   -> normal
  FLOAT   -> slow (rtk_float_g) + enlarge arrival radius (arrival_radius_float_
             scale): dm-level error may never reach the old radius, or reach it
             early.
  SINGLE/no-fix -> STOP and report rtk_unreliable (A-RTK-1): "where am I" is no
             longer trustworthy; continuing is blind driving toward an absolute
             goal, which is meaningless. NOT slow-and-continue (A-RTK-2 reverse).
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

from .types import NavFailReason, NavFailure


class RtkTier(str, Enum):
    """RTK fix quality, mapped to the three RNS behaviors (S3.2.1). Values match
    the fix_type semantics upstream; RNS only cares about these three tiers."""
    FIXED = "fixed"
    FLOAT = "float"
    SINGLE = "single"      # single / no-fix / no solution -> stop


def rtk_stop_and_report(tier: RtkTier) -> Optional[NavFailure]:
    """A-RTK-1: SINGLE/no-fix -> stop and report rtk_unreliable. FIXED/FLOAT do
    not fail here (FLOAT is slow-continue, handled by the caps). mutant: return
    None for SINGLE (keep driving) -> blind driving -> reddens."""
    if tier == RtkTier.SINGLE:
        return NavFailure(reason=NavFailReason.RTK_UNRELIABLE,
                          detail={"fix": tier.value})
    return None


def rtk_speed_factor(tier: RtkTier, rtk_float_g: float) -> float:
    """The speed coefficient for the RTK tier (S3.2.1 / S8.1A). FIXED = 1.0 (no
    limit), FLOAT = rtk_float_g (< 1). SINGLE never reaches here (it stopped)."""
    if tier == RtkTier.FLOAT:
        return rtk_float_g
    return 1.0


def rtk_arrival_radius(tier: RtkTier, base_radius_m: float,
                       float_scale: float) -> float:
    """FLOAT enlarges the arrival radius (S3.2.1): dm-level error would otherwise
    never satisfy the base radius."""
    if tier == RtkTier.FLOAT:
        return base_radius_m * float_scale
    return base_radius_m


def health_speed_capped(invalid_pixel_ratio: float,
                        invalid_ratio_limit: Optional[float]) -> bool:
    """RNS-I-2 / S3.1.8: invalid_pixel_ratio over the limit forces a speed cap --
    the only cover for "both semantic and geometry are blind" (glass/water where
    both channels say no obstacle). invalid_ratio_limit null -> conservatively
    capped (unusable threshold -> worst case, CLAUDE.md 3.1)."""
    if invalid_ratio_limit is None:
        return True
    return invalid_pixel_ratio > invalid_ratio_limit
