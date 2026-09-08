"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_rtk_health.py
Brief: RTK tiers + perception health limiting (P2.6/P2.4 -- 20 S3.2.1/S3.1.8)

Description:
Guards the RTK three-way (A-RTK-1 stop on SINGLE, A-RTK-2 reverse: FLOAT is
slow-not-stop) and the invalid-pixel-ratio speed cap (RNS-I-2). Each names its
mutant.
"""

from __future__ import annotations

from xbrain.p1_motion.rns.classify import (
    RtkTier, health_speed_capped, rtk_arrival_radius, rtk_speed_factor,
    rtk_stop_and_report,
)
from xbrain.p1_motion.rns.types import NavFailReason


def test_single_stops_and_reports():
    # A-RTK-1: SINGLE -> stop + rtk_unreliable. mutant: return None (keep going)
    # -> blind driving -> reddens.
    f = rtk_stop_and_report(RtkTier.SINGLE)
    assert f is not None
    assert f.reason is NavFailReason.RTK_UNRELIABLE


def test_fixed_and_float_do_not_stop():
    # A-RTK-2 (reverse): FLOAT must be slow-continue, NOT stop. mutant: make
    # FLOAT also return a failure -> a valid FLOAT fix aborts navigation -> red.
    assert rtk_stop_and_report(RtkTier.FIXED) is None
    assert rtk_stop_and_report(RtkTier.FLOAT) is None


def test_float_slows_fixed_does_not():
    assert rtk_speed_factor(RtkTier.FIXED, rtk_float_g=0.5) == 1.0
    assert rtk_speed_factor(RtkTier.FLOAT, rtk_float_g=0.5) == 0.5


def test_float_enlarges_arrival_radius():
    assert rtk_arrival_radius(RtkTier.FIXED, 1.0, float_scale=2.0) == 1.0
    assert rtk_arrival_radius(RtkTier.FLOAT, 1.0, float_scale=2.0) == 2.0


def test_health_cap_when_over_limit():
    # RNS-I-2: invalid ratio over limit -> capped.
    assert health_speed_capped(0.3, invalid_ratio_limit=0.2) is True
    assert health_speed_capped(0.1, invalid_ratio_limit=0.2) is False


def test_health_cap_when_limit_null_is_conservative():
    # null limit -> conservatively capped (CLAUDE.md 3.1). mutant: return False
    # on null (no cap) -> a missing threshold silently disables the guard -> red.
    assert health_speed_capped(0.05, invalid_ratio_limit=None) is True
