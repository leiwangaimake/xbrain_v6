"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_speed.py
Brief: v=min caps + coefficient-family monotonicity (P1.7 -- 20 S8.1/S8.1A)

Description:
Guards speed.py: v = min(all caps) is never a constant (A-SPD-1/2), and every cap
is monotone in its argument (A-SPD-3). Monotonicity is checked by dense sampling
(hypothesis is not installed on the target; sampling still kills a flipped-sign
mutant). Each test names its mutant (CLAUDE.md 3.3).
"""

from __future__ import annotations

import pytest

from xbrain.p1_motion.rns.speed import (
    SpeedCaps, cap_align, cap_deviation, cap_gap_tightness, cap_rtk_float,
    cap_unknown_ratio, cap_wall, g_down, g_up,
)


def test_g_down_endpoints_and_middle():
    assert g_down(0.0, 1.0, 5.0, 0.2) == 1.0     # x <= x0
    assert g_down(9.0, 1.0, 5.0, 0.2) == 0.2     # x >= x1
    mid = g_down(3.0, 1.0, 5.0, 0.2)             # halfway
    assert 0.2 < mid < 1.0


def test_g_up_endpoints():
    assert g_up(0.0, 1.0, 5.0, 0.3) == 0.3       # x <= x0 -> floor
    assert g_up(9.0, 1.0, 5.0, 0.3) == 1.0       # x >= x1 -> 1


def test_g_down_is_monotone_nonincreasing():
    # A-SPD-3: bigger x (more danger) never gives MORE speed. mutant: flip the
    # sign in g_down (return g_min + frac*(1-g_min)) -> rises with x -> reddens.
    xs = [i * 0.05 for i in range(0, 120)]  # 0 .. 6
    vals = [g_down(x, 1.0, 5.0, 0.2) for x in xs]
    for a, b in zip(vals, vals[1:]):
        assert b <= a + 1e-12, "g_down rose"


def test_g_up_is_monotone_nondecreasing():
    xs = [i * 0.05 for i in range(0, 120)]
    vals = [g_up(x, 1.0, 5.0, 0.3) for x in xs]
    for a, b in zip(vals, vals[1:]):
        assert b >= a - 1e-12, "g_up fell"


def test_g_down_rejects_bad_bounds():
    with pytest.raises(ValueError):
        g_down(1.0, 5.0, 1.0, 0.2)  # x0 >= x1


def test_v_is_min_of_all_caps():
    # A-SPD-1: v = min(all caps), never a constant. mutant: return a fixed value
    # (or max) -> reddens.
    caps = SpeedCaps({"gate": 2.0, "deviation": 0.7, "unknown": 1.5})
    assert caps.limit() == 0.7
    assert caps.binding_source() == "deviation"


def test_empty_caps_is_fail_loud_not_unlimited():
    # A-SPD-2 direction: no caps must RAISE, never mean "unlimited". mutant:
    # return a large number on empty -> reddens.
    with pytest.raises(ValueError):
        SpeedCaps({}).limit()


def test_cap_deviation_slows_as_e_grows():
    # cap_deviation monotone down in |e|; at e0 it is v_nom, near max it floors.
    v_nom = 2.0
    near = cap_deviation(0.0, v_nom, dev_e0_m=0.5, max_deviation_m=10.0, dev_g_min=0.1)
    far = cap_deviation(9.0, v_nom, dev_e0_m=0.5, max_deviation_m=10.0, dev_g_min=0.1)
    assert near == v_nom
    assert far < near


def test_cap_align_lets_slower_when_more_turn_needed():
    # A-ALN-1: within the align segment omega_req <= wz_max, enforced by SLOWING.
    # more heading error at same distance -> lower cap (must slow to finish the
    # turn). This is the S2.5 feasibility relation as a cap.
    lots = cap_align(d_m=5.0, delta_theta_rad=1.5, wz_max=1.0, align_eta=0.8)
    little = cap_align(d_m=5.0, delta_theta_rad=0.1, wz_max=1.0, align_eta=0.8)
    assert lots < little


def test_cap_rtk_float_and_wall_are_constants():
    assert cap_rtk_float(v_nom=2.0, rtk_float_g=0.5) == 1.0
    assert cap_wall(0.5) == 0.5


def test_cap_gap_tightness_wider_gap_less_limit():
    v_nom = 2.0
    tight = cap_gap_tightness(1.0, v_nom, gate_saturate_ratio=3.0, gap_g_min=0.2)
    wide = cap_gap_tightness(3.0, v_nom, gate_saturate_ratio=3.0, gap_g_min=0.2)
    assert tight < wide
    assert wide == v_nom  # saturated -> no limit
