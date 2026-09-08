"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_align.py
Brief: Progressive alignment + arrival double-condition (P1.4 -- 20 S2.5)

Description:
Guards route.py's alignment: heading blends in over align_dist (never in place,
RNS-N-3), and arrival needs BOTH radius and heading (A-ALN-3). The A-ALN-2 test
is the reverse pair: it asserts heading converges while v>0 (moving), killing a
"drive there then turn in place" implementation. Each test names its mutant.
"""

from __future__ import annotations

import math

import pytest

from xbrain.p1_motion.rns.route import (
    align_omega, align_weight, arrived_with_heading, desired_heading, wrap_angle,
)


def test_wrap_angle():
    assert wrap_angle(math.radians(359)) == pytest.approx(math.radians(-1))
    assert wrap_angle(math.radians(1)) == pytest.approx(math.radians(1))
    assert wrap_angle(math.pi) == pytest.approx(math.pi)


def test_align_weight_blends_over_distance():
    # far out (d >= align_dist): 0 (follow path direction). At endpoint: 1.
    assert align_weight(10.0, align_dist_m=5.0) == 0.0
    assert align_weight(0.0, align_dist_m=5.0) == 1.0
    assert align_weight(2.5, align_dist_m=5.0) == pytest.approx(0.5)


def test_desired_heading_no_goal_is_path_direction():
    # S2.4: no goal heading -> no alignment demand.
    assert desired_heading(0.3, None, 1.0, 5.0) == 0.3


def test_desired_heading_blends_toward_goal_near_end():
    # near the endpoint (d small), theta_des is close to psi_goal, not path dir.
    theta_path = 0.0
    psi_goal = math.radians(90)
    far = desired_heading(theta_path, psi_goal, d_remaining_m=5.0, align_dist_m=5.0)
    near = desired_heading(theta_path, psi_goal, d_remaining_m=0.5, align_dist_m=5.0)
    assert far == pytest.approx(0.0)          # w=0 -> path direction
    assert near > far                          # blending toward 90 deg
    assert near < psi_goal                     # not fully there yet at d=0.5


def test_align_omega_clamps_to_wz_max():
    # big heading error -> omega clamped, not unbounded.
    w = align_omega(theta_des=math.pi, psi_now=0.0, k_yaw=5.0, wz_max=1.0)
    assert w == 1.0
    w2 = align_omega(theta_des=-math.pi + 0.01, psi_now=0.0, k_yaw=5.0, wz_max=1.0)
    assert w2 == -1.0


def test_arrival_needs_both_radius_and_heading():
    # A-ALN-3: position reached but heading 30 deg off -> NOT arrived.
    # mutant: judge radius only -> reddens.
    psi_goal = math.radians(90)
    off = arrived_with_heading(
        dist_to_endpoint_m=0.5, arrival_radius_m=1.0,
        psi_goal=psi_goal, psi_now=math.radians(60),  # 30 deg off
        yaw_tol_rad=math.radians(5))
    assert off is False
    aligned = arrived_with_heading(
        dist_to_endpoint_m=0.5, arrival_radius_m=1.0,
        psi_goal=psi_goal, psi_now=math.radians(88),  # 2 deg off
        yaw_tol_rad=math.radians(5))
    assert aligned is True


def test_arrival_radius_alone_when_no_goal_heading():
    # no goal heading (intermediate-style goto) -> radius alone.
    assert arrived_with_heading(0.5, 1.0, None, 0.0, math.radians(5)) is True
    assert arrived_with_heading(1.5, 1.0, None, 0.0, math.radians(5)) is False


def test_heading_converges_while_moving_not_in_place():
    # A-ALN-2 (reverse): simulate the last align_dist with the robot MOVING
    # (d shrinking) and check heading error monotonically shrinks BEFORE arrival.
    # A "drive there then turn in place" impl keeps heading at path dir until
    # d~0, so the error stays large while moving -> this reddens it.
    theta_path = 0.0
    psi_goal = math.radians(90)
    align_dist = 5.0
    prev_err = None
    for d in [5.0, 4.0, 3.0, 2.0, 1.0, 0.3]:
        theta_des = desired_heading(theta_path, psi_goal, d, align_dist)
        err = abs(wrap_angle(psi_goal - theta_des))  # how far des is from goal
        if prev_err is not None:
            assert err <= prev_err + 1e-9, "heading demand not converging while moving"
        prev_err = err
    assert prev_err < math.radians(20)  # nearly aligned by the time d=0.3
