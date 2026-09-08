"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: grid_motion.py
Brief: MOT-RD-6 grid motion compensation RNS-FC-12 (must-have)

Description:
20 §8 grid motion compensation: perception grid is published in
world frame; RNS operates in robot frame. Each tick, RNS must
transform grid cells from world to robot frame using odometry
delta between grid ts and current tick.

RNS-FC-12: this compensation is MANDATORY. If odometry delta is
unavailable (rtk_fail or clock_fail), RNS shuts down (returns
None candidate). Rationale: an uncompensated grid drifts
under motion; using it for corridor search would steer the robot
into walls that have effectively 'moved' in the RNS view.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


class OdomUnavailable(RuntimeError):
    """FC-12: odometry delta unavailable; RNS must shut down."""


def transform_grid_to_robot(
    grid_x_world: float, grid_y_world: float,
    robot_x_world: float, robot_y_world: float,
    robot_heading_rad: float,
) -> Tuple[float, float]:
    """Transform a world-frame grid cell to robot frame."""
    dx = grid_x_world - robot_x_world
    dy = grid_y_world - robot_y_world
    cos_h = math.cos(-robot_heading_rad)
    sin_h = math.sin(-robot_heading_rad)
    x_robot = dx * cos_h - dy * sin_h
    y_robot = dx * sin_h + dy * cos_h
    return (x_robot, y_robot)


def check_odom_available(odom_available: bool) -> None:
    """FC-12 hard gate: no odom -> RNS shuts down."""
    if not odom_available:
        raise OdomUnavailable(
            "FC-12: odometry delta unavailable; RNS grid compensation "
            "cannot run; module shuts down")
