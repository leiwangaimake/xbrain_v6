"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: module.py
Brief: MOT-RD-1 RNS module skeleton (RNS-M-1..M-5) + input snapshot contract

Description:
20 §1 RNS is a P1-INTERNAL module (NOT a separate process). It
appears externally as behavior source `rns_avoid` priority 700.
Module discipline (RNS-M-1..M-5):
  M-1  in-process only; no Zenoh session, no separate thread
  M-2  input is a SNAPSHOT taken at tick start; NO reads mid-tick
  M-3  output is a candidate (vx, wz) OR None; never mutates state
  M-4  RNS crash confined to the module -- P1 loop continues without it
  M-5  no async / no I/O
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass(frozen=True)
class RnsSnapshot:
    """M-2: single tick's input snapshot. All fields immutable."""
    grid_targets: tuple      # tuple of (dx, dy) target positions
    d_free_forward_m: float
    lidar_available: bool
    grid_age_ms: int
    robot_vx_mps: float
    robot_vy_mps: float


@dataclass(frozen=True)
class RnsCandidate:
    """M-3: output shape. None wire = no proposal this tick."""
    vx_mps: float
    wz_radps: float
    reason: str            # 'left' / 'right' / 'straight' / 'blocked'


class RnsModuleUnavailable(RuntimeError):
    """M-4: raised when the module refuses to produce a candidate
    (e.g. grid stale). P1 loop catches and skips rns_avoid source."""
