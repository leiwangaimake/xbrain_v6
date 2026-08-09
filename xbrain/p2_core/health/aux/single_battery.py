"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: single_battery.py
Brief: CHK-1-01 single-battery mode (spec.max_vx_eff into gate; battery=degraded)

Description:
14 §11.4 single-battery mode: when the second battery pack is
absent or unhealthy, the robot enters a reduced-capability mode:

  1  `spec.max_vx_eff` becomes an additional term inside the speed
     gate's min() -- always <= normal spec.max_vx
  2  ETA and energy-reach estimates switch to READ-BACK velocity
     (v_actual) not requested (v_requested); the single-battery
     current draw is enough to lag actual speed behind requested
  3  battery health item is marked 'degraded' (not 'error' -- the
     robot still moves, just slower)
  4  HMI displays a persistent 'SINGLE BATTERY' badge

Rules:
  * mode is a one-way trip in a given power cycle; recovery
    requires reboot after a healthy second battery is present.
  * exiting single-battery mode without a reboot is forbidden
    (guards against a flapping-battery scenario putting the robot
    into 'normal' mode with intermittent power loss)
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SingleBatteryMode:
    """Persist for the duration of the power cycle. Cannot exit
    at runtime (guarded)."""
    active: bool = False
    entered_reason: str = ""

    def enter(self, reason: str) -> None:
        if self.active:
            return
        self.active = True
        self.entered_reason = reason

    def can_exit_at_runtime(self) -> bool:
        return False       # always no -- requires reboot


def compose_max_vx_min(spec_max_vx: float,
                        spec_max_vx_eff: float,
                        active: bool) -> float:
    """When active, the effective ceiling enters min(); otherwise
    the normal max_vx is returned unchanged."""
    if active:
        return min(spec_max_vx, spec_max_vx_eff)
    return spec_max_vx


def choose_velocity_for_eta(v_requested: float,
                              v_actual: float,
                              active: bool) -> float:
    """When active, ETA uses actual velocity (which lags requested
    under single-battery current)."""
    return v_actual if active else v_requested
