"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: relative_move.py
Brief: MOT-PM-18 relative_move executor (SM + trapezoidal profile + abort closed set)

Description:
relative_move is the one-shot MI-1 chassis motion executor: given a
delta (linear m or angular rad), execute a trapezoidal velocity
profile then arrive. The state machine uses UPPERCASE internal names
(ACCEPTED / RUNNING / ARRIVED / ABORTED / TIMEOUT) but the wire
enum on cmd/motion/relative_move/status is 5-value lowercase; a
mapping table converts. Serialised output MUST NEVER contain the
uppercase forms -- a variant catches that regression.

abort_reason is a 6-value closed set (limit_exceeded /
fence_violation / preempted / timeout / hes_asserted / user_cancel);
the delegate layer's per-source rejection detail lives elsewhere
and does NOT leak into abort_reason.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import FrozenSet, Optional


class _InternalState(str, Enum):
    """Uppercase internal names -- NEVER serialised to the wire."""
    ACCEPTED = "ACCEPTED"
    RUNNING = "RUNNING"
    ARRIVED = "ARRIVED"
    ABORTED = "ABORTED"
    TIMEOUT = "TIMEOUT"


# 5-value lowercase wire enum + mapping.
WIRE_STATE_MAP = {
    _InternalState.ACCEPTED: "accepted",
    _InternalState.RUNNING: "running",
    _InternalState.ARRIVED: "arrived",
    _InternalState.ABORTED: "aborted",
    _InternalState.TIMEOUT: "timeout",
}

ABORT_REASONS: FrozenSet[str] = frozenset({
    "limit_exceeded", "fence_violation", "preempted",
    "timeout", "hes_asserted", "user_cancel",
})


class RelativeMoveError(RuntimeError):
    """abort_reason out of the 6-value closed set."""


@dataclass
class TrapezoidProfile:
    """Trapezoidal velocity profile parameters."""
    v_max_mps: float
    a_max_mps2: float
    d_target_m: float

    def phase_lengths(self) -> tuple:
        """Return (t_accel_s, t_cruise_s, t_decel_s) for the trapezoid.
        If target distance is too short for full v_max, returns a
        triangular profile with reduced v_max."""
        # Distance to reach v_max: v_max^2 / (2*a).
        d_accel = (self.v_max_mps ** 2) / (2.0 * self.a_max_mps2)
        if 2 * d_accel > self.d_target_m:
            # Triangular: reduce v_peak.
            v_peak = (self.a_max_mps2 * self.d_target_m) ** 0.5
            t_accel = v_peak / self.a_max_mps2
            return (t_accel, 0.0, t_accel)
        t_accel = self.v_max_mps / self.a_max_mps2
        d_cruise = self.d_target_m - 2 * d_accel
        t_cruise = d_cruise / self.v_max_mps
        return (t_accel, t_cruise, t_accel)


def to_wire_state(internal: _InternalState) -> str:
    """Convert internal uppercase state to wire lowercase. Serialised
    output must ONLY use these lowercase values."""
    return WIRE_STATE_MAP[internal]


def validate_abort_reason(reason: str) -> None:
    """Refuse an abort_reason outside the 6-value closed set."""
    if reason not in ABORT_REASONS:
        raise RelativeMoveError(
            "abort_reason %r not in closed set %s"
            % (reason, sorted(ABORT_REASONS)))
