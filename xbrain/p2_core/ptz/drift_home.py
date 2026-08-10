"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: drift_home.py
Brief: CHK-2-08 PTZ open-loop drift auto-home (PAY-29 three triggers, ptz.t_drift_s)

Description:
PAY-29 defines THREE independent triggers that cause the PTZ to
return to its 'travelling observation' preset:

  T-1  exit of D-mode OR B-mode          -> immediate goto_preset
  T-2  continuous open-loop tracking has
       accumulated > ptz.t_drift_s
       (measured on MONOTONIC clock; never wall) -> auto-home
  T-3  operator issued a 'zero' command  -> immediate

The `T_drift` is measured on the monotonic clock; a wall-clock
implementation would falsely trigger on NTP step (variant A).

Speed tiers (PAY-38): cmd/ptz uses coarse or fine speed profile;
BOTH values are injected from config (no defaults).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class DriftHomeTrigger(str, Enum):
    MODE_EXIT = "mode_exit"
    T_DRIFT_ACCUMULATED = "t_drift_accumulated"
    OPERATOR_ZERO = "operator_zero"


class DriftHomeConfigError(Exception):
    pass


@dataclass(frozen=True)
class DriftHomeConfig:
    """Zero t_drift_s = no auto-home; refused (fail-silent form)."""
    t_drift_s: float

    def __post_init__(self) -> None:
        if self.t_drift_s <= 0:
            raise DriftHomeConfigError(
                f"t_drift_s must be > 0, got {self.t_drift_s} "
                f"(zero = no auto-home = drift unchecked)")


@dataclass
class DriftHomeState:
    """Tracks accumulated tracking time on MONOTONIC clock."""
    accumulated_mono_ms: int = 0
    last_tick_mono_ms: int = 0
    homed_times: int = 0

    def on_tracking_tick(self, now_mono_ms: int) -> None:
        """Called each control tick while open-loop tracking is
        active. First call establishes the baseline."""
        if self.last_tick_mono_ms == 0:
            self.last_tick_mono_ms = now_mono_ms
            return
        dt = now_mono_ms - self.last_tick_mono_ms
        if dt < 0:
            # Clock ran backwards (shouldn't on monotonic). Ignore.
            self.last_tick_mono_ms = now_mono_ms
            return
        self.accumulated_mono_ms += dt
        self.last_tick_mono_ms = now_mono_ms

    def reset_accumulation(self) -> None:
        self.accumulated_mono_ms = 0
        self.last_tick_mono_ms = 0


def check_t_drift_trigger(state: DriftHomeState,
                            cfg: DriftHomeConfig) -> bool:
    """T-2: True if accumulated tracking has crossed the threshold."""
    return state.accumulated_mono_ms >= cfg.t_drift_s * 1000


def note_home_fired(state: DriftHomeState) -> None:
    """Reset accumulator; increment homed count for observability."""
    state.homed_times += 1
    state.reset_accumulation()


# ---- speed tiers (PAY-38) ------------------------------------------

class PtzSpeedTierConfigError(Exception):
    pass


@dataclass(frozen=True)
class PtzSpeedTiers:
    """Both coarse and fine required; equal is refused (collapsing
    to one tier defeats the operator-fine-tuning use case)."""
    speed_coarse: float
    speed_fine: float

    def __post_init__(self) -> None:
        for name in ("speed_coarse", "speed_fine"):
            v = getattr(self, name)
            if v <= 0:
                raise PtzSpeedTierConfigError(
                    f"{name} must be > 0, got {v}")
        if self.speed_coarse == self.speed_fine:
            raise PtzSpeedTierConfigError(
                f"speed_coarse ({self.speed_coarse}) == speed_fine "
                f"({self.speed_fine}); the two tiers collapsed into "
                f"one -- operator loses the fine-tune ability")
        if self.speed_fine >= self.speed_coarse:
            raise PtzSpeedTierConfigError(
                f"speed_fine ({self.speed_fine}) must be < speed_coarse "
                f"({self.speed_coarse}); tier ordering broken")
