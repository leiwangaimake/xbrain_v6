"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: speed_gate.py
Brief: MOT-PM-6/7 speed gate f(d_free) + rule form

Description:
Four-band linear-speed cap keyed on d_free (front clearance). Bands are left-closed right-open; upgrade to the 2.0 top band requires 3.5 m sustained for 3 s (dropping is instant). The gate rule combines f(d_free) with g(targets), heading factor h, and rtk factor i via min(); a hard upper cap clips the whole thing.
"""



from __future__ import annotations

from dataclasses import dataclass


# 12 S6.2 four-band f(d_free). Left-closed right-open.
# [3.0, inf) -> 2.0 | [1.8, 3.0) -> 0.5 | [1.25, 1.8) -> 0.2 | [0, 1.25) -> 0
def f_speed_gate(d_free_m: float) -> float:
    if d_free_m >= 3.0:
        return 2.0
    if d_free_m >= 1.8:
        return 0.5
    if d_free_m >= 1.25:
        return 0.2
    return 0.0


# Hysteresis: dropping is instantaneous; rising to 2.0 requires
# d_free >= 3.5 m sustained for 3 s.
HYSTERESIS_UP_M = 3.5
HYSTERESIS_UP_DWELL_S = 3.0


@dataclass
class GateHysteresis:
    """Tracks the 'sustained above 3.5 for 3 s' state for band upgrade."""
    _above_since_mono_ms: int = 0
    _at_upper_band: bool = False

    def update(self, d_free_m: float, now_mono_ms: int) -> None:
        if d_free_m >= HYSTERESIS_UP_M:
            if self._above_since_mono_ms == 0:
                self._above_since_mono_ms = now_mono_ms
            if (now_mono_ms - self._above_since_mono_ms
                    >= int(HYSTERESIS_UP_DWELL_S * 1000)):
                self._at_upper_band = True
        else:
            self._above_since_mono_ms = 0
            # dropping is instant; leaves upper band as soon as
            # d_free drops below 3.0 (or actually below HYSTERESIS_UP_M
            # matches the band boundary rule -- being strict here).
            if d_free_m < 3.0:
                self._at_upper_band = False


def gate_rule(f: float, g: float, h: float, i: float,
              hard_upper_mps: float) -> float:
    """12 S6.7 gate rule: v_max = min(f, g*h*i, hard_upper)."""
    return max(0.0, min(f, g * h * i, hard_upper_mps))
