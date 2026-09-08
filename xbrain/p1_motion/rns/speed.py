"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: speed.py
Brief: v = min(all caps) + speed-cap coefficient family (P1.7 -- 20 S8.1/S8.1A)

Description:
The single speed authority (20 S8.1): the tick speed is the MIN of every cap, not
a constant. "Drive at the highest supported speed" is a per-tick minimum, so a
constant here is the exact defect A-SPD-1/2 kill.

The six qualitative caps (20 S8.1A) share one coefficient family g(x): a piecewise
value in [g_min, 1] multiplied by the mission nominal speed. g_down decreases as x
grows (x = danger), g_up increases (x = safety). Monotonicity is a testable
property (A-SPD-3): every cap is monotone in its argument. 20 S8.1A forbids adding
smoothing/hysteresis on top -- jitter is treated at the input side (ageing, grid,
hysteresis keys); doing it in both places fights itself.

This slice (P1.7): the family, the caps that need no other module yet (deviation,
unknown ratio, align, RTK float, wall cap), and the min-combiner. The gap-tightness
cap (needs candidate.py) and the f(d_free) gate (12 S6.2) are folded in as their
producers land (P4 / speed-gate wiring).

Units: caps are Mps (absolute speed limits). The coefficient itself is a Factor
(CFG-CM-18); it is applied to v_nom (Mps) to produce a cap (Mps). Cross-unit
discipline is in common.types.units; here we keep the arithmetic in plain floats
inside the family and wrap at the cap boundary to stay readable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Optional


def g_down(x: float, x0: float, x1: float, g_min: float) -> float:
    """g_down(x): 1 at x <= x0, g_min at x >= x1, linear between (20 S8.1A). x is
    a DANGER variable -- bigger x, slower. Requires x0 < x1."""
    if x1 <= x0:
        raise ValueError("g_down needs x0 < x1 (got x0=%r x1=%r)" % (x0, x1))
    if x <= x0:
        return 1.0
    if x >= x1:
        return g_min
    frac = (x - x0) / (x1 - x0)
    return 1.0 - frac * (1.0 - g_min)


def g_up(x: float, x0: float, x1: float, g_min: float) -> float:
    """g_up(x): g_min at x <= x0, 1 at x >= x1, linear between. x is a SAFETY
    variable -- bigger x, faster (e.g. gap tightness rho: wider gap, less limit)."""
    if x1 <= x0:
        raise ValueError("g_up needs x0 < x1 (got x0=%r x1=%r)" % (x0, x1))
    if x <= x0:
        return g_min
    if x >= x1:
        return 1.0
    frac = (x - x0) / (x1 - x0)
    return g_min + frac * (1.0 - g_min)


@dataclass(frozen=True)
class SpeedCaps:
    """The set of speed caps active this tick, keyed by source name for
    attribution (which limiter is binding -> 20 S9.0.2 watchdog detail, HMI).
    Every value is an absolute speed in m/s; the tick speed is their min."""
    caps: Dict[str, float]

    def limit(self) -> float:
        """v = min(all caps) (20 S8.1). Empty -> raises: a tick with no cap at
        all is a wiring bug, never "unlimited" (fail-loud, not fail-open)."""
        if not self.caps:
            raise ValueError("no speed caps -- min of empty is not 'unlimited' "
                             "(20 S8.1: v is the min of all limits)")
        return min(self.caps.values())

    def binding_source(self) -> str:
        """Which cap is binding (the argmin). Feeds the watchdog's argmax-limiter
        detail (20 S7.3A) and HMI attribution."""
        return min(self.caps.items(), key=lambda kv: kv[1])[0]


def cap_deviation(e_m: float, v_nom: float, dev_e0_m: float,
                  max_deviation_m: float, dev_g_min: float) -> float:
    """Lateral-deviation cap (20 S8.1A): g_down over |e|, x1 REUSES
    max_deviation_m (at that bound S2.7 fails; the cap floors just before)."""
    return g_down(abs(e_m), dev_e0_m, max_deviation_m, dev_g_min) * v_nom


def cap_unknown_ratio(ratio: float, v_nom: float, unk_r0: float,
                      unk_r1: float, unk_g_min: float) -> float:
    """UNKNOWN-fraction cap (20 S8.1A): g_down over the forward-sector UNKNOWN
    bin ratio."""
    return g_down(ratio, unk_r0, unk_r1, unk_g_min) * v_nom


def cap_gap_tightness(rho: float, v_nom: float, gate_saturate_ratio: float,
                      gap_g_min: float) -> float:
    """Gap-tightness cap (20 S8.1A): g_up over rho = w/(2*(r_eff+margin)); at
    rho=1 the gap is exactly body+margin (tightest passable), saturating at
    gate_saturate_ratio (wider -> no limit). rho comes from candidate.py (P4)."""
    return g_up(rho, 1.0, gate_saturate_ratio, gap_g_min) * v_nom


def cap_align(d_m: float, delta_theta_rad: float, wz_max: float,
              align_eta: float) -> float:
    """Align-segment cap (20 S2.5 / S8.1A, absolute form): v <= eta * d * wz_max
    / max(|dtheta|, 1e-3). Ensures the required yaw rate stays within wz_max so
    the robot arrives with heading converged, never needing an in-place turn."""
    return align_eta * d_m * wz_max / max(abs(delta_theta_rad), 1e-3)


def cap_rtk_float(v_nom: float, rtk_float_g: float) -> float:
    """RTK FLOAT cap (20 S3.2.1): constant coefficient on v_nom."""
    return rtk_float_g * v_nom


def cap_wall(wall_v_max_mps: float) -> float:
    """Wall-follow cap (20 S8.1A): constant hat, the wall_follow.v_max_mps key."""
    return wall_v_max_mps
