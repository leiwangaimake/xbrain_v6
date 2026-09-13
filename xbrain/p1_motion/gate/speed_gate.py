"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: speed_gate.py
Brief: P1 speed gate f(d_free) -- the 11 S9.6.2 U54 four-band table and its rise hysteresis

Description:
Two things the free-space term of the speed gate needs and nothing else:
  * f_speed_gate(d): the 11 S9.6.2 / 12 S6.2 four-band table (U54 values),
    left-closed right-open:
        [3.0, inf) -> 2.0 | [1.8, 3.0) -> 0.5 | [1.25, 1.8) -> 0.2 | [0, 1.25) -> 0
  * BandHysteresis: the 12 S6.2 hysteresis table / 12 S6.7 (U54 rescale).
    Dropping to a slower band is instant (safety direction, no debounce);
    rising to a faster band k needs d_free >= lower_edge(k) + d_up_margin_m
    held CONTINUOUSLY for speed_up_hold_s. U54 fixes the top band at
    3.5 m / 3.0 s; the two lower bands follow the same rule by extrapolation
    (2.3 m, 1.75 m), a 12 S6.2 [建议] pending field tuning whose fallback is
    verbatim "按更严者" -- never "no hysteresis". Both numbers are injected
    from the resolved snapshot (speed_gate.hysteresis.*), no defaults.

Why the memory survives an unknown reading: a tick without a forward
distance (all forward bins null, 20 S4.1 UNKNOWN) returns None -- no f term
that tick -- but keeps the current band and breaks every rise timer. Coming
back from a gap therefore re-enters at the SLOWER of the remembered band and
the fresh reading, and any rise still needs its full hold: a robot cannot
"forget" a nearby obstacle by losing sight of it for one frame.

Why per-band timers: a jump from 1.0 m to 5.0 m must reach 2.0 in one hold
(every threshold has been satisfied the whole time), while a slow retreat
that has been past 2.3 m for 3 s but only just past 3.5 m must stop at 0.5
until 3.5 m has ITS own 3 s -- a single "next band" timer gets one of the two
wrong.

Band edges are the U54 constants here on purpose (user ruling 2026-09-13):
12 S6.2 says the edges are common.motion.profiles[*].require_sense_m, but
the sim variant carries 1.0 / 0.3 there against the 3.0 / 1.8 of the U54
table -- aligning the two is a separate registered item (NEXT.md 8.5 R-5).
gate_rule is the legacy MOT-PM-7 combination kept for its tests; the live
gate is nav/host_gate.py.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

#: 11 S9.6.2 U54 four-band table, fastest first: (lower edge m, speed m/s).
#: The last row is the stop band (lower edge 0).
BANDS: Tuple[Tuple[float, float], ...] = ((3.0, 2.0), (1.8, 0.5), (1.25, 0.2), (0.0, 0.0))


def band_of(d_free_m: float) -> int:
    """Index into BANDS (0 = fastest) for a forward clearance; left-closed."""
    for k, (lower, _v) in enumerate(BANDS):
        if d_free_m >= lower:
            return k
    return len(BANDS) - 1            # negative / nonsense distance: stop band


def f_speed_gate(d_free_m: float) -> float:
    """The raw table lookup (no hysteresis)."""
    return BANDS[band_of(d_free_m)][1]


class SpeedGateError(ValueError):
    """A hysteresis parameter missing / non-positive (CLAUDE.md 3.1: refuse)."""


class BandHysteresis:
    """The 12 S6.2 / S6.7 rise hysteresis over the four bands. One instance
    per control loop; update() once per tick with the forward clearance (or
    None when unknown) and use the returned speed as the gate's f term.
    mutant: promote as soon as the raw band is faster -> a clearing obstacle
    lets the speed jump 0.5 -> 2.0 on the next tick ->
    test_rise_needs_margin_and_hold red."""

    def __init__(self, speed_up_hold_ms: int, d_up_margin_m: float) -> None:
        if isinstance(speed_up_hold_ms, bool) or not isinstance(speed_up_hold_ms, int) \
                or speed_up_hold_ms <= 0:
            raise SpeedGateError("speed_gate.hysteresis.speed_up_hold_s must be > 0, got %r"
                                 % (speed_up_hold_ms,))
        if isinstance(d_up_margin_m, bool) or not isinstance(d_up_margin_m, (int, float)) \
                or d_up_margin_m <= 0.0:
            raise SpeedGateError("speed_gate.hysteresis.d_up_margin_m must be > 0, got %r"
                                 % (d_up_margin_m,))
        self._hold_ms = speed_up_hold_ms
        self._margin_m = float(d_up_margin_m)
        self._cur: Optional[int] = None                      # band index, None before any reading
        self._since: List[Optional[int]] = [None] * len(BANDS)   # rise timers per band

    @property
    def band(self) -> Optional[int]:
        return self._cur

    def rise_threshold_m(self, k: int) -> float:
        """d_free needed to start the rise timer of band k (12 S6.2 table:
        lower edge + d_up_margin_m; 3.5 / 2.3 / 1.75 with the U54 values)."""
        return BANDS[k][0] + self._margin_m

    def update(self, d_free_m: Optional[float], now_mono_ms: int) -> Optional[float]:
        """One tick. Returns the f term (m/s) or None when d_free is unknown."""
        if d_free_m is None:
            # unknown clearance: keep the band, break every rise (a rise must be
            # continuously observed), and give the gate no f term this tick.
            self._since = [None] * len(BANDS)
            return None
        raw = band_of(d_free_m)
        # per-band rise timers: running while d_free stays >= the threshold.
        for k in range(len(BANDS) - 1):
            if d_free_m >= self.rise_threshold_m(k):
                if self._since[k] is None:
                    self._since[k] = now_mono_ms
            else:
                self._since[k] = None
        if self._cur is None or raw > self._cur:
            # first reading, or the clearance fell into a slower band: take it
            # NOW (12 S6.7 "降速立即生效, 安全方向, 无去抖").
            self._cur = raw
        elif raw < self._cur:
            # faster band on offer: the fastest one whose threshold has been
            # held for the full hold wins; none -> stay.
            for k in range(self._cur):
                since = self._since[k]
                if since is not None and now_mono_ms - since >= self._hold_ms:
                    self._cur = k
                    break
        return BANDS[self._cur][1]


def gate_rule(f: float, g: float, h: float, i: float,
              hard_upper_mps: float) -> float:
    """Legacy MOT-PM-7 rule form: v_max = min(f, g*h*i, hard_upper), kept for
    its tests; the live combination is nav/host_gate.compute_gate."""
    return max(0.0, min(f, g * h * i, hard_upper_mps))


__all__ = ["BANDS", "band_of", "f_speed_gate", "SpeedGateError", "BandHysteresis",
           "gate_rule"]
