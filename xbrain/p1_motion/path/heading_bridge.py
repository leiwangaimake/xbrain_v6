"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: heading_bridge.py
Brief: IMU yaw bridge over GNSS heading gaps plus a COG consistency check (99 U152)

Description:
What problem this solves. Measured outdoors 2026-11-06 (patrol t-20261106-007,
the robot hit a wall): at a 62 degree route corner RNS slowed to 0.2 m/s and
turned, the dual antenna baseline dropped, COG was blind, the U151 climb took
over with a heading 27 degrees off, and when the robot sped up the first
"valid" COG read 135 deg while the body faced -106 deg (119 degrees off -- a
legged robot turning on a tight arc moves sideways, its track is not its
nose). RNS turned at full rate on the wrong heading into the wall. Over the
same 15 s the chassis odom yaw (IMU-integrated) agreed with the dual antenna
to within 1 degree. The user ruled (2026-11-06, U152): bridge the GNSS heading
gaps with the IMU yaw change, and use COG only when it agrees with it.

How (one step per tick):
  * a TRUSTED GNSS heading re-anchors (heading, odom yaw, time) every tick.
    Trusted = dual antenna, or a COG that passes the consistency check;
  * prediction = anchor heading + wrap(odom yaw now - odom yaw at anchor),
    available for max_s after the anchor (nav.heading_bridge_max_s);
  * a COG heading is accepted when |wrap(cog - prediction)| <= tol
    (nav.heading_consistency_deg). With no prediction (no anchor yet, anchor
    expired) it must agree with the previous COG tick instead: the COG change
    since that tick must match the odom yaw change within tol -- a COG that
    swings while the body does not (or the reverse) is a track, not a nose;
  * an accepted COG PULLS the estimate toward itself with time constant
    nav.cog_blend_tau_s (a first-order filter on top of the IMU prediction)
    instead of replacing it; the dual antenna replaces it. Replacing let a
    COG lagging a slow turn walk the anchor 5 -> 12 -> 29 deg off in steps
    each under the tolerance, after which the CORRECT COG was rejected
    (replay of t-20261106-007: worst error 28.9 deg replacing, 4.2 pulling);
  * no trusted GNSS heading this tick -> the prediction, marked "bridge";
    none -> invalid (the gate stops, or the U151 climb takes over).

Boundaries: decides only the heading P1's gate / RNS see. rtk_driver's levels,
events and state/pose are untouched. A bridged heading is not yaw_capable.
Without an odom yaw (chassis state absent / stale / not valid) nothing can be
checked: GNSS headings pass through unchanged and nothing is bridged.

Looks right but is wrong:
  * anchoring only at the moment GNSS is lost: the anchor must follow every
    trusted tick, or a bridge starts from a heading seconds old;
  * the odom yaw used as a heading: it is relative (chassis boot frame), only
    its CHANGE means anything;
  * re-anchoring on a rejected COG: the next prediction would carry the error;
  * the window counted from the start of the gap rather than from the last
    trusted heading: same thing today, but a rejected-COG stretch would then
    extend the bridge on a stale anchor.
"""
from __future__ import annotations  # postponed annotations

import math  # atan2 / radians
from dataclasses import dataclass  # result value
from typing import Optional, Tuple  # plain types

DUAL = "dual_antenna"  # 11 S3.3 GnssHeading.source L1
COG = "cog"  # 11 S3.3 GnssHeading.source L2
BRIDGE = "bridge"  # this module: the IMU-bridged heading


def _wrap(a: float) -> float:
    """Angle wrapped to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))  # robust for any input


def _positive(name: str, v: float) -> float:
    """v as a positive float or ValueError naming the key (CLAUDE.md 3.1)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0.0:
        raise ValueError("%s must be a positive number, got %r" % (name, v))
    return float(v)


@dataclass(frozen=True)
class BridgedHeading:  # one tick's answer
    """The heading the gate / RNS use this tick."""
    heading_rad: Optional[float]  # ENU radians, None when invalid
    valid: bool  # usable this tick
    source: Optional[str]  # dual_antenna | cog | bridge | None


class HeadingBridge:  # one per process
    """Holds the trusted-heading anchor and the previous COG sample."""

    def __init__(self, max_s: float, tol_deg: float, cog_tau_s: float) -> None:
        self._max_s = _positive("nav.heading_bridge_max_s", max_s)  # bridge window
        self._tol = math.radians(_positive("nav.heading_consistency_deg", tol_deg))  # COG check
        self._tau = _positive("nav.cog_blend_tau_s", cog_tau_s)  # COG pull time constant
        self._last_t: Optional[float] = None  # previous step time (the pull is per second)
        self._anchor: Optional[Tuple[float, float, float]] = None  # (heading, odom yaw, t)
        self._last_cog: Optional[Tuple[float, float, float]] = None  # (cog, odom yaw, t)

    def _predict(self, now_s: float, odom_yaw: float) -> Optional[float]:
        """Anchor + yaw change, or None outside the window."""
        a = self._anchor
        if a is None or now_s - a[2] > self._max_s:
            return None  # nothing to bridge from
        return _wrap(a[0] + _wrap(odom_yaw - a[1]))

    def _cog_ok(self, now_s: float, cog: float, odom_yaw: float,
                pred: Optional[float]) -> bool:
        """The COG consistency check (see the module note)."""
        if pred is not None:  # against the bridge
            return abs(_wrap(cog - pred)) <= self._tol
        p = self._last_cog
        if p is None or now_s - p[2] > 1.0:  # a COG pair must be consecutive (20 Hz)
            return False  # first COG after a gap: wait one tick to check it
        return abs(_wrap(_wrap(cog - p[0]) - _wrap(odom_yaw - p[1]))) <= self._tol

    def step(self, now_s: float, gnss_valid: bool, gnss_heading: Optional[float],
             gnss_source: Optional[str], odom_yaw: Optional[float]) -> BridgedHeading:
        """This tick's heading from the GNSS heading and the chassis odom yaw."""
        g = float(gnss_heading) if gnss_valid and gnss_heading is not None else None
        dt = 0.0 if self._last_t is None else max(0.0, now_s - self._last_t)  # since last tick
        self._last_t = now_s
        if odom_yaw is None:  # nothing to check against: pass GNSS through
            self._anchor, self._last_cog = None, None  # an anchor needs a yaw pair
            return BridgedHeading(g, True, gnss_source) if g is not None \
                else BridgedHeading(None, False, None)
        pred = self._predict(now_s, odom_yaw)
        if g is not None and gnss_source == COG:
            trusted = self._cog_ok(now_s, g, odom_yaw, pred)
            self._last_cog = (g, odom_yaw, now_s)  # the next unanchored check pairs with it
            if trusted:
                # a checked COG PULLS the estimate (time constant tau), it
                # does not replace it: replacing let a COG lagging a slow
                # turn walk the anchor 5 -> 12 -> 29 deg off in steps each
                # under the tolerance (replay of t-20261106-007). With no
                # estimate yet the paired COG starts one.
                est = g if pred is None else _wrap(pred + min(1.0, dt / self._tau) * _wrap(g - pred))
                self._anchor = (est, odom_yaw, now_s)
                return BridgedHeading(est, True, COG)
        elif g is not None:  # dual antenna (or any non-COG source rtk_driver calls valid)
            self._last_cog = None  # COG pairs only across consecutive COG ticks
            self._anchor = (g, odom_yaw, now_s)
            return BridgedHeading(g, True, gnss_source)
        else:
            self._last_cog = None  # a gap breaks the COG pairing
        if pred is not None:  # no trusted GNSS heading: bridge
            return BridgedHeading(pred, True, BRIDGE)
        return BridgedHeading(None, False, None)  # nothing: invalid
