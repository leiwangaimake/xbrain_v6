"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: climb.py
Brief: heading-deadlock climb (99 U151/U152): last straight >= 1 m track heading drives the robot until a heading returns

Description:
What problem this solves. When BOTH GNSS headings are unusable -- the dual
antenna baseline is not fixed and COG is blind (11 S3.3: COG needs >= 0.5 m/s
held 0.5 s) -- the speed gate vetoes motion, a stopped robot never brings COG
back, and the route task dies of watchdog_no_progress (measured outdoors
2026-11-06: a patrol stood 55 s and failed). The user ruled (2026-11-06):
when that deadlock cannot be avoided, take the heading of the robot's last
movement of at least 1 m as the direction to climb in; RNS drives the climb
like any navigation (the current navigation top speed, its avoidance); once
COG (or the dual antenna) is valid again the climb ends and the normal
seamless switching of rtk_driver takes over. Only while a route task runs.

How:
  * TrackHeading keeps the bearing of the robot's latest STRAIGHT completed
    segment of at least seg_m (nav.climb_track_seg_m, 1 m) from RTK
    positions, ALL the time -- every movement counts, not only after a
    heading is lost -- paired with the odom yaw at the middle of that
    segment (99 U152). A segment along which the odom yaw turned by more
    than nav.heading_consistency_deg is a turn and is skipped: a legged
    robot on a tight arc moves sideways, its track is not its nose;
  * Climb enters when no heading is valid (neither GNSS nor the U152 IMU
    bridge, path/heading_bridge.py), a route runs, a track pair and the
    chassis odom yaw exist; it answers track bearing + the odom yaw change
    since that segment (so turns RNS makes are seen, and so a turn made
    before the climb began is not lost);
  * it ends on a valid heading, when the route stops, or after max_s
    (nav.climb_max_s, equal to rtk_driver's heading blind timeout); after a
    timeout it does not re-enter until a heading has been valid again.

Boundaries: decides only the heading P1's gate / RNS see; rtk_driver's
levels, events and state/pose are untouched. The climb heading is not
yaw_capable (no absolute alignment off it).

Looks right but is wrong:
  * a track heading only computed after the heading is lost: then a stopped
    robot has none -- it must be kept continuously;
  * the climb heading held constant: the first RNS turn would never be seen
    and the robot would spin in place below COG speed for good;
  * re-entering right after a timeout: an endless climb on a guess;
  * pairing the track bearing with the yaw at climb entry (U151 as first
    written): after a turn in place the pair is off by the whole turn --
    measured 27 degrees at the corner where patrol t-20261106-007 hit a
    wall (99 U152).
"""
from __future__ import annotations  # postponed annotations

import math  # atan2 / hypot
from dataclasses import dataclass  # result value
from typing import Optional, Tuple  # plain types


def _wrap(a: float) -> float:
    """Angle wrapped to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))  # robust for any input


def _positive(name: str, v: float) -> float:
    """v as a positive float or ValueError naming the key (CLAUDE.md 3.1)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0.0:
        raise ValueError("%s must be a positive number, got %r" % (name, v))
    return float(v)


class TrackHeading:  # the last straight >= seg_m movement, paired with its yaw
    """Bearing (ENU rad) of the latest STRAIGHT completed segment of >= seg_m,
    and the odom yaw the body had along it (99 U152)."""

    def __init__(self, seg_m: float, straight_deg: float) -> None:
        self._seg = _positive("nav.climb_track_seg_m", seg_m)  # 1 m by the ruling
        # 99 U152: a segment whose odom yaw changed by more than this is a
        # turn -- its bearing is not where the nose pointed, it is not used
        self._straight = math.radians(_positive("nav.heading_consistency_deg", straight_deg))
        self._anchor: Optional[Tuple[float, float, Optional[float]]] = None  # (x, y, yaw) at segment start
        self.heading: Optional[float] = None  # latest straight bearing, None until one
        self.heading_yaw: Optional[float] = None  # the odom yaw paired with it

    def update(self, xy: Optional[Tuple[float, float]], yaw: Optional[float]) -> None:
        """Feed this tick's local position and odom yaw (None: no fix / no
        yaw; a missing position changes nothing)."""
        if xy is None:
            return  # no position: keep the last pair
        if self._anchor is None or self._anchor[2] is None:
            self._anchor = (xy[0], xy[1], yaw)  # (re)start a segment with a yaw
            return
        dx, dy = xy[0] - self._anchor[0], xy[1] - self._anchor[1]  # since the segment start
        if math.hypot(dx, dy) < self._seg:
            return  # segment not complete yet
        y0 = self._anchor[2]
        self._anchor = (xy[0], xy[1], yaw)  # next segment starts here
        if yaw is None:
            return  # no yaw at the end: cannot pair, keep the last pair
        turn = _wrap(yaw - y0)  # body rotation along the segment
        if abs(turn) > self._straight:
            return  # a turn: its track is not its nose (patrol t-20261106-007)
        self.heading = math.atan2(dy, dx)  # its bearing
        self.heading_yaw = _wrap(y0 + turn / 2.0)  # the yaw at its middle


@dataclass(frozen=True)
class ClimbHeading:  # one tick's answer
    """The climb heading for this tick, or not climbing."""
    heading_rad: Optional[float]  # ENU radians
    climbing: bool  # True: the gate / RNS use heading_rad


class Climb:  # the deadlock escape
    """Enter / hold / leave the climb (see the module note)."""

    def __init__(self, max_s: float) -> None:
        self._max_s = _positive("nav.climb_max_s", max_s)  # 30 s by the ruling
        self._since: Optional[float] = None  # climb start time
        self._expired = False  # timed out: wait for a valid heading

    def step(self, now_s: float, heading_valid: bool, route_running: bool,
             track: TrackHeading, odom_yaw: Optional[float]) -> ClimbHeading:
        """This tick's climb heading (heading_valid: GNSS or the U152 bridge)."""
        if heading_valid or not route_running:  # a heading again, or no route task
            self._since = None  # leave the climb
            if heading_valid:
                self._expired = False  # a valid heading re-arms the climb
            return ClimbHeading(None, False)
        if (self._expired or track.heading is None or track.heading_yaw is None
                or odom_yaw is None):
            self._since = None  # cannot climb
            return ClimbHeading(None, False)
        if self._since is None:  # enter
            self._since = now_s
        elif now_s - self._since > self._max_s:  # too long on a guess
            self._expired = True
            self._since = None
            return ClimbHeading(None, False)
        # 99 U152: the straight segment's bearing plus the yaw change SINCE
        # THAT SEGMENT -- not since the climb began (U151 paired the bearing
        # with the yaw at entry and was 27 degrees off after a turn)
        return ClimbHeading(_wrap(track.heading + _wrap(odom_yaw - track.heading_yaw)), True)
