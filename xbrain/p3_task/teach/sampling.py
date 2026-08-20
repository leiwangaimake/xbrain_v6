"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: sampling.py
Brief: 11 S12A.6 sampling rule -- 1 Hz timer + 0.5 m dedup, in WGS84

Description:
The U42 rule, verbatim: fire once a second; drop the sample if its fix quality
is below require_fix (counting it as dropped_by_quality); drop it if it is
closer than dedup_min_dist_m to the last KEPT point; otherwise keep it.

*** This module works in WGS84 throughout, and that is the point of it.

The previous implementation (lifecycle/teach.py, removed with this batch) held
samples as ENU x_m / y_m and de-duplicated with a planar hypot. After the geo
model moved to WGS84 (2026-08-20), commit_route expects [(lat, lon)] -- so the
recording path would have had to project between the two, and a missed
projection puts ENU metres into a latitude column, which is a valid float about
a hundred kilometres from anywhere. Sampling in the frame the pose already
publishes (state/pose carries lat/lon directly) removes that conversion
entirely: there is no ENU anywhere in the recording path to get wrong.

The distance test therefore uses the haversine and not hypot. At camp scale the
two agree to well under a millimetre, so this is not about accuracy -- it is
about the units in the buffer being the units in the database.

The 0.5 m threshold exists to stop a stationary robot from stacking hundreds of
identical points; without it, an operator who parks for two minutes mid-record
adds 120 coincident vertices, and commit_route then computes a length over a
path that stands still.

Boundaries: no clock (the caller passes the monotonic time of each sample), no
storage (S12A.6.1 requires the buffer to live in task.db's memory table, which
is the runtime's job), no session state (that is session.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

_EARTH_R_M = 6371000.0

#: 11 S3.2.1 fix quality, best first. A sample is kept when its fix is at least
#: as good as require_fix, so the ORDER is what the comparison means -- a set
#: would only answer "is this a known fix type".
FIX_RANK = ("rtk_fixed", "rtk_float", "dgps", "single")


@dataclass(frozen=True)
class PoseSample:
    """One state/pose reading offered to the recorder. lat/lon are WGS84
    degrees; mono_s is CLOCK_MONOTONIC seconds (CLK-C1 -- the wall clock steps
    at RTK first lock, which is exactly when a recording tends to start)."""
    lat: float
    lon: float
    mono_s: float
    fix_type: Optional[str] = None
    alt: Optional[float] = None
    heading_rad: Optional[float] = None
    manual: bool = False            # a mark point (F05): exempt from both gates


def haversine_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Great-circle distance in metres between two (lat, lon) degree pairs."""
    la1, lo1, la2, lo2 = (math.radians(a[0]), math.radians(a[1]),
                          math.radians(b[0]), math.radians(b[1]))
    h = (math.sin((la2 - la1) / 2.0) ** 2
         + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2.0) ** 2)
    return 2.0 * _EARTH_R_M * math.asin(min(1.0, math.sqrt(h)))


def fix_is_good_enough(fix_type: Optional[str], require: str) -> bool:
    """Is this fix at least as good as `require` (FIX_RANK order)?

    An unknown fix_type answers False rather than raising: the sample simply
    does not enter the route, which is the same outcome as a bad fix and keeps
    a firmware that invents a new fix name from stopping a recording. The
    unknown value is still visible -- it lands in dropped_by_quality.
    """
    if fix_type is None:
        return False
    try:
        return FIX_RANK.index(fix_type) <= FIX_RANK.index(require)
    except ValueError:
        return False


@dataclass
class Recorder:
    """Applies the S12A.6 rule to a stream of pose samples.

    Holds only what the rule needs: the last KEPT point (for the distance gate)
    and the last ACCEPTED time (for the 1 Hz gate). Counters mirror the
    TeachState stats block so the runtime can publish without a second tally.
    """
    dedup_min_dist_m: float
    sample_hz: float
    require_fix: str
    max_points: int
    points: List[PoseSample] = None            # type: ignore[assignment]
    dropped_by_quality: int = 0
    dropped_by_distance: int = 0
    manual_count: int = 0
    length_m: float = 0.0
    _last_kept_mono: float = -1e9

    def __post_init__(self) -> None:
        if self.points is None:
            self.points = []

    @property
    def point_count(self) -> int:
        return len(self.points)

    def offer(self, sample: PoseSample) -> Tuple[bool, str]:
        """Offer one sample. Returns (kept, reason).

        reason names the gate that rejected it -- quality / interval /
        distance / full -- because "the point did not appear" is the operator's
        symptom and each of those four needs a different response from them.
        """
        if len(self.points) >= self.max_points:
            return False, "full"
        # A mark point (F05) bypasses BOTH the interval and the distance gate:
        # the operator is saying "this corner matters", and the whole value of
        # F05 is that it survives simplification. Quality is still enforced --
        # a forced point with a bad fix is a wrong point, not a valuable one.
        if not fix_is_good_enough(sample.fix_type, self.require_fix):
            self.dropped_by_quality += 1
            return False, "quality"
        if not sample.manual:
            min_interval_s = 1.0 / self.sample_hz if self.sample_hz > 0 else 1.0
            # 1e-6 of slack: a timer firing at exactly the period would
            # otherwise alternate between kept and dropped on float rounding.
            if sample.mono_s - self._last_kept_mono < min_interval_s - 1e-6:
                return False, "interval"
            if self.points:
                last = self.points[-1]
                if (haversine_m((last.lat, last.lon),
                                (sample.lat, sample.lon))
                        < self.dedup_min_dist_m):
                    self.dropped_by_distance += 1
                    return False, "distance"
        if self.points:
            self.length_m += haversine_m(
                (self.points[-1].lat, self.points[-1].lon),
                (sample.lat, sample.lon))
        self.points.append(sample)
        self._last_kept_mono = sample.mono_s
        if sample.manual:
            self.manual_count += 1
        return True, "kept"

    def undo(self, count: int) -> int:
        """Remove the last `count` points (S12A.4: 1..10, never below zero).

        Returns how many were actually removed. length_m is recomputed rather
        than decremented: subtracting the removed segments accumulates float
        error across a long session, and the length is quoted back to the
        operator ("320 metres") at save time.
        """
        count = max(1, min(10, int(count)))
        removed = 0
        while removed < count and self.points:
            popped = self.points.pop()
            if popped.manual and self.manual_count > 0:
                self.manual_count -= 1
            removed += 1
        self.length_m = polyline_length_m(
            [(p.lat, p.lon) for p in self.points])
        # The interval gate is re-armed: after undoing a point the operator
        # expects the next one to be takeable immediately, not a second later.
        self._last_kept_mono = -1e9
        return removed

    def latlon_points(self) -> List[Tuple[float, float]]:
        """The buffer as the (lat, lon) list commit_route / commit_fence take."""
        return [(p.lat, p.lon) for p in self.points]


def polyline_length_m(points: Sequence[Tuple[float, float]]) -> float:
    """Accumulated great-circle length of an ordered (lat, lon) list."""
    return sum(haversine_m(points[i], points[i + 1])
               for i in range(len(points) - 1))
