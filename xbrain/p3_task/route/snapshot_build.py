"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: snapshot_build.py
Brief: routes row -> 15 S9.3A task_route_snapshot (the thing pushed to P1)

Description:
11 S7.12.1 R1: "运行中的任务跑的是快照, 不是活对象". This builds that
snapshot from the live geo.db row at dispatch, and 11 S3.5A reason 3 is why
there is only one object: the snapshot IS what gets pushed on
cmd/motion/route, so a second representation would be a second truth.

Two geometry modes, because `routes` carries them XOR (15 S9.3):
  * mode A, waypoint_ids -- named anchors. Each anchor brings its OWN
    arrival_radius (15 S9.3.1 G-2: a gate and an alley cannot share an
    arrival test).
  * mode B, path_points -- a recorded dense polyline of [lat, lon] with no
    per-point radius. 15 S2.4.3 sends that case to "common.recording 侧的
    全局缺省" and marks it 待 T7 / NAV-12.

*** MODE B CANNOT BE PUSHED TODAY, AND THAT IS THE DESIGNED BEHAVIOUR.
configs/common.yaml's `recording` block has five keys (min_dist_m,
session_timeout_s, sample_hz, max_fences, fence_close_tol_m) and ALL FIVE are
null; there is no arrive-radius key there at all, and 15 S2.4.3 marks its
value undecided. CLAUDE.md 3.1 and IRON RULE 3 both forbid inventing one, and
3.1 names the specific trap: a plausible-looking default is indistinguishable
from a calibrated value once it is in the code. So build_snapshot REFUSES a
mode-B route unless the caller injects a radius, and the refusal NAMES the
config key that is missing. A robot that will not start a recorded patrol and
says which key is empty is the correct failure; one that arrives 1 m short or
2 m long at every point, silently, is not.

What this module does NOT do: it does not read the database (the caller
passes rows in, which is what makes every rule here testable without one),
does not write the snapshot (SnapshotDAO), and does not push (route/push.py).
Arc length uses the same spherical model as geo_commit -- total_len_m is a
replay/progress length, not a survey figure, and the two MUST agree or
S7.3A's alpha lands on a different point than the one the route was
committed with.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from xbrain.p3_task.route.push import MAX_POINTS, RoutePoint

#: Same constant and same model as ingest/geo_commit.py. Duplicated as a
#: value rather than imported to avoid a route -> ingest dependency, but it
#: must not drift: geo_commit computes routes.total_len_m and this computes
#: the snapshot's, and 15 S9.5 says route_total_m IS that value.
_EARTH_R_M = 6371000.0


class SnapshotBuildError(ValueError):
    """The routes row cannot become a snapshot. Always names what is missing
    -- 'cannot build snapshot' alone sends the reader to the wrong file."""


def _haversine_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    la1, lo1, la2, lo2 = (math.radians(a[0]), math.radians(a[1]),
                          math.radians(b[0]), math.radians(b[1]))
    h = (math.sin((la2 - la1) / 2.0) ** 2
         + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2.0) ** 2)
    return 2.0 * _EARTH_R_M * math.asin(min(1.0, math.sqrt(h)))


def cumulative_arclen(points: Sequence[RoutePoint]) -> List[float]:
    """15 S9.3A arclen_json: [0, l1, l2, ...], one entry per point.

    Precomputed and stored because S7.3A runs it on every remap and a remap
    happens while a task is suspended -- recomputing 5000 haversines there is
    work done inside the resume path for no reason. SN-2 requires it to be
    written in the SAME transaction as points_json: if the two disagree, step
    1 of the remap computes a wrong L0 AND NOTHING REPORTS AN ERROR.
    """
    out = [0.0]
    for i in range(1, len(points)):
        prev, cur = points[i - 1], points[i]
        out.append(out[-1] + _haversine_m((prev.lat, prev.lon),
                                          (cur.lat, cur.lon)))
    return out


@dataclass(frozen=True)
class RouteSnapshot:
    """One 15 S9.3A row, ready to insert and to push."""
    task_id: str
    route_id: str
    rev: int
    loop_mode: str
    points: Tuple[RoutePoint, ...]
    total_len_m: float
    arclen: Tuple[float, ...]

    @property
    def point_count(self) -> int:
        return len(self.points)

    def points_json(self) -> str:
        return json.dumps(
            [{"lat": p.lat, "lon": p.lon, "seq": i,
              "arrive_radius_m": p.arrive_radius_m}
             for i, p in enumerate(self.points)], separators=(",", ":"))

    def arclen_json(self) -> str:
        return json.dumps(list(self.arclen), separators=(",", ":"))


def _parse_json_list(raw: Any, field: str) -> List[Any]:
    if raw is None:
        return []
    try:
        val = json.loads(raw) if isinstance(raw, str) else raw
    except ValueError as exc:
        raise SnapshotBuildError("routes.%s is not valid JSON: %s"
                                 % (field, exc)) from exc
    if not isinstance(val, list):
        raise SnapshotBuildError("routes.%s must be a JSON array, got %s"
                                 % (field, type(val).__name__))
    return val


def build_snapshot(*, task_id: str, route_id: str, rev: int, loop_mode: str,
                   path_points: Any = None, waypoint_ids: Any = None,
                   anchor_lookup: Optional[Dict[str, Tuple[float, float, float]]] = None,
                   recorded_arrive_radius_m: Optional[float] = None,
                   total_len_m: Optional[float] = None) -> RouteSnapshot:
    """Turn one `routes` row into a snapshot.

    anchor_lookup maps a waypoint geo_id to (rtk_lat, rtk_lon,
    arrival_radius); the caller reads it from geo.db so this stays pure.

    total_len_m: if the routes row carries one it is TRUSTED and reused, so
    the snapshot's T0 is byte-identical to what geo_commit computed and what
    the operator saw. If the column is NULL the length is recomputed here
    rather than refused -- the column is nullable in the DDL and old rows
    predate it. Either way the value written is the one pushed.
    """
    if loop_mode not in ("oneway", "pingpong", "closed"):
        raise SnapshotBuildError("routes.loop_mode %r outside the 15 S9.3 "
                                 "closed set" % (loop_mode,))
    pp = _parse_json_list(path_points, "path_points")
    wi = _parse_json_list(waypoint_ids, "waypoint_ids")
    if bool(pp) == bool(wi):
        # Mirrors the routes XOR CHECK. Stated here too because the message
        # the CHECK gives ("CHECK constraint failed") does not say which row.
        raise SnapshotBuildError(
            "route %r must carry exactly one of path_points / waypoint_ids "
            "(got %d / %d)" % (route_id, len(pp), len(wi)))

    points: List[RoutePoint] = []
    if wi:
        # Mode A. A missing anchor is refused, never skipped: a route whose
        # geometry silently shortens when somebody tidies a keypoint off the
        # map is the failure 15 S7.6 GC-5 and geo_write._fill_anchor_length
        # both already guard against.
        lookup = anchor_lookup or {}
        for wid in wi:
            got = lookup.get(str(wid))
            if got is None:
                raise SnapshotBuildError(
                    "route %r anchor %r is not in geo.db; refusing to push a "
                    "route that is missing a point" % (route_id, wid))
            lat, lon, radius = got
            points.append(RoutePoint(float(lat), float(lon), float(radius)))
    else:
        # Mode B. See the header: no per-point radius exists on the wire-less
        # side and 15 S2.4.3's source for it is undecided.
        if recorded_arrive_radius_m is None:
            raise SnapshotBuildError(
                "route %r is a recorded polyline (mode B) and needs a global "
                "arrive_radius_m, which is NOT configured: "
                "common.recording has no arrive-radius key and 15 S2.4.3 "
                "marks its value undecided (NAV-12, T7). Land that key before "
                "pushing a recorded route; a default invented here would be "
                "indistinguishable from a calibrated one (CLAUDE.md 3.1)"
                % (route_id,))
        radius = float(recorded_arrive_radius_m)
        if radius <= 0.0:
            raise SnapshotBuildError(
                "arrive_radius_m must be positive, got %r -- a zero radius is "
                "an arrival test that never passes" % (recorded_arrive_radius_m,))
        for item in pp:
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                raise SnapshotBuildError(
                    "routes.path_points entries must be [lat, lon], got %r"
                    % (item,))
            points.append(RoutePoint(float(item[0]), float(item[1]), radius))

    if not points:
        raise SnapshotBuildError("route %r resolved to zero points" % route_id)
    if len(points) > MAX_POINTS:
        raise SnapshotBuildError(
            "route %r has %d points, over the 11 S3.5A RG-2 cap of %d"
            % (route_id, len(points), MAX_POINTS))
    arclen = cumulative_arclen(points)
    length = float(total_len_m) if total_len_m else arclen[-1]
    if length <= 0.0:
        raise SnapshotBuildError(
            "route %r has zero length; it is the remap's T0 (11 S7.12.3 "
            "step 1) and a zero divides in the alpha" % route_id)
    return RouteSnapshot(task_id=task_id, route_id=route_id, rev=rev,
                         loop_mode=loop_mode, points=tuple(points),
                         total_len_m=length, arclen=tuple(arclen))
