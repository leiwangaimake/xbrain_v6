"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: route_intake.py
Brief: cmd/motion/route (11 S3.5A RouteGeometry) -> RouteSet / RouteClear in site metres

Description:
P1-11: p3_task pushes the whole route geometry (P1 never reads geo.db --
11 S3.5A "P3 主动推送"), possibly in chunks, and P1 swaps the RNS mission
pointer only once every chunk is in (RG-1 parse off the control thread, RG-3
old geometry stays valid until the swap). This module is the parse + assemble
half; the swap is the adapter's (sources/rns_avoid.py) and the wiring's.

What a message must look like (11 S3.5A, verbatim fields): v == 1, op set|clear
(absent op == set, the contract's own default), cmd_id, route_id, route_rev,
loop_mode, total_len_m, frame == "wgs84", points[] of {lat, lon, seq,
arrive_radius_m} (>= 1 point since v2.0; a single point is a goto -- 20
RNS-N-1), chunk {index, total}. Points are projected through the site
LocalFrame here, so downstream sees metres only.

Assembly. Chunks of one cmd_id are collected by chunk.index; a chunk from a
different cmd_id abandons the pending set (a new push supersedes an unfinished
one; RG-3 keeps the LIVE mission untouched meanwhile). The set completes when
all `total` indices are present; seq must then run 0..n-1 without gaps (a lost
chunk shows as a seq gap even if chunk.total was mis-stated). RG-2: more than
MAX_POINTS points is refused, not grown into. A re-sent chunk index replaces
the earlier copy (Q3 reliable delivery may repeat a frame).

op == "clear" (20 S9.0.3 cancel): points may be empty and chunk may be absent;
the result is a RouteClear the wiring turns into rns.cancel() -- IDLE, audit
"cancelled", NO failure report (12 S4.2c.4).

arrive_radius_m is per point on the wire (11 S3.5A: gate vs alley differ) but
RNS judges arrival at the ENDPOINT only (20 A-RT-2), so RouteSet carries every
radius for observability and exposes the LAST one as the mission's arrival
radius.

What it does NOT do: no route_rev cross-check against BehaviorCommand (that is
the behaviour entry, not built here), no loop handling (loop_mode is carried;
the single-pass follow is what RNS does this phase), no ack message -- 15 S2.4.4:
there is no cmd/motion/route/ack; P3 reads path_progress (RA-1) and
event/{warn,fault}/motion (RA-2) instead.

Trap: a producer that omits chunk on a one-chunk set. The table marks chunk
required; accepting its absence would also accept a two-chunk set whose second
chunk never comes, so it is refused (E_GEO_INCOMPLETE semantics), except on
clear where there is nothing to assemble.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple, Union

from xbrain.p1_motion.path.local_frame import LocalFrame, LocalFrameError

#: RG-2 preallocation cap (11 S3.5A / S14.2): refuse above, never grow.
MAX_POINTS = 5000
#: 11 S3.5B dir_sign note: pingpong may run -1; oneway / closed are +1.
LOOP_MODES = ("oneway", "closed", "pingpong")
OP_SET = "set"
OP_CLEAR = "clear"
FRAME_WGS84 = "wgs84"


class RouteIntakeError(ValueError):
    """Off-contract RouteGeometry; the message is dropped and audited."""


@dataclass(frozen=True)
class RouteSet:
    """A complete route in site metres, ready for the adapter."""
    cmd_id: str
    route_id: str
    route_rev: int
    loop_mode: str
    total_len_m: float
    points_xy: Tuple[Tuple[float, float], ...]
    arrive_radius_m: Tuple[float, ...]     # per point, wire order

    @property
    def endpoint_arrive_radius_m(self) -> float:
        """20 A-RT-2: arrival is judged at the endpoint only."""
        return self.arrive_radius_m[-1]

    @property
    def waypoint_total(self) -> int:
        return len(self.points_xy)


@dataclass(frozen=True)
class RouteClear:
    """op == clear: cancel the navigation mission (no failure report)."""
    cmd_id: str
    route_id: Optional[str]
    route_rev: Optional[int]


def _str(body: Dict[str, Any], key: str) -> str:
    v = body.get(key)
    if not isinstance(v, str) or not v:
        raise RouteIntakeError("%s must be a non-empty string, got %r" % (key, v))
    return v


def _int(body: Dict[str, Any], key: str) -> int:
    v = body.get(key)
    if isinstance(v, bool) or not isinstance(v, int) or v < 0:
        raise RouteIntakeError("%s must be a non-negative int, got %r" % (key, v))
    return v


def _num(body: Dict[str, Any], key: str, *, positive: bool = False) -> float:
    v = body.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise RouteIntakeError("%s must be a number, got %r" % (key, v))
    f = float(v)
    if f != f or (positive and f <= 0.0) or (not positive and f < 0.0):
        raise RouteIntakeError("%s out of range: %r" % (key, v))
    return f


def _parse_point(raw: Any, idx: int) -> Tuple[float, float, int, float]:
    """One points[] entry -> (lat, lon, seq, arrive_radius_m); every field
    required (11 S3.5A: arrive_radius_m is per point and mandatory)."""
    if not isinstance(raw, dict):
        raise RouteIntakeError("points[%d] is not an object" % idx)
    lat = _num(raw, "lat")
    lon = _num(raw, "lon")
    seq = _int(raw, "seq")
    radius = _num(raw, "arrive_radius_m", positive=True)
    return lat, lon, seq, radius


class RouteAssembler:
    """Chunk collector for ONE producer stream. accept() returns None while a
    set is incomplete, a RouteSet on completion, a RouteClear on op=clear."""

    def __init__(self, frame: LocalFrame) -> None:
        self._frame = frame
        self._pending_cmd: Optional[str] = None
        self._pending_meta: Optional[Dict[str, Any]] = None
        self._pending_total = 0
        self._chunks: Dict[int, List[Tuple[float, float, int, float]]] = {}
        self.completed = 0
        self.rejected = 0

    def _reset_pending(self) -> None:
        self._pending_cmd = None
        self._pending_meta = None
        self._pending_total = 0
        self._chunks = {}

    @property
    def pending(self) -> bool:
        return self._pending_cmd is not None

    def accept(self, body: Any) -> Optional[Union[RouteSet, RouteClear]]:
        """Parse one frame. Raises RouteIntakeError / LocalFrameError (counted)
        on an off-contract frame; the pending set is dropped on a defect in
        one of its own chunks (RG-3: the LIVE mission is untouched either
        way)."""
        try:
            return self._accept(body)
        except (RouteIntakeError, LocalFrameError):
            self.rejected += 1
            # drop the pending set only when the bad frame belongs to it (or is
            # too broken to say): a stray malformed frame from another push
            # must not lose a half-assembled valid route.
            bad_cmd = body.get("cmd_id") if isinstance(body, dict) else None
            if self._pending_cmd is None or bad_cmd is None or bad_cmd == self._pending_cmd:
                self._reset_pending()
            raise

    def _accept(self, body: Any) -> Optional[Union[RouteSet, RouteClear]]:
        if not isinstance(body, dict):
            raise RouteIntakeError("route body is not an object")
        v = body.get("v")
        if isinstance(v, bool) or v != 1:
            raise RouteIntakeError("v must be 1, got %r" % (v,))
        op = body.get("op", OP_SET)
        if op not in (OP_SET, OP_CLEAR):
            raise RouteIntakeError("op %r not in (set, clear)" % (op,))
        cmd_id = _str(body, "cmd_id")
        if op == OP_CLEAR:
            # cancel supersedes any half-assembled set too: whatever was
            # pending is no longer wanted.
            self._reset_pending()
            rid = body.get("route_id")
            rev = body.get("route_rev")
            return RouteClear(
                cmd_id,
                rid if isinstance(rid, str) else None,
                rev if isinstance(rev, int) and not isinstance(rev, bool) else None)
        meta = {
            "route_id": _str(body, "route_id"),
            "route_rev": _int(body, "route_rev"),
            "loop_mode": _str(body, "loop_mode"),
            "total_len_m": _num(body, "total_len_m"),
        }
        if meta["loop_mode"] not in LOOP_MODES:
            raise RouteIntakeError(
                "loop_mode %r not in %s" % (meta["loop_mode"], list(LOOP_MODES)))
        if _str(body, "frame") != FRAME_WGS84:
            raise RouteIntakeError("frame must be %r" % FRAME_WGS84)
        chunk = body.get("chunk")
        if not isinstance(chunk, dict):
            raise RouteIntakeError("chunk {index, total} required")
        total = _int(chunk, "total")
        index = _int(chunk, "index")
        if total < 1 or index >= total:
            raise RouteIntakeError(
                "chunk index/total invalid: %d/%d" % (index, total))
        raw_points = body.get("points")
        if not isinstance(raw_points, list):
            raise RouteIntakeError("points must be an array")
        if len(raw_points) > MAX_POINTS:
            raise RouteIntakeError("chunk carries %d points > %d (RG-2)"
                                   % (len(raw_points), MAX_POINTS))
        pts = [_parse_point(p, i) for i, p in enumerate(raw_points)]
        # --- assembly ---
        if self._pending_cmd != cmd_id:
            self._reset_pending()
            self._pending_cmd = cmd_id
            self._pending_meta = meta
            self._pending_total = total
        elif total != self._pending_total or meta != self._pending_meta:
            raise RouteIntakeError(
                "chunk of cmd_id %s disagrees with its earlier chunks" % cmd_id)
        self._chunks[index] = pts
        n = sum(len(c) for c in self._chunks.values())
        if n > MAX_POINTS:
            raise RouteIntakeError("route exceeds %d points (RG-2)" % MAX_POINTS)
        if len(self._chunks) < total:
            return None
        ordered: List[Tuple[float, float, int, float]] = []
        for i in range(total):
            ordered.extend(self._chunks[i])
        if not ordered:
            raise RouteIntakeError("op=set needs >= 1 point (11 S3.5A v2.0)")
        seqs = [p[2] for p in ordered]
        if seqs != list(range(len(ordered))):
            raise RouteIntakeError("points seq not contiguous 0..n-1: %s"
                                   % seqs[:8])
        # the LocalFrame may raise on an out-of-range lat/lon: counted as a
        # rejected frame by accept(), pending dropped, live mission kept.
        xy = tuple(self._frame.to_xy(p[0], p[1]) for p in ordered)
        radii = tuple(p[3] for p in ordered)
        assert self._pending_meta is not None
        rs = RouteSet(cmd_id=cmd_id,
                      route_id=self._pending_meta["route_id"],
                      route_rev=self._pending_meta["route_rev"],
                      loop_mode=self._pending_meta["loop_mode"],
                      total_len_m=self._pending_meta["total_len_m"],
                      points_xy=xy, arrive_radius_m=radii)
        self._reset_pending()
        self.completed += 1
        return rs
