"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_route_intake.py
Brief: RouteAssembler -- 11 S3.5A parse, chunk assembly, clear, RG-2 cap

Description:
The assembler is the only place a lost or duplicated chunk can be caught
before it becomes a shorter polyline; so the tests drive it with out-of-order
chunks, a seq gap, a cmd_id switch mid-set, a missing chunk block and the RG-2
cap, and check op=clear both returns RouteClear and drops a pending set.
Projection is checked end-to-end (a point one milli-degree north of the origin
lands at +111.32 m north).
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.nav.route_intake import (MAX_POINTS, RouteAssembler,
                                               RouteClear, RouteIntakeError,
                                               RouteSet)
from xbrain.p1_motion.path.local_frame import M_PER_DEG, LocalFrame

pytestmark = pytest.mark.no_device

LAT0, LON0 = 31.2304, 121.4737


def _asm():
    return RouteAssembler(LocalFrame(LAT0, LON0))


def _pt(seq, dlat=0.0, dlon=0.0, r=1.0):
    return {"lat": LAT0 + dlat, "lon": LON0 + dlon, "seq": seq, "arrive_radius_m": r}


def _body(points, *, index=0, total=1, cmd_id="rg-1", rev=7, op=None, **kw):
    b = {"v": 1, "cmd_id": cmd_id, "route_id": "r-east", "route_rev": rev,
         "loop_mode": "oneway", "total_len_m": 100.0, "frame": "wgs84",
         "points": points, "chunk": {"index": index, "total": total}}
    if op is not None:
        b["op"] = op
    b.update(kw)
    return b


def test_single_chunk_set_projects_points_and_keeps_endpoint_radius():
    a = _asm()
    rs = a.accept(_body([_pt(0), _pt(1, dlat=0.001, r=1.0), _pt(2, dlat=0.002, r=0.5)]))
    assert isinstance(rs, RouteSet)
    assert rs.route_id == "r-east" and rs.route_rev == 7 and rs.waypoint_total == 3
    assert rs.points_xy[1][1] == pytest.approx(0.001 * M_PER_DEG, rel=1e-9)
    assert rs.points_xy[1][0] == pytest.approx(0.0, abs=1e-9)
    assert rs.endpoint_arrive_radius_m == 0.5
    assert a.completed == 1 and not a.pending


def test_chunks_assemble_in_index_order_regardless_of_arrival():
    a = _asm()
    assert a.accept(_body([_pt(2), _pt(3)], index=1, total=2)) is None
    assert a.pending
    rs = a.accept(_body([_pt(0), _pt(1)], index=0, total=2))
    assert isinstance(rs, RouteSet) and rs.waypoint_total == 4


def test_seq_gap_is_refused_and_pending_dropped():
    """mutant: drop the seq contiguity check -> a lost chunk becomes a
    shorter polyline -> red."""
    a = _asm()
    a.accept(_body([_pt(0), _pt(1)], index=0, total=2))
    with pytest.raises(RouteIntakeError):
        a.accept(_body([_pt(3), _pt(4)], index=1, total=2))
    assert a.rejected == 1 and not a.pending


def test_new_cmd_id_abandons_pending_set():
    a = _asm()
    a.accept(_body([_pt(0)], index=0, total=2, cmd_id="rg-1"))
    rs = a.accept(_body([_pt(0)], index=0, total=1, cmd_id="rg-2"))
    assert isinstance(rs, RouteSet) and rs.cmd_id == "rg-2"


def test_clear_returns_route_clear_and_drops_pending():
    a = _asm()
    a.accept(_body([_pt(0)], index=0, total=2))
    rc = a.accept({"v": 1, "op": "clear", "cmd_id": "rg-9", "route_id": "r-east",
                   "route_rev": 7, "points": []})
    assert isinstance(rc, RouteClear) and rc.route_rev == 7
    assert not a.pending


@pytest.mark.parametrize("mutate", [
    lambda b: b.update(frame="enu"),
    lambda b: b.update(loop_mode="bounce"),
    lambda b: b.pop("chunk"),
    lambda b: b.update(v=2),
    lambda b: b.update(op="stop"),
    lambda b: b.update(points=[]),
    lambda b: b.update(points=[{"lat": LAT0, "lon": LON0, "seq": 0}]),
    lambda b: b.update(points=[_pt(0, r=0.0)]),
    lambda b: b.update(route_rev=-1),
    lambda b: b.update(chunk={"index": 1, "total": 1}),
])
def test_off_contract_frames_are_refused(mutate):
    a = _asm()
    b = _body([_pt(0)])
    mutate(b)
    with pytest.raises(RouteIntakeError):
        a.accept(b)
    assert a.rejected == 1


def test_rg2_cap_refuses_oversize():
    a = _asm()
    pts = [_pt(i, dlat=i * 1e-6) for i in range(MAX_POINTS + 1)]
    with pytest.raises(RouteIntakeError):
        a.accept(_body(pts))


def test_absent_op_means_set():
    a = _asm()
    rs = a.accept(_body([_pt(0)]))
    assert isinstance(rs, RouteSet)


def test_disagreeing_chunk_meta_is_refused():
    a = _asm()
    a.accept(_body([_pt(0)], index=0, total=2, rev=7))
    with pytest.raises(RouteIntakeError):
        a.accept(_body([_pt(1)], index=1, total=2, rev=8))


def test_rg2_cap_counts_across_chunks():
    """mutant: drop the running-total check -> two under-cap chunks assemble
    into an over-cap polyline -> red."""
    a = _asm()
    a.accept(_body([_pt(i, dlat=i * 1e-6) for i in range(3000)], index=0, total=2))
    with pytest.raises(RouteIntakeError):
        a.accept(_body([_pt(3000 + i, dlat=(3000 + i) * 1e-6) for i in range(2001)],
                       index=1, total=2))


def test_malformed_frame_of_another_push_keeps_the_pending_set():
    """mutant: reset pending on every rejection -> the valid half-route is
    lost to a stray bad frame -> red."""
    a = _asm()
    a.accept(_body([_pt(0)], index=0, total=2, cmd_id="rg-1"))
    with pytest.raises(RouteIntakeError):
        a.accept(_body([_pt(0)], cmd_id="rg-other", frame="enu"))
    assert a.pending
    rs = a.accept(_body([_pt(1)], index=1, total=2, cmd_id="rg-1"))
    assert isinstance(rs, RouteSet) and rs.waypoint_total == 2
    # a bad frame of the SAME push does drop it
    a.accept(_body([_pt(0)], index=0, total=2, cmd_id="rg-2"))
    with pytest.raises(RouteIntakeError):
        a.accept(_body([_pt(9)], index=1, total=2, cmd_id="rg-2"))
    assert not a.pending
