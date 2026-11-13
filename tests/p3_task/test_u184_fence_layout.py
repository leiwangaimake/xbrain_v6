"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u184_fence_layout.py
Brief: 99 U184 -- forbid zones may not overlap, unreachable points / routes are reported

Description:
The user's ruling of 2026-11-13 (99 U184), each rule pinned by a case that
names the mutant it kills:
  * a forbid zone may not overlap a warning zone, a speed-limit zone or another
    forbid zone -- refused at save (teach) and at upsert (cloud), from either
    side, against every live fence, switched on or not;
  * a forbid zone may overlap the activity area, and warning / speed-limit
    zones may overlap each other;
  * waypoints / routes an activity area leaves outside, or a forbid zone
    covers, are REPORTED (ack detail.unreachable) when that fence is saved or
    switched on, and when a waypoint / route is saved against the fences in
    force -- never refused;
  * the robot's own position relative to a fence being switched on.
The voice wording of all of this is pinned in tests/p4_agent/runtime/
test_ack_speech.py; the P3 wiring that turns detail.unreachable into an
event/warn/fence is covered by reading its call sites (no loop harness).
"""
from __future__ import annotations  # postponed annotations

import math  # metres -> degrees of longitude

import aiosqlite  # in-memory dbs
import pytest  # marker, parametrize
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_GEO_INVALID, E_TEACH_GEOMETRY  # expected codes
from xbrain.p3_task.fence.reach import (  # under test
    INSIDE_FORBID,
    OUTSIDE_ACTIVITY_AREA,
    object_reports,
    robot_note,
    unreachable_event,
    unreachable_objects,
)
from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload  # cmd/geo path
from xbrain.p3_task.ingest.geo_commit import commit_fence, commit_route, commit_waypoint  # seeds
from xbrain.p3_task.persistence.schema_geo import FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS  # DDL
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task DDL
from xbrain.p3_task.teach.runtime import TeachRuntime  # under test
from xbrain.p3_task.teach.validate import ring_overlap  # under test

pytestmark = pytest.mark.no_device  # pure logic + in-memory sqlite

_LAT, _LON = 34.6970, 135.5050  # a site
_DLAT = 1.0 / 111320.0  # one metre of latitude
_DLON = _DLAT / math.cos(math.radians(_LAT))  # one metre of longitude


def _sq(x0, y0, side):  # [(lat, lon)] corners, metres east / north of the site
    return [(_LAT + y * _DLAT, _LON + x * _DLON)
            for x, y in ((x0, y0), (x0, y0 + side), (x0 + side, y0 + side), (x0 + side, y0))]


def _pt(x, y):  # one (lat, lon), metres east / north
    return (_LAT + y * _DLAT, _LON + x * _DLON)


_AREA = _sq(0, 0, 100)  # the activity area
_PIT = _sq(20, 20, 20)  # a forbid zone inside it
_PIT_SIDE = _sq(30, 30, 20)  # half over _PIT
_FAR = _sq(60, 60, 20)  # clear of _PIT


async def _open(statements):  # in-memory db with a schema
    conn = await aiosqlite.connect(":memory:")  # fresh
    for stmt in statements:  # schema
        await conn.execute(stmt)  # one DDL
    await conn.commit()  # ready
    return conn  # caller closes


async def _seed(conn, fid, name, role, ring, state="draft"):  # a fence row
    await commit_fence(conn, fence_id=fid, role=role, points=ring, now_ms=1, name=name,  # as teach
                       state=state, created_by="teach",  # writes it
                       speed_limit_mps=0.6 if role == "speed_limit" else None)  # FV-13


def _pose(rt, lat, lon, t):  # one rtk_fixed state/pose sample at mono t
    rt.update_pose({"lat": lat, "lon": lon, "fix_type": "rtk_fixed", "heading_rad": 0.0,
                    "heading_valid": True, "heading_level": 1, "speed_mps": 1.0}, t)


@pytest_asyncio.fixture  # a runtime with healthy caches
async def rt():
    dbs = (await _open(ALL_DDL_STATEMENTS), await _open(GEO_DB_STATEMENTS),  # task, geo
           await _open(FENCE_DB_STATEMENTS))  # and fence
    runtime = TeachRuntime(*dbs, boot_id="b0", sample_hz=1.0, min_dist_m=0.5,  # U132 pair
                           snap_max_age_ms=8000, record_duration_s=10800,  # configured values
                           fence_speed_limit={"default": 0.6, "min": 0.1, "max": 2.0})  # U185
    _pose(runtime, _LAT, _LON, 100.0)  # a fresh fix
    runtime.update_health({"allow_motion": True})  # arming check 4
    runtime.update_robot({"hes": False, "estop_path": "up"})  # checks 4 / 7
    runtime.update_power({"soc_pct": 80.0})  # check 5
    runtime.update_teleop({"sources": [{"device": "gamepad", "stale": False, "alive": True}]})  # check 7
    yield runtime  # to the test
    for c in dbs:  # cleanup
        await c.close()  # close


def _cmd(cmd_id, action, **over):  # a voice-issued cmd/teach
    base = {"cmd_id": cmd_id, "action": action,  # S12A.4
            "issuer": {"src": "p4_agent", "channel": "local_voice"}}  # voice issuer
    base.update(over)  # the action block
    return base  # payload


async def _record(rt, kind, name, pts):  # start, drive the points, exit (saves)
    _pose(rt, _LAT, _LON, 100.0)  # fresh at the start
    ack = await rt.handle(_cmd("c-s-" + name, "start", start={"kind": kind, "name": name}),  # named
                          now_mono_s=100.0, now_ms=1000)  # start
    assert ack["result"] == "accepted", ack  # armed
    for i, (la, lo) in enumerate(pts):  # each point
        _pose(rt, la, lo, 101.0 + i)  # there
        await rt.offer_pose(101.0 + i, 1000 + i)  # sampled
    return await rt.handle(_cmd("c-f-" + name, "finish", session_id=ack["detail"]["session_id"]),  # exit
                           now_mono_s=110.0, now_ms=2000)  # = save


# -- geometry -----------------------------------------------------------------
def test_ring_overlap():
    """Crossing and either-way containment overlap; apart, or only touching
    along an edge (no shared area), does not. MUTATION: edges only (no
    containment test) -> the nested pair reads clear -> red; containment one
    way only -> red."""
    assert ring_overlap(_PIT, _PIT_SIDE)  # edges cross
    assert ring_overlap(_sq(25, 25, 5), _PIT) and ring_overlap(_PIT, _sq(25, 25, 5))  # nested
    assert not ring_overlap(_PIT, _sq(45, 20, 20))  # side by side
    assert not ring_overlap(_PIT, _FAR)  # apart


# -- forbid overlap refused (teach) -------------------------------------------
@pytest.mark.asyncio  # async case
@pytest.mark.parametrize("existing_role,existing_name,new_name", [
    ("warning", "东门报警区域围栏", "北门禁止区域围栏"),  # forbid over a warning zone
    ("speed_limit", "路口限速区域围栏", "北门禁止区域围栏"),  # forbid over a speed zone
    ("forbid", "南门禁止区域围栏", "北门禁止区域围栏"),  # forbid over forbid
    ("forbid", "南门禁止区域围栏", "东门报警区域围栏"),  # a warning zone over a forbid zone
])
async def test_a_forbid_overlap_is_refused_at_save(rt, existing_role, existing_name, new_name):
    """Either side, against a DRAFT fence too. MUTATION: drop the forbid_overlap
    check -> saved -> red; count only active fences -> the draft is missed ->
    red; check from the forbid side only -> the last case saves -> red."""
    await _seed(rt._fence_conn, "f-old", existing_name, existing_role, _PIT)  # a draft
    fin = await _record(rt, "fence", new_name, _PIT_SIDE)  # half over it
    assert (fin["result"], fin["code"]) == ("rejected", E_TEACH_GEOMETRY)  # not saved
    assert {"code": "forbid_overlap", "fences": [existing_name]} in fin["detail"]["issues"]  # named


@pytest.mark.asyncio  # async case
@pytest.mark.parametrize("existing_role,existing_name,new_name", [
    ("allow", "营区活动区域围栏", "北门禁止区域围栏"),  # a forbid zone in the activity area
    ("warning", "东门报警区域围栏", "路口限速区域围栏"),  # warning and speed zones nest
])
async def test_the_overlaps_the_user_allows_are_saved(rt, existing_role, existing_name, new_name):
    """MUTATION: refuse any overlap -> red."""
    await _seed(rt._fence_conn, "f-old", existing_name, existing_role,  # the earlier fence
                _AREA if existing_role == "allow" else _PIT)  # around / under the new one
    fin = await _record(rt, "fence", new_name, _PIT_SIDE)  # overlapping it
    assert fin["result"] == "accepted" and fin["detail"]["saved"] is True, fin  # saved


# -- forbid overlap refused (cloud upsert) ------------------------------------
def _upsert(cmd_id, fid, role, ring, rev=0):  # cmd/geo fence upsert as the cloud sends it
    obj = {"name": fid, "geom": {"role": role, "outer": [{"lat": a, "lon": b} for a, b in ring]}}  # 11 S7.8.3
    if role == "speed_limit":  # FV-13
        obj["geom"]["speed_limit_mps"] = 0.6  # required on a speed zone
    return {"cmd_id": cmd_id, "action": "upsert", "type": "fence", "geo_id": fid,  # S7.9.1
            "origin": "cloud", "base_rev": rev, "obj": obj}  # cloud-made


@pytest.mark.asyncio  # async case
async def test_a_cloud_forbid_overlapping_an_alarm_region_is_refused():
    """MUTATION: no check on the upsert path -> accepted -> red; the fence's
    own row not excluded -> re-sending its geometry clashes with itself -> red."""
    geo, fence = await _open(GEO_DB_STATEMENTS), await _open(FENCE_DB_STATEMENTS)  # dbs
    ctx = GeoContext(geo_conn=geo, fence_conn=fence, task_conn=None)  # as p3 builds it
    try:
        assert (await handle_geo_payload(_upsert("c1", "f-gate", "warning", _PIT), ctx,  # an alarm region
                                         now_ms=1))["result"] == "accepted"  # stored
        bad = await handle_geo_payload(_upsert("c2", "f-pit", "forbid", _PIT_SIDE), ctx, now_ms=2)  # over it
        assert (bad["result"], bad["code"]) == ("rejected", E_GEO_INVALID)  # refused
        assert bad["detail"] == {"reason": "forbid_overlap", "fences": ["f-gate"]}  # names it
        assert (await handle_geo_payload(_upsert("c3", "f-hole", "forbid", _FAR), ctx,  # a clear forbid
                                         now_ms=3))["result"] == "accepted"  # stored
        moved = await handle_geo_payload(_upsert("c4", "f-hole", "forbid", _sq(65, 65, 20), rev=1),
                                         ctx, now_ms=4)  # over its own old ring: fine
        assert moved["result"] == "accepted", moved  # no clash with itself
    finally:
        await geo.close()  # cleanup
        await fence.close()  # cleanup


# -- unreachable objects ------------------------------------------------------
def test_unreachable_objects_and_robot_note():
    """Allow: points / routes outside; forbid: inside; other roles: nothing. A
    route counts when any point is. MUTATION: forbid test inverted -> red;
    route judged by its first point only -> red; warning treated as a
    boundary -> red."""
    wps = [{"name": "1号测试点", "lat": _pt(30, 30)[0], "lon": _pt(30, 30)[1]},  # in the pit
           {"name": "返航终点站", "lat": _pt(150, 10)[0], "lon": _pt(150, 10)[1]}]  # outside the area
    routes = [{"name": "1号路径", "points": [{"lat": a, "lon": b}  # starts clear, crosses the pit
                                           for a, b in (_pt(5, 5), _pt(30, 25), _pt(90, 90))]}]
    assert unreachable_objects("allow", _AREA, wps, routes) == ["返航终点站"]  # left outside
    assert unreachable_objects("forbid", _PIT, wps, routes) == ["1号测试点", "1号路径"]  # covered
    assert unreachable_objects("warning", _PIT, wps, routes) == []  # not a boundary
    assert robot_note("allow", _AREA, *_pt(150, 10)) == OUTSIDE_ACTIVITY_AREA  # robot outside
    assert robot_note("forbid", _PIT, *_pt(30, 30)) == INSIDE_FORBID  # robot inside the pit
    assert robot_note("forbid", _PIT, None, None) is None  # unknown position: nothing
    assert unreachable_event({"fence": "北门禁止区域围栏", "role": "forbid",
                              "reason": INSIDE_FORBID, "objects": ["1号测试点"]}, "enable") == {
        "kind": "objects_unreachable", "trigger": "enable", "poly_name": "北门禁止区域围栏",
        "role": "forbid", "reason": INSIDE_FORBID, "objects": ["1号测试点"]}


@pytest.mark.asyncio  # async case
async def test_saving_a_forbid_zone_reports_what_it_covers(rt):
    """Told at save, while the operator is there. MUTATION: drop fence_reports
    in _commit -> no unreachable -> red."""
    await commit_waypoint(rt._geo_conn, geo_id="w-1", name="1号测试点", wtype="poi",  # a point
                          rtk_lat=_pt(30, 30)[0], rtk_lon=_pt(30, 30)[1], now_ms=1)  # in the pit
    await rt._geo_conn.commit()  # visible
    fin = await _record(rt, "fence", "北门禁止区域围栏", _PIT)  # a forbid zone over it
    assert fin["detail"]["saved"] is True  # saved (a report, not a refusal)
    assert fin["detail"]["unreachable"] == [{"fence": "北门禁止区域围栏", "role": "forbid",  # told
                                             "reason": INSIDE_FORBID, "objects": ["1号测试点"]}]


@pytest.mark.asyncio  # async case
async def test_a_route_saved_through_a_forbid_zone_in_force_is_reported(rt):
    """Against the fences IN FORCE (a draft forbid zone blocks nothing yet).
    MUTATION: drop object_reports for routes -> red; count drafts -> the
    second save reports too -> red."""
    await _seed(rt._fence_conn, "f-pit", "北门禁止区域围栏", "forbid", _PIT, state="active")  # in force
    await _seed(rt._fence_conn, "f-far", "西门禁止区域围栏", "forbid", _FAR)  # draft
    fin = await _record(rt, "route", "1号路径", [_pt(5, 5), _pt(15, 15), _pt(30, 30), _pt(50, 30)])  # through it
    assert fin["detail"]["saved"] is True  # saved
    assert fin["detail"]["unreachable"] == [{"fence": "北门禁止区域围栏", "role": "forbid",  # told
                                             "reason": INSIDE_FORBID, "objects": ["1号路径"]}]
    fin = await _record(rt, "route", "2号路径", [_pt(65, 65), _pt(70, 70), _pt(75, 75)])  # through the draft
    assert "unreachable" not in fin["detail"]  # only the draft one covers it


@pytest.mark.asyncio  # async case
async def test_a_point_outside_the_activity_area_in_force_is_reported():
    """The waypoint path (mark_once) calls object_reports with its one point.
    MUTATION: allow test inverted in _blocked -> red."""
    fence = await _open(FENCE_DB_STATEMENTS)  # fence.db
    try:
        await _seed(fence, "f-a", "营区活动区域围栏", "allow", _AREA, state="active")  # in force
        assert await object_reports(fence, name="3号测试点", points=[_pt(150, 10)]) == [  # outside
            {"fence": "营区活动区域围栏", "role": "allow", "reason": OUTSIDE_ACTIVITY_AREA,
             "objects": ["3号测试点"]}]
        assert await object_reports(fence, name="4号测试点", points=[_pt(50, 50)]) == []  # inside: nothing
    finally:
        await fence.close()  # cleanup


@pytest.mark.asyncio  # async case
async def test_switching_an_activity_area_on_reports_what_it_leaves_out():
    """The enable ack carries the report (the wiring raises the event and adds
    the robot note). MUTATION: drop fence_activation_notes from set_state ->
    no unreachable -> red."""
    geo, fence = await _open(GEO_DB_STATEMENTS), await _open(FENCE_DB_STATEMENTS)  # dbs
    ctx = GeoContext(geo_conn=geo, fence_conn=fence, task_conn=None)  # as p3 builds it
    try:
        await commit_route(geo, route_id="r-1", name="1号路径",  # a route that leaves
                           path_points=[_pt(10, 10), _pt(120, 10)], now_ms=1,  # the area at 100 m
                           state="active", created_by="teach")  # live
        await geo.commit()  # visible
        await _seed(fence, "f-a", "营区活动区域围栏", "allow", _AREA)  # a draft area
        on = await handle_geo_payload({"cmd_id": "c-on", "action": "set_state", "type": "fence",  # F15
                                       "geo_id": "f-a", "origin": "cloud", "base_rev": 1,
                                       "obj": {"state": "active"}}, ctx, now_ms=5)  # switched on
        assert on["result"] == "accepted"  # never refused for it
        assert on["detail"]["unreachable"] == [{"fence": "营区活动区域围栏", "role": "allow",  # told
                                                "reason": OUTSIDE_ACTIVITY_AREA,
                                                "objects": ["1号路径"]}]
    finally:
        await geo.close()  # cleanup
        await fence.close()  # cleanup
