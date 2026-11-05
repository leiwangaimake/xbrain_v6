"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_route_plan.py
Brief: 99 U127 -- route-aware goto (B01) and patrol start (B02) plans, and their push wiring

Description:
The user's rulings of 2026-11-05 (99 U127), each pinned by a case that names
the mutant it kills:
  goto: a waypoint within membership_m of a route polyline belongs to it;
    the robot enters at the nearest route point, follows the route the
    shorter way (either direction, through the seam on a ring), and leaves
    at the point nearest the target; among several member routes the
    cheapest total length wins; straight when the robot is no farther from
    the target than from that route; straight when the target belongs to no
    route.
  patrol: an open route is entered at the nearest point, run to the farther
    end first and then back to the other end (the return dropped when the
    near side is within the arrival radius); a ring is run once in the
    recorded direction back to the entry point.
  wiring: push_goto_for_task follows the route when route_membership_m is
    set; push_route_for_task starts the patrol polyline at the robot.
Geometry is built in a local metric plane (the module's own _Plane) so every
expected point reads in metres.
"""
from __future__ import annotations  # postponed annotations

import json  # wire frames

import aiosqlite  # in-memory dbs for the wiring half
import pytest  # parametrize, marker
import pytest_asyncio  # async fixture

from xbrain.p3_task.dao.simple_daos import PatrolProgressDAO, SnapshotDAO  # push dependencies
from xbrain.p3_task.persistence.schema_geo import GEO_DB_STATEMENTS  # geo.db DDL
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task.db DDL
from xbrain.p3_task.route.push import RouteAckWindow, RoutePushTrigger  # push dependencies
from xbrain.p3_task.route.route_plan import RouteGeom, _Plane, plan_goto, plan_patrol  # under test
from xbrain.p3_task.runtime.route_push_runtime import push_goto_for_task, push_route_for_task  # wiring under test

pytestmark = pytest.mark.no_device  # pure geometry + in-memory sqlite

P = _Plane((31.0, 121.0))  # a fixed local plane, metres


def ll(x, y):  # helper
    """local metres -> (lat, lon)"""
    return P.ll((x, y))  # inverse projection


def xy(p):  # helper
    """(lat, lon) -> local metres, rounded to 0.1 m for comparison"""
    return tuple(round(v, 1) for v in P.xy(p))  # forward projection


LINE = RouteGeom("r-a", "A", tuple(ll(x, 0) for x in range(0, 201, 10)), False)  # open, 0..200 m east
RING = RouteGeom("r-c", "C", (ll(0, 0), ll(100, 0), ll(100, 100), ll(0, 100)), True)  # closed 100 m square


# -- goto --------------------------------------------------------------------
def test_goto_follows_the_route_its_target_belongs_to():  # B01 core
    """Target 3 m off the line at x=150, robot beside it at x=20: robot ->
    entry foot (20,0) -> along -> exit foot (150,0) -> target. MUTATION:
    membership against vertices only / no entry leg / no robot point 0 ->
    the point list changes -> red."""
    g = plan_goto(ll(20, 8), ll(150, 3), [LINE], 5.0)  # plan
    pts = [xy(p) for p in g.points]  # in metres
    assert g.route_id == "r-a"  # followed the route
    assert pts[:2] == [(20.0, 8.0), (20.0, 0.0)] and pts[-2:] == [(150.0, 0.0), (150.0, 3.0)]  # robot, entry ... exit, target
    assert all(p[1] == 0.0 for p in pts[1:-1])  # the middle is on the route


def test_goto_runs_against_the_recording_when_that_is_the_way():  # ruling 3
    """Ruling 3: either direction. Robot at x=180, target at x=30.
    MUTATION: forward-only -> no plan or a wrong-way leg -> red."""
    pts = [xy(p) for p in plan_goto(ll(180, 4), ll(30, 2), [LINE], 5.0).points]  # plan
    assert pts[1] == (180.0, 0.0) and pts[2] == (170.0, 0.0) and pts[-2] == (30.0, 0.0)  # backward along it


def test_goto_straight_when_closer_to_target_than_to_route():  # ruling 2B
    """Ruling 2B (re-ruled): 17 m to the target, 20 m to the route -> straight.
    MUTATION: compare with the via-route cost instead (never via a route) ->
    the first goto test reddens; drop the rule -> this one reddens."""
    g = plan_goto(ll(150, 20), ll(150, 3), [LINE], 5.0)  # robot right above the target
    assert g.route_id is None and len(g.points) == 2  # straight


def test_goto_straight_when_target_belongs_to_no_route():  # off route
    """8.5 m off the line > 5 m -> plain PTP. MUTATION: '<' vs '<=' does not
    matter here; drop the membership test -> follows the route -> red."""
    assert plan_goto(ll(20, 8), ll(150, 8.5), [LINE], 5.0).route_id is None  # off route


def test_goto_membership_boundary_is_inclusive():  # boundary
    """Exactly at membership_m belongs (the ruling reads 'within 5 m').
    The boundary is computed in plan_goto's own frame (centred on the robot):
    a WGS84 round trip turns 5.0 into 4.9999999998, which would let a '>='
    mutant survive. MUTATION: '>=' instead of '>' -> red."""
    from xbrain.p3_task.route.route_plan import _Poly  # the module's own projection
    start, target = ll(0, 30), ll(150, 5.0)  # robot, target ~5 m off the line
    plane = _Plane(start)  # plan_goto's frame
    d = _Poly([plane.xy(p) for p in LINE.points], False).project(plane.xy(target))[0]  # exact distance there
    assert plan_goto(start, target, [LINE], d).route_id == "r-a"  # exactly on the boundary: member


def test_goto_takes_the_cheaper_of_two_member_routes():  # ruling 1B
    """Ruling 1B: total length, not entry distance. LINE's entry is closer
    (10 m) but its leg is long; SHORT's entry is 12 m and its leg short.
    MUTATION: pick by entry distance -> LINE -> red."""
    short = RouteGeom("r-s", "S", (ll(140, 8), ll(160, 8)), False)  # a short route 8 m north
    # target (150, 4): 4 m from LINE and 4 m from SHORT -> member of both.
    # LINE: 10 + 150 + 4 = 164; SHORT: ~140 + 10 + 4 = ~154 -> SHORT.
    g = plan_goto(ll(0, 10), ll(150, 4), [LINE, short], 5.0)  # target belongs to both
    assert g.route_id == "r-s"  # cheaper total


def test_goto_on_a_ring_goes_round_through_the_seam():  # ring
    """Ring: from (10,0) to (0,60) the short way is backward through vertex 0,
    not 330 m forward. MUTATION: along = |s_x - s_e| -> long way -> red."""
    pts = [xy(p) for p in plan_goto(ll(10, -6), ll(2, 60), [RING], 5.0).points]  # plan
    assert pts == [(10.0, -6.0), (10.0, 0.0), (0.0, 0.0), (0.0, 60.0), (2.0, 60.0)]  # through the seam


# -- patrol ------------------------------------------------------------------
def test_patrol_open_far_end_first_then_back():  # ruling 4 open
    """Entry at x=60 of 200: far end (200) first, then back to 0. MUTATION:
    near end first / no return leg -> red."""
    pts = [xy(p) for p in plan_patrol(ll(60, 6), LINE.points, False, 1.0)]  # plan
    assert pts[:3] == [(60.0, 6.0), (60.0, 0.0), (70.0, 0.0)]  # robot, entry, toward 200
    assert (200.0, 0.0) in pts and pts[-1] == (0.0, 0.0)  # far end reached, ends at the other end
    assert pts.index((200.0, 0.0)) < len(pts) - 1  # the turn comes before the end


def test_patrol_open_from_an_end_skips_the_return():  # ruling 4 open, edge
    """Entering at the start: nothing behind -> no return leg (it would cover
    the route twice). MUTATION: always return -> ends at 0 -> red."""
    pts = [xy(p) for p in plan_patrol(ll(0, 0.3), LINE.points, False, 1.0)]  # plan
    assert pts[-1] == (200.0, 0.0) and pts.count((0.0, 0.0)) == 1  # one pass


def test_patrol_ring_once_round_recorded_direction():  # ruling 4 ring
    """Ring entered mid-side: once round forward, back to the entry.
    MUTATION: backward / stop at vertex 0 -> red."""
    pts = [xy(p) for p in plan_patrol(ll(50, -4), RING.points, True, 1.0)]  # plan
    assert pts == [(50.0, -4.0), (50.0, 0.0), (100.0, 0.0), (100.0, 100.0),  # expected ring
                   (0.0, 100.0), (0.0, 0.0), (50.0, 0.0)]  # round and back


# -- wiring --------------------------------------------------------------------
async def _open(statements):  # db helper
    conn = await aiosqlite.connect(":memory:")  # in-memory db
    for stmt in statements:  # schema
        await conn.execute(stmt)  # one DDL
    await conn.commit()  # ready
    return conn  # caller closes


@pytest_asyncio.fixture  # per-test dbs
async def dbs():  # task.db + geo.db
    task, geo = await _open(ALL_DDL_STATEMENTS), await _open(GEO_DB_STATEMENTS)  # both dbs
    await geo.execute(  # the line
        "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"  # mode B route
        " total_len_m, rev, content_hash, updated_ms) VALUES"  # as teach commits it
        " ('r-a', '1号路径', ?, 'oneway', 'forward', 200.0, 1, 'h', 0)",  # open, 200 m
        (json.dumps([list(p) for p in LINE.points]),))  # the line, as teach records it
    await geo.execute(  # the target waypoint
        "INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon,"  # on the line
        " arrival_radius, rev, content_hash, updated_ms)"  # 3 m off it
        " VALUES ('w-t', '1号测试点', 'poi', ?, ?, 1.0, 1, 'h', 0)", ll(150, 3))  # on the line
    await geo.commit()  # visible
    yield task, geo  # to the test
    await task.close()  # cleanup
    await geo.close()  # cleanup


class _Pub:  # records frames
    def __init__(self):  # empty
        self.puts = []  # decoded frames

    def put(self, payload):  # one frame
        self.puts.append(json.loads(payload.decode("utf-8")))  # decode


@pytest.mark.asyncio  # async case
@pytest.mark.parametrize("member,via", [(5.0, True), (None, False)])  # configured vs not
async def test_goto_push_follows_the_route_when_configured(dbs, member, via):  # wiring: goto
    """MUTATION: drop the plan in push_goto_for_task -> one point -> red."""
    task, geo = dbs  # the dbs
    pub = _Pub()  # frames out
    res = await push_goto_for_task(  # the push under test
        task_id="tg", params={"waypoint_id": "w-t"}, start=ll(20, 8),  # target and robot
        task_conn=task, geo_conn=geo, snapshot_dao=SnapshotDAO(task),  # dbs and daos
        progress_dao=PatrolProgressDAO(task), route_pub=pub,  # publisher
        ack_window=RouteAckWindow(), trigger=RoutePushTrigger.RP1_DISPATCH,  # dispatch
        now_mono_ms=1000, now_wall_ms=1_785_000_000_000,  # clocks
        route_membership_m=member)  # configured or not
    assert res.pushed  # sent
    pts = [xy((p["lat"], p["lon"])) for p in pub.puts[0]["points"]]  # what P1 receives
    if via:  # configured
        assert pts[1] == (20.0, 0.0) and pts[-1] == (150.0, 3.0)  # entry ... target
    else:  # not configured
        assert pts == [(150.0, 3.0)]  # the single point, unchanged


@pytest.mark.asyncio  # async case
async def test_patrol_push_starts_at_the_robot(dbs):  # wiring: patrol
    """The pushed patrol begins with the robot's position and its entry foot.
    MUTATION: ignore start -> first point is the route start (0,0) -> red;
    a robot 30 m beside the route would then fail max_deviation."""
    task, geo = dbs  # the dbs
    pub = _Pub()  # frames out
    res = await push_route_for_task(  # the push under test
        task_id="tp", route_geo_id="r-a", task_conn=task, geo_conn=geo,  # the route
        snapshot_dao=SnapshotDAO(task), progress_dao=PatrolProgressDAO(task),  # daos
        route_pub=pub, ack_window=RouteAckWindow(),  # publisher
        trigger=RoutePushTrigger.RP1_DISPATCH, now_mono_ms=1000,  # dispatch
        now_wall_ms=1_785_000_000_000, recorded_arrive_radius_m=1.0,  # mode B radius
        start=ll(60, 30))  # 30 m beside x=60
    assert res.pushed  # sent
    pts = [xy((p["lat"], p["lon"])) for p in pub.puts[0]["points"]]  # what P1 receives
    assert pts[:2] == [(60.0, 30.0), (60.0, 0.0)] and pts[-1] == (0.0, 0.0)  # robot, entry ... far end and back


# -- the live P3 loop: both new parameters reach the push (CLAUDE.md 9.1A) ----
@pytest.mark.asyncio  # async case
@pytest.mark.parametrize("task_type", ["patrol", "goto"])  # both kinds
async def test_the_live_loop_plans_from_the_robot(tmp_path, monkeypatch, task_type):  # wiring: live loop
    """main_wiring must hand state/pose to the patrol push (start) and
    route.membership_m to the goto push. MUTATION: pass start=None at RP-1 ->
    the patrol frame starts at the route start -> red; drop
    route_membership_m in _push_goto -> the goto is one point -> red."""
    import asyncio  # the loop runs as a task
    from tests.p3_task.test_recorded_route_radius import _fake_planes  # shared fake session
    from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO  # seed a ready task
    from xbrain.p3_task.runtime import main_wiring as mw  # the live loop

    subs, pubs = {}, {}  # what the fake session saw
    _fake_planes(monkeypatch, subs, pubs)  # no zenoh
    task_db, geo_db = str(tmp_path / "task.db"), str(tmp_path / "geo.db")  # on disk
    async with aiosqlite.connect(geo_db) as g:  # the line and a waypoint on it
        for stmt in GEO_DB_STATEMENTS:  # schema
            await g.execute(stmt)  # one DDL
        await g.execute(  # the line
            "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"  # mode B
            " total_len_m, rev, content_hash, updated_ms) VALUES"  # columns
            " ('r-a', '1号路径', ?, 'oneway', 'forward', 200.0, 1, 'h', 0)",  # open, 200 m
            (json.dumps([list(p) for p in LINE.points]),))  # mode B route
        await g.execute(  # the target
            "INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon,"  # on the line
            " arrival_radius, rev, content_hash, updated_ms)"  # radius 1 m
            " VALUES ('w-t', '1号测试点', 'poi', ?, ?, 1.0, 1, 'h', 0)", ll(150, 3))  # on the line
        await g.commit()  # visible
    stop = {"stop": False}  # loop control
    task = asyncio.create_task(mw._amain(  # start P3
        stop, 99.0, task_db, geo_db_path=geo_db, fence_db_path=str(tmp_path / "fence.db"),  # dbs
        recorded_arrive_radius_m=1.0, route_membership_m=5.0))  # both configured
    try:  # always stop
        for _ in range(300):  # wait for the subscribers
            if mw.CMD_TASK_TOPIC in subs and mw.STATE_POSE_TOPIC in subs:  # subscribers up
                break  # ready
            await asyncio.sleep(0.01)  # poll
        lat, lon = ll(20, 8)  # robot 8 m beside x=20
        subs[mw.STATE_POSE_TOPIC](type("S", (), {"payload": json.dumps(  # state/pose sample
            {"data": {"lat": lat, "lon": lon}}).encode("utf-8")})())  # state/pose
        await asyncio.sleep(0.05)  # let it land
        async with aiosqlite.connect(task_db) as t:  # one ready task
            await TasksDAO(t).insert(TaskRow(  # one ready task
                task_id="t-1", task_type=task_type, state="ready", priority=5,  # kind under test
                submit_seq=1, total_steps=1, current_step=0, step_status_json="[]",  # one step
                mission_json=json.dumps({"params": {"waypoint_id": "w-t"}}),  # goto target
                created_ms=0, updated_ms=0, source="local", trace_id="tr",  # local operator
                resume_policy="continue",  # resumable
                route_geo_id="r-a" if task_type == "patrol" else None))  # patrol names the route
            await t.commit()  # visible
        for _ in range(200):  # a few dispatch passes
            if pubs.get("cmd/motion/route"):  # a frame arrived
                break  # done
            await asyncio.sleep(0.02)  # next pass
        frame = json.loads(pubs["cmd/motion/route"][0].decode("utf-8"))  # first frame
        pts = [xy((p["lat"], p["lon"])) for p in frame["points"]]  # in metres
        assert pts[:2] == [(20.0, 8.0), (20.0, 0.0)]  # robot, entry foot -- both kinds
        assert pts[-1] == ((0.0, 0.0) if task_type == "patrol" else (150.0, 3.0))  # far end and back / the target
    finally:  # cleanup
        stop["stop"] = True  # ask it to stop
        await asyncio.wait_for(task, timeout=10.0)  # and wait


# -- 99 U128: B09 charge and the spoken B08 in the live loop --------------------
@pytest.mark.asyncio
@pytest.mark.parametrize("case", ["named", "nearest_by_travel", "no_dock", "spoken_return_no_home"])
async def test_the_live_loop_dock_and_return_rules(tmp_path, monkeypatch, case):
    """Robot at (0,10); route along y=0 (x 0..200); dock1 at (30,0) on the route
    (straight 31.6 m, by route 10+30 = 40 m); dock2 at (0,45) off any route
    (35 m straight, the same by travel).
      named: charge with waypoint_id=w-d1 -> w-d1 (the named one, U128).
      nearest_by_travel: charge, no name -> w-d2 (35 < 40 by travel; a
        straight-line choice would take w-d1). MUTATION: straight-line
        nearest_dock -> w-d1 -> red.
      no_dock: no dock rows -> failed E_NOT_FOUND. MUTATION: no charge
        branch -> stays running, no frame -> red.
      spoken_return_no_home: return_home with source 'local', no home, docks
        exist -> failed E_NOT_FOUND, NOT sent to a dock. MUTATION: allow_dock
        always True -> a frame to a dock -> red."""
    import asyncio  # the loop runs as a task
    from tests.p3_task.test_recorded_route_radius import _fake_planes  # shared fake session
    from xbrain.common.errors import E_NOT_FOUND  # expected failure code
    from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO  # seed a ready task
    from xbrain.p3_task.runtime import main_wiring as mw  # the live loop

    subs, pubs = {}, {}  # what the fake session saw
    _fake_planes(monkeypatch, subs, pubs)  # no zenoh
    task_db, geo_db = str(tmp_path / "task.db"), str(tmp_path / "geo.db")  # on disk
    async with aiosqlite.connect(geo_db) as g:  # route and dock points
        for stmt in GEO_DB_STATEMENTS:  # schema
            await g.execute(stmt)  # one DDL
        await g.execute(
            "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"
            " total_len_m, rev, content_hash, updated_ms) VALUES"
            " ('r-a', '1号路径', ?, 'oneway', 'forward', 200.0, 1, 'h', 0)",
            (json.dumps([list(p) for p in LINE.points]),))  # the line
        if case != "no_dock":  # two docks
            for gid, name, (x, y) in (("w-d1", "1号充电站", (30, 0)), ("w-d2", "2号充电站", (0, 45))):
                await g.execute(
                    "INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon,"
                    " arrival_radius, rev, content_hash, updated_ms)"
                    " VALUES (?, ?, 'dock', ?, ?, 1.0, 1, 'h', 0)", (gid, name) + ll(x, y))  # a dock point
        await g.commit()  # visible
    stop = {"stop": False}  # loop control
    task = asyncio.create_task(mw._amain(
        stop, 99.0, task_db, geo_db_path=geo_db, fence_db_path=str(tmp_path / "fence.db"),
        recorded_arrive_radius_m=1.0, route_membership_m=5.0))  # configured as in production
    try:
        for _ in range(300):  # wait for the subscribers
            if mw.CMD_TASK_TOPIC in subs and mw.STATE_POSE_TOPIC in subs:  # up
                break  # ready
            await asyncio.sleep(0.01)  # poll
        lat, lon = ll(0, 10)  # robot position
        subs[mw.STATE_POSE_TOPIC](type("S", (), {"payload": json.dumps(
            {"data": {"lat": lat, "lon": lon}}).encode("utf-8")})())  # state/pose
        await asyncio.sleep(0.05)  # let it land
        ttype = "return_home" if case == "spoken_return_no_home" else "charge"  # task kind
        params = {"waypoint_id": "w-d1"} if case == "named" else {}  # named or not
        async with aiosqlite.connect(task_db) as t:  # one ready task
            await TasksDAO(t).insert(TaskRow(
                task_id="t-1", task_type=ttype, state="ready", priority=5,
                submit_seq=1, total_steps=1, current_step=0, step_status_json="[]",
                mission_json=json.dumps({"params": params}), created_ms=0,
                updated_ms=0, source="local", trace_id="tr", resume_policy="continue"))  # a voice task
            await t.commit()  # visible
        row = None  # the task row after dispatch
        for _ in range(200):  # a few dispatch passes
            if pubs.get("cmd/motion/route"):  # a frame went out
                break  # done
            async with aiosqlite.connect(task_db) as t:  # or it failed
                row = await (await t.execute(
                    "SELECT state, error_context_json FROM tasks WHERE task_id='t-1'")).fetchone()  # state
            if row and row[0] == "failed":  # terminal
                break  # done
            await asyncio.sleep(0.02)  # next pass
        frames = [json.loads(p.decode("utf-8")) for p in pubs.get("cmd/motion/route", [])]  # what P1 got
        if case in ("named", "nearest_by_travel"):  # a dock goto
            assert frames and frames[0]["route_id"] == ("goto-w-d1" if case == "named" else "goto-w-d2")  # which dock
        else:  # refused
            assert not frames and row is not None and row[0] == "failed"  # never left running
            assert json.loads(row[1])["code"] == E_NOT_FOUND  # 15 S9.5.1
    finally:
        stop["stop"] = True  # ask it to stop
        await asyncio.wait_for(task, timeout=10.0)  # and wait
