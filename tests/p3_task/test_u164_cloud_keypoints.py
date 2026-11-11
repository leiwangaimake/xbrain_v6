"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u164_cloud_keypoints.py
Brief: 99 U164 -- the cloud GOTO_KEYPOINT goes to its waypoints in order along recorded_path_id

Description:
Until 2026-11-11 the gateway's goto (route_id = recorded_path_id, params.
waypoints = the v2.0 plan) took P3's ROUTE branch at dispatch: the whole
recorded path was patrolled, waypoints[] and arrival_radius_m were stored and
never read, and a missing path or waypoint was accepted and failed later.
User rulings 2026-11-11: string the waypoints into ONE line in order; plan
each leg along recorded_path_id only. Pinned here against the real push, the
real submit path and the live P3 loop. Each case names the mutant it kills.
"""
from __future__ import annotations

import asyncio
import json

import aiosqlite
import pytest

from tests.p3_task.test_goto_precheck import _square
from tests.p3_task.test_route_plan import LINE, _Pub, dbs, ll, xy  # noqa: F401 (fixture)
from xbrain.common.errors import E_NOT_FOUND, E_OUT_OF_FENCE
from xbrain.p3_task.dao.simple_daos import PatrolProgressDAO, SnapshotDAO
from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.ingest.goto_precheck import cloud_goto_refusal
from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload
from xbrain.p3_task.persistence.schema_geo import GEO_DB_STATEMENTS
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.route.push import RouteAckWindow, RoutePushTrigger
from xbrain.p3_task.runtime.route_push_runtime import push_goto_for_task

pytestmark = pytest.mark.no_device


def _plan(*items):
    """params as the gateway passes the v2.0 S2.1 plan (task_router._goto)."""
    return {"coordinate_system": "WGS84",
            "waypoints": [{"id": w, "name": w, "latitude": 0.0, "longitude": 0.0,
                           "altitude": 0.0, "arrival_radius_m": r} for w, r in items]}


async def _add_waypoint(geo, wid, x, y):
    await geo.execute(
        "INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon, arrival_radius,"
        " rev, content_hash, updated_ms) VALUES (?, ?, 'poi', ?, ?, 1.0, 1, 'h', 0)",
        (wid, wid, *ll(x, y)))
    await geo.commit()


async def _push(task, geo, params, start=None, task_id="tc"):
    start = ll(20, 8) if start is None else start  # robot 8 m beside x=20
    pub = _Pub()
    res = await push_goto_for_task(
        task_id=task_id, params=params, start=start, task_conn=task, geo_conn=geo,
        snapshot_dao=SnapshotDAO(task), progress_dao=PatrolProgressDAO(task),
        route_pub=pub, ack_window=RouteAckWindow(),
        trigger=RoutePushTrigger.RP1_DISPATCH, now_mono_ms=1000,
        now_wall_ms=1_785_000_000_000, route_membership_m=5.0)
    return res, pub


@pytest.mark.asyncio
async def test_two_waypoints_are_one_line_in_order_with_the_last_radius(dbs):  # noqa: F811
    """robot (20, 8) -> w-t (150, 3) -> w-u (60, 2) along r-a. MUTATION: keep
    the last waypoint only -> x never reaches 150 -> red; reverse the order ->
    the line ends at 150 -> red; take the waypoint row's radius (1.0) or the
    first item's -> red."""
    task, geo = dbs
    await _add_waypoint(geo, "w-u", 60, 2)
    res, pub = await _push(task, geo, dict(_plan(("w-t", 4.0), ("w-u", 2.5)),
                                           via_route_id="r-a"))
    assert res.pushed
    frame = pub.puts[0]
    pts = [xy((p["lat"], p["lon"])) for p in frame["points"]]
    assert pts[0] == (20.0, 8.0) and pts[1] == (20.0, 0.0)  # robot, entry foot on r-a
    assert (150.0, 3.0) in pts and pts[-1] == (60.0, 2.0)  # through w-t, ends at w-u
    assert pts.index((150.0, 3.0)) < len(pts) - 1
    assert frame["points"][-1]["arrive_radius_m"] == 2.5  # the plan's own radius
    assert frame["route_id"] == "goto-tc"


@pytest.mark.asyncio
async def test_the_plan_follows_recorded_path_id_only(dbs):  # noqa: F811
    """A second route r-b along y = 8 is the cheaper U127 choice for a robot
    at (20, 10) and a target at (150, 4) (4 m from both, inside the 5 m
    membership -- exactly 5 m falls outside on rounding and would make this
    case vacuous). The plan names r-a, so the entry foot is on r-a.
    MUTATION: plan over every route -> entry (20, 8) on r-b -> red."""
    task, geo = dbs
    await geo.execute(
        "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"
        " total_len_m, rev, content_hash, updated_ms) VALUES"
        " ('r-b', '2号路径', ?, 'oneway', 'forward', 200.0, 1, 'h', 0)",
        (json.dumps([list(ll(x, 8)) for x in range(0, 201, 10)]),))
    await _add_waypoint(geo, "w-m", 150, 4)
    res, pub = await _push(task, geo, dict(_plan(("w-m", 1.5)), via_route_id="r-a"),
                           start=ll(20, 10))
    assert res.pushed
    pts = [xy((p["lat"], p["lon"])) for p in pub.puts[0]["points"]]
    assert pts[1] == (20.0, 0.0) and pts[-1] == (150.0, 4.0)
    res, pub = await _push(task, geo, _plan(("w-m", 1.5)), start=ll(20, 10), task_id="tc2")
    assert [xy((p["lat"], p["lon"])) for p in pub.puts[0]["points"]][1] == (20.0, 8.0)  # the case bites


@pytest.mark.asyncio
@pytest.mark.parametrize("params,why", [
    (dict(_plan(("w-t", 2.0)), via_route_id="r-gone"), "route 'r-gone' not found"),
    (dict(_plan(("w-t", 2.0), ("w-gone", 2.0)), via_route_id="r-a"),
     "waypoints[1] 'w-gone' not found"),
])
async def test_a_plan_that_cannot_be_resolved_is_not_pushed(dbs, params, why):  # noqa: F811
    """A path or waypoint deleted after admission: nothing pushed, the reason
    names it. MUTATION: fall back to all routes / skip the missing waypoint
    -> pushed -> red."""
    task, geo = dbs
    res, pub = await _push(task, geo, params)
    assert not res.pushed and why in res.reason and res.cause == "target_missing"
    assert pub.puts == []


# -- submit: v2.0 S3.1 "ID 不存在 ... 必须拒绝并提供结构化定位字段" ------------------------
def _task(route="r-a", *wids):
    return {"type": "goto", "route_id": route, "params": _plan(*[(w, 2.0) for w in wids])}


async def _route_exists(gid):
    return gid == "r-a"


async def _keypoint(wid):
    return {"w-1": (31.1585, 121.3913), "w-2": (31.1590, 121.3913)}.get(wid)


@pytest.mark.asyncio
async def test_the_refusals_carry_the_locators():
    """MUTATION: no path check -> None -> red; report index 0 for every
    waypoint -> red; drop the fence check -> None -> red."""
    r = await cloud_goto_refusal(_task("r-x", "w-1"), fence_set=None,
                                 keypoint=_keypoint, route_exists=_route_exists)
    assert r[0] == E_NOT_FOUND and r[2]["field"] == "recorded_path_id"
    r = await cloud_goto_refusal(_task("r-a", "w-1", "w-9"), fence_set=None,
                                 keypoint=_keypoint, route_exists=_route_exists)
    assert r[0] == E_NOT_FOUND and (r[2]["index"], r[2]["field"], r[2]["waypoint_id"]) == (
        1, "waypoints[1]", "w-9")
    fence = {"polygons": [_square(31.1585, 121.3913, 0.0002, "allow")]}
    r = await cloud_goto_refusal(_task("r-a", "w-1", "w-2"), fence_set=fence,
                                 keypoint=_keypoint, route_exists=_route_exists)
    assert r[0] == E_OUT_OF_FENCE and r[2]["index"] == 1 and r[2]["subject"] == "target"
    assert await cloud_goto_refusal(_task("r-a", "w-1"), fence_set=fence,
                                    keypoint=_keypoint, route_exists=_route_exists) is None


@pytest.mark.asyncio
async def test_a_cloud_goto_is_refused_in_the_ack_and_not_stored():
    """MUTATION: no cloud_goto_precheck call in _submit -> accepted, one row."""
    conn = await aiosqlite.connect(":memory:")
    for stmt in ALL_DDL_STATEMENTS:
        await conn.execute(stmt)
    await conn.commit()

    async def precheck(task):
        return await cloud_goto_refusal(task, fence_set=None, keypoint=_keypoint,
                                        route_exists=_route_exists)
    ctx = TaskContext(conn, TasksDAO(conn), cloud_goto_precheck=precheck)
    try:
        body = {"cmd_id": "c-1", "action": "submit", "source": "cloud",
                "task": dict(_task("r-a", "w-9"), task_id="task-1")}
        ack = await handle_task_payload(body, ctx, now_mono_ms=1, date_str="20261111")
        assert ack["result"] == "rejected" and ack["code"] == E_NOT_FOUND
        assert ack["detail"]["waypoint_id"] == "w-9"
        cur = await conn.execute("SELECT COUNT(*) FROM tasks")
        assert (await cur.fetchone())[0] == 0
    finally:
        await conn.close()


# -- the live loop: dispatch takes the keypoint branch, not the route branch ----------------
@pytest.mark.asyncio
async def test_the_live_loop_drives_to_the_waypoint_not_the_whole_path(tmp_path, monkeypatch):
    """A ready cloud goto (route_geo_id = r-a, params.waypoints = [w-m]); a
    second route r-b along y = 8 is the cheaper U127 choice for the robot
    at (20, 10). MUTATION: drop the keypoint branch in
    _push_route_on_dispatch -> the route branch patrols r-a -> the frame
    ends at (0, 0) -> red; drop via_route_id at dispatch -> entry on r-b
    (20, 8) -> red."""
    from tests.p3_task.test_recorded_route_radius import _fake_planes
    from xbrain.p3_task.dao.tasks_dao import TaskRow
    from xbrain.p3_task.runtime import main_wiring as mw

    subs, pubs = {}, {}
    _fake_planes(monkeypatch, subs, pubs)
    task_db, geo_db = str(tmp_path / "task.db"), str(tmp_path / "geo.db")
    async with aiosqlite.connect(geo_db) as g:
        for stmt in GEO_DB_STATEMENTS:
            await g.execute(stmt)
        await g.execute(
            "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"
            " total_len_m, rev, content_hash, updated_ms) VALUES"
            " ('r-a', '1号路径', ?, 'oneway', 'forward', 200.0, 1, 'h', 0)",
            (json.dumps([list(p) for p in LINE.points]),))
        await g.execute(
            "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"
            " total_len_m, rev, content_hash, updated_ms) VALUES"
            " ('r-b', '2号路径', ?, 'oneway', 'forward', 200.0, 1, 'h', 0)",
            (json.dumps([list(ll(x, 8)) for x in range(0, 201, 10)]),))
        await g.execute(
            "INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon,"
            " arrival_radius, rev, content_hash, updated_ms)"
            " VALUES ('w-m', '1号测试点', 'poi', ?, ?, 1.0, 1, 'h', 0)", ll(150, 4))
        await g.commit()
    stop = {"stop": False}
    loop = asyncio.create_task(mw._amain(
        stop, 99.0, task_db, geo_db_path=geo_db, fence_db_path=str(tmp_path / "fence.db"),
        recorded_arrive_radius_m=1.0, route_membership_m=5.0))
    try:
        for _ in range(300):
            if mw.CMD_TASK_TOPIC in subs and mw.STATE_POSE_TOPIC in subs:
                break
            await asyncio.sleep(0.01)
        lat, lon = ll(20, 10)
        subs[mw.STATE_POSE_TOPIC](type("S", (), {"payload": json.dumps(
            {"data": {"lat": lat, "lon": lon}}).encode("utf-8")})())
        await asyncio.sleep(0.05)
        # the submit check is wired into the live cmd/task path. MUTATION:
        # build TaskContext without cloud_goto_precheck -> accepted -> red.
        subs[mw.CMD_TASK_TOPIC](type("S", (), {"payload": json.dumps(
            {"cmd_id": "c-bad", "action": "submit", "source": "cloud",
             "task": dict(_task("r-a", "w-9"), task_id="task-bad")}).encode("utf-8")})())
        for _ in range(100):
            if pubs.get(mw.CMD_TASK_ACK_TOPIC):
                break
            await asyncio.sleep(0.02)
        ack = json.loads(pubs[mw.CMD_TASK_ACK_TOPIC][0])
        assert ack["result"] == "rejected" and ack["detail"]["field"] == "waypoints[0]"
        async with aiosqlite.connect(task_db) as t:
            await TasksDAO(t).insert(TaskRow(
                task_id="task-c1", task_type="goto", state="ready", priority=5,
                submit_seq=1, total_steps=1, current_step=0, step_status_json="[]",
                mission_json=json.dumps({"source": "cloud", "params": _plan(("w-m", 3.0))}),
                created_ms=0, updated_ms=0, source="cloud", trace_id="tr",
                resume_policy="continue", route_geo_id="r-a"))
            await t.commit()
        for _ in range(200):
            if pubs.get("cmd/motion/route"):
                break
            await asyncio.sleep(0.02)
        frame = json.loads(pubs["cmd/motion/route"][0].decode("utf-8"))
        pts = [xy((p["lat"], p["lon"])) for p in frame["points"]]
        assert pts[:2] == [(20.0, 10.0), (20.0, 0.0)] and pts[-1] == (150.0, 4.0)
        assert frame["points"][-1]["arrive_radius_m"] == 3.0
    finally:
        stop["stop"] = True
        await asyncio.wait_for(loop, timeout=10.0)
