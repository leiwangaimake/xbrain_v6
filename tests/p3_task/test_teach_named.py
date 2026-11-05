"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_teach_named.py
Brief: 99 U133 -- named recording sessions: bind the name at start, save at finish, no task while recording

Description:
The user's rulings of 2026-11-05 (99 U133) on the P3 side, each pinned by a case
that names the mutant it kills:
  * start.name binds the session name; a name already used by a live route
    (or fence, for a fence session) is refused at start with E_NAME_CONFLICT;
  * finish on a named session saves under that name at once ("退出即保存")
    -- a route row, or a draft forbid fence (saving is not enabling);
  * geometry that cannot be saved closes the session as invalid_geometry
    with E_TEACH_GEOMETRY and the issues -- no finalizing left behind;
  * the max_duration auto-finish of a named session saves the same way;
  * while a session records, cmd/task submit is refused with E_BUSY and
    reason teach_session_active (11 S12A.3 CMD-16, never wired before).
The last case drives the real P3 loop on a fake session, because the CMD-16
refusal lives in the wiring (TaskContext.teach_busy) and a unit test of
task_apply alone would stay green with it unwired.
"""
from __future__ import annotations  # postponed annotations

import asyncio  # the live loop runs as a task
import json  # wire frames
import math  # fence square

import aiosqlite  # in-memory dbs
import pytest  # marker
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_BUSY, E_NAME_CONFLICT, E_TEACH_GEOMETRY  # expected codes
from xbrain.p3_task.dao.tasks_dao import TasksDAO  # the submit path's dao
from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload  # CMD-16 under test
from xbrain.p3_task.persistence.schema_geo import FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS  # geo DDL
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task DDL
from xbrain.p3_task.teach.runtime import TeachRuntime  # under test

pytestmark = pytest.mark.no_device  # pure logic + in-memory sqlite

_LAT, _LON = 34.6970, 135.5050  # a site
_DEG_PER_M = 1.0 / 111320.0  # metres -> degrees of latitude


async def _open(statements):  # in-memory db with a schema
    conn = await aiosqlite.connect(":memory:")  # fresh
    for stmt in statements:  # schema
        await conn.execute(stmt)  # one DDL
    await conn.commit()  # ready
    return conn  # caller closes


def _pose(rt, lat, lon, t):  # one state/pose sample at mono t
    rt.update_pose({"lat": lat, "lon": lon, "fix_type": "rtk_fixed", "heading_rad": 0.0,
                    "heading_valid": True, "heading_level": 1, "speed_mps": 1.0}, t)  # rtk_fixed


@pytest_asyncio.fixture  # a runtime with healthy caches
async def rt():
    task, geo, fence = (await _open(ALL_DDL_STATEMENTS), await _open(GEO_DB_STATEMENTS),
                        await _open(FENCE_DB_STATEMENTS))  # three dbs
    runtime = TeachRuntime(task, geo, fence, boot_id="b0", sample_hz=1.0, min_dist_m=0.5,  # U132 pair
                           snap_max_age_ms=8000)  # p3_task.yaml value
    _pose(runtime, _LAT, _LON, 100.0)  # a fresh fix
    runtime.update_health({"allow_motion": True})  # arming check 4
    runtime.update_robot({"hes": False, "estop_path": "up"})  # check 4 / 7
    runtime.update_power({"soc_pct": 80.0})  # check 5
    runtime.update_teleop({"sources": [{"device": "gamepad", "stale": False, "alive": True}]})  # check 7
    yield runtime  # to the test
    for c in (task, geo, fence):  # cleanup
        await c.close()  # close


def _cmd(cmd_id, action, **over):  # a voice-issued cmd/teach
    base = {"cmd_id": cmd_id, "action": action,
            "issuer": {"src": "p4_agent", "channel": "local_voice"}}  # S12A.4 issuer
    base.update(over)  # the action block
    return base  # payload


async def _start(rt, name, kind="route", t=100.0):  # a named start
    _pose(rt, _LAT, _LON, t)  # fresh at t
    return await rt.handle(_cmd("c-start-%s-%s" % (kind, name), "start",  # one cmd_id per start
                                start={"kind": kind, "name": name}),
                           now_mono_s=t, now_ms=1000)  # the ack


async def _drive(rt, metres, count, t0=101.0):  # count samples, metres apart, a second apart
    for i in range(count):  # each sample
        _pose(rt, _LAT + metres * (i + 1) * _DEG_PER_M, _LON, t0 + i)  # further north
        await rt.offer_pose(t0 + i, 1000 + i)  # the sampler


@pytest.mark.asyncio  # async case
async def test_a_named_finish_saves_the_route_at_once(rt):  # "退出即保存"
    """MUTATION: drop the named branch in finish -> state finalizing, no row -> red."""
    ack = await _start(rt, "1号路径")  # named start
    sid = ack["detail"]["session_id"]  # session
    assert rt.active  # recording mode
    await _drive(rt, 2.0, 5)  # five points
    fin = await rt.handle(_cmd("c-fin", "finish", session_id=sid), now_mono_s=110.0, now_ms=2000)  # exit
    assert fin["result"] == "accepted" and fin["detail"]["saved"] is True  # saved
    assert fin["detail"]["applied"]["name"] == "1号路径" and fin["detail"]["applied"]["point_count"] == 5  # as named
    cur = await rt._geo_conn.execute("SELECT name FROM routes")  # the row
    assert await cur.fetchone() == ("1号路径",)  # written
    assert not rt.active and rt.teach_state_payload(111.0)["session"] == {"state": "idle"}  # mode left


@pytest.mark.asyncio  # async case
async def test_a_named_fence_is_saved_as_a_draft(rt):  # same flow for fences
    """Saving is not enabling (S12A.7 constraint 1). MUTATION: commit active -> red."""
    ack = await _start(rt, "东门围栏", kind="fence")  # named fence start
    d = 20.0 * _DEG_PER_M  # 20 m in latitude
    dlon = d / math.cos(math.radians(_LAT))  # 20 m in longitude
    for i, (la, lo) in enumerate([(_LAT, _LON), (_LAT + d, _LON), (_LAT + d, _LON + dlon),
                                  (_LAT, _LON + dlon)]):  # a square
        _pose(rt, la, lo, 101.0 + i)  # a corner
        await rt.offer_pose(101.0 + i, 1000 + i)  # sampled
    fin = await rt.handle(_cmd("c-fin", "finish", session_id=ack["detail"]["session_id"]),
                          now_mono_s=110.0, now_ms=2000)  # exit
    assert fin["result"] == "accepted" and fin["detail"]["activated"] is False  # not in force
    cur = await rt._fence_conn.execute("SELECT role, state FROM fences WHERE name='东门围栏'")  # the row
    assert await cur.fetchone() == ("forbid", "draft")  # 15 design


@pytest.mark.asyncio  # async case
async def test_unsavable_geometry_closes_the_session_and_says_why(rt):  # no finalizing left
    """MUTATION: leave the session in finalizing -> rt still holds a session -> red."""
    ack = await _start(rt, "2号路径")  # named start
    await _drive(rt, 2.0, 1)  # one point: too_few_points
    fin = await rt.handle(_cmd("c-fin", "finish", session_id=ack["detail"]["session_id"]),
                          now_mono_s=110.0, now_ms=2000)  # exit
    assert fin["result"] == "rejected" and fin["code"] == E_TEACH_GEOMETRY  # not saved
    assert fin["detail"]["close_reason"] == "invalid_geometry"  # why it closed
    assert [i["code"] for i in fin["detail"]["issues"]] == ["too_few_points"]  # the reason P4 speaks
    assert rt.teach_state_payload(111.0)["session"] == {"state": "idle"}  # nothing left open
    cur = await rt._geo_conn.execute("SELECT COUNT(*) FROM routes")  # no row
    assert (await cur.fetchone())[0] == 0  # nothing written


@pytest.mark.asyncio  # async case
async def test_a_taken_name_is_refused_at_start(rt):  # bound at start
    """MUTATION: drop the start-time name check -> the session arms -> red."""
    await rt._geo_conn.execute(
        "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction, total_len_m, rev,"
        " content_hash, updated_ms) VALUES ('r-1', '1号路径', '[]', 'oneway', 'forward', 1.0, 1,"
        " 'h', 0)")  # a live route of that name
    await rt._geo_conn.commit()  # visible
    ack = await _start(rt, "1号路径")  # same name
    assert ack["result"] == "rejected" and ack["code"] == E_NAME_CONFLICT  # refused
    assert not rt.active  # never armed
    fence_ack = await _start(rt, "1号路径", kind="fence", t=101.0)  # a fence may share a route's name
    assert fence_ack["result"] == "accepted"  # different table


@pytest.mark.asyncio  # async case
async def test_a_named_session_auto_finished_at_its_deadline_is_saved(rt):  # max_duration
    """MUTATION: save_named ignores sessions finished by expire -> no row -> red."""
    ack = await rt.handle(_cmd("c-s", "start", start={"kind": "route", "name": "3号路径",
                                                      "max_duration_s": 10}),
                          now_mono_s=100.0, now_ms=1000)  # a short session
    assert ack["result"] == "accepted"  # armed
    await _drive(rt, 2.0, 3)  # three points
    assert rt.expire(111.0) == "max_duration"  # auto-finished
    detail = await rt.save_named(3000)  # what the loop does next
    assert detail["saved"] is True and detail["applied"]["name"] == "3号路径"  # saved


@pytest.mark.asyncio  # async case
async def test_a_discard_names_what_it_threw_away(rt):  # 99 U137
    """The spoken answer needs the name and count. MUTATION: drop name from
    the discard detail -> red."""
    ack = await _start(rt, "6号路径")  # named
    await _drive(rt, 2.0, 3)  # three points
    d = await rt.handle(_cmd("c-d", "discard", session_id=ack["detail"]["session_id"],
                             reason="operator_cancel"), now_mono_s=110.0, now_ms=2000)  # F04
    assert d["detail"]["name"] == "6号路径" and d["detail"]["discarded_points"] == 3  # said back
    assert d["detail"]["kind"] == "route" and not rt.active  # mode left


@pytest.mark.asyncio  # async case
async def test_criterion_7_reads_estop_path_from_state_link(rt):  # 2026-11-05 runtime test
    """Arming criterion 7 (2): state/robot reachable AND state/link estop_path
    not down (11 S4.6 -- RobotState has no estop_path). The gate read
    state/robot.estop_path, which never exists, so with no local gamepad
    recording could never arm on the robot. MUTATION: read robot.get(
    "estop_path") again -> the first start is refused -> red."""
    rt.update_teleop({"sources": [{"device": "keyboard_cloud", "stale": False, "alive": True}]})  # network only
    rt.update_robot({"hes": False})  # the real RobotState: no estop_path field
    rt.update_estop_path("ok")  # P5's probe verdict on state/link
    ok = await _start(rt, "7号路径")  # arms
    assert ok["result"] == "accepted"  # criterion 7 (2) passed
    await rt.handle(_cmd("c-x", "discard", session_id=ok["detail"]["session_id"]),
                    now_mono_s=101.0, now_ms=1)  # close it
    rt.update_estop_path("down")  # probe lost
    rt.update_robot({"hes": False, "estop_path": "ok"})  # a robot field must not count
    bad = await _start(rt, "8号路径", t=102.0)  # refused
    assert bad["result"] == "rejected" and bad["detail"]["reason"] == "no_nonvoice_estop"  # fail-safe


@pytest.mark.asyncio  # async case
async def test_task_submit_is_refused_while_recording(rt):  # CMD-16
    """MUTATION: no teach_busy check in _submit -> the task is stored -> red."""
    await _start(rt, "4号路径")  # recording
    ctx = TaskContext(rt._task_conn, TasksDAO(rt._task_conn), teach_busy=lambda: rt.active)  # as wired
    ack = await handle_task_payload(
        {"cmd_id": "c-t", "action": "submit", "source": "voice",
         "task": {"type": "patrol", "route_id": "r-x", "params": {}}},
        ctx, now_mono_ms=1, date_str="20261105")  # a patrol
    assert ack["result"] == "rejected" and ack["code"] == E_BUSY  # refused
    assert ack["detail"]["reason"] == "teach_session_active"  # P4 speaks this


@pytest.mark.asyncio  # async case
async def test_the_live_loop_refuses_a_task_during_a_recording(tmp_path, monkeypatch):  # wiring
    """The real loop: state caches in, a named cmd/teach start, then a cmd/task
    submit must come back rejected teach_session_active. MUTATION: build the
    loop's TaskContext without teach_busy -> accepted -> red."""
    from tests.p3_task.test_recorded_route_radius import _fake_planes  # shared fake session
    from xbrain.p3_task.runtime import main_wiring as mw  # the live loop

    subs, pubs = {}, {}  # what the fake session saw
    _fake_planes(monkeypatch, subs, pubs)  # no zenoh

    def _sample(body):  # a zenoh sample stand-in
        return type("S", (), {"payload": json.dumps(body).encode("utf-8")})()  # payload only

    stop = {"stop": False}  # loop control
    task = asyncio.create_task(mw._amain(
        stop, 99.0, str(tmp_path / "task.db"), geo_db_path=str(tmp_path / "geo.db"),
        fence_db_path=str(tmp_path / "fence.db"), recorded_arrive_radius_m=1.0,
        teach_sample_hz=1.0, teach_min_dist_m=0.5))  # configured as in production
    try:  # always stop
        for _ in range(300):  # wait for the subscribers
            if mw.CMD_TEACH_TOPIC in subs and mw.CMD_TASK_TOPIC in subs:  # up
                break  # ready
            await asyncio.sleep(0.01)  # poll
        for key, body in ((mw.STATE_POSE_TOPIC, {"lat": _LAT, "lon": _LON, "fix_type": "rtk_fixed",
                                                 "heading_rad": 0.0, "heading_valid": True,
                                                 "heading_level": 1, "speed_mps": 0.0}),
                          (mw.HEALTH_SUMMARY_TOPIC, {"allow_motion": True}),
                          (mw.STATE_ROBOT_TOPIC, {"hes": False, "estop_path": "up"}),
                          (mw.STATE_POWER_TOPIC, {"soc_pct": 80.0}),
                          (mw.STATE_TELEOP_TOPIC, {"sources": [{"device": "gamepad", "stale": False,
                                                                "alive": True}]})):  # healthy
            subs[key](_sample({"data": body}))  # enveloped, as published
        await asyncio.sleep(0.1)  # cached
        subs[mw.CMD_TEACH_TOPIC](_sample(_cmd("c-live", "start",
                                              start={"kind": "route", "name": "5号路径"})))  # start
        for _ in range(100):  # until answered
            if pubs.get(mw.CMD_TEACH_ACK_TOPIC):  # an ack
                break  # done
            await asyncio.sleep(0.02)  # next pass
        assert json.loads(pubs[mw.CMD_TEACH_ACK_TOPIC][0])["result"] == "accepted"  # recording
        subs[mw.CMD_TASK_TOPIC](_sample({"cmd_id": "c-task", "action": "submit", "source": "voice",
                                         "task": {"type": "patrol", "route_id": "r-x",
                                                  "params": {}}}))  # a patrol
        for _ in range(100):  # until answered
            if pubs.get(mw.CMD_TASK_ACK_TOPIC):  # an ack
                break  # done
            await asyncio.sleep(0.02)  # next pass
        ack = json.loads(pubs[mw.CMD_TASK_ACK_TOPIC][0])  # the answer
        assert ack["result"] == "rejected" and ack["detail"]["reason"] == "teach_session_active"  # CMD-16
    finally:  # cleanup
        stop["stop"] = True  # ask it to stop
        await asyncio.wait_for(task, timeout=10.0)  # and wait
