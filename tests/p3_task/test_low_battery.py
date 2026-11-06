"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_low_battery.py
Brief: 99 U146 low-battery return -- episode events, dock / home choice, the injected task, 5 % stop, cloud alarms, dock-on-route

Description:
The user's low-battery rule (2026-11-05 ruling 3, restated 2026-11-06):
below 20 % one voice alarm and a cloud alarm every 2 s until the chassis
reports charging; drive (normal speed) to the nearest 充电桩 / 充电站 point
that lies on a route, else to 返航终点站; say unreachable when
remain_mile_km cannot cover it; at 5 % stop and lie down; at boot report a
missing home point; a dock off every route is refused when it is recorded.
Each case names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import asyncio  # the live loop
import json  # path_points

import aiosqlite  # geo.db
import pytest  # marks, parametrize
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_GEO_INVALID, E_LOW_BATTERY  # refusal codes
from xbrain.p3_task.lifecycle import low_battery as lb  # under test
from xbrain.p3_task.persistence.schema_geo import GEO_DB_STATEMENTS  # geo.db DDL
from xbrain.p3_task.runtime.route_push_runtime import load_route_geoms  # routes as P3 reads them
from tests.p3_task.live_loop import (  # the real loop over fake planes
    goto_row, route_ops, start_p3, stop_p3, task_state, wait_for)

pytestmark = pytest.mark.no_device  # pure logic plus an in-process loop

ROUTE = [[35.0, 135.0], [35.002, 135.0]]  # ~222 m north from the robot (35.0, 135.0)
ROUTE_SQL = ("INSERT INTO routes (geo_id, name, path_points, loop_mode, direction, total_len_m,"  # one mode B route
             " rev, content_hash, updated_ms) VALUES ('r-a', '1号路径', ?, 'oneway', 'forward',"
             " 222.0, 1, 'h', 0)", (json.dumps(ROUTE),))  # a mode B route


def _wp(gid, name, wtype, lat, lon):  # a waypoint row
    """(sql, params) of one waypoint row."""
    return ("INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon, arrival_radius, rev,"  # columns as P3 stores them
            " content_hash, updated_ms) VALUES (?, ?, ?, ?, ?, 1.0, 1, 'h', 0)",
            (gid, name, wtype, lat, lon))  # as P3 stores it


DOCK_ON = _wp("w-d1", "1号充电站", "dock", 35.0015, 135.00002)  # ~2 m off the route
DOCK_OFF = _wp("w-d2", "2号充电站", "dock", 35.0, 135.01)  # ~900 m off any route, but closer straight
HOME = _wp("w-home", "返航终点站", "home", 35.0005, 135.0)  # on the route


def _power(soc, charging=False, remain_km=5.0, charge="idle"):  # a chassis power frame
    """A state/power body as quadruped publishes it (11 S4.2, 13 BAT-1)."""
    return {"soc_pct": soc, "remain_mile_km": remain_km, "charge": charge,  # 11 S4.2 fields
            "list": [{"index": 0, "level_pct": soc, "charging": charging, "present": True}]}


# -- the episode ----------------------------------------------------------------------
def test_low_starts_once_and_alarms_every_period():  # trigger and cadence
    """MUTATION: no cadence -> an alarm every frame -> red; '<=' 20 -> 20.0
    starts an episode -> red."""
    m = lb.LowBatteryMonitor(20.0, 5.0, 2.0)  # the shipped rule
    assert m.update(_power(20.0), 0.0) == []  # 20 % is not below 20
    assert m.update(_power(19.0), 1.0) == ["low", "alarm"]  # starts, first alarm
    assert m.update(_power(19.0), 2.0) == []  # 1 s later: not due
    assert m.update(_power(19.0), 3.0) == ["alarm"]  # 2 s after the last
    assert m.update(_power(25.0), 4.0) == []  # soc up without charging: still on


@pytest.mark.parametrize("body", [_power(19.0, charging=True), _power(19.0, charge="charging")])
def test_only_charging_closes_the_episode(body):  # either charging signal
    """99 U125: either the battery flag or the six-state field. MUTATION:
    read one of the two only -> red for the other; close on soc -> red above."""
    m = lb.LowBatteryMonitor(20.0, 5.0, 2.0)  # the shipped rule
    m.update(_power(19.0), 0.0)  # an episode
    assert m.update(body, 1.0) == ["charging"] and not m.low  # closed by charging


def test_stop_once_at_five_percent():  # the 5 % latch
    """MUTATION: '<' 5 -> 5.0 does not stop -> red; no once-latch -> red."""
    m = lb.LowBatteryMonitor(20.0, 5.0, 2.0)  # the shipped rule
    m.update(_power(19.0), 0.0)  # episode
    assert "stop" in m.update(_power(5.0), 3.0)  # the last 5 %
    assert "stop" not in m.update(_power(4.0), 6.0)  # once


@pytest.mark.parametrize("args", [(None, 5.0, 2.0), (20.0, 0.0, 2.0), (20.0, 20.0, 2.0),
                                  (20.0, 5.0, 0)])
def test_thresholds_are_required_and_ordered(args):  # config refusals
    """CLAUDE.md 3.1. MUTATION: accept stop >= trigger -> red."""
    with pytest.raises(lb.LowBatteryConfigError):  # each bad tuple
        lb.LowBatteryMonitor(*args)  # refused


# -- the target ------------------------------------------------------------------------
@pytest_asyncio.fixture
async def geo():  # catalogue fixture
    """An in-memory geo.db with the route."""
    conn = await aiosqlite.connect(":memory:")  # CLAUDE.md 7.2
    for stmt in GEO_DB_STATEMENTS:  # real schema
        await conn.execute(stmt)  # real schema
    await conn.execute(*ROUTE_SQL)  # the route
    await conn.commit()  # no open transaction left for the appliers' BEGIN IMMEDIATE
    yield conn  # to the case
    await conn.close()  # cleanup


async def _seed(conn, *rows):  # seed helper
    """Insert waypoint rows and commit."""
    for sql, params in rows:  # each row
        await conn.execute(sql, params)  # one row
    await conn.commit()  # visible


@pytest.mark.asyncio
@pytest.mark.parametrize("rows,want", [
    ((DOCK_ON, DOCK_OFF, HOME), ("w-d1", "dock")),  # the on-route dock, not the closer off-route one
    ((DOCK_OFF, HOME), ("w-home", "home")),  # off-route docks never qualify -> home
    ((DOCK_OFF,), None),  # nothing qualifies
])
async def test_pick_target(geo, rows, want):  # dock vs home choice
    """MUTATION: drop the on-route filter -> w-d2 / no home fallback -> red."""
    await _seed(geo, *rows)  # the catalogue
    routes = await load_route_geoms(geo)  # as P3 reads them
    got = await lb.pick_target(geo, (35.0, 135.0), routes, 5.0)  # from the robot
    assert (None if got is None else (got.geo_id, got.kind)) == want  # chosen or none


@pytest.mark.asyncio
async def test_unreachable_compares_remaining_metres(geo):  # remaining range check
    """MUTATION: treat a missing remain_mile_km as 0 -> True -> red."""
    await _seed(geo, DOCK_ON)  # one dock, ~167 m away along the route
    routes = await load_route_geoms(geo)  # routes as P3 reads them
    tgt = await lb.pick_target(geo, (35.0, 135.0), routes, 5.0)  # the dock
    assert lb.unreachable(_power(19.0, remain_km=0.1), (35.0, 135.0), tgt, routes, 5.0) is True  # 100 m left, ~167 m to go
    assert lb.unreachable(_power(19.0, remain_km=1.0), (35.0, 135.0), tgt, routes, 5.0) is False  # 1 km left: enough
    assert lb.unreachable({"soc_pct": 19.0}, (35.0, 135.0), tgt, routes, 5.0) is None  # not judged


@pytest.mark.asyncio
@pytest.mark.parametrize("lat,lon,refused", [(35.0015, 135.00002, False), (35.0, 135.01, True)])
async def test_a_dock_off_every_route_is_refused(geo, lat, lon, refused):  # upsert path
    """99 U146 dock-on-route, cmd/geo upsert path. MUTATION: skip the check
    -> the off-route dock is stored -> red."""
    from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload  # the applier
    ctx = GeoContext(geo_conn=geo, fence_conn=geo, route_membership_m=5.0)  # as wired
    ack = await handle_geo_payload({  # an HMI create
        "cmd_id": "g-1", "action": "upsert", "type": "waypoint", "geo_id": "w-new",
        "origin": "hmi", "base_rev": 0,
        "obj": {"name": "3号充电站", "geom": {"lat": lat, "lon": lon}}}, ctx, now_ms=1)
    assert (ack["result"] == "rejected") is refused  # off-route refused
    if refused:  # the refusal names why
        assert ack["code"] == E_GEO_INVALID and ack["detail"]["reason"] == "dock_off_route"


@pytest.mark.asyncio
@pytest.mark.parametrize("lat,lon,refused", [(35.0015, 135.00002, False), (35.0, 135.01, True)])
async def test_renaming_into_a_dock_off_every_route_is_refused(geo, lat, lon, refused):  # rename path
    """99 U146, rename path: a poi renamed "X充电站" becomes a dock, so it must
    be on a route. MUTATION: skip the rename check -> stored -> red."""
    from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload  # the applier
    await _seed(geo, _wp("w-p", "旧点", "poi", lat, lon))  # a plain point
    ctx = GeoContext(geo_conn=geo, fence_conn=geo, route_membership_m=5.0)  # as wired
    ack = await handle_geo_payload({  # a voice rename
        "cmd_id": "g-2", "action": "rename", "type": "waypoint", "geo_id": "w-p",
        "origin": "voice", "base_rev": 1, "obj": {"name": "4号充电站"}}, ctx, now_ms=2)
    assert (ack["result"] == "rejected") is refused, ack  # off-route refused
    if refused:  # the refusal names why
        assert ack["code"] == E_GEO_INVALID and ack["detail"]["reason"] == "dock_off_route"


# -- through the real P3 loop ------------------------------------------------------------
LB = {"trigger_soc_pct": 20.0, "stop_soc_pct": 5.0, "cloud_alarm_period_s": 2.0}  # the rule


async def _feed(h, body, seconds):  # power frames at 2 Hz
    """state/power at 2 Hz for `seconds` (the chassis sends 1 Hz; faster is fine)."""
    from xbrain.p3_task.runtime.main_wiring import STATE_POWER_TOPIC  # CR-5 key
    for _ in range(int(seconds * 2)):  # frame count
        h.send(STATE_POWER_TOPIC, {"data": body})  # as the relay forwards it
        await asyncio.sleep(0.5)


@pytest.mark.asyncio
async def test_the_episode_end_to_end(tmp_path, monkeypatch):  # the whole episode
    """Low -> charge task to the on-route dock, alarms every 2 s with only the
    first marked; 5 % -> running task suspended, P1 cleared, prone sent,
    submits refused; charging -> the alarm closes and the low-battery task
    is cancelled (left suspended, "继续" would drive back to the dock).
    MUTATION: no task / no cadence / no prone / no refusal / alarm not closed
    / task left suspended -> red."""
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "gj-test")  # the prone envelope needs a rid
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, task_db = await start_p3(  # loop with the rule on
        tmp_path, monkeypatch, rows=[goto_row("t-run", "ready", 1)],
        geo_sql=[ROUTE_SQL, DOCK_ON, DOCK_OFF, HOME],
        route_membership_m=5.0, low_battery=LB)  # the rule on
    try:
        assert await wait_for(lambda: route_ops(h))  # t-run running
        await _feed(h, _power(18.0), 5.0)  # low for 5 s
        alarms = h.frames("event/alarm/charging")  # cloud reports
        lows = [a for a in alarms if a["detail"]["type"] == "low_battery"]  # the low reports
        assert 2 <= len(lows) <= 4  # every 2 s, not every frame
        assert [a["detail"]["first"] for a in lows][:2] == [True, False]  # one voice only
        assert lows[0]["detail"]["target_name"] == "1号充电站"  # the on-route dock
        tid = lows[0]["detail"]["task_id"]  # the injected task
        assert (await task_state(task_db, tid))[0] in ("ready", "running")  # injected
        await _feed(h, _power(4.0), 2.0)  # the last 5 %
        assert await wait_for(lambda: h.frames("cmd/chassis/ctrl"))  # prone sent
        ctrl = h.frames("cmd/chassis/ctrl")[0]  # the envelope
        assert ctrl["data"]["action"] == "prone" and ctrl["src"] == "p3_task"  # enveloped
        assert any(a["detail"]["type"] == "low_battery_stop" for a in h.frames("event/alarm/charging"))
        h.send(CMD_TASK_TOPIC, {"cmd_id": "c-x", "action": "submit", "source": "voice",  # a submit while lying down
                                "task": {"type": "goto", "priority": 5,
                                         "params": {"waypoint_id": "w-gate"}}})
        assert await wait_for(lambda: h.frames("cmd/task/ack"))  # answered
        assert h.frames("cmd/task/ack")[-1]["code"] == E_LOW_BATTERY  # refused
        await _feed(h, _power(4.0, charging=True), 1.0)  # docked and charging
        assert any(e["detail"]["type"] == "low_battery_cleared"
                   for e in h.frames("event/info/charging"))  # closed
        for _ in range(100):  # the close is written by the loop, poll it
            if (await task_state(task_db, tid))[0] == "cancelled":
                break  # closed
            await asyncio.sleep(0.02)  # give the loop a turn
        assert (await task_state(task_db, tid))[0] == "cancelled"  # the task is closed too
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_boot_reports_a_missing_home(tmp_path, monkeypatch):  # the boot report
    """MUTATION: drop the boot check -> no report -> red."""
    h, stop, loop, _db = await start_p3(tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_OFF],  # no home, one off-route dock
                                        route_membership_m=5.0, low_battery=LB)
    try:
        assert await wait_for(lambda: h.frames("event/alarm/charging"))  # reported at boot
        d = h.frames("event/alarm/charging")[0]["detail"]  # its detail
        assert d["type"] == "return_target_missing"  # the type
        assert d["missing"] == ["返航终点站", "充电桩/充电站"] and d["docks_off_route"] == ["2号充电站"]
    finally:
        await stop_p3(stop, loop)  # cleanup
