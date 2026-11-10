"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_low_battery.py
Brief: 99 U146/U157 low-battery return -- episode phases, held tasks, dock / home choice, the injected task, 5 % stop, cloud alarms, dock-on-route

Description:
The user's low-battery rule (2026-11-05 ruling 3, restated 2026-11-06):
below 20 % one voice alarm and a cloud alarm every 2 s; on the charger the
alarm goes on worded "charging" until 100 %, a swap ends it at 80 % (99 U154
/ U157), and the tasks the episode interrupted or held resume only then;
drive (normal speed) to
the nearest 充电桩 / 充电站 point
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
RULE = (20.0, 5.0, 2.0, 100.0, 80.0)  # trigger, stop, period, charge_full, swap_resume (U157)


def _mon():  # the shipped rule
    """A monitor with the shipped thresholds (configs/p3_task.yaml)."""
    return lb.LowBatteryMonitor(*RULE)


def test_low_starts_once_and_alarms_every_period():  # trigger and cadence
    """MUTATION: no cadence -> an alarm every frame -> red; '<=' 20 -> 20.0
    starts an episode -> red."""
    m = _mon()
    assert m.update(_power(20.0), 0.0) == []  # 20 % is not below 20
    assert m.update(_power(19.0), 1.0) == ["low", "alarm"]  # starts, first alarm
    assert m.update(_power(19.0), 2.0) == []  # 1 s later: not due
    assert m.update(_power(19.0), 3.0) == ["alarm"]  # 2 s after the last
    assert m.update(_power(25.0), 4.0) == []  # soc up without charging: still on


@pytest.mark.parametrize("body", [_power(19.0, charging=True), _power(19.0, charge="charging")])
def test_charging_turns_the_alarm_into_charging_not_an_end(body):  # either charging signal
    """99 U125: either the battery flag or the six-state field starts the
    charging phase. 99 U157: charging no longer ENDS the episode -- the alarm
    goes on (worded by phase) at once and every period after. MUTATION: read
    one of the two signals only -> red for the other; end the episode on
    charging (the U146 rule) -> `low` false -> red."""
    m = _mon()
    m.update(_power(19.0), 0.0)  # an episode
    assert m.update(body, 1.0) == ["charging", "alarm"]  # phase change, alarm at once
    assert m.low and m.phase == lb.PHASE_CHARGING  # still an episode
    assert m.update(body, 2.0) == []  # not due
    assert m.update(body, 3.0) == ["alarm"]  # the charging alarm keeps its cadence


@pytest.mark.parametrize("soc,ends", [(99.0, False), (100.0, True)])
def test_the_charging_alarm_ends_only_at_charge_full(soc, ends):  # U157 charge_full_soc_pct
    """MUTATION: '>' instead of '>=' -> 100 does not end -> red; end on any
    charging soc -> 99 ends -> red."""
    m = _mon()
    m.update(_power(19.0), 0.0)
    m.update(_power(30.0, charging=True), 1.0)  # charging phase
    got = m.update(_power(soc, charging=True), 4.0)
    assert (got == ["charge_full"]) is ends and m.low is not ends


def test_unplugged_before_full_holds_or_drives_back():  # U157 ruling 2
    """Unplugged at or above the trigger -> interrupted (still alarmed);
    drained below it -> low again (a new return task); full while unplugged
    ends it. MUTATION: end on unplug -> red; skip the interrupted -> low
    re-entry -> no second "low" -> red."""
    m = _mon()
    m.update(_power(19.0), 0.0)
    m.update(_power(40.0, charging=True), 1.0)  # charging
    assert m.update(_power(60.0), 2.0) == ["interrupted", "alarm"]  # pulled at 60 %
    assert m.phase == lb.PHASE_INTERRUPTED and m.low
    assert m.update(_power(19.0), 5.0)[:1] == ["low"]  # drained: back to low
    m.update(_power(19.0, charging=True), 6.0)  # replugged
    assert m.update(_power(15.0), 7.0)[:1] == ["low"]  # pulled again while low: low
    m.update(_power(50.0, charging=True), 8.0)
    assert m.update(_power(100.0), 10.0) == ["charge_full"]  # unplugged at full: ends


def test_the_five_percent_latch_rearms_after_charging():  # stop once per low phase
    """The 5 % pause lifts on the charger (U146) and a later low phase may stop
    again. MUTATION: keep `stopped` across the charging phase -> no second
    stop -> red."""
    m = _mon()
    assert "stop" in m.update(_power(4.0), 0.0)
    m.update(_power(4.0, charging=True), 1.0)
    assert not m.stopped  # the dispatch pause is lifted on the charger
    assert m.update(_power(4.0), 2.0)[:2] == ["low", "stop"]  # unplugged at 4 %


# -- the battery swap (99 U154 / U157) --------------------------------------------------
def _packs(soc, *packs):  # a power frame with explicit slots
    """A state/power body whose `list` has one entry per (present, level) slot;
    soc is soc_pct as quadruped computes it (min over present slots, 13 V-68)."""
    return {"soc_pct": soc, "remain_mile_km": 5.0, "charge": "idle",  # 11 S4.2 fields
            "list": [{"index": i, "level_pct": lvl, "charging": False, "present": pres}
                     for i, (pres, lvl) in enumerate(packs)]}


def test_a_swap_closes_the_episode_once_the_level_is_back():  # pulled, refilled, enough
    """User ruling 2026-11-10: a battery swap is a lawful end. MUTATION: no
    swap rule -> still low -> red; close while the slot is still empty -> red."""
    m = _mon()
    assert m.update(_packs(4.0, (True, 4.0)), 0.0) == ["low", "stop", "alarm"]  # low and lying down
    assert m.update(_packs(None, (False, 0.0)), 1.0) == []  # pack out: no soc, nothing closes
    assert m.update(_packs(90.0, (True, 90.0)), 3.0) == ["swapped"]  # a full pack in
    assert not m.low and not m.stopped  # the dispatch pause is lifted with it


def test_soc_back_without_a_swap_does_not_close():  # a wobble is not a swap
    """MUTATION: drop the presence condition -> 60 % closes it -> red."""
    m = _mon()
    m.update(_packs(19.0, (True, 19.0)), 0.0)  # an episode
    got = m.update(_packs(60.0, (True, 60.0)), 1.0)
    assert not {"swapped", "swapped_partial"} & set(got) and m.low  # never pulled out


@pytest.mark.parametrize("soc,want", [(19.0, None), (20.0, "swapped_partial"),
                                      (79.0, "swapped_partial"), (80.0, "swapped")])
def test_a_swap_resumes_at_swap_resume_and_parks_below(soc, want):  # U154 floor, U157 split
    """soc_pct is the minimum over present packs: swapping one of two low
    packs leaves the other low. Below the trigger nothing ends; from the
    trigger to swap_resume the alarm ends but the tasks wait for a person;
    at swap_resume it ends and they resume. MUTATION: '>' for '>=' at either
    boundary -> 20 or 80 lands in the wrong bucket -> red; one threshold for
    both -> 79 resumes -> red."""
    m = _mon()
    m.update(_packs(15.0, (True, 15.0), (True, 15.0)), 0.0)  # two low packs
    m.update(_packs(15.0, (False, 0.0), (True, 15.0)), 1.0)  # slot 0 pulled
    got = m.update(_packs(soc, (True, 95.0), (True, soc)), 2.0)  # slot 0 refilled, slot 1 at soc
    if want is None:
        assert not {"swapped", "swapped_partial"} & set(got) and m.low  # still low
    else:
        assert got == [want] and not m.low  # ended, in the right bucket


def test_a_slot_empty_when_the_episode_starts_counts():  # single_battery, the second slot filled
    """A pack put into a slot that was already empty is a swap too. MUTATION:
    no presence tracking on the starting frame -> never closes -> red."""
    m = _mon()
    m.update(_packs(18.0, (False, 0.0), (True, 18.0)), 0.0)  # starts with slot 0 empty
    assert m.update(_packs(85.0, (True, 95.0), (True, 85.0)), 1.0) == ["swapped"]  # slot 0 filled


def test_a_swap_during_charging_ends_it_too():  # any episode phase
    """MUTATION: judge swaps in the low phase only -> red."""
    m = _mon()
    m.update(_packs(18.0, (True, 18.0)), 0.0)
    m.update(dict(_packs(30.0, (True, 30.0)), charge="charging"), 1.0)  # charging
    m.update(dict(_packs(None, (False, 0.0)), charge="charging"), 2.0)  # pulled
    assert m.update(_packs(90.0, (True, 90.0)), 3.0) == ["swapped"]


def test_swap_history_belongs_to_one_episode():  # reset on close
    """MUTATION: keep the presence history after the close -> the next
    episode closes on a wobble -> red."""
    m = _mon()
    m.update(_packs(18.0, (True, 18.0)), 0.0)  # episode 1
    m.update(_packs(None, (False, 0.0)), 1.0)  # pulled
    assert m.update(_packs(90.0, (True, 90.0)), 2.0) == ["swapped"]  # swapped
    assert "low" in m.update(_packs(19.0, (True, 19.0)), 3.0)  # episode 2, no swap since
    got = m.update(_packs(25.0, (True, 25.0)), 4.0)
    assert not {"swapped", "swapped_partial"} & set(got) and m.low  # a wobble only


def test_stop_once_at_five_percent():  # the 5 % latch
    """MUTATION: '<' 5 -> 5.0 does not stop -> red; no once-latch -> red."""
    m = _mon()
    m.update(_power(19.0), 0.0)  # episode
    assert "stop" in m.update(_power(5.0), 3.0)  # the last 5 %
    assert "stop" not in m.update(_power(4.0), 6.0)  # once


@pytest.mark.parametrize("args", [
    (None, 5.0, 2.0, 100.0, 80.0), (20.0, 0.0, 2.0, 100.0, 80.0),
    (20.0, 20.0, 2.0, 100.0, 80.0), (20.0, 5.0, 0, 100.0, 80.0),
    (20.0, 5.0, 2.0, None, 80.0), (20.0, 5.0, 2.0, 100.0, None),
    (20.0, 5.0, 2.0, 101.0, 80.0),   # full could never be reached
    (20.0, 5.0, 2.0, 70.0, 80.0),    # swap_resume above full
    (20.0, 5.0, 2.0, 100.0, 20.0)])  # swap_resume not above the trigger
def test_thresholds_are_required_and_ordered(args):  # config refusals
    """CLAUDE.md 3.1 + U157 order stop < trigger < swap_resume <= full <= 100.
    MUTATION: drop any one comparison in check_thresholds -> its row -> red."""
    with pytest.raises(lb.LowBatteryConfigError):  # each bad tuple
        lb.LowBatteryMonitor(*args)  # refused


def test_equal_swap_resume_and_full_is_allowed():  # the '<=' ends of the order
    """MUTATION: '<' for '<=' -> a legal 100/100 config refused -> red."""
    lb.LowBatteryMonitor(20.0, 5.0, 2.0, 100.0, 100.0)


# -- __main__ (U157 keys) ---------------------------------------------------------------
_FULL_LB = {"trigger_soc_pct": 20.0, "stop_soc_pct": 5.0, "cloud_alarm_period_s": 2.0,
            "charge_full_soc_pct": 100.0, "swap_resume_soc_pct": 80.0}


@pytest.mark.parametrize("patch,rc", [
    ({}, 0), ({"charge_full_soc_pct": None}, 78), ({"swap_resume_soc_pct": None}, 78),
    ({"charge_full_soc_pct": 99.0, "swap_resume_soc_pct": 99.5}, 78),  # swap_resume above full
    ({"charge_full_soc_pct": 99.0}, 0)])  # a chassis that tops out at 99 %
def test_main_refuses_without_the_u157_keys(monkeypatch, patch, rc):
    """EX_CONFIG 78 by key path, and the five values reach the wiring.
    MUTATION: drop either new key from the __main__ check, or the order check
    -> its row starts -> red; forget to pass a key -> KeyError / wrong dict
    -> red."""
    import xbrain.common.config.resolved as resolved  # where main loads from
    import xbrain.p3_task.runtime.main_wiring as mw  # the wiring main calls
    from xbrain.p3_task import __main__ as p3main  # unit under test
    cfg = {"geo": {"enu_origin": {"lat": 31.0, "lon": 121.0, "alt": 0.0}},  # minimal resolved config
           "teach": {"snap_max_age_ms": 8000, "min_dist_m": 0.5, "sample_hz": 1.0},
           "route": {"recorded_arrive_radius_m": 1.0, "membership_m": 5.0},
           "rtk_loss": {"lost_sustain_s": 1.0, "recover_sustain_s": 2.0},
           "low_battery": dict(_FULL_LB, **patch)}
    monkeypatch.setattr(resolved, "load_resolved", lambda *_a, **_k: cfg)  # no file
    seen = {}  # what the wiring received
    monkeypatch.setattr(mw, "run_voice_loop_wiring", lambda **kw: seen.update(kw) or 0)
    assert p3main.main(["--voice-loop"]) == rc  # the exit code
    if rc == 0:
        assert seen["low_battery"] == dict(_FULL_LB, **patch)  # all five, as configured


# -- the persisted episode (U157) -------------------------------------------------------
def test_episode_record_round_trips_and_garbage_holds_nothing():
    """MUTATION: decode an idle / unreadable row as an episode -> red."""
    raw = lb.encode_episode(lb.PHASE_CHARGING, {"t-2", "t-1"}, None)
    assert lb.decode_episode(raw) == (lb.PHASE_CHARGING, {"t-1", "t-2"}, None)
    assert lb.decode_episode(lb.encode_episode(lb.PHASE_IDLE, set(), None)) is None
    assert lb.decode_episode(b"{not json") is None and lb.decode_episode(None) is None
    assert lb.decode_episode(b'{"phase":"bogus","held":[]}') is None


def test_restore_rejects_an_unknown_phase_and_restarts_the_cadence():
    """MUTATION: restore without resetting the cadence -> no alarm on the first
    frame after a restart -> red."""
    m = _mon()
    with pytest.raises(ValueError):
        m.restore("nap")
    m.restore(lb.PHASE_CHARGING)
    assert m.update(_power(50.0, charging=True), 0.0) == ["alarm"]  # no phase event, alarm at once


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
LB = {"trigger_soc_pct": 20.0, "stop_soc_pct": 5.0, "cloud_alarm_period_s": 2.0,  # the rule
      "charge_full_soc_pct": 100.0, "swap_resume_soc_pct": 80.0}  # U157


async def _feed(h, body, seconds):  # power frames at 2 Hz
    """state/power at 2 Hz for `seconds` (the chassis sends 1 Hz; faster is fine)."""
    from xbrain.p3_task.runtime.main_wiring import STATE_POWER_TOPIC  # CR-5 key
    for _ in range(int(seconds * 2)):  # frame count
        h.send(STATE_POWER_TOPIC, {"data": body})  # as the relay forwards it
        await asyncio.sleep(0.5)


async def _poll_state(task_db, tid, want, n=100):  # the loop writes, poll it
    """True once task `tid` reaches state `want` (committed)."""
    for _ in range(n):
        if (await task_state(task_db, tid))[0] == want:
            return True
        await asyncio.sleep(0.02)  # give the loop a turn
    return False


@pytest.mark.asyncio
async def test_the_episode_end_to_end(tmp_path, monkeypatch):  # the whole episode
    """Low -> charge task to the on-route dock preempts t-run, alarms every 2 s
    with only the first marked; 5 % -> running task suspended, P1 cleared,
    prone sent, submits refused; charging -> the low-battery task is
    cancelled, the alarm goes on worded battery_charging and t-run STAYS
    suspended (99 U157 -- until 2026-11-10 it resumed here); 100 % -> cleared
    with cause charge_full and t-run resumes by itself.
    MUTATION: no task / no cadence / no prone / no refusal / task left
    suspended -> red; end the episode on charging, or drop the held filter in
    the scheduler -> t-run running while charging -> red; never end at full
    -> no cleared / t-run never resumes -> red."""
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
        assert (await task_state(task_db, "t-run"))[:3] == ("suspended", "yielding", "preempted")
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
        await _feed(h, _power(40.0, charging=True), 3.0)  # docked and charging
        assert await _poll_state(task_db, tid, "cancelled")  # the return task is closed
        charging = [a for a in h.frames("event/alarm/charging")
                    if a["detail"]["type"] == "battery_charging"]
        assert len(charging) >= 1 and not any(a["detail"]["first"] for a in charging)  # never voiced
        assert not [e for e in h.frames("event/info/charging")
                    if e["detail"]["type"] == "low_battery_cleared"]  # not ended by charging
        assert (await task_state(task_db, "t-run"))[0] == "suspended"  # held on the charger
        await _feed(h, _power(100.0, charging=True), 1.0)  # full
        cleared = [e for e in h.frames("event/info/charging")
                   if e["detail"]["type"] == "low_battery_cleared"]
        assert len(cleared) == 1 and cleared[0]["detail"]["cause"] == "charge_full"
        assert cleared[0]["detail"]["auto_resume"] is True and "t-run" in cleared[0]["detail"]["held"]
        assert await _poll_state(task_db, "t-run", "running", n=300)  # resumed by itself
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_queued_task_waits_for_the_episode_and_a_human_resume_does_not(
        tmp_path, monkeypatch):  # U157 ruling 3 + the manual release
    """A task queued before the episode does not dispatch when the return task
    arrives at the dock (it waits for the end); a held task a person resumes
    leaves the hold and runs. MUTATION: hold preempted tasks only -> t-q
    dispatches on arrival -> red; ignore the resume release -> t-run stays
    suspended -> red."""
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, rows=[goto_row("t-run", "ready", 1),
                                     goto_row("t-q", "pending", 2)],
        geo_sql=[ROUTE_SQL, DOCK_ON, HOME], route_membership_m=5.0, low_battery=LB)
    try:
        assert await wait_for(lambda: route_ops(h))  # t-run running
        await _feed(h, _power(18.0), 2.0)  # low: t-run preempted
        tid = h.frames("event/alarm/charging")[0]["detail"]["task_id"]
        async with aiosqlite.connect(task_db) as t:  # the return task arrives
            await t.execute("UPDATE tasks SET state='done' WHERE task_id=?", (tid,))
            await t.commit()
        await _feed(h, _power(18.0), 2.0)  # a few ticks with nothing running
        assert (await task_state(task_db, "t-q"))[0] in ("pending", "ready")  # held
        assert (await task_state(task_db, "t-run"))[0] == "suspended"  # held
        h.send(CMD_TASK_TOPIC, {"cmd_id": "c-r", "action": "resume", "source": "hmi",
                                "task_id": "t-run"})  # a person resumes it
        assert await _poll_state(task_db, "t-run", "running", n=300)  # their call
        assert (await task_state(task_db, "t-q"))[0] in ("pending", "ready")  # t-q still held
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_battery_swap_ends_the_episode_end_to_end(tmp_path, monkeypatch):  # the swap path
    """99 U154 / U157: lying down at 4 %, the pack is pulled and a 95 % one put
    in -> low_battery_cleared with cause battery_swap and auto_resume, the
    low-battery task cancelled, and submits no longer refused with
    E_LOW_BATTERY. MUTATION: "swapped" not wired -> no clear / task left /
    still refused -> red."""
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "gj-test")  # the prone envelope needs a rid
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, task_db = await start_p3(  # loop with the rule on
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)  # the rule on
    try:
        await _feed(h, _packs(4.0, (True, 4.0)), 2.0)  # low and the last 5 % at once
        assert await wait_for(lambda: h.frames("cmd/chassis/ctrl"))  # prone sent
        tid = h.frames("event/alarm/charging")[0]["detail"]["task_id"]  # the injected task
        await _feed(h, _packs(None, (False, 0.0)), 1.0)  # the pack pulled out
        assert not h.frames("event/info/charging")  # an empty slot closes nothing
        await _feed(h, _packs(95.0, (True, 95.0)), 1.0)  # a full pack put in
        cleared = [e for e in h.frames("event/info/charging")
                   if e["detail"]["type"] == "low_battery_cleared"]  # the close report
        assert len(cleared) == 1 and cleared[0]["detail"]["cause"] == "battery_swap"  # once, by swap
        assert cleared[0]["detail"]["auto_resume"] is True
        assert await _poll_state(task_db, tid, "cancelled")  # the task is closed too
        h.send(CMD_TASK_TOPIC, {"cmd_id": "c-y", "action": "submit", "source": "voice",  # a submit after the swap
                                "task": {"type": "goto", "priority": 5,
                                         "params": {"waypoint_id": "w-home"}}})
        assert await wait_for(lambda: [a for a in h.frames("cmd/task/ack") if a["cmd_id"] == "c-y"])
        ack = [a for a in h.frames("cmd/task/ack") if a["cmd_id"] == "c-y"][-1]  # its answer
        assert ack["code"] != E_LOW_BATTERY  # the 5 % refusal is lifted
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_partial_swap_parks_the_interrupted_task_for_a_person(
        tmp_path, monkeypatch):  # U157 ruling 1
    """Swapped to 60 %: the alarm ends (auto_resume false) and the preempted
    task is re-tagged passive / low_battery -- it never resumes by itself.
    MUTATION: treat 60 % as a full swap, or skip the re-tag -> t-run running
    -> red."""
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, rows=[goto_row("t-run", "ready", 1)],
        geo_sql=[ROUTE_SQL, DOCK_ON, HOME], route_membership_m=5.0, low_battery=LB)
    try:
        assert await wait_for(lambda: route_ops(h))  # t-run running
        await _feed(h, _packs(18.0, (True, 18.0)), 2.0)  # low: t-run preempted
        await _feed(h, _packs(None, (False, 0.0)), 1.0)  # pulled
        await _feed(h, _packs(60.0, (True, 60.0)), 2.0)  # a 60 % pack in
        cleared = [e for e in h.frames("event/info/charging")
                   if e["detail"]["type"] == "low_battery_cleared"]
        assert len(cleared) == 1 and cleared[0]["detail"]["auto_resume"] is False
        assert cleared[0]["detail"]["parked"] == ["t-run"]
        await _feed(h, _packs(60.0, (True, 60.0)), 1.0)  # ticks with nothing running
        assert (await task_state(task_db, "t-run")) == ("suspended", "passive", "low_battery")
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_restart_mid_charge_keeps_holding(tmp_path, monkeypatch):  # the memory row
    """P3 restarted while charging: the episode comes back from task.db memory,
    the preempted task stays suspended, and full charge resumes it.
    MUTATION: skip the restore -> nothing held -> t-run resumes at once -> red."""
    from xbrain.p3_task.dao.simple_daos import MemoryDAO
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
    task_db = str(tmp_path / "task.db")
    async with aiosqlite.connect(task_db) as t:  # what the previous P3 left behind
        for stmt in ALL_DDL_STATEMENTS:
            await t.execute(stmt)
        await TasksDAO(t).insert(goto_row("t-run", "suspended", 1,
                                          suspend_kind="yielding", suspend_reason="preempted"))
        await MemoryDAO(t).put(lb.EPISODE_MEMORY_KEY,
                               lb.encode_episode(lb.PHASE_CHARGING, {"t-run"}, None), 0)
        await t.commit()
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, geo_sql=[ROUTE_SQL, DOCK_ON, HOME],
        route_membership_m=5.0, low_battery=LB)
    try:
        await _feed(h, _power(50.0, charging=True), 2.0)  # still charging after the restart
        assert (await task_state(task_db, "t-run"))[0] == "suspended"  # held
        assert any(a["detail"]["type"] == "battery_charging"
                   for a in h.frames("event/alarm/charging"))  # the phase came back
        await _feed(h, _power(100.0, charging=True), 1.0)  # full
        assert await _poll_state(task_db, "t-run", "running", n=300)  # resumed
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
