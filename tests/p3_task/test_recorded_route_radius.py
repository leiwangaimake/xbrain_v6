"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_recorded_route_radius.py
Brief: 99 U126 -- a teach-recorded route (mode B) is pushed with common.recording.arrive_radius_m

Description:
Until 2026-11-05 P3 passed recorded_arrive_radius_m=None to every route push,
so build_snapshot refused EVERY teach-recorded route (mode B dense polyline,
15 S2.4.3): a voice patrol on a recorded route was accepted and then failed
with E_GEO_INCOMPLETE. The user ruled the radius 1.0 m (99 U126); it lives
in common.recording.arrive_radius_m and reaches P3 through p3_task.yaml
route.recorded_arrive_radius_m. These cases pin the three links: the live P3
loop pushes a mode B route with that radius on every point; without the value
the push is still refused (no number is invented in code); and __main__
refuses to start when the resolved key is missing (EX_CONFIG 78).
"""
from __future__ import annotations  # postponed annotations

import asyncio  # the live P3 loop runs as a task
import json  # frames and mission_json

import aiosqlite  # task.db / geo.db on disk for the live loop
import pytest  # parametrize, marker

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO  # seed a ready patrol
from xbrain.p3_task.persistence.schema_geo import GEO_DB_STATEMENTS  # geo.db DDL

pytestmark = pytest.mark.no_device  # fake zenoh session, no robot


def _fake_planes(monkeypatch, subs, pubs):
    """Replace open_planes with a fake session that records keys and frames."""
    class _WPub:  # one declared publisher
        def __init__(self, key):  # remember the key
            self.key = key  # the key it publishes on
            pubs.setdefault(key, [])  # declared, maybe never used

        def put(self, payload):  # a frame on this key
            pubs[self.key].append(payload)  # record the frame

    class _Session:  # the fake zenoh session
        def declare_subscriber(self, key, cb):  # subscriber registration
            subs[key] = cb  # the test can feed it
            return object()  # a handle to hold

        def declare_publisher(self, key):  # publisher registration
            return _WPub(key)  # recording publisher

        def declare_queryable(self, key, cb):  # queryable registration
            subs[key] = cb  # same table
            return object()  # a handle to hold

        def put(self, key, payload):  # session-level put
            pubs.setdefault(key, []).append(payload)  # session.put path

    class _Planes:  # open_planes() context manager
        def __enter__(self):  # enter: hand out the session
            return _Session()  # the session p3 uses

        def __exit__(self, *_a):  # exit: propagate errors
            return False  # do not swallow errors

    import xbrain.common.runtime.session_ctx as sctx  # where p3 imports it from
    monkeypatch.setattr(sctx, "open_planes", lambda _p: _Planes())  # fake it


@pytest.mark.asyncio  # the live loop is async
@pytest.mark.parametrize("radius,want_frames", [(1.0, True), (None, False)])  # configured value vs none
async def test_a_recorded_route_is_pushed_with_the_configured_radius(
        tmp_path, monkeypatch, radius, want_frames):
    """MUTATION: pass None again at the RP-1 call site (the pre-U126 shape) ->
    the 1.0 case pushes nothing -> red; invent a radius in code -> the None
    case pushes -> red."""
    from xbrain.p3_task.runtime import main_wiring as mw  # the live loop

    subs, pubs = {}, {}  # what the fake session saw
    _fake_planes(monkeypatch, subs, pubs)  # no zenoh
    task_db, geo_db = str(tmp_path / "task.db"), str(tmp_path / "geo.db")  # on disk
    async with aiosqlite.connect(geo_db) as g:  # a recorded route, as teach commits it
        for stmt in GEO_DB_STATEMENTS:  # the geo.db schema
            await g.execute(stmt)  # one DDL statement
        await g.execute(  # a mode B route row
            "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"  # inline polyline, no anchors
            " total_len_m, rev, content_hash, updated_ms)"  # as commit_route writes it
            " VALUES ('r-rec', '1号路径', ?, 'oneway', 'forward', 22.0, 1, 'h', 0)",  # oneway, 22 m
            (json.dumps([[35.0, 135.0], [35.0001, 135.0], [35.0002, 135.0]]),))  # three [lat, lon] points
        await g.commit()  # visible to the loop
    stop = {"stop": False}  # loop control
    task = asyncio.create_task(mw._amain(  # start P3
        stop, 99.0, task_db, geo_db_path=geo_db,  # heartbeat, dbs
        fence_db_path=str(tmp_path / "fence.db"),  # fence db unused here
        recorded_arrive_radius_m=radius))  # the value under test
    try:  # always stop the loop
        for _ in range(300):  # wait for the subscribers
            if mw.CMD_TASK_TOPIC in subs:  # subscribers declared
                break  # loop is up
            await asyncio.sleep(0.01)  # poll gently
        async with aiosqlite.connect(task_db) as t:  # a ready patrol on it
            await TasksDAO(t).insert(TaskRow(  # insert directly: admission is out of scope
                task_id="t-rec", task_type="patrol", state="ready",  # a ready patrol
                priority=5, submit_seq=1, mission_json="{}", total_steps=1,  # first in queue
                current_step=0, step_status_json="[]", created_ms=0,  # one step
                updated_ms=0, source="local", trace_id="tr",  # local operator
                resume_policy="continue", route_geo_id="r-rec"))  # on the recorded route
            await t.commit()  # visible to the scheduler
        for _ in range(200):  # give the dispatch a few passes
            if pubs.get("cmd/motion/route"):  # a frame arrived
                break  # done waiting
            await asyncio.sleep(0.02)  # next pass
        frames = [json.loads(p.decode("utf-8"))  # decode every frame
                  for p in pubs.get("cmd/motion/route", [])]  # what reached P1
        if want_frames:  # radius configured
            assert frames and frames[0]["route_id"] == "r-rec"  # pushed
            assert {p["arrive_radius_m"] for p in frames[0]["points"]} == {1.0}  # every point
        else:  # radius not configured
            assert not frames  # refused, no number invented
    finally:  # stop the loop either way
        stop["stop"] = True  # ask it to stop
        await asyncio.wait_for(task, timeout=10.0)  # and wait for it


@pytest.mark.parametrize("route,rc", [({"membership_m": 5.0}, 78),  # missing radius
                                      ({"recorded_arrive_radius_m": None, "membership_m": 5.0}, 78),  # null radius
                                      ({"recorded_arrive_radius_m": 0.0, "membership_m": 5.0}, 78),  # zero is not a radius
                                      ({"recorded_arrive_radius_m": 1.0}, 78),  # 99 U127: missing membership
                                      ({"recorded_arrive_radius_m": 1.0, "membership_m": 0.0}, 78),  # zero membership
                                      ({"recorded_arrive_radius_m": 1.0, "membership_m": 5.0}, 0)])  # the shipped values
def test_main_refuses_to_start_without_the_radius(monkeypatch, route, rc):
    """EX_CONFIG 78 with the key path when the resolved value is missing / null
    / not positive; 1.0 reaches run_voice_loop_wiring. MUTATION: drop the check
    -> the null cases start and every recorded patrol fails later."""
    import xbrain.common.config.resolved as resolved  # where main imports it from
    import xbrain.p3_task.runtime.main_wiring as mw  # the wiring main calls
    from xbrain.p3_task import __main__ as p3main  # unit under test

    cfg = {"geo": {"enu_origin": {"lat": 31.0, "lon": 121.0, "alt": 0.0}},  # minimal resolved config
           "teach": {"snap_max_age_ms": 8000, "min_dist_m": 0.5, "sample_hz": 1.0},  # U132 pair
           "route": route,  # a resolved p3 config
           "rtk_loss": {"lost_sustain_s": 1.0, "recover_sustain_s": 2.0},  # 99 U142 pair
           "low_battery": {"trigger_soc_pct": 20.0, "stop_soc_pct": 5.0,
                           "cloud_alarm_period_s": 2.0,  # 99 U146 rule
                           "charge_full_soc_pct": 100.0,  # 99 U157
                           "swap_resume_soc_pct": 80.0,
                           "swap_absent_min_s": 5.0, "cold_swap_min_rise_pct": 10.0}}
    monkeypatch.setattr(resolved, "load_resolved", lambda *_a, **_k: cfg)  # no file
    seen = {}  # what the wiring received
    monkeypatch.setattr(mw, "run_voice_loop_wiring",  # replace the loop
                        lambda **kw: seen.update(kw) or 0)  # do not run the loop
    assert p3main.main(["--voice-loop"]) == rc  # the exit code
    if rc == 0:  # the started case
        assert seen["recorded_arrive_radius_m"] == 1.0  # passed through
        assert seen["route_membership_m"] == 5.0  # 99 U127, passed through
