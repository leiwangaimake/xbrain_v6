"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_patrol_laps.py
Brief: 99 U131 -- multi-pass patrols counted by P3, and B04 repeat of the latest completed patrol

Description:
The user's rulings of 2026-11-05 (99 U131), each pinned by a case that names
the mutant it kills:
  laps: one pass = one push of the U127 plan; "N圈" = N passes; "来回N次"
    = 2N passes on an open route (A -> B -> A is ONE round trip) and N on a
    ring; loops 0 is the endless patrol. Validated at admission (V-8a).
  P3 counts passes: an arrived pass with passes left advances loop_index,
    re-pushes (RP-5) and keeps the task running; the old pass's re-sent
    arrived frames are ignored until the next pass is seen running; the row's
    loop columns are P3's, never the frame's; dist_done_m is cumulative.
  B04: a route-less patrol_repeat gets the route of the latest DONE patrol
    at submit; none -> E_NOT_FOUND with reason no_completed_patrol.
The last case drives the real P3 loop (_amain on a fake session) through two
passes, because every unit above would stay green with the sink's next_pass
left unwired in main_wiring.
"""
from __future__ import annotations  # postponed annotations

import time  # P1-like mono stamps

import asyncio  # the live loop runs as a task
import json  # wire frames

import aiosqlite  # in-memory dbs
import pytest  # parametrize, marker
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_NOT_FOUND, E_SCHEMA  # expected codes
from xbrain.p3_task.dao.simple_daos import PatrolProgressDAO, SnapshotDAO  # rows under test
from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO  # seed tasks
from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload  # submit path
from xbrain.p3_task.ingest.task_command import TaskCommandError, parse_task_command  # admission
from xbrain.p3_task.persistence.schema_geo import GEO_DB_STATEMENTS  # geo.db DDL
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task.db DDL
from xbrain.p3_task.route.laps import LapSpec, lap_spec_from_params  # under test
from xbrain.p3_task.route.push import RouteAckWindow, RoutePushTrigger  # push dependencies
from xbrain.p3_task.runtime.progress_sink import apply_path_progress, predates_push  # under test
from xbrain.p3_task.runtime.route_push_runtime import push_route_for_task  # under test
from xbrain.p3_task.state.path_progress import ProgressTracker  # PP-1 tracker
from tests.p3_task.test_route_plan import LINE, ll, xy  # the 200 m line and its metric plane

pytestmark = pytest.mark.no_device  # pure logic + in-memory sqlite


# -- the count rule ---------------------------------------------------------
@pytest.mark.parametrize("loops,rt,mode,want", [  # (said, mode) -> pushes
    (1, False, "oneway", 1), (3, False, "oneway", 3),  # plain laps on a line
    (1, True, "oneway", 2), (3, True, "oneway", 6),  # round trips on a line: A-B-A each
    (3, True, "closed", 3), (3, False, "closed", 3),  # a ring has no "back"
    (2, True, "pingpong", 4), (0, True, "oneway", 0),  # pingpong is a line; 0 stays endless
])
def test_passes_follow_the_ruling(loops, rt, mode, want):  # U131 rule
    """MUTATION: ignore round_trip -> (1, True, oneway) gives 1 -> red;
    double on a ring too -> (3, True, closed) gives 6 -> red."""
    assert LapSpec(loops=loops, round_trip=rt).passes(mode) == want  # pushes


def test_lap_spec_defaults_and_reads_the_slots():  # 16 S6.7.3 [3]
    """loops not said -> 1; said -> as said. MUTATION: default 0 -> an
    unsaid count would be an endless patrol -> red."""
    assert lap_spec_from_params({"slots": {}}) == LapSpec(1, False)  # nothing said
    assert lap_spec_from_params(None) == LapSpec(1, False)  # no params at all
    assert lap_spec_from_params({"slots": {"loops": 3, "round_trip": True}}) == LapSpec(3, True)  # as said


@pytest.mark.parametrize("slots", [  # every value admission must refuse
    {"loops": "3"}, {"loops": True}, {"loops": -1}, {"loops": 100},  # not a count
    {"round_trip": "yes"}, {"round_trip": 1},  # not a bool
])
def test_bad_lap_slots_are_refused_at_admission(slots):  # V-8a
    """MUTATION: drop the admission check -> parse accepts -> red."""
    payload = {"cmd_id": "c1", "action": "submit", "source": "voice",  # 11 S7.2
               "task": {"type": "patrol", "route_id": "r-a", "params": {"slots": slots}}}  # a patrol
    with pytest.raises(TaskCommandError) as ei:  # refused
        parse_task_command(payload)  # admission
    assert ei.value.code == E_SCHEMA  # V-8a code


def test_only_patrol_repeat_may_omit_the_route():  # 11 S7.2 exception
    """MUTATION: accept any route-less patrol -> the second parse passes -> red."""
    base = {"cmd_id": "c1", "action": "submit", "source": "voice"}  # envelope
    parse_task_command({**base, "task": {"type": "patrol", "params": {"intent": "patrol_repeat"}}})  # B04: ok
    with pytest.raises(TaskCommandError):  # a plain patrol still needs one
        parse_task_command({**base, "task": {"type": "patrol", "params": {"intent": "patrol_route"}}})  # B02


# -- the db half --------------------------------------------------------------
@pytest_asyncio.fixture  # per-test task.db
async def db():  # in memory
    async with aiosqlite.connect(":memory:") as c:  # fresh
        for stmt in ALL_DDL_STATEMENTS:  # schema
            await c.execute(stmt)  # one DDL
        await c.commit()  # ready
        yield c  # to the test


async def _seed_task(conn, task_id, state, route, finished_at, seq):  # one past patrol
    await TasksDAO(conn).insert(TaskRow(  # a row as the scheduler leaves it
        task_id=task_id, task_type="patrol", state=state, priority=5,  # finished patrol
        submit_seq=seq, mission_json="{}", total_steps=1, current_step=0,  # one step
        step_status_json="[]", created_ms=0, updated_ms=0, source="local",  # local
        trace_id="tr", resume_policy="continue", route_geo_id=route))  # its route
    await conn.execute("UPDATE tasks SET finished_at=? WHERE task_id=?",  # when it ended
                       (finished_at, task_id))  # ISO wall time
    await conn.commit()  # visible


def _repeat_payload():  # what P4 sends for "再巡一圈"
    return {"cmd_id": "c-rep", "action": "submit", "source": "voice",  # 11 S7.2
            "task": {"type": "patrol", "params": {"intent": "patrol_repeat", "id": "B04",  # no route
                                                  "slots": {"loops": 1}}}}  # one lap


@pytest.mark.asyncio  # async case
async def test_repeat_takes_the_latest_done_patrol(db):  # B04 fill
    """The latest DONE patrol, not the latest finished one. MUTATION: drop
    state='done' -> the failed r-fail (finished last) is taken -> red;
    order ascending -> r-old -> red."""
    await _seed_task(db, "t-a", "done", "r-old", "2026-11-05T01:00:00.000Z", 1)  # earlier
    await _seed_task(db, "t-b", "done", "r-new", "2026-11-05T02:00:00.000Z", 2)  # the one
    await _seed_task(db, "t-c", "failed", "r-fail", "2026-11-05T03:00:00.000Z", 3)  # did not complete
    ack = await handle_task_payload(_repeat_payload(), TaskContext(db, TasksDAO(db)),  # submit
                                    now_mono_ms=1000, date_str="20261105")  # today
    assert ack["result"] == "accepted"  # stored
    assert ack["detail"]["applied"]["route_id"] == "r-new"  # the latest completed patrol


@pytest.mark.asyncio  # async case
async def test_repeat_with_nothing_done_is_refused_with_its_reason(db):  # B04 none
    """MUTATION: store it anyway -> a patrol with no route -> red."""
    await _seed_task(db, "t-c", "failed", "r-fail", "2026-11-05T03:00:00.000Z", 1)  # nothing completed
    ack = await handle_task_payload(_repeat_payload(), TaskContext(db, TasksDAO(db)),  # submit
                                    now_mono_ms=1000, date_str="20261105")  # today
    assert ack["result"] == "rejected" and ack["code"] == E_NOT_FOUND  # refused
    assert ack["detail"]["reason"] == "no_completed_patrol"  # the P4 ack speaker keys on this


async def _running(conn, loop_total):  # a dispatched patrol on r-a rev 1
    dao = TasksDAO(conn)  # tasks
    await dao.insert(TaskRow(  # pending row
        task_id="t1", task_type="patrol", state="pending", priority=5,  # a patrol
        submit_seq=1, mission_json="{}", total_steps=1, current_step=0,  # one step
        step_status_json="[]", created_ms=0, updated_ms=0, source="local",  # local
        trace_id="tr", resume_policy="continue"))  # resumable
    await dao.dispatch_task("t1", 0, started_at="2026-11-05T00:00:00Z",  # running
                            started_mono=0.0, started_boot="b1")  # this boot
    await dao.update_state("t1", "running", 0)  # running
    pp = PatrolProgressDAO(conn)  # the run
    await pp.start_run(task_id="t1", route_name="1号路径", route_geo_id="r-a",  # opened as RP-1 does
                       route_rev=1, direction="forward", loop_mode="oneway",  # an open line
                       waypoint_total=5, route_total_m=200.0, loop_total=loop_total,  # passes
                       now_iso="2026-11-05T00:00:00.000Z")  # now
    await conn.commit()  # visible
    return dao, pp  # handles


def _pframe(state, dist=0.0):  # a P1 frame: single pass, says 0 / 1 always
    return {"v": 1, "route_id": "r-a", "route_rev": 1, "loading": False,  # 11 S3.5B
            "waypoint_index": 4 if state == "arrived" else 1, "waypoint_total": 5,  # where
            "seg_done_m": 0.0, "dist_done_m": dist, "fail_reason": None,  # metres
            "loop_index": 0, "loop_total": 1, "dir_sign": 1,  # what single-pass P1 says
            # mono is P1's tick time on the shared CLOCK_MONOTONIC: a frame
            # stamped before P3's push is dropped as the previous run's
            "state": state, "ts": 1.0, "mono": time.monotonic()}


async def _noop(*_a, **_k):  # on_transition stub
    return None  # nothing published


@pytest.mark.asyncio  # async case
async def test_two_passes_count_relatch_and_close(db):  # the sink
    """Two passes (one round trip on a line).
    MUTATION: close on the first arrived (ignore passes) -> state done after
    pass 1 -> red. Drop the latch -> the re-sent arrived counts pass 2 and
    closes the task -> red. Write loop_total from the frame -> 1 -> red.
    Non-cumulative dist -> 10.0 instead of 210.0 -> red."""
    dao, pp = await _running(db, 2)  # two passes asked
    pushed, latch = [], set()  # next_pass calls, the latch

    async def _next(task_id):  # what main_wiring hands in
        pushed.append(task_id)  # RP-5 recorded

    kw = dict(conn=db, dao=dao, progress_dao=pp, tracker=ProgressTracker(),  # shared args
              now_mono_ms=1000, now_wall_ms=1_785_000_000_000, on_transition=_noop,  # clocks
              boot_id="b1", next_pass=_next, pass_latch=latch)  # U131 hooks
    out = await apply_path_progress(_pframe("arrived", 200.0), **kw)  # pass 1 done
    assert out["next_pass"] and pushed == ["t1"] and "t1" in latch  # re-pushed, latched
    assert (await dao.fetch_by_id("t1")).state == "running"  # not done
    assert (await pp.fetch_active("t1"))[10] == 1  # loop_index advanced
    out = await apply_path_progress(_pframe("arrived", 200.0), **kw)  # the same arrived, re-sent
    assert out["reason"] == "pass_pending" and pushed == ["t1"]  # ignored
    assert (await pp.fetch_active("t1"))[10] == 1  # not counted twice
    out = await apply_path_progress(_pframe("running", 10.0), **kw)  # pass 2 loaded
    assert out["accepted"] and "t1" not in latch  # latch cleared
    row = await pp.fetch_active("t1")  # the run
    assert row[11] == 2 and row[7] == pytest.approx(210.0)  # P3's loop_total; cumulative distance
    out = await apply_path_progress(_pframe("arrived", 200.0), **kw)  # pass 2 done
    assert out["closed"] and pushed == ["t1"]  # closed, no third push
    assert (await dao.fetch_by_id("t1")).state == "done"  # done


@pytest.mark.asyncio  # async case
async def test_an_endless_patrol_never_closes_on_arrival(db):  # LM-4 / PP-3b
    """MUTATION: drop the loop_total == 0 short-circuit -> 0 + 1 < 0 is
    false -> the first arrival closes it -> red."""
    dao, pp = await _running(db, 0)  # endless
    latch = set()  # the latch

    async def _next(_task_id):  # re-push stub
        return None  # nothing to do

    kw = dict(conn=db, dao=dao, progress_dao=pp, tracker=ProgressTracker(),  # shared args
              now_mono_ms=1000, now_wall_ms=1_785_000_000_000, on_transition=_noop,  # clocks
              boot_id="b1", next_pass=_next, pass_latch=latch)  # U131 hooks
    for _ in range(3):  # three passes
        await apply_path_progress(_pframe("arrived", 200.0), **kw)  # a pass done
        await apply_path_progress(_pframe("running", 1.0), **kw)  # the next one loaded
    assert (await dao.fetch_by_id("t1")).state == "running"  # still patrolling
    assert (await pp.fetch_active("t1"))[10] == 3  # three counted


# -- RP-1 opens the run with the pass count --------------------------------------
@pytest_asyncio.fixture  # task.db + geo.db with the line
async def dbs():  # both dbs
    task = await aiosqlite.connect(":memory:")  # task.db
    geo = await aiosqlite.connect(":memory:")  # geo.db
    for conn, ddl in ((task, ALL_DDL_STATEMENTS), (geo, GEO_DB_STATEMENTS)):  # schemas
        for stmt in ddl:  # one DDL
            await conn.execute(stmt)  # apply
    await geo.execute(  # the line, mode B
        "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"  # columns
        " total_len_m, rev, content_hash, updated_ms) VALUES"  # as teach commits it
        " ('r-a', '1号路径', ?, 'oneway', 'forward', 200.0, 1, 'h', 0)",  # open, 200 m
        (json.dumps([list(p) for p in LINE.points]),))  # the line
    await geo.commit()  # visible
    yield task, geo  # to the test
    await task.close()  # cleanup
    await geo.close()  # cleanup


class _Pub:  # records frames
    def __init__(self):  # empty
        self.puts = []  # decoded frames

    def put(self, payload):  # one frame
        self.puts.append(json.loads(payload.decode("utf-8")))  # decode


def _push(task, geo, trigger, lap_spec):  # one push of r-a from the robot at (0, 1)
    return push_route_for_task(  # under test
        task_id="t1", route_geo_id="r-a", task_conn=task, geo_conn=geo,  # the line
        snapshot_dao=SnapshotDAO(task), progress_dao=PatrolProgressDAO(task),  # daos
        route_pub=_Pub(), ack_window=RouteAckWindow(), trigger=trigger,  # outputs
        now_mono_ms=1000, now_wall_ms=1_785_000_000_000,  # clocks
        recorded_arrive_radius_m=1.0, start=ll(0, 1), lap_spec=lap_spec)  # mode B radius, robot


@pytest.mark.asyncio  # async case
async def test_rp1_opens_the_run_with_the_pass_count(dbs):  # wiring
    """MUTATION: keep loop_total = 1 -> red; use the composite snapshot's
    loop_mode ('oneway' always) -- not caught here, the line IS oneway; the
    ring case of test_passes_follow_the_ruling pins the rule itself."""
    task, geo = dbs  # dbs
    res = await _push(task, geo, RoutePushTrigger.RP1_DISPATCH, LapSpec(3, True))  # 来回3次
    assert res.pushed  # sent
    assert (await PatrolProgressDAO(task).fetch_active("t1"))[11] == 6  # six passes


@pytest.mark.asyncio  # async case
async def test_rp1_without_a_lap_spec_is_a_caller_bug(dbs):  # no guessed count
    """MUTATION: default to one pass -> no raise -> red."""
    task, geo = dbs  # dbs
    with pytest.raises(ValueError):  # refused loudly
        await _push(task, geo, RoutePushTrigger.RP1_DISPATCH, None)  # no spec


# -- the live P3 loop: two passes end to end (CLAUDE.md 9.1A) ----------------------
@pytest.mark.asyncio  # async case
async def test_the_live_loop_runs_a_round_trip(tmp_path, monkeypatch):  # wiring: live loop
    """"来回巡逻1次" on the 200 m line from its start: pass 1 runs to the far
    end, pass 2 is pushed from there back to the start, the re-sent arrived
    of pass 1 pushes nothing, and the task is done after pass 2.
    MUTATION: next_pass=None in main_wiring -> done after pass 1, no second
    frame -> red. Push RP-5 with start=None -> the second frame is the
    recorded line again (ends at x=200) -> red."""
    from tests.p3_task.test_recorded_route_radius import _fake_planes  # shared fake session
    from xbrain.p3_task.runtime import main_wiring as mw  # the live loop

    subs, pubs = {}, {}  # what the fake session saw
    _fake_planes(monkeypatch, subs, pubs)  # no zenoh
    task_db, geo_db = str(tmp_path / "task.db"), str(tmp_path / "geo.db")  # on disk
    async with aiosqlite.connect(geo_db) as g:  # the line
        for stmt in GEO_DB_STATEMENTS:  # schema
            await g.execute(stmt)  # one DDL
        await g.execute(  # the line
            "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"  # mode B
            " total_len_m, rev, content_hash, updated_ms) VALUES"  # columns
            " ('r-a', '1号路径', ?, 'oneway', 'forward', 200.0, 1, 'h', 0)",  # open, 200 m
            (json.dumps([list(p) for p in LINE.points]),))  # the line
        await g.commit()  # visible

    def _sample(body):  # a zenoh sample stand-in
        return type("S", (), {"payload": json.dumps(body).encode("utf-8")})()  # payload only

    def _pose(x, y):  # state/pose at (x, y)
        lat, lon = ll(x, y)  # WGS84
        subs[mw.STATE_POSE_TOPIC](_sample({"data": {"lat": lat, "lon": lon}}))  # one sample

    async def _state():  # (task state, loop_index)
        async with aiosqlite.connect(task_db) as t:  # read back
            st = (await (await t.execute("SELECT state FROM tasks WHERE task_id='t-1'")).fetchone())[0]  # task
            li = await (await t.execute(  # the run, any status
                "SELECT loop_index FROM patrol_progress WHERE task_id='t-1'")).fetchone()  # run
        return st, (li[0] if li else None)  # pair

    async def _frames(n):  # wait for n route frames
        for _ in range(250):  # bounded
            if len(pubs.get("cmd/motion/route", [])) >= n:  # enough
                break  # done
            await asyncio.sleep(0.02)  # next pass
        return [json.loads(p.decode("utf-8")) for p in pubs.get("cmd/motion/route", [])]  # decoded

    stop = {"stop": False}  # loop control
    task = asyncio.create_task(mw._amain(  # start P3
        stop, 99.0, task_db, geo_db_path=geo_db, fence_db_path=str(tmp_path / "fence.db"),  # dbs
        recorded_arrive_radius_m=1.0, route_membership_m=5.0))  # configured as in production
    try:  # always stop
        for _ in range(300):  # wait for the subscribers
            if mw.STATE_PATH_PROGRESS_TOPIC in subs and mw.STATE_POSE_TOPIC in subs:  # up
                break  # ready
            await asyncio.sleep(0.01)  # poll
        _pose(0, 1)  # beside the start
        await asyncio.sleep(0.05)  # let it land
        async with aiosqlite.connect(task_db) as t:  # one ready patrol, one round trip
            await TasksDAO(t).insert(TaskRow(  # as admission stores it
                task_id="t-1", task_type="patrol", state="ready", priority=5,  # a patrol
                submit_seq=1, total_steps=1, current_step=0, step_status_json="[]",  # one step
                mission_json=json.dumps({"params": {"slots": {"loops": 1, "round_trip": True}}}),  # 来回1次
                created_ms=0, updated_ms=0, source="local", trace_id="tr",  # local
                resume_policy="continue", route_geo_id="r-a"))  # the line
            await t.commit()  # visible
        f = await _frames(1)  # pass 1
        assert xy((f[0]["points"][-1]["lat"], f[0]["points"][-1]["lon"])) == (200.0, 0.0)  # to the far end
        progress = subs[mw.STATE_PATH_PROGRESS_TOPIC]  # P1's frames in
        progress(_sample({"data": _pframe("running", 5.0)}))  # pass 1 loaded
        await asyncio.sleep(0.1)  # applied
        _pose(200, 1)  # at the far end
        await asyncio.sleep(0.05)  # landed
        progress(_sample({"data": _pframe("arrived", 200.0)}))  # pass 1 done
        f = await _frames(2)  # pass 2
        assert len(f) == 2  # re-pushed
        assert xy((f[1]["points"][-1]["lat"], f[1]["points"][-1]["lon"])) == (0.0, 0.0)  # back to the start
        progress(_sample({"data": _pframe("arrived", 200.0)}))  # pass 1's arrived, re-sent
        await asyncio.sleep(0.2)  # would have pushed by now
        assert len(await _frames(2)) == 2 and await _state() == ("running", 1)  # ignored
        progress(_sample({"data": _pframe("running", 3.0)}))  # pass 2 loaded
        await asyncio.sleep(0.1)  # applied
        progress(_sample({"data": _pframe("arrived", 200.0)}))  # pass 2 done
        for _ in range(100):  # until closed
            if (await _state())[0] == "done":  # closed
                break  # done
            await asyncio.sleep(0.02)  # next pass
        assert await _state() == ("done", 1) and len(await _frames(2)) == 2  # done, no third pass
    finally:  # cleanup
        stop["stop"] = True  # ask it to stop
        await asyncio.wait_for(task, timeout=10.0)  # and wait


# -- 99 U147 nav audit: a frame from before the push is the previous run's ------
def test_predates_push_compares_p1_mono_with_the_push():  # the gate itself
    """MUTATION: <= instead of < / ignore the push time -> red."""
    assert predates_push({"mono": 9.999}, 10_000) is True  # stamped before the push
    assert predates_push({"mono": 10.0}, 10_000) is False  # at the push: P1 may have loaded it
    assert predates_push({"mono": 5.0}, 0) is False  # no push yet: nothing to compare
    assert predates_push({"mono": "x"}, 10_000) is False  # malformed: the parser's to reject


@pytest.mark.asyncio  # async case
async def test_a_stale_arrived_does_not_close_the_next_run_on_the_same_route(tmp_path, monkeypatch):
    """Patrol t-1 on r-a finishes; P1 keeps re-sending its arrived at 2 Hz.
    t-2 ("再巡一圈") is dispatched on the SAME route at the SAME rev: the
    re-sent arrived, built before the push, must neither confirm nor close
    it. Found on the bench 2026-11-06 ("再巡一圈" ended before the robot
    moved). MUTATION: drop the predates_push gate in main_wiring -> t-2 done
    on the stale frame -> red."""
    from tests.p3_task.test_recorded_route_radius import _fake_planes  # shared fake session
    from xbrain.p3_task.runtime import main_wiring as mw  # the live loop

    subs, pubs = {}, {}  # what the fake session saw
    _fake_planes(monkeypatch, subs, pubs)  # no zenoh
    task_db, geo_db = str(tmp_path / "task.db"), str(tmp_path / "geo.db")  # on disk
    async with aiosqlite.connect(geo_db) as g:  # the line
        for stmt in GEO_DB_STATEMENTS:  # schema
            await g.execute(stmt)  # one DDL
        await g.execute(  # the line, open, 200 m
            "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction,"
            " total_len_m, rev, content_hash, updated_ms) VALUES"
            " ('r-a', '1号路径', ?, 'oneway', 'forward', 200.0, 1, 'h', 0)",
            (json.dumps([list(p) for p in LINE.points]),))
        await g.commit()  # visible

    def _sample(body):  # a zenoh sample stand-in
        return type("S", (), {"payload": json.dumps(body).encode("utf-8")})()

    async def _state(tid):  # a task's state
        async with aiosqlite.connect(task_db) as t:  # read back
            return (await (await t.execute("SELECT state FROM tasks WHERE task_id=?", (tid,))).fetchone())[0]

    async def _until(cond):  # poll a condition
        for _ in range(250):  # bounded
            if await cond():
                return True  # met
            await asyncio.sleep(0.02)  # next look
        return False  # timed out

    async def _patrol(tid, seq):  # one ready single-pass patrol on r-a
        async with aiosqlite.connect(task_db) as t:
            await TasksDAO(t).insert(TaskRow(
                task_id=tid, task_type="patrol", state="ready", priority=5,
                submit_seq=seq, total_steps=1, current_step=0, step_status_json="[]",
                mission_json=json.dumps({"params": {"slots": {}}}),
                created_ms=0, updated_ms=0, source="local", trace_id="tr",
                resume_policy="continue", route_geo_id="r-a"))
            await t.commit()  # visible

    stop = {"stop": False}  # loop control
    task = asyncio.create_task(mw._amain(  # start P3
        stop, 99.0, task_db, geo_db_path=geo_db, fence_db_path=str(tmp_path / "fence.db"),
        recorded_arrive_radius_m=1.0, route_membership_m=5.0))
    try:  # always stop
        for _ in range(300):  # wait for the subscribers
            if mw.STATE_PATH_PROGRESS_TOPIC in subs and mw.STATE_POSE_TOPIC in subs:
                break  # ready
            await asyncio.sleep(0.01)  # poll
        lat, lon = ll(0, 1)  # beside the start
        subs[mw.STATE_POSE_TOPIC](_sample({"data": {"lat": lat, "lon": lon}}))
        progress = subs[mw.STATE_PATH_PROGRESS_TOPIC]  # P1's frames in
        await _patrol("t-1", 1)  # the first run
        assert await _until(lambda: _frames_n(pubs, 1))  # pushed
        progress(_sample({"data": _pframe("running", 5.0)}))  # loaded
        await asyncio.sleep(0.1)  # applied
        progress(_sample({"data": _pframe("arrived", 200.0)}))  # t-1 done
        assert await _until(lambda: _is(_state("t-1"), "done"))  # closed
        stale = _pframe("arrived", 200.0)  # P1 re-sends it, stamped NOW (before t-2's push)
        await asyncio.sleep(0.01)  # the push comes strictly later
        await _patrol("t-2", 2)  # "再巡一圈": same route, same rev
        assert await _until(lambda: _frames_n(pubs, 2))  # t-2 pushed
        progress(_sample({"data": stale}))  # the stale arrived lands after the push
        await asyncio.sleep(1.2)  # > two passes of the 0.5 s loop: it would have closed t-2
        assert await _state("t-2") == "running"  # ignored
        progress(_sample({"data": _pframe("running", 5.0)}))  # t-2 loaded
        await asyncio.sleep(0.1)  # applied
        progress(_sample({"data": _pframe("arrived", 200.0)}))  # t-2 really done
        assert await _until(lambda: _is(_state("t-2"), "done"))  # closed by the fresh frame
    finally:
        stop["stop"] = True  # stop the loop
        await asyncio.wait_for(task, 10.0)  # joined


async def _frames_n(pubs, n):  # at least n route frames out
    return len(pubs.get("cmd/motion/route", [])) >= n


async def _is(coro, want):  # an awaited value equals want
    return (await coro) == want

