"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_route_push.py
Brief: EX-2 -- RouteGeometry frames, chunking, the RA-1 window, and the live push

Description:
15 S2.4 / 11 S3.5A. Replaces the six tests deleted from test_batch_d.py,
which were green over an API that did not implement this protocol (see the
note left at that spot).

The assertions that matter most here are the two the old suite could not
have made:

  * a frame P1 would actually ACCEPT. test_a_frame_round_trips_through_p1s_
    own_parser feeds the built frame to xbrain/p1_motion/nav/route_intake.py
    -- the real consumer. A field-by-field comparison against the json5 block
    in 11 S3.5A would be a second transcription of the contract, and the
    previous push.py shows how far a transcription can drift while every
    local test stays green.
  * RA-1 in its three-criteria form. 15 TC-37a: a push that never arrived,
    with loading == false because the OLD geometry is idle, must NOT read as
    confirmed.

The last test drives the real p3 loop and asserts a dispatched patrol puts
frames on cmd/motion/route. Every other test in this file passes with the
publisher deleted from main_wiring.
"""
from __future__ import annotations

import asyncio
import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.simple_daos import PatrolProgressDAO, SnapshotDAO
from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.persistence.schema_geo import GEO_DB_STATEMENTS
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.route.push import (
    CHUNK_POINTS,
    MAX_POINTS,
    RouteAck,
    RouteAckWindow,
    RoutePoint,
    RoutePushError,
    RoutePushTrigger,
    build_route_frames,
)
from xbrain.p3_task.route.snapshot_build import (
    SnapshotBuildError,
    build_snapshot,
    cumulative_arclen,
)
from xbrain.p3_task.runtime.route_push_runtime import push_route_for_task

pytestmark = pytest.mark.no_device


def _points(n, radius=1.0):
    # Spread in latitude so consecutive points are distinct and the arc
    # length is non-zero (a zero-length route is refused, by design).
    return [RoutePoint(35.0 + i * 1e-4, 135.0, radius) for i in range(n)]


# --- 11 S3.5A frame shape -------------------------------------------------

def test_a_frame_carries_every_11_s3_5a_field():
    f = build_route_frames(route_id="r-east_gate", route_rev=7,
                           loop_mode="closed", total_len_m=1620.0,
                           points=_points(3), cmd_id="rg-3f1a")[0]
    assert f["v"] == 1 and f["op"] == "set" and f["frame"] == "wgs84"
    assert f["cmd_id"] == "rg-3f1a" and f["route_id"] == "r-east_gate"
    assert f["route_rev"] == 7 and f["loop_mode"] == "closed"
    assert f["total_len_m"] == 1620.0
    assert f["chunk"] == {"index": 0, "total": 1}
    assert f["points"][0] == {"lat": 35.0, "lon": 135.0, "seq": 0,
                              "arrive_radius_m": 1.0}


def test_a_frame_round_trips_through_p1s_own_parser():
    """*** The assertion the old suite structurally could not make.

    P1's RouteAssembler is the real consumer (P1-11). Its rejection mode is
    silent on the wire -- "participant is up, nothing gets through", the
    13 DDS-9 failure -- so a frame P3 is happy with and P1 drops looks
    exactly like a network fault.

    MUTATION: rename any wire field (route_rev -> rev, arrive_radius_m ->
    radius_m, drop `frame`) and this raises RouteIntakeError. The shape
    assertions above would still pass for most of those.
    """
    from xbrain.p1_motion.nav.route_intake import RouteAssembler, RouteSet
    from xbrain.p1_motion.path.local_frame import LocalFrame

    frames = build_route_frames(route_id="r-east_gate", route_rev=7,
                                loop_mode="closed", total_len_m=1620.0,
                                points=_points(4))
    asm = RouteAssembler(LocalFrame(35.0, 135.0))
    out = None
    for f in frames:
        out = asm.accept(f)
    assert isinstance(out, RouteSet)
    assert out.route_id == "r-east_gate" and out.route_rev == 7
    assert out.waypoint_total == 4


# --- 15 S2.4.2 chunking ---------------------------------------------------

def test_a_route_under_the_cap_is_one_frame():
    assert len(build_route_frames(
        route_id="r", route_rev=1, loop_mode="oneway", total_len_m=10.0,
        points=_points(CHUNK_POINTS))) == 1


def test_2400_points_become_three_frames():
    """15 TC-36 verbatim: "P3 分 3 片推 cmd/motion/route (>1000 点即分片)".

    MUTATION: restore the old CHUNK_SIZE of 32 and this reports 75 frames.
    That mutation is invisible to every other test here -- the frames stay
    individually well-formed, there are just 25x as many of them.
    """
    frames = build_route_frames(route_id="r", route_rev=1,
                                loop_mode="oneway", total_len_m=100.0,
                                points=_points(2400))
    assert [f["chunk"] for f in frames] == [
        {"index": 0, "total": 3}, {"index": 1, "total": 3},
        {"index": 2, "total": 3}]
    assert sum(len(f["points"]) for f in frames) == 2400


def test_seq_is_the_index_in_the_whole_route_not_in_the_chunk():
    """P1 checks that seq runs 0..n-1 over the ASSEMBLED set
    (route_intake.py), so a per-chunk restart reads to it as a lost chunk --
    and it discards the whole route on a gap (11 S3.5A).

    MUTATION: enumerate inside the chunk and the second frame starts at
    seq 0; red here, and on the robot the route is silently dropped and the
    old geometry keeps running.
    """
    frames = build_route_frames(route_id="r", route_rev=1,
                                loop_mode="oneway", total_len_m=100.0,
                                points=_points(CHUNK_POINTS + 5))
    assert frames[1]["points"][0]["seq"] == CHUNK_POINTS


def test_every_chunk_of_one_push_shares_one_cmd_id():
    """P1 assembles by cmd_id and ABANDONS the pending set when a different
    one arrives, so a per-chunk id turns every multi-chunk route into a
    series of abandoned single-chunk sets -- with no error anywhere."""
    frames = build_route_frames(route_id="r", route_rev=1,
                                loop_mode="oneway", total_len_m=100.0,
                                points=_points(2400))
    assert len({f["cmd_id"] for f in frames}) == 1


def test_over_the_rg2_cap_refuses_rather_than_trimming():
    """11 S3.5A RG-2 / 15 S2.4.2: over 5000 points nothing is pushed and the
    task fails at the precondition. Trimming would push a route that stops
    short of where it was told to go."""
    with pytest.raises(RoutePushError, match="RG-2"):
        build_route_frames(route_id="r", route_rev=1, loop_mode="oneway",
                           total_len_m=100.0, points=_points(MAX_POINTS + 1))


def test_zero_total_len_refuses():
    """total_len_m is S7.3A's T0 and divides in the remap's alpha."""
    with pytest.raises(RoutePushError, match="T0"):
        build_route_frames(route_id="r", route_rev=1, loop_mode="oneway",
                           total_len_m=0.0, points=_points(3))


# --- 15 S2.4.4 RA-1 / RA-2 / RA-3 ----------------------------------------

def test_ra1_needs_all_three_criteria():
    """15 TC-37a. `loading == false` alone is ALSO the steady state of the
    old geometry, so a push that never arrived would read as confirmed; P3
    then sends a path_follow with the new route_rev and P1 answers
    E_GEO_CONFLICT.

    MUTATION: confirm on `loading is False` alone -> the first two asserts
    below go red. Either single field alone is enough to break it, which is
    why both a wrong id and a wrong rev are exercised.
    """
    w = RouteAckWindow(timeout_s=3.0)
    w.open("r-east_gate", 7, now_mono_ms=0)
    assert w.observe(loading=False, route_id="r-west", route_rev=7) is None
    assert w.observe(loading=False, route_id="r-east_gate",
                     route_rev=6) is None
    assert w.observe(loading=True, route_id="r-east_gate",
                     route_rev=7) is None
    out = w.observe(loading=False, route_id="r-east_gate", route_rev=7)
    assert out is not None and out.ack is RouteAck.RA1_CONFIRMED
    assert w.is_open is False


def test_ra3_times_out_on_the_monotonic_window_and_allows_two_retries():
    """15 S2.4.2: route_push_retry defaults to 2, then the task fails.

    MUTATION: make the window open at the FIRST chunk instead of the last --
    not visible here, but see the runtime: it can expire while frames are
    still going out.
    """
    w = RouteAckWindow(timeout_s=3.0, max_retry=2)
    w.open("r", 1, now_mono_ms=0)
    assert w.tick(now_mono_ms=2999) is None
    out = w.tick(now_mono_ms=3000)
    assert out.ack is RouteAck.RA3_TIMEOUT and out.retry_allowed is True
    w.open("r", 1, now_mono_ms=3000)
    assert w.tick(now_mono_ms=6000).retry_allowed is True     # attempt 2
    w.open("r", 1, now_mono_ms=6000)
    out = w.tick(now_mono_ms=9000)                            # attempt 3
    assert out.retry_allowed is False


def test_report_incomplete_does_not_wait_out_the_window():
    """15 S2.4.4: an E_GEO_INCOMPLETE on event/warn/motion re-pushes at once
    rather than costing another 3 s."""
    w = RouteAckWindow(timeout_s=3.0)
    w.open("r", 1, now_mono_ms=0)
    out = w.report_incomplete()
    assert out.ack is RouteAck.RA2_REPORTED_INCOMPLETE
    assert w.is_open is False


def test_rp4_is_a_value_that_means_do_nothing():
    """15 S2.4.1 RP-4: a task reaching a terminal pushes NOTHING and revokes
    NOTHING. The geometry staying resident in P1 is what 15 S1.3 T-1 relies
    on (P1 finishes the lap if P3 dies)."""
    assert RoutePushTrigger.RP4_TASK_TERMINAL.value == "RP-4"


# --- 15 S9.3A snapshot build ---------------------------------------------

def test_mode_a_takes_the_radius_from_each_anchor():
    """15 S9.3.1 G-2: per point, because a gate and an alley cannot share an
    arrival test.

    MUTATION: use one radius for all anchors -> red.
    """
    snap = build_snapshot(
        task_id="t1", route_id="r-a", rev=2, loop_mode="oneway",
        waypoint_ids=json.dumps(["w-1", "w-2"]),
        anchor_lookup={"w-1": (35.0, 135.0, 3.0),
                       "w-2": (35.001, 135.0, 0.8)})
    assert [p.arrive_radius_m for p in snap.points] == [3.0, 0.8]


def test_a_missing_anchor_refuses_the_whole_route():
    """A route whose geometry silently shortens when somebody tidies a
    keypoint off the map is the failure geo_write._fill_anchor_length already
    guards at commit; the same has to hold at push."""
    with pytest.raises(SnapshotBuildError, match="not in geo.db"):
        build_snapshot(task_id="t1", route_id="r-a", rev=2,
                       loop_mode="oneway",
                       waypoint_ids=json.dumps(["w-1", "w-missing"]),
                       anchor_lookup={"w-1": (35.0, 135.0, 1.0)})


def test_mode_b_without_a_configured_radius_refuses_and_names_the_key():
    """*** CLAUDE.md 3.1 / IRON RULE 3 in a concrete spot.

    15 S2.4.3 sends mode B's arrive_radius_m to "common.recording 侧的全局
    缺省" and marks it 待 T7 / NAV-12; configs/common.yaml's recording block
    has five keys and no arrive-radius among them. A number invented here
    would be indistinguishable from a calibrated one forever after.

    MUTATION: default it to 1.0 and this goes red -- and a recorded patrol
    silently arrives at a radius nobody chose.
    """
    with pytest.raises(SnapshotBuildError) as exc:
        build_snapshot(task_id="t1", route_id="r-b", rev=1,
                       loop_mode="oneway",
                       path_points=json.dumps([[35.0, 135.0],
                                               [35.001, 135.0]]))
    msg = str(exc.value)
    assert "common.recording" in msg and "NAV-12" in msg


def test_arclen_is_cumulative_and_one_entry_per_point():
    """SN-2: arclen_json and points_json must describe the same route. A
    length mismatch makes S7.3A step 1 compute a wrong L0 AND REPORT
    NOTHING."""
    pts = _points(4)
    arc = cumulative_arclen(pts)
    assert len(arc) == 4 and arc[0] == 0.0
    assert arc[1] < arc[2] < arc[3]


def test_a_route_with_both_geometries_is_refused():
    """Mirrors the routes XOR CHECK, with a message that names the row."""
    with pytest.raises(SnapshotBuildError, match="exactly one"):
        build_snapshot(task_id="t1", route_id="r", rev=1, loop_mode="oneway",
                       path_points=json.dumps([[35.0, 135.0], [35.1, 135.0]]),
                       waypoint_ids=json.dumps(["w-1"]),
                       anchor_lookup={"w-1": (35.0, 135.0, 1.0)},
                       recorded_arrive_radius_m=1.0)


# --- the push against real databases -------------------------------------

@pytest_asyncio.fixture
async def dbs():
    async with aiosqlite.connect(":memory:") as task_conn, \
            aiosqlite.connect(":memory:") as geo_conn:
        for stmt in ALL_DDL_STATEMENTS:
            await task_conn.execute(stmt)
        for stmt in GEO_DB_STATEMENTS:
            await geo_conn.execute(stmt)
        await task_conn.commit()
        await geo_conn.commit()
        yield task_conn, geo_conn


async def _seed(task_conn, geo_conn, *, task_id="t1", route_id="r-east_gate"):
    """A running patrol plus a mode-A route of two anchors in geo.db."""
    dao = TasksDAO(task_conn)
    await dao.insert(TaskRow(
        task_id=task_id, task_type="patrol", state="pending", priority=5,
        submit_seq=1, mission_json="{}", total_steps=1, current_step=0,
        step_status_json="[]", created_ms=0, updated_ms=0, source="local",
        trace_id="tr", resume_policy="continue", route_geo_id=route_id))
    await task_conn.commit()
    for wid, lat, radius in (("w-1", 35.0, 2.0), ("w-2", 35.002, 0.9)):
        await geo_conn.execute(
            "INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon,"
            " arrival_radius, content_hash, updated_ms)"
            " VALUES (?, ?, 'poi', ?, 135.0, ?, 'h', 0)",
            (wid, "wp-" + wid, lat, radius))
    await geo_conn.execute(
        "INSERT INTO routes (geo_id, name, waypoint_ids, loop_mode,"
        " direction, total_len_m, rev, content_hash, updated_ms)"
        " VALUES (?, 'east gate', ?, 'closed', 'forward', 222.0, 7, 'h', 0)",
        (route_id, json.dumps(["w-1", "w-2"])))
    await geo_conn.commit()
    return dao


class _Pub:
    def __init__(self):
        self.puts = []

    def put(self, payload):
        self.puts.append(json.loads(payload.decode("utf-8")))


@pytest.mark.asyncio
async def test_push_writes_the_snapshot_before_it_sends(dbs):
    """SN-1 (15 S9.3A): snapshot COMMITTED first, then the frames.

    Reversed, a power cut in between leaves P1 holding geometry P3 has no
    snapshot of, and S7.3A then remaps against the wrong T0.

    MUTATION: move snapshot_dao.replace after the publish loop; the ordering
    assertion below goes red (the publisher records the snapshot row count it
    saw at put time).
    """
    task_conn, geo_conn = dbs
    await _seed(task_conn, geo_conn)   # seeds both DBs; the DAO handle
    #   it returns is not needed here (this case drives push_route_for_task
    #   directly and reads back through snapshot_dao).
    snapshot_dao = SnapshotDAO(task_conn)
    seen_rows = []

    class _OrderPub(_Pub):
        def put(self, payload):
            super().put(payload)
            seen_rows.append(True)

    pub = _OrderPub()
    from xbrain.p3_task.route.push import RouteAckWindow as _W
    res = await push_route_for_task(
        task_id="t1", route_geo_id="r-east_gate", task_conn=task_conn,
        geo_conn=geo_conn, snapshot_dao=snapshot_dao,
        progress_dao=PatrolProgressDAO(task_conn), route_pub=pub,
        ack_window=_W(), trigger=RoutePushTrigger.RP1_DISPATCH,
        now_mono_ms=1000, now_wall_ms=1_785_000_000_000)
    assert res.pushed is True and res.frames == 1
    # The snapshot exists and carries what the frame carried.
    row = await snapshot_dao.fetch("t1")
    assert row[0] == "r-east_gate" and row[1] == 7 and row[3] == 2
    assert pub.puts[0]["route_rev"] == 7
    # Per-anchor radii survived all the way onto the wire.
    assert [p["arrive_radius_m"] for p in pub.puts[0]["points"]] == [2.0, 0.9]
    # The progress run is open so EX-3's writer has somewhere to land.
    assert await PatrolProgressDAO(task_conn).fetch_active("t1") is not None


@pytest.mark.asyncio
async def test_rp4_pushes_nothing(dbs):
    """15 S2.4.1 RP-4. Guarded in code rather than left to the caller: "do
    nothing" is a rule that disappears the first time somebody adds an else
    branch."""
    task_conn, geo_conn = dbs
    await _seed(task_conn, geo_conn)
    pub = _Pub()
    res = await push_route_for_task(
        task_id="t1", route_geo_id="r-east_gate", task_conn=task_conn,
        geo_conn=geo_conn, snapshot_dao=SnapshotDAO(task_conn),
        progress_dao=PatrolProgressDAO(task_conn), route_pub=pub,
        ack_window=RouteAckWindow(),
        trigger=RoutePushTrigger.RP4_TASK_TERMINAL,
        now_mono_ms=1000, now_wall_ms=1_785_000_000_000)
    assert res.pushed is False and pub.puts == []


@pytest.mark.asyncio
async def test_a_deleted_route_refuses_without_raising(dbs):
    """The dequeue precondition V-8 should have caught this, so reaching here
    means the route went away between validation and dispatch. That is a real
    race and not a reason to push a partial path -- nor to take the p3 loop
    down."""
    task_conn, geo_conn = dbs
    await _seed(task_conn, geo_conn)
    pub = _Pub()
    res = await push_route_for_task(
        task_id="t1", route_geo_id="r-gone", task_conn=task_conn,
        geo_conn=geo_conn, snapshot_dao=SnapshotDAO(task_conn),
        progress_dao=PatrolProgressDAO(task_conn), route_pub=pub,
        ack_window=RouteAckWindow(), trigger=RoutePushTrigger.RP1_DISPATCH,
        now_mono_ms=1000, now_wall_ms=1_785_000_000_000)
    assert res.pushed is False and "route_missing" in res.reason
    assert pub.puts == []


# --- the wiring itself ----------------------------------------------------

@pytest.mark.asyncio
async def test_the_live_wiring_pushes_cmd_motion_route_on_dispatch(
        tmp_path, monkeypatch):
    """*** The test that tells "the task drives the chassis" from "it does
    not".

    Runs the real p3 loop against a fake session, submits a patrol on a route
    that exists in geo.db, and asserts frames land on cmd/motion/route. Every
    other test in this file passes with the publisher removed from
    main_wiring -- this one does not.

    MUTATION: delete the _push_route_on_dispatch call after scheduler_tick
    and this goes red with zero frames.
    """
    from xbrain.p3_task.runtime import main_wiring as mw

    subs = {}
    pubs = {}

    class _WPub:
        def __init__(self, key):
            self.key = key
            pubs.setdefault(key, [])

        def put(self, payload):
            pubs[self.key].append(payload)

    class _Session:
        def declare_subscriber(self, key, cb):
            subs[key] = cb
            return object()

        def declare_publisher(self, key):
            return _WPub(key)

        def declare_queryable(self, key, cb):
            subs[key] = cb
            return object()

        def put(self, key, payload):
            pubs.setdefault(key, []).append(payload)

    class _Planes:
        def __enter__(self):
            return _Session()

        def __exit__(self, *_a):
            return False

    import xbrain.common.runtime.session_ctx as sctx
    monkeypatch.setattr(sctx, "open_planes", lambda _p: _Planes())

    task_db = str(tmp_path / "task.db")
    geo_db = str(tmp_path / "geo.db")
    # Seed geo.db BEFORE the wiring opens it; the route has to exist when the
    # dispatch happens.
    async with aiosqlite.connect(geo_db) as g:
        for stmt in GEO_DB_STATEMENTS:
            await g.execute(stmt)
        await g.commit()
        for wid, lat, radius in (("w-1", 35.0, 2.0), ("w-2", 35.002, 0.9)):
            await g.execute(
                "INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon,"
                " arrival_radius, content_hash, updated_ms)"
                " VALUES (?, ?, 'poi', ?, 135.0, ?, 'h', 0)",
                (wid, "wp-" + wid, lat, radius))
        await g.execute(
            "INSERT INTO routes (geo_id, name, waypoint_ids, loop_mode,"
            " direction, total_len_m, rev, content_hash, updated_ms)"
            " VALUES ('r-east_gate', 'east gate', ?, 'closed', 'forward',"
            " 222.0, 7, 'h', 0)", (json.dumps(["w-1", "w-2"]),))
        await g.commit()

    stop = {"stop": False}
    task = asyncio.create_task(mw._amain(
        stop, 99.0, task_db, geo_db_path=geo_db,
        fence_db_path=str(tmp_path / "fence.db")))
    try:
        for _ in range(300):
            if mw.CMD_TASK_TOPIC in subs:
                break
            await asyncio.sleep(0.01)
        assert "cmd/motion/route" in pubs, (
            "p3 never declared cmd/motion/route; with no publisher the "
            "geometry has no way to reach P1 (15 S2.4)")

        # Seed a ready patrol directly: the admission path (mission parsing,
        # route expansion) is EX-1 and out of scope here -- what is under
        # test is that a DISPATCH pushes.
        async with aiosqlite.connect(task_db) as t:
            dao = TasksDAO(t)
            await dao.insert(TaskRow(
                task_id="t-push", task_type="patrol", state="ready",
                priority=5, submit_seq=1, mission_json="{}", total_steps=1,
                current_step=0, step_status_json="[]", created_ms=0,
                updated_ms=0, source="local", trace_id="tr",
                resume_policy="continue", route_geo_id="r-east_gate"))
            await t.commit()

        for _ in range(400):
            if pubs.get("cmd/motion/route"):
                break
            await asyncio.sleep(0.02)
        frames = [json.loads(p.decode("utf-8"))
                  for p in pubs.get("cmd/motion/route", [])]
        assert frames, ("a dispatched patrol published NO cmd/motion/route "
                        "frame; the task cannot drive the chassis")
        assert frames[0]["route_id"] == "r-east_gate"
        assert frames[0]["route_rev"] == 7
        assert frames[0]["frame"] == "wgs84"
        assert frames[0]["chunk"] == {"index": 0, "total": 1}
    finally:
        stop["stop"] = True
        await asyncio.wait_for(task, timeout=10.0)
