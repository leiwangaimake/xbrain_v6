"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_commit.py
Brief: commit a recorded route/fence to geo.db/fence.db + closure checks (PB7)

Description:
Tests the teach COMMIT writer (geo_commit) against real in-memory geo.db /
fence.db, plus the fence perimeter closure checks (geom). These were the audit
gaps: teach had dedup but no writer, validate_polygon had no closure notion,
and record_fence was not a task-create. Each assertion has a mutation guard.
"""
from __future__ import annotations

import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.fence.geom import (
    InvalidPolygon, assert_perimeter_closed, close_ring,
)
from xbrain.p3_task.ingest.geo_commit import (
    GeoCommitError, commit_fence, commit_route, commit_waypoint,
)
from xbrain.p3_task.persistence.schema_geo import (
    FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS,
)


pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def geo_conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in GEO_DB_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield c


@pytest_asyncio.fixture
async def fence_conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in FENCE_DB_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield c


def _path(n, lat0=31.2000, lon0=121.5000):
    # A WGS84 (lat, lon) polyline heading north; 0.0001 deg lat ~ 11 m, so the
    # segments are non-degenerate and total_len_m > 0.
    return [(lat0 + i * 0.0001, lon0) for i in range(n)]


# -- commit_route: mode B (inline path_points), the 15 S9.3 model --------

@pytest.mark.asyncio
async def test_commit_route_writes_inline_geometry_no_waypoints(geo_conn):
    """PLAN A: a route is ONE routes row with inline path_points -- no waypoints
    and no assoc rows. MUTATION: the old model wrote one waypoint per point +
    assoc, so asserting waypoints==0 and assoc==0 catches a regression to it."""
    await commit_route(geo_conn, route_id="r-eastgate", name="东门路线",
                       path_points=_path(3), now_ms=1)
    cur = await geo_conn.execute("SELECT COUNT(*) FROM waypoints")
    assert (await cur.fetchone())[0] == 0
    cur = await geo_conn.execute("SELECT COUNT(*) FROM route_waypoint_assoc")
    assert (await cur.fetchone())[0] == 0
    cur = await geo_conn.execute(
        "SELECT name, path_points, waypoint_ids, total_len_m "
        "FROM routes WHERE geo_id=?", ("r-eastgate",))
    name, pp, wi, total = await cur.fetchone()
    assert name == "东门路线"
    assert wi is None                            # XOR: path mode leaves anchors NULL
    assert json.loads(pp) == [[31.2, 121.5], [31.2001, 121.5], [31.2002, 121.5]]
    assert total > 20.0                          # ~22 m over the two 11 m segments


@pytest.mark.asyncio
async def test_commit_route_rejects_both_or_neither_geometry(geo_conn):
    """MUTATION: dropping the XOR guard lets a route with BOTH (or NEITHER)
    geometry through the Python layer to a cryptic DB CHECK abort."""
    with pytest.raises(GeoCommitError, match="exactly one"):
        await commit_route(geo_conn, route_id="r-x", name="x", now_ms=1)
    with pytest.raises(GeoCommitError, match="exactly one"):
        await commit_route(geo_conn, route_id="r-y", name="y",
                           path_points=_path(2), waypoint_ids=["w-a"], now_ms=1)


@pytest.mark.asyncio
async def test_commit_route_rejects_too_few_points(geo_conn):
    with pytest.raises(GeoCommitError, match=">= 2"):
        await commit_route(geo_conn, route_id="r-x", name="x",
                           path_points=_path(1), now_ms=1)


@pytest.mark.asyncio
async def test_commit_route_rejects_over_cap(geo_conn):
    """MUTATION: dropping the cap check lets a >5000-point route (11 S7.8.3
    RouteGeometry bound) through."""
    with pytest.raises(GeoCommitError, match="cap"):
        await commit_route(geo_conn, route_id="r-x", name="x",
                           path_points=_path(5001), now_ms=1)


@pytest.mark.asyncio
async def test_commit_route_mode_a_anchors(geo_conn):
    """Mode A: waypoint_ids reference committed keypoints; total_len_m is the
    anchor polyline length. MUTATION: a missing anchor must raise, not store a
    route pointing at a non-existent keypoint."""
    await commit_waypoint(geo_conn, geo_id="w-a", name="甲", wtype="poi",
                          rtk_lat=31.2000, rtk_lon=121.5000, now_ms=1)
    await commit_waypoint(geo_conn, geo_id="w-b", name="乙", wtype="poi",
                          rtk_lat=31.2010, rtk_lon=121.5000, now_ms=1)
    await commit_route(geo_conn, route_id="r-anchored", name="锚点线",
                       waypoint_ids=["w-a", "w-b"], now_ms=2)
    cur = await geo_conn.execute(
        "SELECT waypoint_ids, path_points, total_len_m FROM routes WHERE geo_id=?",
        ("r-anchored",))
    wi, pp, total = await cur.fetchone()
    assert pp is None and json.loads(wi) == ["w-a", "w-b"]
    assert total > 100.0                         # ~111 m for 0.001 deg lat
    with pytest.raises(GeoCommitError, match="not found"):
        await commit_route(geo_conn, route_id="r-bad", name="坏线",
                           waypoint_ids=["w-a", "w-missing"], now_ms=3)


@pytest.mark.asyncio
async def test_commit_route_is_atomic(geo_conn):
    """A duplicate geo_id fails the second commit; the first route survives and
    no partial row from the failed attempt remains (BEGIN IMMEDIATE rollback)."""
    await commit_route(geo_conn, route_id="r-a", name="a",
                       path_points=_path(3), now_ms=1)
    with pytest.raises(Exception):
        await commit_route(geo_conn, route_id="r-a", name="a2",
                           path_points=_path(2), now_ms=2)
    cur = await geo_conn.execute("SELECT COUNT(*) FROM routes")
    assert (await cur.fetchone())[0] == 1        # only the first route


# -- commit_fence --------------------------------------------------------

@pytest.mark.asyncio
async def test_commit_fence_stores_role_and_polygon(fence_conn):
    # WGS84 vertex ring; a forbid fence hard-enforces. MUTATION: not writing role
    # (or defaulting it) loses the S9A.2 classification the HMI + P1 need.
    pts = [(31.20, 121.50), (31.21, 121.50), (31.21, 121.51), (31.20, 121.51)]
    await commit_fence(fence_conn, fence_id="f-forbid", role="forbid",
                       name="东北禁止区", points=pts, now_ms=1)
    cur = await fence_conn.execute(
        "SELECT name, role, kind, hard_enforce, geom_json FROM fences "
        "WHERE fence_id='f-forbid'")
    name, role, kind, hard, geom = await cur.fetchone()
    assert name == "东北禁止区" and role == "forbid" and kind == "polygon" and hard == 1
    assert json.loads(geom)["points"][0] == [31.20, 121.50]


@pytest.mark.asyncio
async def test_commit_fence_warning_never_hard_enforces(fence_conn):
    """11 S9A.2: role=warning hard_enforce is ALWAYS 0. MUTATION: writing 1 (or
    the caller's value) would let a warning zone hard-clip motion."""
    pts = [(31.20, 121.50), (31.21, 121.50), (31.205, 121.51)]
    await commit_fence(fence_conn, fence_id="f-warn", role="warning",
                       points=pts, now_ms=1)
    cur = await fence_conn.execute(
        "SELECT hard_enforce FROM fences WHERE fence_id='f-warn'")
    assert (await cur.fetchone())[0] == 0


@pytest.mark.asyncio
async def test_commit_fence_rejects_bad_role(fence_conn):
    """MUTATION: not checking role lets a typo ('zone', the retired name) reach
    the DB and fail a CHECK with an opaque error."""
    pts = [(31.20, 121.50), (31.21, 121.50), (31.205, 121.51)]
    with pytest.raises(GeoCommitError, match="role"):
        await commit_fence(fence_conn, fence_id="f-x", role="zone",
                           points=pts, now_ms=1)


@pytest.mark.asyncio
async def test_commit_fence_rejects_degenerate(fence_conn):
    """MUTATION: skipping validate_polygon would store a zero-area 'fence'."""
    with pytest.raises(InvalidPolygon):
        await commit_fence(fence_conn, fence_id="f-bad", role="forbid",
                           points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], now_ms=1)


# -- closure checks ------------------------------------------------------

def test_perimeter_closed_ok():
    # last point returns to within tol of the first.
    pts = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.05, 0.05)]
    assert_perimeter_closed(pts, tol_m=0.2)          # no raise


def test_perimeter_open_rejected():
    """MUTATION: dropping the closure check would accept an un-closed walk."""
    pts = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (3.0, 4.0)]   # ends far from start
    with pytest.raises(InvalidPolygon, match="not closed"):
        assert_perimeter_closed(pts, tol_m=0.2)


def test_close_ring_drops_duplicate_closing_vertex():
    """A walked ring whose last point == first must store WITHOUT the dup, or
    polygon_area double-counts the zero-length seam. MUTATION: returning points
    unchanged keeps the duplicate."""
    pts = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 0.0)]   # closed exactly
    ring = close_ring(pts, tol_m=0.2)
    assert ring == [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)]
    # An open ring is returned unchanged.
    open_pts = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)]
    assert close_ring(open_pts, tol_m=0.2) == open_pts


# -- record_fence is now a task-create -----------------------------------

def test_record_fence_start_is_a_task_create():
    from xbrain.p4_agent.runtime.task_request import (
        _TASK_CREATE_INTENTS, is_task_create_intent,
    )
    assert is_task_create_intent("record_fence_start")
    assert _TASK_CREATE_INTENTS["record_fence_start"] == "teach"


# -- commit_waypoint: F06 named keypoint (11 S7.10A / 18 F06) -----------------

@pytest.mark.asyncio
async def test_commit_waypoint_stores_name_and_wgs84(geo_conn):
    """F06 record_waypoint (把这里记为X) -> a named WGS84 keypoint in geo.db, so
    the HMI keypoint layer can label it. MUTATION: not writing name/type breaks
    the NOT NULL columns; wrong coords column drops the point off the map."""
    await commit_waypoint(geo_conn, geo_id="w-eastgate", name="东门岗亭",
                          wtype="poi", rtk_lat=31.2003, rtk_lon=121.5007,
                          yaw_deg=90.0, now_ms=1000)
    cur = await geo_conn.execute(
        "SELECT name, type, rtk_lat, rtk_lon, yaw_deg, arrival_radius "
        "FROM waypoints WHERE geo_id=?", ("w-eastgate",))
    row = await cur.fetchone()
    assert row == ("东门岗亭", "poi", 31.2003, 121.5007, 90.0, 1.0)


@pytest.mark.asyncio
async def test_commit_waypoint_idempotent_on_reid(geo_conn):
    """A redelivered record command (same geo_id) overwrites, never duplicates.
    MUTATION: a plain INSERT would raise on the UNIQUE geo_id the second time."""
    await commit_waypoint(geo_conn, geo_id="w-1", name="旧名", wtype="poi",
                          rtk_lat=31.20, rtk_lon=121.50, now_ms=1)
    await commit_waypoint(geo_conn, geo_id="w-1", name="新名", wtype="poi",
                          rtk_lat=31.21, rtk_lon=121.51, now_ms=2)
    cur = await geo_conn.execute("SELECT COUNT(*), MAX(name) FROM waypoints "
                                 "WHERE geo_id='w-1'")
    assert await cur.fetchone() == (1, "新名")
