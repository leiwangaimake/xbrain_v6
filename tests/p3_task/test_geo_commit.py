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
from xbrain.p3_task.lifecycle.teach import TeachSample
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


def _samples(n):
    return [TeachSample(x_m=float(i), y_m=0.0, heading_rad=0.0)
            for i in range(n)]


# -- commit_route --------------------------------------------------------

@pytest.mark.asyncio
async def test_commit_route_writes_waypoints_and_assoc(geo_conn):
    await commit_route(geo_conn, route_id="r-eastgate", name="东门路线",
                       samples=_samples(3), now_ms=1)
    cur = await geo_conn.execute("SELECT COUNT(*) FROM waypoints")
    assert (await cur.fetchone())[0] == 3
    cur = await geo_conn.execute(
        "SELECT waypoint_id FROM route_waypoint_assoc "
        "WHERE route_id='r-eastgate' ORDER BY seq")
    assert [r[0] for r in await cur.fetchall()] == [
        "r-eastgate-w000", "r-eastgate-w001", "r-eastgate-w002"]
    cur = await geo_conn.execute("SELECT name FROM routes WHERE route_id=?",
                                 ("r-eastgate",))
    assert (await cur.fetchone())[0] == "东门路线"


@pytest.mark.asyncio
async def test_commit_route_rejects_too_few_points(geo_conn):
    with pytest.raises(GeoCommitError, match=">= 2"):
        await commit_route(geo_conn, route_id="r-x", name="x",
                           samples=_samples(1), now_ms=1)


@pytest.mark.asyncio
async def test_commit_route_rejects_over_cap(geo_conn):
    """MUTATION: dropping the cap check lets a 17-point route through the
    Python layer (the DB trigger also aborts, but with a less clear error)."""
    with pytest.raises(GeoCommitError, match="cap"):
        await commit_route(geo_conn, route_id="r-x", name="x",
                           samples=_samples(17), now_ms=1)


@pytest.mark.asyncio
async def test_commit_route_is_atomic(geo_conn):
    """A duplicate route_id fails the second commit and leaves NO orphan
    waypoints from the failed attempt (BEGIN IMMEDIATE rollback)."""
    await commit_route(geo_conn, route_id="r-a", name="a",
                       samples=_samples(3), now_ms=1)
    with pytest.raises(Exception):
        await commit_route(geo_conn, route_id="r-a", name="a2",
                           samples=_samples(2), now_ms=2)
    cur = await geo_conn.execute("SELECT COUNT(*) FROM waypoints")
    assert (await cur.fetchone())[0] == 3        # only the first route's 3


# -- commit_fence --------------------------------------------------------

@pytest.mark.asyncio
async def test_commit_fence_stores_validated_polygon(fence_conn):
    pts = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    await commit_fence(fence_conn, fence_id="f-perimeter", points=pts, now_ms=1)
    cur = await fence_conn.execute(
        "SELECT kind, geom_json FROM fences WHERE fence_id='f-perimeter'")
    kind, geom = await cur.fetchone()
    assert kind == "polygon"
    assert json.loads(geom)["points"][0] == [0.0, 0.0]


@pytest.mark.asyncio
async def test_commit_fence_rejects_degenerate(fence_conn):
    """MUTATION: skipping validate_polygon would store a zero-area 'fence'."""
    with pytest.raises(InvalidPolygon):
        await commit_fence(fence_conn, fence_id="f-bad",
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
async def test_commit_waypoint_stores_name_and_coords(geo_conn):
    """F06 record_waypoint (把这里记为X) -> a named keypoint in geo.db, so the HMI
    keypoint layer can label it. MUTATION: not writing name -> the keypoint shows
    as an unlabelled dot forever (the pre-F06 gap this closes)."""
    await commit_waypoint(geo_conn, waypoint_id="w-东门岗亭", name="东门岗亭",
                          x_m=12.0, y_m=-3.5, heading_rad=1.57, now_ms=1000)
    cur = await geo_conn.execute(
        "SELECT name, x_m, y_m, heading_rad FROM waypoints WHERE waypoint_id=?",
        ("w-东门岗亭",))
    row = await cur.fetchone()
    assert row == ("东门岗亭", 12.0, -3.5, 1.57)


@pytest.mark.asyncio
async def test_commit_waypoint_idempotent_on_reid(geo_conn):
    """A redelivered record command (same id) overwrites, never duplicates.
    MUTATION: a plain INSERT would raise on the PK the second time."""
    await commit_waypoint(geo_conn, waypoint_id="w-1", name="旧名",
                          x_m=1.0, y_m=1.0, heading_rad=None, now_ms=1)
    await commit_waypoint(geo_conn, waypoint_id="w-1", name="新名",
                          x_m=2.0, y_m=2.0, heading_rad=None, now_ms=2)
    cur = await geo_conn.execute("SELECT COUNT(*), MAX(name) FROM waypoints "
                                 "WHERE waypoint_id='w-1'")
    assert await cur.fetchone() == (1, "新名")
