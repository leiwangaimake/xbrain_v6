"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_objects.py
Brief: read_geo_objects -> state/geo/objects payload (11 S7.10A)

Description:
Pins the geo read/shape that feeds the HMI map: waypoints/docks come out with
name + ENU metres, a route's geometry is its ordered waypoints' coordinates
(route_waypoint_assoc join, in seq order), tombstoned rows are excluded, a
name-less object keeps name=None (never fabricated), and catalog_rev tracks the
newest edit. Each check names the mutation it reddens (CLAUDE.md 3.3).
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.geo.objects import read_geo_objects
from xbrain.p3_task.persistence.schema_geo import GEO_DB_STATEMENTS

pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def geo():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in GEO_DB_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield c


async def _wp(c, wid, name, x, y, ms, heading=None):
    await c.execute(
        "INSERT INTO waypoints (waypoint_id, name, x_m, y_m, heading_rad, "
        "content_hash, updated_ms) VALUES (?,?,?,?,?,?,?)",
        (wid, name, x, y, heading, "h", ms))


async def _seed(c):
    await _wp(c, "w-1", "东门岗亭", 12.0, -3.5, 100, heading=1.57)
    await _wp(c, "w-2", "西门入口", 20.4, 8.1, 110)
    await c.execute("INSERT INTO routes (route_id, name, content_hash, "
                    "updated_ms) VALUES (?,?,?,?)", ("r-1", "营区日常", "h", 120))
    # assoc: seq 0 -> w-2, seq 1 -> w-1  (deliberately NOT id order, to prove
    # the route follows assoc seq, not waypoint_id order)
    await c.execute("INSERT INTO route_waypoint_assoc (route_id, seq, "
                    "waypoint_id) VALUES ('r-1', 0, 'w-2')")
    await c.execute("INSERT INTO route_waypoint_assoc (route_id, seq, "
                    "waypoint_id) VALUES ('r-1', 1, 'w-1')")
    await c.execute("INSERT INTO docks (dock_id, name, x_m, y_m, heading_rad, "
                    "content_hash, updated_ms) VALUES (?,?,?,?,?,?,?)",
                    ("d-01", "1号充电桩", -40.0, 22.0, 0.0, "h", 130))
    await c.commit()


@pytest.mark.asyncio
async def test_payload_shape_and_names(geo):
    await _seed(geo)
    p = await read_geo_objects(geo)
    assert p["schema"] == "geo_objects_v1"
    assert {w["geo_id"]: w["name"] for w in p["waypoints"]} == {
        "w-1": "东门岗亭", "w-2": "西门入口"}
    # ENU metres passed straight through (frontend places [e_m,n_m] w/o origin).
    w1 = next(w for w in p["waypoints"] if w["geo_id"] == "w-1")
    assert w1["e_m"] == 12.0 and w1["n_m"] == -3.5
    assert p["docks"][0]["name"] == "1号充电桩"


@pytest.mark.asyncio
async def test_route_points_follow_assoc_seq(geo):
    await _seed(geo)
    p = await read_geo_objects(geo)
    r = p["routes"][0]
    assert r["geo_id"] == "r-1" and r["name"] == "营区日常"
    # points in assoc seq order (w-2 then w-1), NOT waypoint_id order. MUTATION:
    # ordering by waypoint_id (or dropping ORDER BY a.seq) reverses the route.
    assert r["points"] == [[20.4, 8.1], [12.0, -3.5]]


@pytest.mark.asyncio
async def test_tombstoned_excluded(geo):
    await _seed(geo)
    await geo.execute("UPDATE waypoints SET tombstone=1 WHERE waypoint_id='w-1'")
    await geo.commit()
    p = await read_geo_objects(geo)
    # MUTATION: dropping the tombstone=0 filter resurrects a deleted point.
    assert [w["geo_id"] for w in p["waypoints"]] == ["w-2"]


@pytest.mark.asyncio
async def test_nameless_object_keeps_null_name(geo):
    # A waypoint with no name (pre-F06 rows) -> name None, never fabricated (GO-4).
    await _wp(geo, "w-x", None, 1.0, 2.0, 200)
    await geo.commit()
    p = await read_geo_objects(geo)
    assert p["waypoints"][0]["name"] is None


@pytest.mark.asyncio
async def test_catalog_rev_tracks_newest_edit(geo):
    await _seed(geo)
    p1 = await read_geo_objects(geo)
    assert p1["catalog_rev"] == 130          # newest updated_ms (the dock)
    # A newer edit bumps catalog_rev so P5 can detect the change (GO-2).
    await _wp(geo, "w-9", "新点", 0.0, 0.0, 999)
    await geo.commit()
    p2 = await read_geo_objects(geo)
    assert p2["catalog_rev"] == 999


@pytest.mark.asyncio
async def test_empty_geo_is_empty_payload(geo):
    p = await read_geo_objects(geo)
    assert p["waypoints"] == [] and p["routes"] == [] and p["docks"] == []
    assert p["catalog_rev"] == 0             # never None -> P5 compare is simple
