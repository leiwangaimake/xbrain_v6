"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_objects.py
Brief: read_geo_objects -> state/geo/objects payload (11 S7.10A)

Description:
Pins the geo read/shape that feeds the HMI map (v1.5 PLAN A / 15 S9.3): waypoints
are NAMED WGS84 keypoints emitted as lat/lon, a route's geometry is INLINE
(path_points mode B verbatim, or waypoint_ids mode A resolved to the anchors'
lat/lon), tombstoned rows are excluded, and catalog_rev tracks the newest edit.
Route points come out as {lat,lon} OBJECTS (an array would be read as ENU by the
frontend). Each check names the mutation it reddens (CLAUDE.md 3.3).
"""
from __future__ import annotations

import json

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


async def _wp(c, gid, name, lat, lon, ms, yaw=None, wtype="poi"):
    await c.execute(
        "INSERT INTO waypoints (geo_id, name, type, rtk_lat, rtk_lon, yaw_deg, "
        "content_hash, updated_ms) VALUES (?,?,?,?,?,?,?,?)",
        (gid, name, wtype, lat, lon, yaw, "h", ms))


async def _route_pp(c, rid, name, path_points, ms):
    # mode B: inline path_points [[lat,lon],...]
    await c.execute(
        "INSERT INTO routes (geo_id, name, path_points, loop_mode, direction, "
        "total_len_m, content_hash, updated_ms) VALUES (?,?,?,?,?,?,?,?)",
        (rid, name, json.dumps(path_points), "oneway", "forward", 100.0, "h", ms))


async def _route_wi(c, rid, name, waypoint_ids, ms):
    # mode A: named anchors [geo_id,...]
    await c.execute(
        "INSERT INTO routes (geo_id, name, waypoint_ids, loop_mode, direction, "
        "total_len_m, content_hash, updated_ms) VALUES (?,?,?,?,?,?,?,?)",
        (rid, name, json.dumps(waypoint_ids), "oneway", "forward", 100.0, "h", ms))


async def _seed(c):
    # two named WGS84 keypoints -> the dots layer.
    await _wp(c, "w-1", "东门岗亭", 31.2003, 121.5007, 100, yaw=90.0)
    await _wp(c, "w-2", "西门入口", 31.2010, 121.4990, 110)
    # a mode-B route: inline path_points (its own geometry, NOT keypoints).
    await _route_pp(c, "r-1", "营区日常",
                    [[31.2010, 121.4990], [31.2003, 121.5007]], 120)
    # dock: full 15 S9.3 WGS84 model (body + handover point). Not rendered on the
    # map, but emitted as lat/lon for coordinate consistency.
    await c.execute(
        "INSERT INTO docks (geo_id, name, rtk_lat, rtk_lon, dock_heading_rad, "
        "handover_lat, handover_lon, handover_heading_rad, content_hash, updated_ms) "
        "VALUES (?,?,?,?,?,?,?,?,?,?)",
        ("d-01", "1号充电桩", 31.2005, 121.5005, 0.0, 31.2006, 121.5006, 0.0, "h", 130))
    await c.commit()


@pytest.mark.asyncio
async def test_payload_shape_and_names(geo):
    await _seed(geo)
    p = await read_geo_objects(geo)
    assert p["schema"] == "geo_objects_v1"
    assert {w["geo_id"]: w["name"] for w in p["waypoints"]} == {
        "w-1": "东门岗亭", "w-2": "西门入口"}
    # WGS84 lat/lon emitted (frontend projects via enu_origin). MUTATION: reading
    # the wrong column (or the old e_m/n_m) drops the keypoint off the map.
    w1 = next(w for w in p["waypoints"] if w["geo_id"] == "w-1")
    assert w1["lat"] == 31.2003 and w1["lon"] == 121.5007
    assert p["docks"][0]["name"] == "1号充电桩"


@pytest.mark.asyncio
async def test_waypoint_type_rides_along(geo):
    """2026-11-12: the HMI draws charging / return-home points by type (17
    S6.10.2 U6). MUTATION: drop type from the SELECT / dict -> None -> red."""
    await _wp(geo, "w-9", "1号充电站", 31.2, 121.5, 100, wtype="dock")
    await _wp(geo, "w-8", "返航终点站", 31.2, 121.5, 100, wtype="home")
    await geo.commit()
    p = await read_geo_objects(geo)
    assert {w["geo_id"]: w["type"] for w in p["waypoints"]} == {"w-9": "dock", "w-8": "home"}


@pytest.mark.asyncio
async def test_route_points_from_path_points_in_order(geo):
    await _seed(geo)
    p = await read_geo_objects(geo)
    r = p["routes"][0]
    assert r["geo_id"] == "r-1" and r["name"] == "营区日常"
    # points are {lat,lon} OBJECTS in path_points order. MUTATION: emitting [a,b]
    # arrays makes the frontend read them as ENU; reversing drops the order.
    assert r["points"] == [{"lat": 31.2010, "lon": 121.4990},
                           {"lat": 31.2003, "lon": 121.5007}]


@pytest.mark.asyncio
async def test_route_mode_a_resolves_anchor_coords(geo):
    # mode A: a route referencing named keypoints resolves to their lat/lon; a
    # tombstoned/missing anchor drops from the line (never fabricated). MUTATION:
    # not resolving waypoint_ids leaves the route with no geometry.
    await _seed(geo)
    await _route_wi(geo, "r-2", "锚点巡线", ["w-1", "w-2", "w-missing"], 140)
    await geo.commit()
    p = await read_geo_objects(geo)
    r2 = next(r for r in p["routes"] if r["geo_id"] == "r-2")
    assert r2["points"] == [{"lat": 31.2003, "lon": 121.5007},
                            {"lat": 31.2010, "lon": 121.4990}]   # missing one dropped


@pytest.mark.asyncio
async def test_tombstoned_excluded(geo):
    await _seed(geo)
    await geo.execute("UPDATE waypoints SET tombstone=1 WHERE geo_id='w-1'")
    await geo.commit()
    p = await read_geo_objects(geo)
    # MUTATION: dropping the tombstone=0 filter resurrects a deleted point.
    assert [w["geo_id"] for w in p["waypoints"]] == ["w-2"]


@pytest.mark.asyncio
async def test_catalog_rev_tracks_newest_edit(geo):
    await _seed(geo)
    p1 = await read_geo_objects(geo)
    assert p1["catalog_rev"] == 130          # newest updated_ms (the dock)
    # A newer edit bumps catalog_rev so P5 can detect the change (GO-2).
    await _wp(geo, "w-9", "新点", 31.2020, 121.5000, 999)
    await geo.commit()
    p2 = await read_geo_objects(geo)
    assert p2["catalog_rev"] == 999


@pytest.mark.asyncio
async def test_empty_geo_is_empty_payload(geo):
    p = await read_geo_objects(geo)
    assert p["waypoints"] == [] and p["routes"] == [] and p["docks"] == []
    assert p["catalog_rev"] == 0             # never None -> P5 compare is simple


# -- 99 U183: fences ride state/geo/objects, every live fence with its state ---
@pytest.mark.asyncio
async def test_every_live_fence_rides_the_payload_with_its_state(geo):
    """The catalogue, not the enforced set: a fence just recorded (draft) or
    switched off must reach the HMI and the cloud too; a tombstone must not.
    MUTATION: filter fences on state='active' -> the draft is missing -> red;
    drop the fence updated_ms from catalog_rev -> a fence switched on changes
    nothing a consumer compares -> red."""
    from xbrain.p3_task.ingest.geo_commit import commit_fence  # the writer teach uses
    from xbrain.p3_task.persistence.schema_geo import FENCE_DB_STATEMENTS  # fence DDL
    await _seed(geo)
    async with aiosqlite.connect(":memory:") as fc:
        for stmt in FENCE_DB_STATEMENTS:
            await fc.execute(stmt)
        ring = [(31.20, 121.50), (31.21, 121.50), (31.21, 121.51)]
        await commit_fence(fc, fence_id="f-a", role="allow", points=ring, now_ms=900,
                           name="营区活动区域围栏", state="active", created_by="teach")
        await commit_fence(fc, fence_id="f-s", role="speed_limit", points=ring,
                           now_ms=950, name="路口限速区域围栏", state="draft",
                           created_by="teach", speed_limit_mps=0.6)
        await commit_fence(fc, fence_id="f-x", role="forbid", points=ring, now_ms=990,
                           name="旧禁止区域围栏", state="draft", created_by="teach")
        await fc.execute("UPDATE fences SET state='deleted', tombstone=1 WHERE fence_id='f-x'")
        await fc.commit()
        p = await read_geo_objects(geo, fc)
    assert [(f["geo_id"], f["role"], f["state"]) for f in p["fences"]] == [
        ("f-a", "allow", "active"), ("f-s", "speed_limit", "draft")]  # no tombstone
    assert p["fences"][0]["vertices"][0] == {"lat": 31.20, "lon": 121.50}  # {lat,lon}
    assert p["fences"][1]["speed_limit_mps"] == 0.6  # only the speed zone has it
    assert "speed_limit_mps" not in p["fences"][0]
    assert p["catalog_rev"] == 990  # the tombstone's write counts too (GO-2 token)


@pytest.mark.asyncio
async def test_a_geo_only_read_has_no_fences_key(geo):
    """The delete sweep and older callers pass no fence connection: no fences
    key at all, so P5 greys the layer rather than drawing "no fences"."""
    await _seed(geo)
    assert "fences" not in await read_geo_objects(geo)
