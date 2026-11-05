"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_waypoint_kind.py
Brief: 99 U119 -- waypoint type from its name on create / rename, and the return-home target

Description:
The user ruled (2026-11-05) that a waypoint named exactly 返航终点站 is the
home point and a name containing 充电桩 / 充电站 is a dock point. These
cases pin the rule at every write path (voice F06 via teach, cmd/geo upsert,
cmd/geo rename) and the lookup that the return_home dispatch uses. The
dispatch half itself is in test_route_push.py (the live wiring).
"""
from __future__ import annotations

import aiosqlite  # in-memory geo.db (CLAUDE.md 7.2)
import pytest  # parametrize, marker
import pytest_asyncio  # async fixtures

from xbrain.p3_task.geo.waypoint_kind import (  # unit under test
    renamed_waypoint_type,  # rename rule
    resolve_return_target,  # dispatch lookup
    waypoint_type_for_name,  # create rule
)
from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload  # the cmd/geo path
from xbrain.p3_task.persistence.schema_geo import FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS  # DDL

pytestmark = pytest.mark.no_device  # pure host-side logic


@pytest.mark.parametrize("name,want", [
    ("返航终点站", "home"),  # the one exact name
    ("旧返航终点站", None), ("返航终点站2", None),  # MUTATION: substring match makes these homes
    ("1号充电桩", "dock"), ("东门充电站", "dock"),  # dock by fragment
    ("1号测试点", None),  # an ordinary keypoint
])
def test_type_for_name(name, want):
    """MUTATION: substring for home / equality for dock -> reddens."""
    assert waypoint_type_for_name(name) == want  # the rule


@pytest.mark.parametrize("new,cur,want", [
    ("返航终点站", "poi", "home"),  # renamed into the home name
    ("2号充电桩", "poi", "dock"),  # into a dock name
    ("1号测试点", "home", "poi"),  # out of a role: dropped back
    ("1号测试点", "gate", None),  # an unrelated type is left alone
    ("返航终点站", "home", None),  # no change, no write
])
def test_rename_rule(new, cur, want):
    """MUTATION: keep the old role after a rename out of it -> case 3 red."""
    assert renamed_waypoint_type(new, cur) == want  # the type to write, or None


async def _open(statements):
    conn = await aiosqlite.connect(":memory:")  # in-memory db
    for stmt in statements:  # schema
        await conn.execute(stmt)
    await conn.commit()  # ready
    return conn  # caller closes


@pytest_asyncio.fixture
async def ctx():
    geo, fence = await _open(GEO_DB_STATEMENTS), await _open(FENCE_DB_STATEMENTS)  # both dbs
    yield GeoContext(geo_conn=geo, fence_conn=fence, task_conn=None)  # the applier context
    await geo.close()
    await fence.close()


def _upsert(cmd_id, geo_id, name, lat=35.0, **extra):
    obj = {"name": name, "geom": {"lat": lat, "lon": 135.0}}  # a point
    obj.update(extra)  # e.g. wtype
    return {"cmd_id": cmd_id, "action": "upsert", "type": "waypoint", "geo_id": geo_id,
            "origin": "hmi", "base_rev": 0, "obj": obj}  # 11 S7.9 upsert


async def _type(conn, geo_id):
    cur = await conn.execute("SELECT type FROM waypoints WHERE geo_id=?", (geo_id,))  # one row
    return (await cur.fetchone())[0]  # its type


@pytest.mark.asyncio
async def test_cmd_geo_create_and_rename_follow_the_name(ctx):
    """The HMI / cloud write paths. MUTATION: keep obj.wtype over the name ->
    the home is stored poi; skip the rename hook -> a renamed point keeps or
    never gets its role."""
    ack = await handle_geo_payload(_upsert("c-1", "w-h", "返航终点站", wtype="poi"), ctx, now_ms=1)
    assert ack["result"] == "accepted" and await _type(ctx.geo_conn, "w-h") == "home"  # name wins
    await handle_geo_payload(_upsert("c-2", "w-p", "1号测试点"), ctx, now_ms=2)  # ordinary point
    assert await _type(ctx.geo_conn, "w-p") == "poi"  # default kept
    await handle_geo_payload({"cmd_id": "c-3", "action": "rename", "type": "waypoint",
                              "geo_id": "w-p", "origin": "hmi", "base_rev": 1,
                              "obj": {"name": "1号充电桩"}}, ctx, now_ms=3)  # into a dock name
    assert await _type(ctx.geo_conn, "w-p") == "dock"  # role set by rename
    await handle_geo_payload({"cmd_id": "c-4", "action": "rename", "type": "waypoint",
                              "geo_id": "w-h", "origin": "hmi", "base_rev": 1,
                              "obj": {"name": "旧终点"}}, ctx, now_ms=4)  # out of the home name
    assert await _type(ctx.geo_conn, "w-h") == "poi"  # role dropped


@pytest.mark.asyncio
async def test_resolve_skips_deleted_and_disabled_points(ctx):
    """MUTATION: drop state='active' / tombstone=0 -> a deleted home wins."""
    await handle_geo_payload(_upsert("c-1", "w-h", "返航终点站"), ctx, now_ms=1)  # a home
    await ctx.geo_conn.execute("UPDATE waypoints SET state='deleted', tombstone=1 WHERE geo_id='w-h'")
    await ctx.geo_conn.commit()  # close the raw write before the applier opens its own
    await handle_geo_payload(_upsert("c-2", "w-d", "1号充电桩", lat=35.01), ctx, now_ms=2)  # a dock
    assert await resolve_return_target(ctx.geo_conn, (35.0, 135.0)) == ("w-d", "dock")  # the live dock
    assert await resolve_return_target(ctx.geo_conn, None) == ("w-d", "dock")  # no pose: oldest dock
