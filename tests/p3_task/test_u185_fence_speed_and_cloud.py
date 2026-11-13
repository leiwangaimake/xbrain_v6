"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u185_fence_speed_and_cloud.py
Brief: 99 U185 -- the speed-limit default of 限速区域 fences, set_fence_defaults, cloud-only disable, in_force

Description:
The user's ruling of 2026-11-13 (99 U185), each rule pinned by a case that
names the mutant it kills:
  * three levels: a zone's own limit (sent with it), else the cloud default
    (persisted in fence.db, holds after a restart, overridable, deletable),
    else the configured 0.6;
  * a zone without its own limit follows the default: changing or deleting the
    default rewrites it (rev moves), a zone with its own limit is untouched;
  * set_fence_defaults is cloud-only and range-checked; switching a fence off
    is cloud-only ("关闭围栏只能云端"), switching it on stays open to voice;
  * every fence write says whether it changed what P1 enforces (detail
    in_force), so P5 stops timing out configurations that changed nothing;
  * a recorded speed zone takes the current default.
"""
from __future__ import annotations  # postponed annotations

import math  # metres -> degrees of longitude

import aiosqlite  # in-memory dbs
import pytest  # marker, parametrize
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_CHANNEL_DENIED, E_GEO_INVALID  # expected codes
from xbrain.p3_task.fence.settings import (  # under test
    effective_speed_default,
    reconcile_speed_default,
    stored_speed_default,
)
from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload  # cmd/geo path
from xbrain.p3_task.persistence.schema_geo import FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS  # DDL
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task DDL
from xbrain.p3_task.teach.runtime import TeachRuntime  # under test

pytestmark = pytest.mark.no_device  # pure logic + in-memory sqlite

_LAT, _LON = 34.6970, 135.5050  # a site
_DLAT = 1.0 / 111320.0  # one metre of latitude
_DLON = _DLAT / math.cos(math.radians(_LAT))  # one metre of longitude
_SPEED = {"default": 0.6, "min": 0.1, "max": 2.0}  # p3_task.yaml fence.speed_limit_*_mps


def _sq(x0, y0, side):  # [(lat, lon)] corners, metres east / north of the site
    return [(_LAT + y * _DLAT, _LON + x * _DLON)  # one corner
            for x, y in ((x0, y0), (x0, y0 + side), (x0 + side, y0 + side), (x0 + side, y0))]


async def _open(statements):  # in-memory db with a schema
    conn = await aiosqlite.connect(":memory:")  # fresh
    for stmt in statements:  # schema
        await conn.execute(stmt)  # one DDL
    await conn.commit()  # ready
    return conn  # caller closes


@pytest_asyncio.fixture  # the cmd/geo context as p3 builds it, speed config included
async def ctx():
    geo, fence = await _open(GEO_DB_STATEMENTS), await _open(FENCE_DB_STATEMENTS)  # dbs
    yield GeoContext(geo_conn=geo, fence_conn=fence, task_conn=None, speed_limit=_SPEED)  # U185
    await geo.close()  # cleanup
    await fence.close()  # cleanup


# cmd_ids must be unique per command: P3 replays a seen cmd_id as a duplicate
_N = [0]  # cmd_id counter


def _cmd(action, origin="cloud", **over):  # one cmd/geo payload
    _N[0] += 1  # unique cmd_id
    base = {"cmd_id": "c-%d" % _N[0], "action": action, "origin": origin}  # envelope
    base.update(over)  # the action's fields
    return base  # payload


def _speed_zone(fid, ring, limit=None):  # upsert a speed_limit fence
    geom = {"role": "speed_limit", "outer": [{"lat": a, "lon": b} for a, b in ring]}  # 11 S7.8.3
    if limit is not None:  # its own limit
        geom["speed_limit_mps"] = limit  # sent with it
    return _cmd("upsert", type="fence", geo_id=fid, base_rev=0, obj={"name": fid, "geom": geom})


async def _zone(ctx, fid):  # (speed_limit_mps, speed_limit_own, rev) of one fence
    cur = await ctx.fence_conn.execute(
        "SELECT speed_limit_mps, speed_limit_own, rev FROM fences WHERE fence_id=?", (fid,))
    return await cur.fetchone()  # one row


async def _on(ctx, fid, origin="cloud"):  # switch a fence on (F15 / the cloud)
    rev = (await _zone(ctx, fid))[2]  # voice goes through optimistic concurrency
    return await handle_geo_payload(_cmd("set_state", origin=origin, type="fence", geo_id=fid,
                                         force=origin == "cloud", base_rev=rev,
                                         obj={"state": "active"}), ctx, now_ms=50)


async def _defaults(ctx, value, origin="cloud"):  # set_fence_defaults
    return await handle_geo_payload(_cmd("set_fence_defaults", origin=origin,
                                         obj={"speed_limit_default_mps": value}), ctx, now_ms=60)


# -- three levels -------------------------------------------------------------
@pytest.mark.asyncio  # async case
async def test_a_zone_without_its_own_limit_follows_the_default(ctx):
    """Level 3 when nothing else is set; level 1 kept when sent.
    MUTATION: own flag always 1 -> the follower is not rewritten later -> red;
    the configured default not used -> the row has no limit -> red."""
    assert (await handle_geo_payload(_speed_zone("f-yard", _sq(0, 0, 30)), ctx,
                                     now_ms=1))["result"] == "accepted"  # no limit sent
    assert (await handle_geo_payload(_speed_zone("f-gate", _sq(50, 0, 30), limit=1.2), ctx,
                                     now_ms=2))["result"] == "accepted"  # its own limit
    assert (await _zone(ctx, "f-yard"))[:2] == (0.6, 0)  # follows the configured 0.6
    assert (await _zone(ctx, "f-gate"))[:2] == (1.2, 1)  # keeps its own


@pytest.mark.asyncio  # async case
async def test_the_cloud_default_overrides_and_its_deletion_restores(ctx):
    """Set: followers move (rev too), own-limit zones do not; the value is
    persisted. Delete: back to 0.6. MUTATION: rewrite own = 1 rows too -> red;
    no rev bump -> crc / catalogue detectors miss it -> red; persist nothing ->
    stored_speed_default None -> red."""
    await handle_geo_payload(_speed_zone("f-yard", _sq(0, 0, 30)), ctx, now_ms=1)  # follower
    await handle_geo_payload(_speed_zone("f-gate", _sq(50, 0, 30), limit=1.2), ctx, now_ms=2)  # own
    rev0 = (await _zone(ctx, "f-yard"))[2]  # before
    ack = await _defaults(ctx, 0.8)  # the cloud sets 0.8
    assert ack["result"] == "accepted" and ack["detail"] == {
        "speed_limit_default_mps": 0.8, "source": "cloud", "zones_updated": 1,
        "in_force": False}  # the follower is a draft: nothing in force moved
    assert (await _zone(ctx, "f-yard"))[:2] == (0.8, 0) and (await _zone(ctx, "f-yard"))[2] == rev0 + 1
    assert (await _zone(ctx, "f-gate"))[:2] == (1.2, 1)  # untouched
    assert await stored_speed_default(ctx.fence_conn) == 0.8  # persisted (holds a restart)
    gone = await _defaults(ctx, None)  # the cloud deletes it
    assert gone["detail"]["speed_limit_default_mps"] == 0.6 and gone["detail"]["source"] == "config"
    assert (await _zone(ctx, "f-yard"))[0] == 0.6  # back to the configured value
    assert await stored_speed_default(ctx.fence_conn) is None  # deleted


@pytest.mark.asyncio  # async case
async def test_a_default_change_reaching_an_active_zone_is_in_force(ctx):
    """P5 waits for state/fence only then. MUTATION: in_force from the row
    count instead of the ACTIVE rows -> a draft-only change waits -> red (the
    case above); active rows ignored -> red here."""
    await handle_geo_payload(_speed_zone("f-yard", _sq(0, 0, 30)), ctx, now_ms=1)  # follower
    await _on(ctx, "f-yard")  # in force
    assert (await _defaults(ctx, 1.0))["detail"]["in_force"] is True  # P1 sees the new limit


@pytest.mark.asyncio  # async case
@pytest.mark.parametrize("value", [0.05, 2.5, True, "0.8"])
async def test_a_default_outside_the_bounds_is_refused(ctx, value):
    """MUTATION: drop the range / type check -> accepted -> red."""
    ack = await _defaults(ctx, value)  # out of [0.1, 2.0] or not a number
    assert ack["result"] == "rejected" and ack["code"] == E_GEO_INVALID
    assert await stored_speed_default(ctx.fence_conn) is None  # nothing stored


@pytest.mark.asyncio  # async case
async def test_a_zone_limit_outside_the_bounds_is_refused(ctx):
    """MUTATION: no range check on a sent limit -> a 3 m/s "limit" -> red."""
    ack = await handle_geo_payload(_speed_zone("f-x", _sq(0, 0, 30), limit=3.0), ctx, now_ms=1)
    assert ack["result"] == "rejected" and ack["detail"]["reason"] == "speed_limit_out_of_range"


@pytest.mark.asyncio  # async case
async def test_reconcile_brings_followers_to_the_effective_default(ctx):
    """At start, with the configured value changed since the last run.
    MUTATION: reconcile does nothing -> red."""
    await handle_geo_payload(_speed_zone("f-yard", _sq(0, 0, 30)), ctx, now_ms=1)  # at 0.6
    assert await reconcile_speed_default(ctx.fence_conn, 0.5, 9) == 1  # the yaml now says 0.5
    assert (await _zone(ctx, "f-yard"))[0] == 0.5  # moved
    assert await reconcile_speed_default(ctx.fence_conn, 0.5, 10) == 0  # nothing left to move
    assert await effective_speed_default(ctx.fence_conn, 0.5) == 0.5  # level 3


# -- channels -----------------------------------------------------------------
@pytest.mark.asyncio  # async case
async def test_only_the_cloud_sets_defaults_or_switches_a_fence_off(ctx):
    """"关闭围栏只能云端"; voice may still switch on (F15). MUTATION: no
    disable_fence cell -> voice switches a fence off -> red; set_fence_defaults
    open to voice -> red; the cell applied to switching on -> F15 breaks -> red."""
    await handle_geo_payload(_speed_zone("f-yard", _sq(0, 0, 30)), ctx, now_ms=1)  # a draft
    assert (await _defaults(ctx, 0.8, origin="voice"))["code"] == E_CHANNEL_DENIED
    on = await _on(ctx, "f-yard", origin="voice")  # F15
    assert on["result"] == "accepted" and on["detail"]["in_force"] is True  # switched on
    off = await handle_geo_payload(_cmd("set_state", origin="voice", type="fence", geo_id="f-yard",
                                        base_rev=2, obj={"state": "disabled"}), ctx, now_ms=70)
    assert off["result"] == "rejected" and off["code"] == E_CHANNEL_DENIED  # voice may not
    cloud_off = await handle_geo_payload(_cmd("set_state", type="fence", geo_id="f-yard", force=True,
                                              obj={"state": "disabled"}), ctx, now_ms=71)
    assert cloud_off["result"] == "accepted" and cloud_off["detail"]["in_force"] is True


# -- in_force -----------------------------------------------------------------
@pytest.mark.asyncio  # async case
async def test_in_force_marks_only_writes_that_change_what_p1_enforces(ctx):
    """A new fence (draft) no; switching on yes; on again no; an edit of an
    active fence yes; deleting an active fence yes. MUTATION: in_force True for
    every write -> red; False for the delete -> red."""
    ctx.task_conn = await _open(ALL_DDL_STATEMENTS)  # delete computes task refs
    try:  # a failing assert must still close it: its worker thread is not a daemon
        new = await handle_geo_payload(_speed_zone("f-yard", _sq(0, 0, 30)), ctx, now_ms=1)  # draft
        assert new["detail"]["in_force"] is False  # nothing in force yet
        assert (await _on(ctx, "f-yard"))["detail"]["in_force"] is True  # now in force
        again = await handle_geo_payload(_cmd("set_state", type="fence", geo_id="f-yard", force=True,
                                              obj={"state": "active"}), ctx, now_ms=80)
        assert again["detail"]["in_force"] is False  # already on: nothing changed
        edit = await handle_geo_payload(_cmd("upsert", type="fence", geo_id="f-yard", base_rev=3,
                                             obj={"name": "f-yard", "geom": {
                                                 "role": "speed_limit",
                                                 "outer": [{"lat": a, "lon": b} for a, b in _sq(0, 0, 40)]}}),
                                        ctx, now_ms=81)
        assert edit["result"] == "accepted" and edit["detail"]["in_force"] is True  # in force, edited
        gone = await handle_geo_payload(_cmd("delete", type="fence", geo_id="f-yard", base_rev=4),
                                        ctx, now_ms=82)
        assert gone["result"] == "accepted" and gone["detail"]["in_force"] is True  # deleted in force
    finally:
        await ctx.task_conn.close()  # cleanup, or pytest hangs at exit


# -- teach ----------------------------------------------------------------------
@pytest.mark.asyncio  # async case
async def test_a_recorded_speed_zone_takes_the_current_default(ctx):
    """The cloud value when there is one. MUTATION: record with the configured
    value regardless -> 0.6 -> red."""
    task = await _open(ALL_DDL_STATEMENTS)  # task.db
    try:
        await _defaults(ctx, 0.9)  # the cloud default
        rt = TeachRuntime(task, ctx.geo_conn, ctx.fence_conn, boot_id="b0", sample_hz=1.0,
                          min_dist_m=0.5, snap_max_age_ms=8000, record_duration_s=10800,
                          fence_speed_limit=_SPEED)  # U185 config
        pose = {"fix_type": "rtk_fixed", "heading_rad": 0.0, "heading_valid": True,
                "heading_level": 1, "speed_mps": 1.0}  # a usable fix
        rt.update_pose(dict(pose, lat=_LAT, lon=_LON), 100.0)  # fresh
        rt.update_health({"allow_motion": True})  # arming checks
        rt.update_robot({"hes": False, "estop_path": "up"})
        rt.update_power({"soc_pct": 80.0})
        rt.update_teleop({"sources": [{"device": "gamepad", "stale": False, "alive": True}]})
        start = await rt.handle({"cmd_id": "s1", "action": "start", "start": {
            "kind": "fence", "name": "路口限速区域围栏"},
            "issuer": {"src": "p4_agent", "channel": "local_voice"}}, now_mono_s=100.0, now_ms=1)
        assert start["result"] == "accepted", start  # never refused for the limit now
        for i, (la, lo) in enumerate(_sq(0, 0, 30)):  # drive the corners
            rt.update_pose(dict(pose, lat=la, lon=lo), 101.0 + i)  # there
            await rt.offer_pose(101.0 + i, 1000 + i)  # sampled
        fin = await rt.handle({"cmd_id": "f1", "action": "finish",
                               "session_id": start["detail"]["session_id"],
                               "issuer": {"src": "p4_agent", "channel": "local_voice"}},
                              now_mono_s=110.0, now_ms=2)  # exit = save
        assert fin["detail"]["saved"] is True  # saved
        cur = await ctx.fence_conn.execute(
            "SELECT speed_limit_mps, speed_limit_own FROM fences WHERE name='路口限速区域围栏'")
        assert await cur.fetchone() == (0.9, 0)  # the cloud default, following it
    finally:
        await task.close()  # cleanup
