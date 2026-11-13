"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u194_activity_area.py
Brief: 99 U194 -- P3 itself keeps "the activity area is recorded only, at most one in force"

Description:
Measured 2026-11-13 (second audit of the fence plan): over cmd/geo, three
activity-area drafts could be made (origin voice, hmi, cloud), and switching
on a second one was refused only by the FS-5A trigger, reaching the sender
as E_INTERNAL. The gateway already refused cloud keep_in upserts and HMI
fence writes, and P4 sends none -- but P3 is the single writer and now
holds the rule itself (user: "把这些缺口补上"). Each case names the mutant it
kills.
"""
from __future__ import annotations  # postponed annotations

import aiosqlite  # in-memory dbs
import pytest  # markers, parametrize

from xbrain.common.errors import E_GEO_INVALID  # expected code
from xbrain.p3_task.ingest.geo_apply import (  # cmd/geo path
    GeoContext,  # the context P3 builds
    handle_geo_payload,  # the cmd/geo entry
)
from xbrain.p3_task.ingest.geo_commit import commit_fence  # the recording's writer
from xbrain.p3_task.persistence.schema_geo import (  # DDL
    FENCE_DB_STATEMENTS,  # fence.db
    GEO_DB_STATEMENTS,  # geo.db
)

pytestmark = pytest.mark.no_device  # pure logic + in-memory sqlite

_RING = [(34.6970, 135.5050), (34.6975, 135.5050), (34.6975, 135.5055), (34.6970, 135.5055)]
_N = [0]  # cmd_id counter: a repeated cmd_id is replayed as a duplicate


async def _ctx():  # a fresh cmd/geo context
    dbs = []  # opened connections
    for stmts in (GEO_DB_STATEMENTS, FENCE_DB_STATEMENTS):  # geo, fence
        c = await aiosqlite.connect(":memory:")  # in memory
        for s in stmts:  # schema
            await c.execute(s)  # one DDL
        await c.commit()  # ready
        dbs.append(c)  # keep
    return GeoContext(geo_conn=dbs[0], fence_conn=dbs[1])  # the geo path alone


async def _close(ctx):  # cleanup
    await ctx.geo_conn.close()  # geo
    await ctx.fence_conn.close()  # fence


async def _geo(ctx, action, origin="cloud", **kw):  # one cmd/geo command
    _N[0] += 1  # unique cmd_id
    return await handle_geo_payload(dict({"cmd_id": "u%d" % _N[0], "action": action,
                                          "origin": origin}, **kw), ctx, now_ms=_N[0])


def _upsert_obj(name, role):  # the fence object cmd/geo carries
    return {"name": name, "geom": {"role": role,
                                   "outer": [{"lat": a, "lon": b} for a, b in _RING]}}


async def _rows(ctx):  # (fence_id, role) of every stored fence
    cur = await ctx.fence_conn.execute("SELECT fence_id, role FROM fences")  # all
    return await cur.fetchall()  # rows


@pytest.mark.asyncio
@pytest.mark.parametrize("origin", ["voice", "hmi", "cloud"])
async def test_cmd_geo_never_creates_an_activity_area(origin):
    """MUTATION: the role check removed -> stored as a draft -> red."""
    ctx = await _ctx()  # fresh
    try:
        ack = await _geo(ctx, "upsert", origin, type="fence", geo_id="f-camp", base_rev=0,
                         obj=_upsert_obj("营区活动区域", "allow"))  # the measured write
        assert ack["result"] == "rejected" and ack["code"] == E_GEO_INVALID  # refused
        assert ack["detail"]["reason"] == "activity_area_recording_only"  # named
        assert await _rows(ctx) == []  # nothing stored
    finally:  # always
        await _close(ctx)  # cleanup


@pytest.mark.asyncio
async def test_a_recorded_activity_area_cannot_be_rewritten_or_retyped():
    """The recorded one is neither replaced nor turned into another role.
    MUTATION: only the NEW role checked -> retyping to forbid passes -> red."""
    ctx = await _ctx()  # fresh
    try:
        await commit_fence(ctx.fence_conn, fence_id="f-camp", role="allow", points=_RING,
                           now_ms=1, state="draft", name="营区活动区域")  # recorded
        for role in ("allow", "forbid"):  # rewrite, retype
            ack = await _geo(ctx, "upsert", type="fence", geo_id="f-camp", base_rev=1,
                             force=True, obj=_upsert_obj("营区活动区域", role))
            assert ack["detail"]["reason"] == "activity_area_recording_only", ack  # refused
        assert await _rows(ctx) == [("f-camp", "allow")]  # untouched
    finally:  # always
        await _close(ctx)  # cleanup


@pytest.mark.asyncio
async def test_switching_on_off_stays_open_and_a_second_one_is_named():
    """The cloud may switch the recorded activity area on and off (plan
    section 1); a second one in force is refused with the first one's name.
    MUTATION: the second-area check removed -> E_INTERNAL from the trigger -> red;
    the check applied to every role -> a forbid zone cannot be switched on -> red."""
    ctx = await _ctx()  # fresh
    try:
        for gid, name in (("f-a", "甲活动区域"), ("f-b", "乙活动区域")):  # two recordings
            await commit_fence(ctx.fence_conn, fence_id=gid, role="allow", points=_RING,
                               now_ms=1, state="draft", name=name)
        on = await _geo(ctx, "set_state", type="fence", geo_id="f-a", base_rev=1,
                        obj={"state": "active"})  # the first
        assert on["result"] == "accepted"  # in force
        second = await _geo(ctx, "set_state", type="fence", geo_id="f-b", base_rev=1,
                            obj={"state": "active"})  # the second
        assert (second["code"], second["detail"]) == (
            E_GEO_INVALID, {"reason": "activity_area_exists", "existing": "甲活动区域"})
        off = await _geo(ctx, "set_state", type="fence", geo_id="f-a", base_rev=2,
                         obj={"state": "disabled"})  # cloud switches it off
        assert off["result"] == "accepted"  # allowed
        assert (await _geo(ctx, "set_state", type="fence", geo_id="f-b", base_rev=1,
                           obj={"state": "active"}))["result"] == "accepted"  # now free
        forbid = await _geo(ctx, "upsert", type="fence", geo_id="f-pit", base_rev=0,
                            obj=_upsert_obj("北门禁止区域", "forbid"))  # other roles unaffected
        assert forbid["result"] == "accepted"  # still writable over cmd/geo
        assert (await _geo(ctx, "set_state", type="fence", geo_id="f-pit", base_rev=1,
                           obj={"state": "active"}))["result"] == "accepted"
    finally:  # always
        await _close(ctx)  # cleanup
