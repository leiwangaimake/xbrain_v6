"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_read.py
Brief: cmd/geo read actions get / list / resync (11 S7.9.1, S7.10, S7.11.2)

Description:
The read half against real databases. What each case is for:

  * get returns the geometry a consumer acts on -- P1 pulls a fence polygon
    this way after active_fence changes -- so the shapes are asserted per type,
    not just "something came back".
  * a get on a TOMBSTONE returns the object rather than E_NOT_FOUND. If it
    answered not-found, the cloud comparing manifests would read the deleted
    object as missing and push its own copy back (S7.11.1).
  * list is summaries only: geometry in a 0.1 Hz manifest is what the size
    budget of S7.10 exists to prevent.
  * resync{prune:true} is REFUSED, and that refusal is the batch's load-bearing
    assertion -- see the test's own docstring.
"""
from __future__ import annotations

import pytest

from xbrain.common.errors import E_NOT_FOUND, E_NOT_IMPLEMENTED, E_SCHEMA
from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload
from xbrain.p3_task.persistence.schema_geo import (
    FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS,
)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS

import aiosqlite
import pytest_asyncio

pytestmark = pytest.mark.no_device

_LAT, _LON = 34.6970, 135.5050
_PATH = [{"lat": _LAT, "lon": _LON}, {"lat": _LAT + 0.0001, "lon": _LON},
         {"lat": _LAT + 0.0002, "lon": _LON}]
_RING = [{"lat": _LAT, "lon": _LON}, {"lat": _LAT + 0.0005, "lon": _LON},
         {"lat": _LAT + 0.0005, "lon": _LON + 0.0005},
         {"lat": _LAT, "lon": _LON + 0.0005}]


async def _open(statements):
    conn = await aiosqlite.connect(":memory:")
    for stmt in statements:
        await conn.execute(stmt)
    await conn.commit()
    return conn


@pytest_asyncio.fixture
async def ctx():
    geo = await _open(GEO_DB_STATEMENTS)
    fence = await _open(FENCE_DB_STATEMENTS)
    task = await _open(ALL_DDL_STATEMENTS)
    yield GeoContext(geo_conn=geo, fence_conn=fence, task_conn=task)
    for c in (geo, fence, task):
        await c.close()


async def _seed(ctx):
    for payload in (
        {"cmd_id": "s-r", "action": "upsert", "type": "route",
         "geo_id": "r-east", "origin": "cloud", "base_rev": 0,
         "obj": {"name": "east gate route", "num": 3, "alias": ["east line"],
                 "geom": {"points": _PATH, "loop_mode": "closed"}}},
        {"cmd_id": "s-w", "action": "upsert", "type": "waypoint",
         "geo_id": "w-gate", "origin": "cloud", "base_rev": 0,
         "obj": {"name": "east gate",
                 "geom": {"lat": _LAT, "lon": _LON, "yaw_deg": 90.0}}},
        {"cmd_id": "s-f", "action": "upsert", "type": "fence",
         "geo_id": "f-north", "origin": "cloud", "base_rev": 0,
         "obj": {"name": "north zone",
                 "geom": {"role": "forbid", "outer": _RING}}},
    ):
        ack = await handle_geo_payload(payload, ctx, now_ms=1000)
        assert ack["result"] == "accepted", ack


def _read(action, **over):
    base = {"cmd_id": f"q-{action}", "action": action, "origin": "hmi"}
    base.update(over)
    return base


# ------------------------------------------------------------------ get ----

@pytest.mark.asyncio
async def test_get_route_returns_geometry(ctx):
    await _seed(ctx)
    ack = await handle_geo_payload(
        _read("get", type="route", geo_id="r-east"), ctx, now_ms=1)
    obj = ack["detail"]["obj"]
    assert obj["geo_id"] == "r-east" and obj["type"] == "route"
    assert obj["name"] == "east gate route" and obj["num"] == 3
    assert obj["alias"] == ["east line"] and obj["state"] == "active"
    assert obj["geom"]["loop_mode"] == "closed"
    assert obj["geom"]["point_count"] == 3
    assert obj["geom"]["points"][0] == {"lat": _LAT, "lon": _LON}
    # updated_ts is SECONDS (S7.8.2) while the column is ms. MUTATION: pass the
    # ms through and every consumer's date reads as the year 56000.
    assert 0.9 < obj["updated_ts"] < 1.1


@pytest.mark.asyncio
async def test_get_fence_returns_the_ring_p1_needs(ctx):
    """This is the P1 path: active_fence changes in the manifest, P1 issues
    get, and clips against what comes back. MUTATION: return geom_json verbatim
    (a {"points": [[lat,lon]]} blob) and P1 reads a dict where it expects a
    vertex list."""
    await _seed(ctx)
    ack = await handle_geo_payload(
        _read("get", type="fence", geo_id="f-north"), ctx, now_ms=1)
    geom = ack["detail"]["obj"]["geom"]
    assert geom["role"] == "forbid" and geom["hard_enforce"] is True
    assert len(geom["outer"]) == 4
    assert geom["outer"][0] == {"lat": _LAT, "lon": _LON}


@pytest.mark.asyncio
async def test_get_waypoint_shape(ctx):
    await _seed(ctx)
    ack = await handle_geo_payload(
        _read("get", type="waypoint", geo_id="w-gate"), ctx, now_ms=1)
    geom = ack["detail"]["obj"]["geom"]
    assert geom["lat"] == _LAT and geom["yaw_deg"] == 90.0


@pytest.mark.asyncio
async def test_get_missing_object(ctx):
    ack = await handle_geo_payload(
        _read("get", type="route", geo_id="r-nope"), ctx, now_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_NOT_FOUND


@pytest.mark.asyncio
async def test_get_returns_a_tombstone_rather_than_not_found(ctx):
    """*** S7.11.1: the cloud pulls objects it is behind on. If a deleted object
    answered E_NOT_FOUND, the cloud would classify it as MISSING and push its
    own copy back -- the delete would undo itself on the next sync.

    MUTATION: filter tombstones out of _read_object and that resurrection loop
    is what you get.
    """
    await _seed(ctx)
    await handle_geo_payload(
        {"cmd_id": "d-1", "action": "delete", "type": "route",
         "geo_id": "r-east", "origin": "cloud", "base_rev": 1}, ctx, now_ms=2)
    ack = await handle_geo_payload(
        _read("get", type="route", geo_id="r-east"), ctx, now_ms=3)
    assert ack["result"] == "accepted"
    assert ack["detail"]["obj"]["state"] == "deleted"


# ----------------------------------------------------------------- list ----

@pytest.mark.asyncio
async def test_list_is_summaries_without_geometry(ctx):
    """S7.10: the manifest goes out at 0.1 Hz, so geometry must not be in it.
    MUTATION: include geom and a few dozen keypoints plus a 2000-point route
    turn a heartbeat into a megabyte."""
    await _seed(ctx)
    ack = await handle_geo_payload(_read("list"), ctx, now_ms=1)
    d = ack["detail"]
    assert d["counts"] == {"route": 1, "waypoint": 1, "fence": 1, "dock": 0}
    assert len(d["items"]) == 3
    for item in d["items"]:
        assert "geom" not in item
        assert {"geo_id", "type", "rev", "hash", "state"} <= set(item)


@pytest.mark.asyncio
async def test_list_filters_by_type(ctx):
    await _seed(ctx)
    ack = await handle_geo_payload(
        _read("list", obj={"types": ["fence"]}), ctx, now_ms=1)
    assert [i["type"] for i in ack["detail"]["items"]] == ["fence"]
    bad = await handle_geo_payload(
        _read("list", obj={"types": ["zone"]}), ctx, now_ms=1)
    assert bad["result"] == "rejected" and bad["code"] == E_SCHEMA


@pytest.mark.asyncio
async def test_active_fence_lists_only_active_ones(ctx):
    """F15's data source, and the two halves of it in one case.

    A newly created fence is a DRAFT (batch 2 / 11 S12A.7 constraint 1), so it
    must NOT be in active_fence -- P1 pulls exactly these ids and clips against
    them, and a fence appearing here is a fence in force. F15 (set_state ->
    active) is what puts it there.

    MUTATION: build active_fence from tombstone=0 instead of state=='active'
    (which is what the code did before batch 2) -- the first assertion goes red
    and every saved fence starts being enforced the moment it is stored.
    """
    await _seed(ctx)
    ack = await handle_geo_payload(_read("list"), ctx, now_ms=1)
    assert ack["detail"]["active_fence"] == [], "a saved fence is not enforced"
    await handle_geo_payload(
        {"cmd_id": "ss-1", "action": "set_state", "type": "fence",
         "geo_id": "f-north", "origin": "cloud", "base_rev": 1,
         "obj": {"state": "active"}}, ctx, now_ms=2)
    after = await handle_geo_payload(_read("list"), ctx, now_ms=3)
    assert after["detail"]["active_fence"] == ["f-north"]


@pytest.mark.asyncio
async def test_catalog_hash_moves_when_a_delete_happens(ctx):
    """S7.10: a tombstone must change the catalog hash, or the cloud comparing
    hashes concludes nothing happened and never mirrors the deletion."""
    await _seed(ctx)
    before = (await handle_geo_payload(_read("list"), ctx,
                                       now_ms=1))["detail"]["catalog_hash"]
    await handle_geo_payload(
        {"cmd_id": "d-2", "action": "delete", "type": "waypoint",
         "geo_id": "w-gate", "origin": "cloud", "base_rev": 1}, ctx, now_ms=2)
    after = (await handle_geo_payload(_read("list"), ctx,
                                      now_ms=3))["detail"]["catalog_hash"]
    assert before != after


def test_catalog_hash_covers_state_independently_of_rev():
    """The contract says the hash is over (geo_id, rev, state), and the case
    above cannot show the state part: every path that changes state also bumps
    rev, so it stays green with state dropped from the hash.

    That makes the case above true but not sufficient -- exactly the shape
    CLAUDE.md 3.2 form 1 warns about. This one holds rev fixed and varies only
    state, which is the only way the third element of the tuple is observable.

    MUTATION: drop state from _catalog_hash -- this goes red, the one above
    does not.
    """
    from xbrain.p3_task.ingest.geo_read import _catalog_hash
    live = [{"geo_id": "r-a", "rev": 4, "state": "active"}]
    draft = [{"geo_id": "r-a", "rev": 4, "state": "draft"}]
    assert _catalog_hash(live) != _catalog_hash(draft)


@pytest.mark.asyncio
async def test_dirty_is_null_not_false(ctx):
    """S7.8.2 dirty means rev > synced_rev, and synced_rev is maintained by
    nothing in this build. Emitting false would be a claim we cannot support --
    and it is the claim that makes prune safe to run.

    MUTATION: emit false and the manifest asserts every object is in sync with
    a cloud that may never have seen it.
    """
    await _seed(ctx)
    ack = await handle_geo_payload(_read("list"), ctx, now_ms=1)
    assert all(i["dirty"] is None for i in ack["detail"]["items"])


# --------------------------------------------------------------- resync ----

@pytest.mark.asyncio
async def test_resync_prune_is_refused_with_its_reason(ctx):
    """*** The load-bearing case of this batch.

    S7.11.2 permits prune only while skipping dirty objects, because those are
    the routes and fences recorded while the link was down -- pruning them
    destroys field work silently. dirty needs synced_rev; nothing maintains it.
    So the guard cannot be evaluated, and the answer is a refusal that says so.

    MUTATION: implement prune with a guard that reads "not dirty" for
    everything (the only thing the current schema can answer) -- it passes by
    construction while deleting the operator's recordings, which is the
    always-green assertion in its most expensive form.
    """
    await _seed(ctx)
    ack = await handle_geo_payload(
        _read("resync", origin="cloud",
              obj={"direction": "push", "prune": True, "objects": []}),
        ctx, now_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_NOT_IMPLEMENTED
    assert ack["detail"]["reason"] == "prune_guard_unavailable"
    # And nothing was tombstoned on the way to refusing.
    cur = await ctx.geo_conn.execute(
        "SELECT COUNT(*) FROM routes WHERE state='deleted'")
    assert (await cur.fetchone())[0] == 0


@pytest.mark.asyncio
async def test_resync_pull_returns_every_object_in_full(ctx):
    await _seed(ctx)
    ack = await handle_geo_payload(
        _read("resync", origin="cloud", obj={"direction": "pull"}), ctx,
        now_ms=1)
    d = ack["detail"]
    assert d["object_count"] == 3
    assert {o["geo_id"] for o in d["objects"]} == {"r-east", "w-gate",
                                                   "f-north"}
    assert all("geom" in o for o in d["objects"])


@pytest.mark.asyncio
async def test_resync_push_applies_through_the_normal_upsert(ctx):
    """Reusing apply_upsert is what keeps rev arbitration, name conflicts and
    geometry validation identical in a resync. MUTATION: write a bulk INSERT
    path and a pushed object skips every one of those rules."""
    await _seed(ctx)
    ack = await handle_geo_payload(
        _read("resync", origin="cloud", obj={
            "direction": "push",
            "objects": [
                {"type": "route", "geo_id": "r-east",
                 "name": "east gate route v2",
                 "geom": {"points": _PATH, "loop_mode": "oneway"}},
                {"type": "waypoint", "geo_id": "w-new", "name": "north gate",
                 "geom": {"lat": _LAT, "lon": _LON}},
                {"type": "route", "geo_id": "bad-id", "name": "x",
                 "geom": {"points": _PATH}},
            ]}), ctx, now_ms=2)
    d = ack["detail"]
    assert d["applied"] == 2
    # The malformed one is reported per object rather than aborting the batch.
    assert len(d["failed"]) == 1 and d["failed"][0]["geo_id"] == "bad-id"
    cur = await ctx.geo_conn.execute(
        "SELECT name, rev, loop_mode FROM routes WHERE geo_id='r-east'")
    assert await cur.fetchone() == ("east gate route v2", 2, "oneway")


@pytest.mark.asyncio
async def test_resync_direction_is_closed(ctx):
    ack = await handle_geo_payload(
        _read("resync", origin="cloud", obj={"direction": "sideways"}), ctx,
        now_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_SCHEMA


@pytest.mark.asyncio
async def test_resync_stays_cloud_only(ctx):
    """The S7.9.5 matrix, reached through the read path this time: an HMI
    resync is denied before any of the above runs."""
    ack = await handle_geo_payload(
        _read("resync", obj={"direction": "pull"}), ctx, now_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == "E_CHANNEL_DENIED"


@pytest.mark.asyncio
async def test_every_contract_action_now_has_an_applier():
    """Guards the whole cmd/geo surface: S7.9.1 lists eight actions, and an
    unimplemented one answers E_NOT_IMPLEMENTED at runtime -- which reads as
    "not built" rather than as a registration that was forgotten."""
    from xbrain.common.enums import GEO_ACTION
    from xbrain.p3_task.ingest.geo_apply import APPLIERS
    assert set(APPLIERS) == set(GEO_ACTION)
