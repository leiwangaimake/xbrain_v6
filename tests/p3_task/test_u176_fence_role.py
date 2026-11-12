"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u176_fence_role.py
Brief: 99 U176 -- a recorded fence's type from its name, every other fence inside the activity area

Description:
The user's ruling of 2026-11-12 (99 U176), each rule pinned by a case that
names the mutant it kills:
  * the type comes from a keyword in the name: 活动区域 allow, 报警区域
    warning, 限速区域 speed_limit, 禁止区域 forbid; no keyword, or two, is
    no type (until then every recorded fence was a forbid zone, and nothing
    could create the activity area FS-5A needs, so none ever took effect);
  * a recording start is refused before anything is recorded: no type
    (fence_type_missing), a second activity area (activity_area_exists), any
    other fence with no activity area (activity_area_missing), a 限速区域
    while teach.fence_speed_limit_mps is unset (by key name);
  * at save, a fence other than the activity area must lie inside it
    (outside_activity_area), and an activity area must enclose the fences
    already there (activity_area_excludes) -- both close the session as
    invalid_geometry;
  * enabling (cmd/geo set_state active, F15 and the cloud) a fence other than
    the activity area needs the activity area enabled and the fence inside it.
"""
from __future__ import annotations  # postponed annotations

import math  # metres -> degrees of longitude

import aiosqlite  # in-memory dbs
import pytest  # marker, parametrize
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_GEO_INVALID, E_TEACH_GEOMETRY, E_TEACH_STATE  # expected codes
from xbrain.common.geo.roles import fence_role_for_name  # under test
from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload  # the set_state path
from xbrain.p3_task.ingest.geo_commit import commit_fence  # seed rows
from xbrain.p3_task.persistence.schema_geo import FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS  # geo DDL
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task DDL
from xbrain.p3_task.teach.runtime import TeachRuntime  # under test
from xbrain.p3_task.teach.validate import BLOCKING_ISSUES, ring_within  # under test

pytestmark = pytest.mark.no_device  # pure logic + in-memory sqlite

_LAT, _LON = 34.6970, 135.5050  # a site
_DLAT = 1.0 / 111320.0  # one metre of latitude
_DLON = _DLAT / math.cos(math.radians(_LAT))  # one metre of longitude


def _square(x0, y0, side):  # [(lat, lon)] corners, metres east / north of the site
    return [(_LAT + y * _DLAT, _LON + x * _DLON)
            for x, y in ((x0, y0), (x0, y0 + side), (x0 + side, y0 + side), (x0 + side, y0))]


_AREA = _square(0, 0, 100)  # the activity area, 100 m square
_INSIDE = _square(20, 20, 20)  # well inside it
_ACROSS = _square(90, 20, 20)  # half outside


async def _open(statements):  # in-memory db with a schema
    conn = await aiosqlite.connect(":memory:")  # fresh
    for stmt in statements:  # schema
        await conn.execute(stmt)  # one DDL
    await conn.commit()  # ready
    return conn  # caller closes


def _pose(rt, lat, lon, t):  # one rtk_fixed state/pose sample at mono t
    rt.update_pose({"lat": lat, "lon": lon, "fix_type": "rtk_fixed", "heading_rad": 0.0,
                    "heading_valid": True, "heading_level": 1, "speed_mps": 1.0}, t)


async def _make(speed=None):  # a runtime with healthy caches
    dbs = (await _open(ALL_DDL_STATEMENTS), await _open(GEO_DB_STATEMENTS),
           await _open(FENCE_DB_STATEMENTS))  # task, geo, fence
    rt = TeachRuntime(*dbs, boot_id="b0", sample_hz=1.0, min_dist_m=0.5,
                      snap_max_age_ms=8000, record_duration_s=10800,
                      fence_speed_limit_mps=speed)  # U176 key under test
    _pose(rt, _LAT, _LON, 100.0)  # a fresh fix
    rt.update_health({"allow_motion": True})  # arming check 4
    rt.update_robot({"hes": False, "estop_path": "up"})  # check 4 / 7
    rt.update_power({"soc_pct": 80.0})  # check 5
    rt.update_teleop({"sources": [{"device": "gamepad", "stale": False, "alive": True}]})  # check 7
    return rt, dbs  # caller closes dbs


@pytest_asyncio.fixture  # speed limit unset, as shipped
async def rt():
    runtime, dbs = await _make()  # fresh
    yield runtime  # to the test
    for c in dbs:  # cleanup
        await c.close()  # close


async def _seed(conn, fid, name, role, ring, state="draft", speed=None):  # a fence row
    await commit_fence(conn, fence_id=fid, role=role, points=ring, now_ms=1, name=name,
                       state=state, created_by="teach", speed_limit_mps=speed)  # as teach writes it


def _cmd(cmd_id, action, **over):  # a voice-issued cmd/teach
    base = {"cmd_id": cmd_id, "action": action,
            "issuer": {"src": "p4_agent", "channel": "local_voice"}}  # S12A.4 issuer
    base.update(over)  # the action block
    return base  # payload


async def _start(rt, name, t=100.0):  # a named fence start
    _pose(rt, _LAT, _LON, t)  # fresh at t
    return await rt.handle(_cmd("c-start-" + name, "start", start={"kind": "fence", "name": name}),
                           now_mono_s=t, now_ms=1000)  # the ack


async def _record(rt, name, ring):  # start, drive the corners, exit (saves under the name)
    ack = await _start(rt, name)  # named start
    assert ack["result"] == "accepted", ack  # armed
    for i, (la, lo) in enumerate(ring):  # each corner
        _pose(rt, la, lo, 101.0 + i)  # there
        await rt.offer_pose(101.0 + i, 1000 + i)  # sampled
    return await rt.handle(_cmd("c-fin-" + name, "finish", session_id=ack["detail"]["session_id"]),
                           now_mono_s=110.0, now_ms=2000)  # exit = save


def _blocking(fin):  # the blocking issues of a refused finish (warnings ride along)
    return [i for i in fin["detail"]["issues"] if i["code"] in BLOCKING_ISSUES]


async def _row(rt, name):  # (role, state, speed_limit_mps) of a saved fence
    cur = await rt._fence_conn.execute(
        "SELECT role, state, speed_limit_mps FROM fences WHERE name=?", (name,))
    return await cur.fetchone()  # None when not saved


# -- the name rule ------------------------------------------------------------
@pytest.mark.parametrize("name,role", [
    ("营区活动区域围栏", "allow"), ("东门报警区域围栏", "warning"),  # the user's words
    ("路口限速区域围栏", "speed_limit"), ("1号禁止区域围栏", "forbid"),
    ("东门围栏", None), ("活动区域禁止区域围栏", None),  # none / two: no type
    ("营区活动围栏", None),  # 活动 alone is not the keyword
])
def test_the_type_is_the_keyword_in_the_name(name, role):  # roles.py
    """MUTATION: first keyword wins -> two types read as one -> red; 活动 alone
    accepted -> red."""
    assert fence_role_for_name(name) == role  # exactly one keyword or None


def test_ring_within():  # containment
    """MUTATION: vertices only (no edge crossing test) -> a ring whose corners
    are inside but whose edge cuts a notch of the outer ring passes -> red."""
    assert ring_within(_INSIDE, _AREA) and not ring_within(_ACROSS, _AREA)  # in / across
    notch = [(_LAT, _LON), (_LAT + 100 * _DLAT, _LON), (_LAT + 100 * _DLAT, _LON + 100 * _DLON),
             (_LAT + 60 * _DLAT, _LON + 100 * _DLON), (_LAT + 60 * _DLAT, _LON + 40 * _DLON),
             (_LAT + 40 * _DLAT, _LON + 40 * _DLON), (_LAT + 40 * _DLAT, _LON + 100 * _DLON),
             (_LAT, _LON + 100 * _DLON)]  # a C shape opening east, notch 40..60 m north
    bridge = [(_LAT + 30 * _DLAT, _LON + 50 * _DLON), (_LAT + 70 * _DLAT, _LON + 50 * _DLON),
              (_LAT + 70 * _DLAT, _LON + 60 * _DLON), (_LAT + 30 * _DLAT, _LON + 60 * _DLON)]
    assert not ring_within(bridge, notch)  # corners inside the C, edges cross the notch


# -- start refusals -----------------------------------------------------------
@pytest.mark.asyncio  # async case
async def test_an_untyped_name_is_refused_at_start(rt):  # fence_type_missing
    """MUTATION: drop the start check -> the session arms -> red."""
    ack = await _start(rt, "东门围栏")  # no keyword
    assert (ack["result"], ack["code"], ack["detail"]["reason"]) == (
        "rejected", E_GEO_INVALID, "fence_type_missing")  # refused
    assert not rt.active  # never armed


@pytest.mark.asyncio  # async case
async def test_another_fence_needs_an_activity_area_first(rt):  # activity_area_missing
    """MUTATION: let a forbid start with no allow -> armed -> red."""
    for name in ("北门禁止区域围栏", "东门报警区域围栏"):  # two non-allow kinds
        ack = await _start(rt, name)  # nothing to sit in
        assert ack["result"] == "rejected" and ack["detail"]["reason"] == "activity_area_missing"
    assert not rt.active  # never armed


@pytest.mark.asyncio  # async case
async def test_only_one_activity_area(rt):  # activity_area_exists
    """FS-5A. MUTATION: allow a second allow recording -> armed -> red."""
    await _seed(rt._fence_conn, "f-a", "营区活动区域围栏", "allow", _AREA)  # a draft one
    ack = await _start(rt, "二号活动区域围栏")  # a second
    assert ack["result"] == "rejected" and ack["detail"] == {
        "reason": "activity_area_exists", "name": "二号活动区域围栏", "existing": "营区活动区域围栏"}


@pytest.mark.asyncio  # async case
async def test_a_speed_limit_fence_needs_the_configured_limit(rt):  # CLAUDE.md 3.1
    """MUTATION: save with a made-up limit -> armed -> red."""
    await _seed(rt._fence_conn, "f-a", "营区活动区域围栏", "allow", _AREA)  # area there
    ack = await _start(rt, "路口限速区域围栏")  # key unset
    assert (ack["result"], ack["code"], ack["detail"]) == ("rejected", E_TEACH_STATE, {
        "reason": "recording_config_missing", "missing": ["teach.fence_speed_limit_mps"]})


# -- save ---------------------------------------------------------------------
@pytest.mark.asyncio  # async case
async def test_the_activity_area_then_a_fence_inside_it(rt):  # the user's order
    """MUTATION: role hard-coded forbid again -> the area saves as forbid -> red."""
    fin = await _record(rt, "营区活动区域围栏", _AREA)  # first the area
    assert fin["detail"]["saved"] is True and await _row(rt, "营区活动区域围栏") == ("allow", "draft", None)
    fin = await _record(rt, "东门报警区域围栏", _INSIDE)  # then one inside (a draft area counts)
    assert fin["detail"]["saved"] is True and await _row(rt, "东门报警区域围栏") == ("warning", "draft", None)


@pytest.mark.asyncio  # async case
async def test_a_fence_across_the_activity_area_is_not_saved(rt):  # outside_activity_area
    """MUTATION: drop check_activity_area at save -> saved -> red."""
    await _seed(rt._fence_conn, "f-a", "营区活动区域围栏", "allow", _AREA)  # the area
    fin = await _record(rt, "北门禁止区域围栏", _ACROSS)  # half outside
    assert fin["result"] == "rejected" and fin["code"] == E_TEACH_GEOMETRY  # closed
    assert fin["detail"]["close_reason"] == "invalid_geometry"  # no finalizing left
    assert _blocking(fin) == [{"code": "outside_activity_area",
                               "activity_area": "营区活动区域围栏"}]  # P4 says it
    assert await _row(rt, "北门禁止区域围栏") is None  # nothing written


@pytest.mark.asyncio  # async case
async def test_an_activity_area_must_enclose_the_fences_already_there(rt):  # excludes
    """The area was deleted and is re-recorded smaller. MUTATION: skip the
    enclosed check for allow -> saved, leaving the warning fence outside -> red."""
    await _seed(rt._fence_conn, "f-w", "东门报警区域围栏", "warning", _ACROSS)  # left behind
    fin = await _record(rt, "营区活动区域围栏", _AREA)  # does not reach it
    assert fin["result"] == "rejected"  # closed
    assert _blocking(fin) == [{"code": "activity_area_excludes",
                               "fences": ["东门报警区域围栏"]}]  # names it


@pytest.mark.asyncio  # async case
async def test_a_speed_limit_fence_is_saved_with_the_configured_limit():  # FV-13
    """MUTATION: pass no speed_limit_mps -> commit_fence refuses (FV-13) -> red."""
    rt, dbs = await _make(speed=0.8)  # key set
    try:
        await _seed(rt._fence_conn, "f-a", "营区活动区域围栏", "allow", _AREA)  # the area
        fin = await _record(rt, "路口限速区域围栏", _INSIDE)  # inside
        assert fin["detail"]["saved"] is True  # saved
        assert await _row(rt, "路口限速区域围栏") == ("speed_limit", "draft", 0.8)  # the limit
    finally:
        for c in dbs:  # cleanup
            await c.close()  # close


# -- enabling -----------------------------------------------------------------
async def _enable(ctx, fid, n):  # cmd/geo set_state active, as F15 / the cloud send it
    return await handle_geo_payload({"cmd_id": "c-on-%d" % n, "action": "set_state",
                                     "type": "fence", "geo_id": fid, "origin": "cloud",
                                     "base_rev": 1, "obj": {"state": "active"}}, ctx, now_ms=10 + n)


@pytest.mark.asyncio  # async case
async def test_enabling_needs_the_activity_area_enabled_and_the_fence_inside_it():
    """MUTATION: drop fence_activation_refusal -> the warning fence is enabled
    under a draft area (and the FenceSet is never broadcast) -> red."""
    geo, fence = await _open(GEO_DB_STATEMENTS), await _open(FENCE_DB_STATEMENTS)  # dbs
    ctx = GeoContext(geo_conn=geo, fence_conn=fence, task_conn=None)  # as p3 builds it
    try:
        await _seed(fence, "f-a", "营区活动区域围栏", "allow", _AREA)  # draft area
        await _seed(fence, "f-w", "东门报警区域围栏", "warning", _INSIDE)  # inside
        await _seed(fence, "f-x", "北门禁止区域围栏", "forbid", _ACROSS)  # across (cloud-made)
        early = await _enable(ctx, "f-w", 1)  # area not enabled yet
        assert (early["result"], early["code"], early["detail"]["reason"]) == (
            "rejected", E_GEO_INVALID, "activity_area_inactive")
        assert (await _enable(ctx, "f-a", 2))["result"] == "accepted"  # the area itself
        assert (await _enable(ctx, "f-w", 3))["result"] == "accepted"  # now it takes effect
        across = await _enable(ctx, "f-x", 4)  # half outside
        assert across["result"] == "rejected" and across["detail"] == {
            "reason": "outside_activity_area", "name": "北门禁止区域围栏",
            "activity_area": "营区活动区域围栏"}
        cur = await fence.execute("SELECT fence_id FROM fences WHERE state='active' ORDER BY fence_id")
        assert [r[0] for r in await cur.fetchall()] == ["f-a", "f-w"]  # exactly those
    finally:
        await geo.close()  # cleanup
        await fence.close()  # cleanup
