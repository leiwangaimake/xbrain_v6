"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_delete.py
Brief: cmd/geo delete -- tombstone + 15 S7.6 linkage + S7.9.4 refs (batch 3)

Description:
The delete path against real geo.db / fence.db / task.db. Two families of case,
and the second is the unusual one:

  * what delete DOES: tombstone (never a row removal), refs in the ack, and the
    single state change 15 S7.6 calls for -- a suspended task whose route was
    deleted becomes failed with fail_reason=route_deleted.

  * what delete MUST NOT DO. Most of 15 S7.6 is a list of things not to do, and
    each is a real assertion here: a running patrol keeps running when its route
    is deleted (GC-3), a waypoint delete changes no task at all (GC-5), a fence
    delete produces zero task-side action (GC-4 / GX-3). The previous
    implementation failed all three -- it aborted the running task -- and its
    tests passed, because they asserted the classifier's answer rather than the
    contract's.

That is why the linkage cases below drive handle_geo_payload against a real
task row and then assert the TASK STATE, instead of asserting what classify()
returned.
"""
from __future__ import annotations

import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.common.errors import E_GEO_CONFLICT, E_NOT_FOUND
from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload
from xbrain.p3_task.persistence.schema_geo import (
    FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS,
)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.route.geo_linkage import (
    GeoChange, UnknownGeoChange, classify,
)

pytestmark = pytest.mark.no_device

_PATH = [{"lat": 34.6970, "lon": 135.5050}, {"lat": 34.6971, "lon": 135.5051},
         {"lat": 34.6972, "lon": 135.5052}]
_RING = [{"lat": 34.6970, "lon": 135.5050}, {"lat": 34.6975, "lon": 135.5050},
         {"lat": 34.6975, "lon": 135.5055}, {"lat": 34.6970, "lon": 135.5055}]


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


async def _make_task(ctx, task_id, state, task_type="patrol",
                     route_geo_id="r-east", slots=None):
    """One task row referencing a geo object, in the given state."""
    mission = {"source": "voice", "intent": "patrol_route", "id": "B02",
               "slots": slots if slots is not None else {"route": "r-east"}}
    # suspend_kind / suspend_reason are non-null IFF state == 'suspended' (a
    # paired CHECK in the tasks DDL), so a suspended fixture has to carry them
    # -- and the delete path has to clear them when it moves the task to failed.
    kind = "passive" if state == "suspended" else None
    reason = "operator_pause" if state == "suspended" else None
    await ctx.task_conn.execute(
        "INSERT INTO tasks (task_id, task_type, state, priority, submit_seq, "
        " mission_json, total_steps, current_step, step_status_json, source, "
        " resume_policy, route_geo_id, trace_id, created_ms, updated_ms, "
        " suspend_kind, suspend_reason) "
        "VALUES (?, ?, ?, 50, 1, ?, 0, 0, '[]', 'local', 'restart', ?, "
        " 'tr-1', 1, 1, ?, ?)",
        (task_id, task_type, state,
         json.dumps(mission, ensure_ascii=False), route_geo_id, kind, reason))
    await ctx.task_conn.commit()


async def _seed_route(ctx, geo_id="r-east", name="east gate route"):
    ack = await handle_geo_payload(
        {"cmd_id": f"seed-{geo_id}", "action": "upsert", "type": "route",
         "geo_id": geo_id, "origin": "cloud", "base_rev": 0,
         "obj": {"name": name, "geom": {"points": _PATH}}}, ctx, now_ms=1000)
    assert ack["result"] == "accepted", ack
    return ack


def _delete(geo_id, gtype="route", base_rev=1, origin="cloud", cmd_id="d-1"):
    return {"cmd_id": cmd_id, "action": "delete", "type": gtype,
            "geo_id": geo_id, "origin": origin, "base_rev": base_rev}


async def _state_of(ctx, task_id):
    cur = await ctx.task_conn.execute(
        "SELECT state, error_context_json FROM tasks WHERE task_id=?",
        (task_id,))
    return await cur.fetchone()


# -------------------------------------------------------------- tombstone ---

@pytest.mark.asyncio
async def test_delete_tombstones_and_keeps_the_row(ctx):
    """11 S7.11.4: the row survives so the deletion can be pushed to the cloud
    and so an audit can still resolve the id. MUTATION: DELETE FROM the table --
    the object vanishes, the cloud never learns it was deleted, and every
    finished task that referenced it dangles."""
    await _seed_route(ctx)
    ack = await handle_geo_payload(_delete("r-east"), ctx, now_ms=2000)
    assert ack["result"] == "accepted" and ack["detail"]["rev"] == 2
    cur = await ctx.geo_conn.execute(
        "SELECT state, tombstone FROM routes WHERE geo_id='r-east'")
    assert await cur.fetchone() == ("deleted", 1)
    # And it leaves the live broadcast set.
    from xbrain.p3_task.geo.objects import read_geo_objects
    assert (await read_geo_objects(ctx.geo_conn))["routes"] == []


@pytest.mark.asyncio
async def test_delete_is_idempotent_and_rev_checked(ctx):
    await _seed_route(ctx)
    stale = await handle_geo_payload(_delete("r-east", base_rev=99), ctx,
                                     now_ms=2000)
    assert stale["code"] == E_GEO_CONFLICT
    await handle_geo_payload(_delete("r-east"), ctx, now_ms=2000)
    again = await handle_geo_payload(_delete("r-east", base_rev=2,
                                             cmd_id="d-2"), ctx, now_ms=3000)
    assert again["result"] == "accepted"
    assert again["detail"]["already_deleted"] is True


@pytest.mark.asyncio
async def test_delete_missing_object(ctx):
    res = await handle_geo_payload(_delete("r-nope"), ctx, now_ms=1)
    assert res["code"] == E_NOT_FOUND


# -------------------------------------------------- 15 S7.6 linkage rules ---

@pytest.mark.asyncio
async def test_gc3_running_patrol_is_not_stopped(ctx):
    """*** The case the previous implementation got backwards.

    15 S7.6 GC-3: deleting a route does NOT interrupt a running patrol. It
    finishes the current lap with the remaining laps clamped. MUTATION: classify
    a running route delete as 'fail' (which is what the code did before
    2026-08-20) -- this goes red, and on the robot it means the patrol halts in
    the middle of a camp roadway because somebody tidied up the map.
    """
    await _seed_route(ctx)
    await _make_task(ctx, "t-run", "running")
    ack = await handle_geo_payload(_delete("r-east"), ctx, now_ms=2000)
    state, _err = await _state_of(ctx, "t-run")
    assert state == "running", "GC-3: the patrol must keep driving"
    # The operator is still told, at warn level.
    assert any(e for e in ack["detail"].get("linkage", [])
               if e["task_id"] == "t-run")


@pytest.mark.asyncio
async def test_gc3_suspended_task_fails_with_reason(ctx):
    """The one state change delete makes. MUTATION: drop the fail branch and a
    suspended task stays resumable onto a route that no longer exists."""
    await _seed_route(ctx)
    await _make_task(ctx, "t-susp", "suspended")
    ack = await handle_geo_payload(_delete("r-east"), ctx, now_ms=2000)
    state, err = await _state_of(ctx, "t-susp")
    assert state == "failed"
    assert json.loads(err)["fail_reason"] == "route_deleted"
    assert {"task_id": "t-susp", "action": "failed",
            "reason": "route_deleted"} in ack["detail"]["linkage"]


@pytest.mark.asyncio
async def test_gc3_queued_task_is_left_for_the_dequeue_check(ctx):
    """GC-3's queued column defers to the dequeue precondition rather than
    failing now. MUTATION: fail queued tasks here -- a task whose route is
    re-created before it ever runs is killed for nothing."""
    await _seed_route(ctx)
    await _make_task(ctx, "t-q", "ready")
    await handle_geo_payload(_delete("r-east"), ctx, now_ms=2000)
    state, _err = await _state_of(ctx, "t-q")
    assert state == "ready"


@pytest.mark.asyncio
async def test_gc5_waypoint_delete_touches_no_task(ctx):
    """GC-5: patrol geometry is inline on the route (15 S9.3 PLAN A) and never
    reads the waypoints table, so a waypoint delete affects nothing that runs.
    MUTATION: the old 'suspend_system' answer -- a running goto gets suspended
    for a keypoint edit it does not depend on."""
    await handle_geo_payload(
        {"cmd_id": "seed-w", "action": "upsert", "type": "waypoint",
         "geo_id": "w-gate", "origin": "cloud", "base_rev": 0,
         "obj": {"name": "east gate",
                 "geom": {"lat": 34.697, "lon": 135.505}}}, ctx, now_ms=1)
    await _make_task(ctx, "t-goto", "running", task_type="goto",
                     route_geo_id="", slots={"waypoint": "w-gate"})
    ack = await handle_geo_payload(
        _delete("w-gate", gtype="waypoint"), ctx, now_ms=2)
    assert ack["result"] == "accepted"
    state, _err = await _state_of(ctx, "t-goto")
    assert state == "running"
    assert "linkage" not in ack["detail"]


@pytest.mark.asyncio
async def test_gc4_fence_delete_produces_no_task_action(ctx):
    """GC-4 + GX-3: fence enforcement is P1's; P3 changes no task. MUTATION:
    re-evaluate and suspend tasks here and every fence edit ripples into the
    task queue, which 15 S7.6 GX-2 forbids."""
    await handle_geo_payload(
        {"cmd_id": "seed-f", "action": "upsert", "type": "fence",
         "geo_id": "f-north", "origin": "cloud", "base_rev": 0,
         "obj": {"name": "north", "geom": {"role": "forbid",
                                           "outer": _RING}}}, ctx, now_ms=1)
    await _make_task(ctx, "t-run", "running", slots={"fence": "f-north"})
    ack = await handle_geo_payload(
        _delete("f-north", gtype="fence"), ctx, now_ms=2)
    assert ack["result"] == "accepted"
    state, _err = await _state_of(ctx, "t-run")
    assert state == "running" and "linkage" not in ack["detail"]


def test_rename_never_affects_a_reference():
    """GC-2, at the classifier level: this is the value of separating geo_id
    from name (G-5). MUTATION: fold rename into modify and a renamed route
    starts producing route_changed handling for tasks nothing happened to."""
    for state in ("running", "suspended", "ready"):
        out = classify(GeoChange("route", "rename", "r-east"), state, "patrol")
        assert out.action == "none"


def test_unknown_kind_raises():
    """An unclassified edit raises instead of defaulting to 'do nothing' -- a
    silent no-op is how a task keeps driving a route that no longer exists."""
    with pytest.raises(UnknownGeoChange):
        classify(GeoChange("hologram", "delete", "h-1"), "running", "patrol")


# -------------------------------------------------------- S7.9.4 refs ------

@pytest.mark.asyncio
async def test_delete_ack_carries_the_impact_set(ctx):
    """CMD-31: the L2 confirmation text must state the impact, and 11 S7.9.4
    says those numbers come from P3 -- not from the LLM. MUTATION: return an
    empty refs block and the operator is asked to confirm a delete described as
    affecting nothing, while a patrol is running on it."""
    await _seed_route(ctx)
    await _make_task(ctx, "t-run", "running")
    await _make_task(ctx, "t-sch", "scheduled")
    await _make_task(ctx, "t-susp", "suspended")
    ack = await handle_geo_payload(_delete("r-east"), ctx, now_ms=2)
    refs = ack["detail"]["refs"]
    assert refs["running_task"] == ["t-run"]
    assert refs["queued_task"] == ["t-sch"] and refs["schedules"] == 1
    # *** refs is read BEFORE the linkage runs, and this is the case that pins
    # the order: the linkage FAILS t-susp (GC-3), so a refs block computed
    # afterwards would find it in state 'failed' -- terminal, hence excluded --
    # and the operator would be shown an impact set missing the one task the
    # delete actually destroyed. MUTATION: swap the two calls in apply_delete.
    assert refs["suspended_task"] == ["t-susp"]
    assert (await _state_of(ctx, "t-susp"))[0] == "failed"


@pytest.mark.asyncio
async def test_refs_action_answers_before_the_delete(ctx):
    """action=refs is the query the HMI runs to BUILD the dialog, so it must
    work without deleting anything. MUTATION: log refs into geo_cmd_log like a
    write and a polling UI gets 'duplicate' on its second call."""
    await _seed_route(ctx)
    await _make_task(ctx, "t-susp", "suspended")
    first = await handle_geo_payload(
        {"cmd_id": "q-1", "action": "refs", "type": "route",
         "geo_id": "r-east", "origin": "hmi"}, ctx, now_ms=1)
    second = await handle_geo_payload(
        {"cmd_id": "q-1", "action": "refs", "type": "route",
         "geo_id": "r-east", "origin": "hmi"}, ctx, now_ms=2)
    assert first["result"] == "accepted" and second["result"] == "accepted"
    assert first["detail"]["refs"]["suspended_task"] == ["t-susp"]
    # Nothing was deleted by asking.
    cur = await ctx.geo_conn.execute(
        "SELECT state FROM routes WHERE geo_id='r-east'")
    assert (await cur.fetchone())[0] == "active"


@pytest.mark.asyncio
async def test_refs_matches_by_name_as_well_as_id(ctx):
    """Nothing writes tasks.route_geo_id yet and voice tasks keep the spoken
    NAME in their slots, so an id-only match reports 'referenced by nothing' for
    a route three tasks are about to run. MUTATION: drop the name match -- this
    case goes red and every voice-created task becomes invisible to the impact
    set."""
    await _seed_route(ctx)
    await _make_task(ctx, "t-name", "running", route_geo_id="",
                     slots={"route": "east gate route"})
    ack = await handle_geo_payload(
        {"cmd_id": "q-2", "action": "refs", "type": "route",
         "geo_id": "r-east", "origin": "hmi"}, ctx, now_ms=1)
    assert ack["detail"]["refs"]["running_task"] == ["t-name"]


@pytest.mark.asyncio
async def test_docks_on_route_are_listed(ctx):
    """CHG-02: deleting a route changes which docks a patrol can reach, so the
    docks that name it are part of the impact."""
    await _seed_route(ctx)
    await handle_geo_payload(
        {"cmd_id": "seed-d", "action": "upsert", "type": "dock",
         "geo_id": "d-north", "origin": "cloud", "base_rev": 0,
         "obj": {"name": "north dock",
                 "geom": {"lat": 34.697, "lon": 135.505,
                          "dock_heading_rad": 0.0, "on_route": ["r-east"],
                          "handover": {"lat": 34.6971, "lon": 135.5051,
                                       "heading_rad": 0.0}}}},
        ctx, now_ms=1)
    ack = await handle_geo_payload(
        {"cmd_id": "q-3", "action": "refs", "type": "route",
         "geo_id": "r-east", "origin": "hmi"}, ctx, now_ms=2)
    assert ack["detail"]["refs"]["docks_on_route"] == ["d-north"]


@pytest.mark.asyncio
async def test_refs_without_task_db_raises_rather_than_answering_empty(ctx):
    """An empty impact set reads as 'nothing references this'. MUTATION: return
    {} when task_conn is None and a misconfigured wiring silently answers every
    delete confirmation with 'affects nothing'."""
    from xbrain.p3_task.route.geo_refs import compute_refs
    with pytest.raises(ValueError):
        await compute_refs(None, ctx.geo_conn, gtype="route", geo_id="r-east")
