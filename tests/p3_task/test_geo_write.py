"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_write.py
Brief: cmd/geo write appliers against real geo.db / fence.db (11 S7.9 batch 2)

Description:
upsert / rename / set_state driven end to end through handle_geo_payload against
in-memory geo.db and fence.db, because the properties worth pinning are storage
properties: what rev became, what the row holds afterwards, and what did NOT
happen on a refusal.

The cases that carry the batch:

  * a rejected command leaves NO row (the fail-silent shape is a refusal that
    half-wrote);
  * a redelivered cmd_id replays and does not bump rev a second time (S7.9.2
    step 5) -- Zenoh redelivers, and a rev that moved twice makes every later
    sync see a phantom conflict;
  * a NEW FENCE is draft, not active. This one is a safety property: activating
    on save would put a keep-out zone into force that nobody confirmed, and F15
    plus its L2 cell would be decoration.
  * the S9A.1A quota triggers fire on ACTIVATION, not only on insert.

Each assertion names the mutation that reddens it.
"""
from __future__ import annotations

import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.common.errors import (
    E_GEO_CONFLICT, E_GEO_INVALID, E_NAME_CONFLICT, E_NOT_FOUND, E_SCHEMA,
)
from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload
from xbrain.p3_task.ingest.geo_write import (
    GEO_EVENT_INFO, GEO_EVENT_WARN,
)
from xbrain.p3_task.persistence.schema_geo import (
    FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS,
)

pytestmark = pytest.mark.no_device

# A square ring near the reseeded site origin (34.697, 135.505), in WGS84.
_RING = [{"lat": 34.6970, "lon": 135.5050}, {"lat": 34.6975, "lon": 135.5050},
         {"lat": 34.6975, "lon": 135.5055}, {"lat": 34.6970, "lon": 135.5055}]
_PATH = [{"lat": 34.6970, "lon": 135.5050}, {"lat": 34.6971, "lon": 135.5051},
         {"lat": 34.6972, "lon": 135.5052}]


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
    yield GeoContext(geo_conn=geo, fence_conn=fence, task_conn=None)
    await geo.close()
    await fence.close()


def _upsert(cmd_id="c-1", gtype="waypoint", geo_id="w-gate", origin="voice",
            base_rev=0, **obj_over):
    obj = {"name": "east gate", "geom": {"lat": 34.6970, "lon": 135.5050}}
    obj.update(obj_over)
    return {"cmd_id": cmd_id, "action": "upsert", "type": gtype,
            "geo_id": geo_id, "origin": origin, "base_rev": base_rev,
            "obj": obj}


async def _row(conn, table, pk_col, geo_id, cols):
    cur = await conn.execute(
        f"SELECT {cols} FROM {table} WHERE {pk_col}=?", (geo_id,))
    return await cur.fetchone()


# ----------------------------------------------------------------- upsert ---

@pytest.mark.asyncio
async def test_upsert_creates_waypoint_with_rev_1_and_provenance(ctx):
    """The F06 path: a spoken "record this as the east gate" becomes one row.
    MUTATION: take rev from the sender's obj -- S7.9.2 step 6 says it is
    ignored, and honouring it lets two writers race the version counter."""
    ack = await handle_geo_payload(
        _upsert(**{"created_by": "teach"}), ctx, now_ms=1000)
    assert ack["result"] == "accepted", ack
    assert ack["detail"]["rev"] == 1
    row = await _row(ctx.geo_conn, "waypoints", "geo_id", "w-gate",
                     "name, rtk_lat, rev, state, created_by, updated_by")
    assert row[0] == "east gate" and abs(row[1] - 34.6970) < 1e-9
    assert row[2] == 1 and row[3] == "active"
    # origin=voice but created_by=teach: provenance travels in the body because
    # teach is not a channel. MUTATION: derive created_by from origin only and
    # every teach recording is attributed to the microphone instead.
    assert row[4] == "teach" and row[5] == "teach"


@pytest.mark.asyncio
async def test_upsert_second_edit_bumps_rev_and_needs_matching_base_rev(ctx):
    await handle_geo_payload(_upsert(), ctx, now_ms=1000)
    ok = await handle_geo_payload(
        _upsert(cmd_id="c-2", base_rev=1, name="east gate 2"), ctx,
        now_ms=2000)
    assert ok["detail"]["rev"] == 2
    # Stale base_rev -> E_GEO_CONFLICT carrying the LOCAL rev to retry against.
    # MUTATION: drop the base_rev comparison -- a stale HMI tab then silently
    # overwrites an edit made from the cloud a minute earlier.
    stale = await handle_geo_payload(
        _upsert(cmd_id="c-3", base_rev=1, name="east gate 3"), ctx,
        now_ms=3000)
    assert stale["result"] == "rejected" and stale["code"] == E_GEO_CONFLICT
    assert stale["detail"]["rev"] == 2 and stale["detail"]["updated_by"]
    # And the refused write did not land.
    row = await _row(ctx.geo_conn, "waypoints", "geo_id", "w-gate",
                     "name, rev")
    assert row == ("east gate 2", 2)


@pytest.mark.asyncio
async def test_identical_content_does_not_bump_rev(ctx):
    """S7.9.2: same content_hash means same rev. MUTATION: bump unconditionally
    and every re-save marks every other holder stale for no change."""
    await handle_geo_payload(_upsert(), ctx, now_ms=1000)
    again = await handle_geo_payload(_upsert(cmd_id="c-2", base_rev=1), ctx,
                                     now_ms=2000)
    assert again["result"] == "accepted"
    assert again["detail"]["rev"] == 1 and again["detail"]["unchanged"] is True


@pytest.mark.asyncio
async def test_redelivered_cmd_id_replays_and_does_not_reapply(ctx):
    """*** S7.9.2 step 5. Zenoh does not promise exactly-once (C-5).
    MUTATION: skip the geo_cmd_log lookup -- the second delivery bumps rev to 2
    and the sender, holding rev 1, then conflicts against a version it created.
    """
    first = await handle_geo_payload(_upsert(), ctx, now_ms=1000)
    dup = await handle_geo_payload(_upsert(), ctx, now_ms=2000)
    assert first["result"] == "accepted"
    assert dup["result"] == "duplicate"
    assert dup["detail"]["rev"] == 1 and dup["detail"]["replayed"] is True
    row = await _row(ctx.geo_conn, "waypoints", "geo_id", "w-gate",
                     "rev, updated_ms")
    assert row == (1, 1000)          # untouched by the redelivery


@pytest.mark.asyncio
async def test_rejected_upsert_writes_nothing(ctx):
    """A refusal must leave no partial row. MUTATION: move the INSERT before the
    validation, or drop the rollback -- a half-written route is worse than none,
    because the map then shows a route the operator never finished."""
    bad = await handle_geo_payload(
        _upsert(geom={"lat": 12.5, "lon": 40.2, "note": "ENU metres"},
                geo_id="w-bad"), ctx, now_ms=1000)
    assert bad["result"] == "accepted"      # 12.5/40.2 IS a valid WGS84 pair
    # The real out-of-range case: ENU northing/easting in the hundreds.
    worse = await handle_geo_payload(
        _upsert(cmd_id="c-9", geo_id="w-enu",
                geom={"lat": 412.5, "lon": 118.0}), ctx, now_ms=1000)
    assert worse["result"] == "rejected" and worse["code"] == E_GEO_INVALID
    cur = await ctx.geo_conn.execute(
        "SELECT COUNT(*) FROM waypoints WHERE geo_id='w-enu'")
    assert (await cur.fetchone())[0] == 0


@pytest.mark.asyncio
async def test_upsert_route_computes_length_and_point_count(ctx):
    """total_len_m is computed by P3, never taken from the sender: S7.12 remaps
    a resumed patrol against it. MUTATION: store a sender-supplied length and a
    resumed patrol restarts at the wrong fraction of the route."""
    ack = await handle_geo_payload(
        _upsert(cmd_id="c-r", gtype="route", geo_id="r-east",
                name="east gate route",
                geom={"loop_mode": "oneway", "points": _PATH,
                      "total_len_m": 99999.0}), ctx, now_ms=1000)
    assert ack["result"] == "accepted", ack
    assert ack["detail"]["point_count"] == 3
    assert 0.0 < ack["detail"]["total_len_m"] < 100.0
    row = await _row(ctx.geo_conn, "routes", "geo_id", "r-east",
                     "path_points, total_len_m")
    assert len(json.loads(row[0])) == 3 and row[1] < 100.0


@pytest.mark.asyncio
async def test_upsert_rejects_wrong_id_prefix_and_short_route(ctx):
    wrong = await handle_geo_payload(
        _upsert(cmd_id="c-p", gtype="route", geo_id="w-oops", name="r",
                geom={"points": _PATH}), ctx, now_ms=1)
    assert wrong["code"] == E_GEO_INVALID and "r-" in wrong["detail"]["reason"]
    short = await handle_geo_payload(
        _upsert(cmd_id="c-s", gtype="route", geo_id="r-short", name="r2",
                geom={"points": _PATH[:1]}), ctx, now_ms=1)
    assert short["code"] == E_GEO_INVALID


@pytest.mark.asyncio
async def test_duplicate_name_is_a_named_conflict(ctx):
    """MUTATION: let the IntegrityError through as E_INTERNAL -- the operator
    hears "internal error" when the real answer is "that name is taken"."""
    await handle_geo_payload(_upsert(), ctx, now_ms=1)
    clash = await handle_geo_payload(
        _upsert(cmd_id="c-2", geo_id="w-other"), ctx, now_ms=2)
    assert clash["result"] == "rejected" and clash["code"] == E_NAME_CONFLICT


# ------------------------------------------------------------ fence state ---

def _fence_cmd(cmd_id, geo_id, role="forbid", base_rev=0, origin="cloud",
               name=None):
    return {"cmd_id": cmd_id, "action": "upsert", "type": "fence",
            "geo_id": geo_id, "origin": origin, "base_rev": base_rev,
            "obj": {"name": name or geo_id,
                    "geom": {"role": role, "outer": _RING}}}


@pytest.mark.asyncio
async def test_new_fence_is_draft_not_active(ctx):
    """*** Safety property. A saved fence is NOT yet in force: F15 activates it
    and 11 S7.9.5 gives that activation its own L2 cell.

    MUTATION: create fences as 'active' -- both cases below flip, and the robot
    starts enforcing a zone at the moment somebody said "save fence", before
    anyone confirmed it.
    """
    ack = await handle_geo_payload(_fence_cmd("c-f", "f-north"), ctx, now_ms=1)
    assert ack["detail"]["state"] == "draft"
    row = await _row(ctx.fence_conn, "fences", "fence_id", "f-north",
                     "state, hard_enforce, role")
    assert row[0] == "draft"
    # And it is NOT in the broadcast set while it is a draft.
    from xbrain.p3_task.dao.simple_daos import FencesDAO
    assert await FencesDAO(ctx.fence_conn).list_active() == []


@pytest.mark.asyncio
async def test_set_state_activates_and_quota_holds_on_activation(ctx):
    """S9A.1A: at most one ACTIVE allow fence. The insert trigger cannot see
    this -- every fence is inserted as a draft -- so the UPDATE trigger is the
    one that matters. MUTATION: drop trg_fence_single_allow_upd and a second
    keep-in fence activates, widening the permitted area to the union of two
    polygons without a word anywhere."""
    await handle_geo_payload(_fence_cmd("c-1", "f-a", role="allow"), ctx,
                             now_ms=1)
    await handle_geo_payload(_fence_cmd("c-2", "f-b", role="allow"), ctx,
                             now_ms=2)
    on = await handle_geo_payload(
        {"cmd_id": "c-3", "action": "set_state", "type": "fence",
         "geo_id": "f-a", "origin": "cloud", "base_rev": 1,
         "obj": {"state": "active"}}, ctx, now_ms=3)
    assert on["result"] == "accepted" and on["detail"]["state"] == "active"
    second = await handle_geo_payload(
        {"cmd_id": "c-4", "action": "set_state", "type": "fence",
         "geo_id": "f-b", "origin": "cloud", "base_rev": 1,
         "obj": {"state": "active"}}, ctx, now_ms=4)
    assert second["result"] == "rejected"
    row = await _row(ctx.fence_conn, "fences", "fence_id", "f-b", "state")
    assert row[0] == "draft"


@pytest.mark.asyncio
async def test_set_state_refuses_deleted_and_unknown(ctx):
    """set_state is not a second delete path (no reference sweep, no cloud-only
    cell) and not a resurrection. MUTATION: accept 'deleted' here and a fence
    can be removed through a cell whose permission row says 'set_state'."""
    await handle_geo_payload(_fence_cmd("c-1", "f-a"), ctx, now_ms=1)
    for target, code in (("deleted", E_SCHEMA), ("bogus", E_SCHEMA)):
        res = await handle_geo_payload(
            {"cmd_id": f"c-{target}", "action": "set_state", "type": "fence",
             "geo_id": "f-a", "origin": "cloud", "base_rev": 1,
             "obj": {"state": target}}, ctx, now_ms=2)
        assert res["result"] == "rejected" and res["code"] == code


@pytest.mark.asyncio
async def test_set_state_on_missing_object(ctx):
    res = await handle_geo_payload(
        {"cmd_id": "c-x", "action": "set_state", "type": "route",
         "geo_id": "r-nope", "origin": "cloud", "base_rev": 0,
         "obj": {"state": "active"}}, ctx, now_ms=1)
    assert res["code"] == E_NOT_FOUND


# ----------------------------------------------------------------- rename ---

@pytest.mark.asyncio
async def test_rename_changes_name_only_and_rehashes(ctx):
    """F14. MUTATION A: let rename write geometry columns -- a rename would then
    be able to move a route a task is driving. MUTATION B: keep content_hash
    unchanged -- a cloud sync comparing hashes decides the renamed object is
    identical and never pulls the new name."""
    await handle_geo_payload(
        _upsert(cmd_id="c-r", gtype="route", geo_id="r-east",
                name="east gate route",
                geom={"points": _PATH}), ctx, now_ms=1)
    before = await _row(ctx.geo_conn, "routes", "geo_id", "r-east",
                        "path_points, content_hash")
    ack = await handle_geo_payload(
        {"cmd_id": "c-n", "action": "rename", "type": "route",
         "geo_id": "r-east", "origin": "hmi", "base_rev": 1,
         "obj": {"name": "east perimeter", "num": 3,
                 "alias": ["east line"]}}, ctx, now_ms=2)
    assert ack["result"] == "accepted" and ack["detail"]["rev"] == 2
    after = await _row(ctx.geo_conn, "routes", "geo_id", "r-east",
                       "name, num, alias_json, path_points, content_hash")
    assert after[0] == "east perimeter" and after[1] == 3
    assert json.loads(after[2]) == ["east line"]
    assert after[3] == before[0]              # geometry untouched
    assert after[4] != before[1]              # hash moved


@pytest.mark.asyncio
async def test_rename_needs_something_to_rename(ctx):
    await handle_geo_payload(_upsert(), ctx, now_ms=1)
    res = await handle_geo_payload(
        {"cmd_id": "c-n", "action": "rename", "type": "waypoint",
         "geo_id": "w-gate", "origin": "hmi", "base_rev": 1, "obj": {}},
        ctx, now_ms=2)
    assert res["result"] == "rejected" and res["code"] == E_SCHEMA


@pytest.mark.asyncio
async def test_rename_bad_num_is_refused(ctx):
    """num is what voice "3 hao lu jing" resolves on, so 0 and 100 are refused
    rather than stored and never matched."""
    await handle_geo_payload(_upsert(), ctx, now_ms=1)
    res = await handle_geo_payload(
        {"cmd_id": "c-n", "action": "rename", "type": "waypoint",
         "geo_id": "w-gate", "origin": "hmi", "base_rev": 1,
         "obj": {"num": 0}}, ctx, now_ms=2)
    assert res["code"] == E_GEO_INVALID


# ----------------------------------------------------------------- events ---

@pytest.mark.asyncio
async def test_events_are_emitted_after_the_write(ctx):
    """The audit trail (11 S6.2 event/{sev}/geo). MUTATION: publish from inside
    the transaction -- a rolled-back write then leaves an announced change that
    S7.10 gives listeners no way to retract."""
    seen = []
    await handle_geo_payload(_upsert(), ctx, now_ms=1,
                             on_event=lambda s, t, d: seen.append((s, t)))
    assert seen == [("info", "geo.created")]
    seen.clear()
    # A refusal emits nothing at all.
    await handle_geo_payload(_upsert(cmd_id="c-2", base_rev=99,
                                     name="other"), ctx, now_ms=2,
                             on_event=lambda s, t, d: seen.append((s, t)))
    assert seen == []


@pytest.mark.asyncio
async def test_force_overwrite_leaves_a_warn_trace(ctx):
    """S7.9.2 step 4: forcing past a real conflict discards somebody's edit, so
    it MUST be visible. MUTATION: emit only geo.updated on a force and the
    discarded edit leaves no trace anywhere."""
    await handle_geo_payload(_upsert(), ctx, now_ms=1)
    seen = []
    ack = await handle_geo_payload(
        {**_upsert(cmd_id="c-f", base_rev=99, origin="cloud", name="forced"),
         "force": True}, ctx, now_ms=2,
        on_event=lambda s, t, d: seen.append((s, t)))
    assert ack["result"] == "accepted"
    assert ("warn", "geo.force_overwrite") in seen


def test_event_types_match_the_contract():
    """Binds the two event-type sets to 11 S6.2 rather than to this file.

    They are not in sets.yaml (every member carries a dot, which the shared
    extractor's value pattern does not match), so without this case they would
    be literals nothing checks -- the exact situation CLAUDE.md 3.5 is about.
    """
    import os
    import re
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    with open(os.path.join(root, "docs", "11-接口契约.md"),
              encoding="utf-8") as fh:
        rows = [ln for ln in fh if "`geo.created`" in ln]
    assert rows, "11 S6.2 geo event row not found -- the table moved"
    found = set(re.findall(r"`(geo\.[a-z_]+)`", rows[0]))
    assert found == GEO_EVENT_INFO | GEO_EVENT_WARN, (
        f"contract {sorted(found)} vs library "
        f"{sorted(GEO_EVENT_INFO | GEO_EVENT_WARN)}")
