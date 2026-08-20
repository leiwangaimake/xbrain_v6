"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_read.py
Brief: cmd/geo read actions -- get / list / resync (11 S7.9.1, S7.10, S7.11.2)

Description:
The read half of the single writer. Three actions, and each answers a different
question:

  get     one object in full, geometry included (S7.9.1: the result rides in
          ack.detail.obj). This is how P1 pulls a fence polygon after seeing
          active_fence change in the manifest, and how G42 answers "how big is
          that fence" -- the only request-response query in the whole G class.
  list    the manifest subset (S7.10): summaries only, no geometry, so it stays
          small enough to send at 0.1 Hz.
  resync  full synchronisation (S7.11.2), for when incremental comparison is no
          longer worth it -- more than resync_threshold differences, a
          catalog_rev that went BACKWARDS (the robot restored its SQLite from a
          backup), or the cloud adopting a robot it has never seen.

*** On prune, which is deliberately NOT implemented.

S7.11.2 allows resync{direction:"push", prune:true} to tombstone every local
object the cloud's full set does not mention, and states the one safety
condition in bold: P3 must SKIP objects with dirty == true, because those are
the routes and fences the operator recorded while the link was down -- the cloud
has never heard of them, and pruning them destroys field work silently.

Evaluating `dirty` needs synced_rev (S7.8.2: dirty means rev > synced_rev), and
nothing in this system maintains synced_rev -- there is no path by which a cloud
acknowledgement reaches back into the row. So the guard the contract requires
cannot be evaluated, which makes a prune here a delete with its safety condition
stubbed out. It is refused with E_NOT_IMPLEMENTED and the reason, rather than
run without the guard or run with a guard that always says "not dirty" (which
would be the always-green assertion CLAUDE.md 3.2 catalogues, in its most
expensive form: a check that passes by construction while deleting data).

The rest of resync works: push without prune upserts what it is given, pull
returns the full set.
"""
from __future__ import annotations

import hashlib
import json
from typing import Any, Dict, List, Optional, Tuple

from xbrain.common.enums import GEO_TYPE
from xbrain.common.errors import E_NOT_FOUND, E_NOT_IMPLEMENTED, E_SCHEMA
from xbrain.p3_task.ingest.geo_apply import (
    ApplyResult, GeoContext, register_applier,
)
from xbrain.p3_task.ingest.geo_command import GeoCommand, GeoCommandError
from xbrain.p3_task.ingest.geo_object import TABLE_FOR_TYPE
from xbrain.p3_task.ingest.geo_write import apply_upsert, conn_for

#: The metadata columns every geo table carries (S7.8.2 common block). Selected
#: by name rather than with * so a column added to one table does not silently
#: change the shape of every object this returns.
_META_COLS = ("name", "num", "alias_json", "state", "created_by", "updated_by",
              "rev", "content_hash", "updated_ms")


def _meta(row: Tuple, geo_id: str, gtype: str) -> Dict[str, Any]:
    """The S7.8.2 common metadata block from a _META_COLS row."""
    (name, num, alias_json, state, created_by, updated_by, rev, chash,
     updated_ms) = row
    return {
        "geo_id": geo_id, "type": gtype, "name": name, "num": num,
        "alias": json.loads(alias_json or "[]"),
        "rev": rev, "content_hash": chash, "state": state,
        "created_by": created_by, "updated_by": updated_by,
        # S7.8.2 keeps *_ts in wall-clock SECONDS; the column is ms. These are
        # display and audit values only -- 11 S7.8.2 is explicit that they take
        # no part in any timeout or age decision (conflict resolution uses the
        # integer rev precisely so the RTK wall-clock step cannot pick a winner).
        "updated_ts": updated_ms / 1000.0,
    }


async def _read_object(conn, gtype: str, geo_id: str) -> Optional[Dict[str, Any]]:
    """One full GeoObject (S7.8.2 metadata + S7.8.3 geom), or None."""
    table, pk_col, _prefix = TABLE_FOR_TYPE[gtype]
    meta_sql = ", ".join(_META_COLS)
    if gtype == "route":
        extra = "waypoint_ids, path_points, loop_mode, direction, max_speed, " \
                "total_len_m, description"
    elif gtype == "waypoint":
        extra = "type, rtk_lat, rtk_lon, rtk_alt, yaw_deg, arrival_radius, " \
                "description"
    elif gtype == "dock":
        extra = "rtk_lat, rtk_lon, rtk_alt, dock_heading_rad, handover_lat, " \
                "handover_lon, handover_heading_rad, handover_tol_m, " \
                "handover_tol_rad, on_route_json, enabled, description"
    else:
        extra = "role, kind, geom_json, hard_enforce, soft_margin_m"
    cur = await conn.execute(
        f"SELECT {meta_sql}, {extra} FROM {table} WHERE {pk_col}=?", (geo_id,))
    row = await cur.fetchone()
    if row is None:
        return None
    obj = _meta(row[:len(_META_COLS)], geo_id, gtype)
    rest = row[len(_META_COLS):]
    obj["geom"] = _geom(gtype, rest)
    return obj


def _geom(gtype: str, rest: Tuple) -> Dict[str, Any]:
    """The S7.8.3 geometry block for this type."""
    if gtype == "route":
        (waypoint_ids, path_points, loop_mode, direction, max_speed,
         total_len_m, _description) = rest
        points: List[Dict[str, float]] = []
        if path_points:
            points = [{"lat": la, "lon": lo}
                      for la, lo in json.loads(path_points)]
        return {"loop_mode": loop_mode, "direction": direction,
                "max_speed": max_speed, "total_len_m": total_len_m,
                "point_count": len(points), "points": points,
                # Mode A keeps its anchors instead of resolved coordinates: the
                # consumer that cares (a route editor) needs to know the route
                # is anchored, and one that only draws it can resolve them.
                "waypoint_ids": json.loads(waypoint_ids or "null")
                if waypoint_ids else None}
    if gtype == "waypoint":
        (wtype, lat, lon, alt, yaw_deg, arrive_radius, _description) = rest
        return {"lat": lat, "lon": lon, "alt": alt, "type": wtype,
                "yaw_deg": yaw_deg, "arrive_radius_m": arrive_radius}
    if gtype == "dock":
        (lat, lon, alt, heading, h_lat, h_lon, h_heading, tol_m, tol_rad,
         on_route_json, enabled, _description) = rest
        return {"lat": lat, "lon": lon, "alt": alt,
                "dock_heading_rad": heading,
                "handover": {"lat": h_lat, "lon": h_lon,
                             "heading_rad": h_heading, "tol_m": tol_m,
                             "tol_rad": tol_rad},
                "on_route": json.loads(on_route_json or "[]"),
                "enabled": bool(enabled)}
    role, kind, geom_json, hard_enforce, soft_margin = rest
    ring = json.loads(geom_json or "{}").get("points", [])
    return {"role": role, "kind": kind, "hard_enforce": bool(hard_enforce),
            "soft_margin_m": soft_margin,
            "outer": [{"lat": la, "lon": lo} for la, lo in ring]}


async def _summaries(conn, gtype: str) -> List[Dict[str, Any]]:
    """The manifest items for one type (S7.10: summaries, never geometry)."""
    table, pk_col, _prefix = TABLE_FOR_TYPE[gtype]
    cur = await conn.execute(
        f"SELECT {pk_col}, name, num, rev, content_hash, state, updated_ms "
        f"FROM {table} ORDER BY {pk_col}")
    return [{"geo_id": r[0], "type": gtype, "name": r[1], "num": r[2],
             "rev": r[3], "hash": r[4], "state": r[5],
             "updated_ts": r[6] / 1000.0,
             # S7.8.2 dirty means rev > synced_rev, and synced_rev is not
             # maintained anywhere (see the module docstring on prune). Emitting
             # a computed false would be a claim we cannot support, so the field
             # is emitted as null: "not known", which is the truth.
             "dirty": None}
            for r in await cur.fetchall()]


def _catalog_hash(items: List[Dict[str, Any]]) -> str:
    """sha256 over (geo_id, rev, state) sorted by geo_id, first 12 hex (S7.10).

    state is part of it deliberately: an object that was tombstoned changes no
    geometry and would otherwise leave the catalog hash untouched, and the
    cloud would never learn to tombstone its copy.
    """
    body = ";".join("%s:%s:%s" % (i["geo_id"], i["rev"], i["state"])
                    for i in sorted(items, key=lambda i: i["geo_id"]))
    return hashlib.sha256(body.encode("utf-8")).hexdigest()[:12]


async def build_manifest(ctx: GeoContext,
                         types: Optional[List[str]] = None) -> Dict[str, Any]:
    """The S7.10 GeoManifest (or the subset `types` names)."""
    wanted = [t for t in (types or sorted(GEO_TYPE)) if t in GEO_TYPE]
    items: List[Dict[str, Any]] = []
    counts: Dict[str, int] = {}
    for gtype in wanted:
        rows = await _summaries(conn_for(ctx, gtype), gtype)
        items.extend(rows)
        # The count is of LIVE objects: a tombstone is still an item (the cloud
        # has to see the deletion) but it is not one of "6 routes".
        counts[gtype] = sum(1 for r in rows if r["state"] != "deleted")
    active_fence = [i["geo_id"] for i in items
                    if i["type"] == "fence" and i["state"] == "active"]
    # catalog_rev is max(updated_ms) across the live rows -- the same token
    # state/geo/objects publishes (11 S7.10A GO-2). Two different numbers under
    # one name in one system is how a consumer ends up comparing them.
    catalog_rev = max((int(i["updated_ts"] * 1000) for i in items
                       if i["state"] != "deleted"), default=0)
    return {"catalog_rev": catalog_rev, "catalog_hash": _catalog_hash(items),
            "counts": counts, "active_fence": active_fence, "items": items}


async def apply_get(cmd: GeoCommand, ctx: GeoContext,
                    now_ms: int) -> ApplyResult:
    """11 S7.9.1 get: one object in full. Read-only, no cmd_log (see apply_refs
    in geo_delete for why a query is not logged for idempotency)."""
    obj = await _read_object(conn_for(ctx, cmd.type), cmd.type, cmd.geo_id)
    if obj is None:
        raise GeoCommandError(E_NOT_FOUND,
                              f"{cmd.type} {cmd.geo_id!r} does not exist")
    # A tombstone is RETURNED, not hidden: S7.11.1 has the cloud pull an object
    # it is behind on, and if a deleted object answered "not found" the cloud
    # would treat it as missing and push its own copy back.
    return ApplyResult("accepted", "OK", {"obj": obj})


async def apply_list(cmd: GeoCommand, ctx: GeoContext,
                     now_ms: int) -> ApplyResult:
    """11 S7.9.1 list: the manifest, optionally filtered by type."""
    types = None
    if cmd.obj and isinstance(cmd.obj.get("types"), list):
        types = [t for t in cmd.obj["types"] if isinstance(t, str)]
        unknown = [t for t in types if t not in GEO_TYPE]
        if unknown:
            raise GeoCommandError(E_SCHEMA, f"unknown geo types {unknown}")
    return ApplyResult("accepted", "OK", await build_manifest(ctx, types))


async def apply_resync(cmd: GeoCommand, ctx: GeoContext,
                       now_ms: int) -> ApplyResult:
    """11 S7.11.2 resync. Cloud-only (the S7.9.5 matrix already enforced that).

    pull returns the full set; push applies the objects it was given. prune is
    refused -- see the module docstring.
    """
    body = cmd.obj or {}
    direction = body.get("direction", "pull")
    if direction not in ("push", "pull"):
        raise GeoCommandError(
            E_SCHEMA, f"resync.direction must be push|pull, got {direction!r}")
    types = body.get("types") if isinstance(body.get("types"), list) else None
    if body.get("prune"):
        raise GeoCommandError(
            E_NOT_IMPLEMENTED,
            "prune is refused: it must skip dirty objects (11 S7.11.2), and "
            "dirty needs synced_rev, which nothing in this build maintains -- "
            "pruning without that guard silently destroys geometry recorded "
            "while the link was down",
            {"reason": "prune_guard_unavailable"})
    if direction == "pull":
        return await _resync_pull(ctx, types)
    return await _resync_push(cmd, ctx, body, now_ms)


async def _resync_pull(ctx: GeoContext,
                       types: Optional[List[str]]) -> ApplyResult:
    """Upload the full set. Tombstones included -- the receiver needs them to
    mirror the deletions (S7.11.1)."""
    wanted = [t for t in (types or sorted(GEO_TYPE)) if t in GEO_TYPE]
    objects: List[Dict[str, Any]] = []
    for gtype in wanted:
        conn = conn_for(ctx, gtype)
        for summary in await _summaries(conn, gtype):
            obj = await _read_object(conn, gtype, summary["geo_id"])
            if obj is not None:
                objects.append(obj)
    manifest = await build_manifest(ctx, types)
    # No chunking here. S7.9.3 puts the split on the SENDER of a large object
    # and it is not built; at camp scale (a handful of routes and a few dozen
    # keypoints) the full set is tens of kilobytes. object_count rides in the
    # detail so a receiver can tell a truncated answer from a small estate --
    # this side never truncates, and says so by always reporting the real count.
    return ApplyResult("accepted", "OK",
                       {"direction": "pull", "object_count": len(objects),
                        "objects": objects,
                        "catalog_rev": manifest["catalog_rev"],
                        "catalog_hash": manifest["catalog_hash"]})


async def _resync_push(cmd: GeoCommand, ctx: GeoContext, body: Dict[str, Any],
                       now_ms: int) -> ApplyResult:
    """Apply a full set from the cloud, object by object, through the ordinary
    upsert applier.

    Reusing apply_upsert rather than writing a bulk path is the point: rev
    arbitration, name conflicts, geometry validation and the audit columns then
    behave identically whether an object arrived one at a time or in a resync.
    A bulk writer is where those rules quietly stop applying.
    """
    objects = body.get("objects")
    if not isinstance(objects, list):
        raise GeoCommandError(E_SCHEMA, "resync push needs obj.objects[]")
    applied, failed = [], []
    for i, obj in enumerate(objects):
        if not isinstance(obj, dict):
            failed.append({"index": i, "reason": "not an object"})
            continue
        gtype, geo_id = obj.get("type"), obj.get("geo_id")
        if gtype not in GEO_TYPE or not isinstance(geo_id, str):
            failed.append({"index": i, "reason": "missing type/geo_id"})
            continue
        # base_rev is taken from the LOCAL row, not from the pushed object: a
        # resync is the cloud asserting its set, and forcing each item through
        # the normal conflict check would reject every object the robot has
        # touched since -- which is the situation resync exists to resolve.
        local = await _read_object(conn_for(ctx, gtype), gtype, geo_id)
        sub = GeoCommand(
            cmd_id="%s:%d" % (cmd.cmd_id, i), action="upsert",
            origin=cmd.origin, type=gtype, geo_id=geo_id,
            base_rev=local["rev"] if local else 0, force=False, obj=obj,
            chunk=None)
        try:
            res = await apply_upsert(sub, ctx, now_ms)
            applied.append({"geo_id": geo_id, "rev": res.detail.get("rev")})
        except GeoCommandError as exc:
            # One bad object does not abort the resync: the remaining ones are
            # still worth applying, and the failures are reported per object so
            # the cloud can retry exactly those.
            failed.append({"index": i, "geo_id": geo_id, "code": exc.code,
                           "reason": str(exc)})
    manifest = await build_manifest(ctx, None)
    return ApplyResult(
        "accepted", "OK",
        {"direction": "push", "applied": len(applied), "failed": failed,
         "catalog_rev": manifest["catalog_rev"],
         "catalog_hash": manifest["catalog_hash"]},
        (("info", "geo.updated",
          {"resync": True, "applied": len(applied),
           "failed": len(failed)}),))


register_applier("get", apply_get)
register_applier("list", apply_list)
register_applier("resync", apply_resync)
