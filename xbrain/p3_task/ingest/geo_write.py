"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_write.py
Brief: cmd/geo write appliers -- upsert / rename / set_state (11 S7.9)

Description:
The three write actions of the single writer (11 S7.9). Each one is ONE
BEGIN IMMEDIATE transaction that does all of:

  1. read the current row -> arbitrate base_rev (S7.9.2 optimistic concurrency);
  2. write the row, with rev = base_rev + 1 and the audit columns;
  3. write the geo_cmd_log entry that makes a redelivery idempotent (S7.9.2
     step 5).

All three in one transaction is the point of the file. Splitting the log from
the mutation would leave a window where a crash loses the log but keeps the
write, and the redelivered command -- Zenoh does not promise exactly-once (C-5)
-- then applies a second time and bumps rev again, which is precisely the
"local version is newer" state that makes every later sync conflict.

Two rules that look like details and are not:

  * rev comes from P3, never from the sender (S7.9.2 step 6). The parser does
    not even read obj.rev.
  * a newly created FENCE is state='draft', while a new route/waypoint/dock is
    state='active'. F15 (18) is a separate spoken command that ACTIVATES a
    fence, and 11 S7.9.5 gives that activation its own L2 confirmation cell --
    both are meaningless if saving a fence already enforced it. The failure
    direction matters too: a route that is active early can be driven only when
    a task names it, whereas a fence that is active early changes where the
    robot may go, immediately, without anyone approving it.

Boundaries: no clock (now_ms injected), no publishing (events are RETURNED for
the wiring to publish), no channel policy (geo_command's matrix already ran).
"""
from __future__ import annotations

import json
from typing import Any, Dict, List, Optional, Tuple

from xbrain.common.enums import GEO_CREATED_BY, GEO_STATE
from xbrain.common.errors import (
    E_GEO_CONFLICT, E_GEO_INVALID, E_NAME_CONFLICT, E_NOT_FOUND, E_SCHEMA,
)
from xbrain.p3_task.ingest.geo_apply import ApplyResult, GeoContext, register_applier
from xbrain.p3_task.ingest.geo_command import GeoCommand, GeoCommandError
from xbrain.p3_task.ingest.geo_object import (
    TABLE_FOR_TYPE, ParsedObject, parse_geo_object, polyline_len_m,
    resolvable_anchor_ids,
)
from xbrain.p3_task.state.geo_rev import content_hash

#: 11 S6.2 event/{sev}/geo detail.type closed set, info half + warn half. These
#: are the audit trail of who changed the map; S7.10 is explicit that the event
#: stream does NOT carry synchronisation duty, so nothing downstream may rebuild
#: state from these -- they are for the HMI's activity list and the cloud audit.
#:
#: Not promoted into sets.yaml with the other geo sets: every member carries a
#: dot, and the shared extractor's VALUE_RE matches backticked [a-z_]+ only, so
#: adding them would need a second value shape in the metatest. The binding to
#: the contract is kept instead by a case in test_geo_write that reads the S6.2
#: row directly -- so the set is still checked against 11, just not from there.
GEO_EVENT_INFO = frozenset({"geo.created", "geo.updated", "geo.deleted",
                            "geo.renamed"})
GEO_EVENT_WARN = frozenset({"geo.conflict", "geo.force_overwrite",
                            "geo.route_changed", "geo.route_deleted",
                            "geo.route_remap_failed"})

#: The lifecycle state a newly created object starts in, by type. See the module
#: docstring for why fence differs.
_INITIAL_STATE = {"route": "active", "waypoint": "active", "dock": "active",
                  "fence": "draft"}

#: Columns rename may touch. 11 S7.9.1 states it as: name / num / alias only,
#: never the geometry and never the geo_id. Held as data so the rename SQL
#: cannot be widened by accident -- a rename that reached geometry would edit a
#: route under a task that is driving it.
_RENAME_COLUMNS = ("name", "num", "alias_json")


def conn_for(ctx: GeoContext, gtype: str):
    """The database that owns this type. 15 S9 splits fences into their own
    file so they can be reloaded without touching route edits."""
    return ctx.fence_conn if gtype == "fence" else ctx.geo_conn


async def lookup_cmd_log(conn, cmd_id: str) -> Optional[Tuple[str, str, str]]:
    """The (result, code, detail_json) of a previously applied cmd_id, or None.

    Absent table -> None: task.db-shaped connections have no geo_cmd_log, and a
    caller that reaches here with one should fail on the write, not on the read.
    """
    cur = await conn.execute(
        "SELECT result, code, detail_json FROM geo_cmd_log WHERE cmd_id=?",
        (cmd_id,))
    row = await cur.fetchone()
    return None if row is None else (row[0], row[1], row[2])


async def _write_cmd_log(conn, cmd: GeoCommand, result: str, code: str,
                         detail: Optional[Dict[str, Any]], now_ms: int) -> None:
    """Record the outcome INSIDE the caller's open transaction (S7.9.2 step 5)."""
    await conn.execute(
        "INSERT INTO geo_cmd_log (cmd_id, action, geo_id, result, code, "
        " detail_json, applied_ms) VALUES (?, ?, ?, ?, ?, ?, ?)",
        (cmd.cmd_id, cmd.action, cmd.geo_id, result, code,
         None if detail is None else json.dumps(detail, ensure_ascii=False),
         now_ms))


def _duplicate(logged: Tuple[str, str, str]) -> ApplyResult:
    """Replay a previous outcome for a redelivered cmd_id.

    result is "duplicate" rather than the original "accepted": the sender must
    be able to tell a second delivery from a second effect, or a retry loop that
    sees "accepted" will believe it applied twice and re-read the object to
    reconcile a rev that never moved.
    """
    result, code, detail_json = logged
    detail = json.loads(detail_json) if detail_json else {}
    detail = dict(detail)
    detail["replayed"] = True
    return ApplyResult("duplicate", code, detail)


async def _current_row(conn, table: str, pk_col: str, geo_id: str):
    """(rev, content_hash, state, created_by, updated_by, updated_ms) or None."""
    cur = await conn.execute(
        f"SELECT rev, content_hash, state, created_by, updated_by, updated_ms "
        f"FROM {table} WHERE {pk_col}=?", (geo_id,))
    return await cur.fetchone()


def _conflict(cmd: GeoCommand, row) -> GeoCommandError:
    """S7.9.2 step 3: the refusal carries the LOCAL version so the sender can
    re-read and retry, or escalate to force (cloud only)."""
    rev, chash, _state, _created_by, updated_by, updated_ms = row
    return GeoCommandError(
        E_GEO_CONFLICT,
        f"base_rev {cmd.base_rev} != local rev {rev}",
        {"geo_id": cmd.geo_id, "rev": rev, "content_hash": chash,
         "updated_by": updated_by,
         # S7.8.2 keeps *_ts in wall-clock SECONDS; the column is stored in ms.
         "updated_ts": updated_ms / 1000.0})


def _created_by(cmd: GeoCommand, obj: Optional[Dict[str, Any]]) -> str:
    """Provenance for the audit columns (S7.8.2 created_by / updated_by).

    Taken from the object body when it names a valid provenance, else from the
    channel. The body is consulted FIRST because it is the only way `teach` can
    ever be recorded: teach is not a channel (it has no S7.9.5 column), so a
    teach recording arrives as origin=voice and says created_by=teach in its
    body. An off-set value in the body is refused rather than ignored -- silently
    substituting the origin would erase provenance the operator can see in the
    HMI and never notice.
    """
    if obj is not None and "created_by" in obj:
        value = obj.get("created_by")
        if value not in GEO_CREATED_BY:
            raise GeoCommandError(
                E_SCHEMA,
                f"created_by {value!r} is not in {sorted(GEO_CREATED_BY)}")
        return value
    if cmd.origin not in GEO_CREATED_BY:
        # wecom is an origin with no provenance member. It cannot reach a write
        # applier (S7.9.5 denies every write cell), so this is a guard against a
        # future matrix change silently storing an off-set provenance value.
        raise GeoCommandError(
            E_SCHEMA, f"origin {cmd.origin!r} has no created_by counterpart")
    return cmd.origin


async def _fill_anchor_length(conn, parsed: ParsedObject) -> None:
    """Compute total_len_m for a mode-A route by resolving its anchors.

    Done here rather than in the parser because it needs the database. A missing
    anchor is refused: a route whose length was measured over the anchors that
    happened to exist would silently shorten the moment a keypoint is deleted.
    """
    anchors = resolvable_anchor_ids(parsed)
    if not anchors:
        return
    coords: List[Tuple[float, float]] = []
    for wid in anchors:
        cur = await conn.execute(
            "SELECT rtk_lat, rtk_lon FROM waypoints WHERE geo_id=? "
            "AND tombstone=0", (wid,))
        row = await cur.fetchone()
        if row is None:
            raise GeoCommandError(
                E_GEO_INVALID, f"anchor waypoint {wid!r} does not exist")
        coords.append((row[0], row[1]))
    total = polyline_len_m(coords)
    if total <= 0.0:
        raise GeoCommandError(E_GEO_INVALID,
                              "route has zero length (anchors coincide)")
    parsed.columns["total_len_m"] = total


async def apply_upsert(cmd: GeoCommand, ctx: GeoContext,
                       now_ms: int) -> ApplyResult:
    """11 S7.9.1 upsert: create or WHOLE-object replace (never a field merge --
    a partly merged geometry has no safe meaning)."""
    conn = conn_for(ctx, cmd.type)
    parsed = parse_geo_object(cmd.type, cmd.geo_id, cmd.obj)
    chash = content_hash(parsed.content)
    created_by = _created_by(cmd, cmd.obj)
    await conn.execute("BEGIN IMMEDIATE")
    try:
        logged = await lookup_cmd_log(conn, cmd.cmd_id)
        if logged is not None:
            await conn.rollback()
            return _duplicate(logged)
        row = await _current_row(conn, parsed.table, parsed.pk_col, cmd.geo_id)
        await _fill_anchor_length(conn, parsed)
        events: List[Tuple[str, str, Dict[str, Any]]] = []
        if row is None:
            rev = 1
            state = _INITIAL_STATE[cmd.type]
            cols = dict(parsed.columns)
            cols.update({parsed.pk_col: cmd.geo_id, "rev": rev,
                         "state": state, "created_by": created_by,
                         "updated_by": created_by, "content_hash": chash,
                         "updated_ms": now_ms})
            names = ", ".join(cols)
            marks = ", ".join("?" for _ in cols)
            await conn.execute(
                f"INSERT INTO {parsed.table} ({names}) VALUES ({marks})",
                tuple(cols.values()))
            events.append(("info", "geo.created",
                           {"geo_id": cmd.geo_id, "type": cmd.type,
                            "state": state}))
        else:
            cur_rev, cur_hash = row[0], row[1]
            if cur_hash == chash and not cmd.force:
                # S7.9.2: identical content is not a new version. Returning
                # accepted with the unchanged rev keeps a re-sent save from
                # inflating rev, which would make every other holder stale for
                # no reason.
                await _write_cmd_log(conn, cmd, "accepted", "OK",
                                     {"geo_id": cmd.geo_id, "rev": cur_rev,
                                      "unchanged": True}, now_ms)
                await conn.commit()
                return ApplyResult(
                    "accepted", "OK",
                    {"geo_id": cmd.geo_id, "rev": cur_rev, "unchanged": True})
            if cmd.base_rev != cur_rev and not cmd.force:
                raise _conflict(cmd, row)
            rev = cur_rev + 1
            state = row[2]                  # upsert does not change lifecycle
            cols = dict(parsed.columns)
            cols.update({"rev": rev, "updated_by": created_by,
                         "content_hash": chash, "updated_ms": now_ms})
            sets = ", ".join(f"{k}=?" for k in cols)
            await conn.execute(
                f"UPDATE {parsed.table} SET {sets} WHERE {parsed.pk_col}=?",
                tuple(cols.values()) + (cmd.geo_id,))
            events.append(("info", "geo.updated",
                           {"geo_id": cmd.geo_id, "type": cmd.type,
                            "rev": rev}))
            if cmd.force and cmd.base_rev != cur_rev:
                # S7.9.2 step 4: a forced overwrite past a real conflict MUST
                # leave a warn-level trace -- somebody's edit was discarded.
                events.append(("warn", "geo.force_overwrite",
                               {"geo_id": cmd.geo_id, "overwritten_rev": cur_rev,
                                "base_rev": cmd.base_rev,
                                "origin": cmd.origin}))
        detail = {"geo_id": cmd.geo_id, "rev": rev, "state": state}
        if parsed.columns.get("total_len_m") is not None:
            # S12A.9's spoken confirmation quotes these two back to the operator
            # ("64 points, 320 metres"), so they ride in the ack.
            detail["total_len_m"] = round(parsed.columns["total_len_m"], 1)
        if parsed.columns.get("path_points"):
            detail["point_count"] = len(
                json.loads(parsed.columns["path_points"]))
        await _write_cmd_log(conn, cmd, "accepted", "OK", detail, now_ms)
        await conn.commit()
        return ApplyResult("accepted", "OK", detail, tuple(events))
    except Exception as exc:
        await conn.rollback()
        raise _as_name_conflict(exc, cmd)


async def apply_rename(cmd: GeoCommand, ctx: GeoContext,
                       now_ms: int) -> ApplyResult:
    """11 S7.9.1 rename: name / num / alias only. Geometry and geo_id are
    untouchable here, which is what makes rename safe to run on an object a
    task is currently driving."""
    conn = conn_for(ctx, cmd.type)
    table, pk_col, _prefix = TABLE_FOR_TYPE[cmd.type]
    obj = cmd.obj or {}
    # Build only the columns actually supplied: a rename that sent just a new
    # name must not blank the aliases.
    cols: Dict[str, Any] = {}
    if "name" in obj:
        from xbrain.p3_task.ingest.geo_object import _name  # noqa: PLC0415
        cols["name"] = _name(obj, required=True)
    if "num" in obj:
        from xbrain.p3_task.ingest.geo_object import _num   # noqa: PLC0415
        cols["num"] = _num(obj)
    if "alias" in obj:
        from xbrain.p3_task.ingest.geo_object import _alias_json  # noqa: PLC0415
        cols["alias_json"] = _alias_json(obj)
    if not cols:
        raise GeoCommandError(
            E_SCHEMA, "rename needs obj with at least one of name / num / alias")
    unknown = set(cols) - set(_RENAME_COLUMNS)
    if unknown:                       # unreachable unless _RENAME_COLUMNS drifts
        raise GeoCommandError(E_SCHEMA, f"rename cannot touch {sorted(unknown)}")
    await conn.execute("BEGIN IMMEDIATE")
    try:
        logged = await lookup_cmd_log(conn, cmd.cmd_id)
        if logged is not None:
            await conn.rollback()
            return _duplicate(logged)
        row = await _current_row(conn, table, pk_col, cmd.geo_id)
        if row is None:
            raise GeoCommandError(E_NOT_FOUND,
                                  f"{cmd.type} {cmd.geo_id!r} does not exist")
        if cmd.base_rev != row[0] and not cmd.force:
            raise _conflict(cmd, row)
        rev = row[0] + 1
        # content_hash covers name as well as geometry (S7.8.2), so a rename is
        # a content change and must re-hash -- otherwise a cloud sync comparing
        # hashes would conclude the renamed object is identical and skip it.
        cur = await conn.execute(
            f"SELECT content_hash FROM {table} WHERE {pk_col}=?", (cmd.geo_id,))
        old_hash = (await cur.fetchone())[0]
        new_hash = content_hash({"prev": old_hash,
                                 "name": cols.get("name"),
                                 "num": cols.get("num"),
                                 "alias": cols.get("alias_json")})
        cols.update({"rev": rev, "updated_by": _created_by(cmd, cmd.obj),
                     "content_hash": new_hash, "updated_ms": now_ms})
        sets = ", ".join(f"{k}=?" for k in cols)
        await conn.execute(f"UPDATE {table} SET {sets} WHERE {pk_col}=?",
                           tuple(cols.values()) + (cmd.geo_id,))
        detail = {"geo_id": cmd.geo_id, "rev": rev,
                  "name": cols.get("name")}
        await _write_cmd_log(conn, cmd, "accepted", "OK", detail, now_ms)
        await conn.commit()
        return ApplyResult("accepted", "OK", detail,
                           (("info", "geo.renamed",
                             {"geo_id": cmd.geo_id, "type": cmd.type,
                              "name": cols.get("name")}),))
    except Exception as exc:
        await conn.rollback()
        raise _as_name_conflict(exc, cmd)


async def apply_set_state(cmd: GeoCommand, ctx: GeoContext,
                          now_ms: int) -> ApplyResult:
    """11 S7.9.1 set_state: draft <-> active <-> disabled. F15 ("activate the
    camp fence") is this action.

    The target state travels in obj.state. 11 S7.9.1 does not name a field for
    it -- the envelope has no state member -- so it is read from the object
    body, the only place a per-action parameter can ride; the 2026-08-20 note in
    S7.9.1 records that choice.

    'deleted' is NOT reachable here: a tombstone is what `delete` produces, with
    its own cloud-only permission cell for fences and its own reference sweep
    (GC-1..7). Allowing set_state to write 'deleted' would be a second delete
    path with neither of those.
    """
    conn = conn_for(ctx, cmd.type)
    table, pk_col, _prefix = TABLE_FOR_TYPE[cmd.type]
    target = (cmd.obj or {}).get("state")
    if target not in GEO_STATE:
        raise GeoCommandError(
            E_SCHEMA,
            f"set_state needs obj.state in {sorted(GEO_STATE)}, got {target!r}")
    if target == "deleted":
        raise GeoCommandError(
            E_SCHEMA, "set_state cannot delete; use action=delete (S7.11.4)")
    await conn.execute("BEGIN IMMEDIATE")
    try:
        logged = await lookup_cmd_log(conn, cmd.cmd_id)
        if logged is not None:
            await conn.rollback()
            return _duplicate(logged)
        row = await _current_row(conn, table, pk_col, cmd.geo_id)
        if row is None:
            raise GeoCommandError(E_NOT_FOUND,
                                  f"{cmd.type} {cmd.geo_id!r} does not exist")
        if row[2] == "deleted":
            # Resurrecting a tombstone would revive an object the reference
            # sweep already acted on (tasks suspended/aborted against it).
            raise GeoCommandError(
                E_GEO_INVALID,
                f"{cmd.geo_id!r} is deleted; set_state cannot revive it")
        if cmd.base_rev != row[0] and not cmd.force:
            raise _conflict(cmd, row)
        rev = row[0] + 1
        # The fence quota triggers fire on THIS update (S9A.1A: <= 5 active, at
        # most 1 allow), which is why activation is where they had to exist --
        # every fence can be inserted as a draft without limit.
        await conn.execute(
            f"UPDATE {table} SET state=?, rev=?, updated_by=?, updated_ms=? "
            f"WHERE {pk_col}=?",
            (target, rev, _created_by(cmd, cmd.obj), now_ms, cmd.geo_id))
        detail = {"geo_id": cmd.geo_id, "rev": rev, "state": target,
                  "previous_state": row[2]}
        await _write_cmd_log(conn, cmd, "accepted", "OK", detail, now_ms)
        await conn.commit()
        return ApplyResult("accepted", "OK", detail,
                           (("info", "geo.updated",
                             {"geo_id": cmd.geo_id, "type": cmd.type,
                              "state": target}),))
    except Exception as exc:
        await conn.rollback()
        raise _as_name_conflict(exc, cmd)


def _as_name_conflict(exc: Exception, cmd: GeoCommand) -> Exception:
    """Turn the UNIQUE(name) violation into E_NAME_CONFLICT, leave the rest.

    The name column is UNIQUE so voice navigation resolves a spoken name to one
    object; when an operator names a second route "east gate" the honest answer
    is "that name is taken", not the sqlite text. Everything else propagates
    unchanged -- mapping broadly here would relabel real storage failures.
    """
    text = str(exc)
    if "UNIQUE constraint failed" in text and ".name" in text:
        return GeoCommandError(
            E_NAME_CONFLICT,
            f"the name given for {cmd.geo_id!r} is already used by another "
            f"{cmd.type}", {"geo_id": cmd.geo_id})
    return exc


register_applier("upsert", apply_upsert)
register_applier("rename", apply_rename)
register_applier("set_state", apply_set_state)
