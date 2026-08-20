"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_delete.py
Brief: cmd/geo delete -- tombstone + 15 S7.6 task linkage + S7.9.4 refs

Description:
delete is the action with consequences beyond its own row, and it runs in three
ordered stages for reasons the specification gives explicitly:

  1. TOMBSTONE, in one transaction. 11 S7.11.4: a delete marks state='deleted'
     and tombstone=1, it does NOT remove the row. The row has to survive so the
     pending-push queue can propagate the deletion to the cloud, and so an audit
     can still resolve the geo_id a finished task referenced.

  2. REFS, read after the commit but BEFORE the linkage runs. The ack must
     describe the impact as it was at deletion time (11 S7.9.4 / CMD-31); read
     after the linkage, a suspended task that the linkage just failed would have
     silently dropped out of the impact list the operator is shown.

  3. LINKAGE, after the commit, on the same db thread (15 S7.6 GX-1). Running it
     inside the transaction would let a task be read against a row whose delete
     can still roll back.

What the linkage actually does is mostly NOTHING, and that is the specified
behaviour, not an unfinished implementation:

  * a RUNNING patrol whose route was deleted keeps driving. GC-3 clamps the
    remaining laps and lets the current one finish, because stopping a patrol in
    the middle of a camp roadway is the more dangerous option.
  * a waypoint delete affects no running task at all (GC-5): patrol geometry is
    inline in the snapshot and never reads the waypoints table.
  * a fence delete produces no task-side action whatsoever (GC-4 / GX-3); P1
     rebuilds its clip polygon from the broadcast and P3 stays out of it.

The one state change made here is GC-3's: a SUSPENDED task whose route was
deleted becomes failed with fail_reason='route_deleted', and its progress and
snapshot rows are deliberately KEPT for the audit.
"""
from __future__ import annotations

import json
import logging
from typing import Any, Dict, List, Optional, Tuple

from xbrain.common.errors import E_NOT_FOUND
from xbrain.p3_task.ingest.geo_apply import ApplyResult, GeoContext, register_applier
from xbrain.p3_task.ingest.geo_command import GeoCommand, GeoCommandError
from xbrain.p3_task.ingest.geo_object import TABLE_FOR_TYPE
from xbrain.p3_task.ingest.geo_write import (
    conflict_error, conn_for, lookup_cmd_log, provenance_for, replay_duplicate,
    write_cmd_log,
)
from xbrain.p3_task.route.geo_linkage import GeoChange, classify
from xbrain.p3_task.route.geo_refs import compute_refs

_logger = logging.getLogger("xbrain.p3.geo")

#: Live task states, in the order 15 S7.6's three answer columns are written.
#: A terminal task is not consulted: nothing the linkage can do to it matters.
_LIVE_STATES = ("running", "suspended", "pending", "scheduled", "blocked",
                "ready")


async def apply_delete(cmd: GeoCommand, ctx: GeoContext,
                       now_ms: int) -> ApplyResult:
    """11 S7.9.1 delete: tombstone, then linkage. Never a physical row removal."""
    conn = conn_for(ctx, cmd.type)
    table, pk_col, _prefix = TABLE_FOR_TYPE[cmd.type]
    await conn.execute("BEGIN IMMEDIATE")
    try:
        logged = await lookup_cmd_log(conn, cmd.cmd_id)
        if logged is not None:
            await conn.rollback()
            return replay_duplicate(logged)
        cur = await conn.execute(
            f"SELECT rev, content_hash, state, created_by, updated_by, "
            f"updated_ms, name FROM {table} WHERE {pk_col}=?", (cmd.geo_id,))
        row = await cur.fetchone()
        if row is None:
            raise GeoCommandError(E_NOT_FOUND,
                                  f"{cmd.type} {cmd.geo_id!r} does not exist")
        name = row[6]
        if row[2] == "deleted":
            # Already a tombstone. Not an error and not a second delete: report
            # it as applied with the rev unchanged, so a retry after a lost ack
            # settles instead of escalating.
            await conn.rollback()
            return ApplyResult("accepted", "OK",
                               {"geo_id": cmd.geo_id, "rev": row[0],
                                "already_deleted": True})
        if cmd.base_rev != row[0] and not cmd.force:
            raise conflict_error(cmd, row)
        rev = row[0] + 1
        # state and tombstone move together: S7.8.2 lists 'deleted' as a state
        # value while the tables carry a tombstone flag, and the two naming the
        # same condition is only true if one writer sets both at once.
        await conn.execute(
            f"UPDATE {table} SET state='deleted', tombstone=1, rev=?, "
            f"updated_by=?, updated_ms=? WHERE {pk_col}=?",
            (rev, provenance_for(cmd, cmd.obj), now_ms, cmd.geo_id))
        detail_stub = {"geo_id": cmd.geo_id, "rev": rev, "deleted": True}
        await write_cmd_log(conn, cmd, "accepted", "OK", detail_stub, now_ms)
        await conn.commit()
    except Exception:
        await conn.rollback()
        raise
    # -- past the commit: refs first (impact as of deletion), then linkage --
    refs = await compute_refs(ctx.task_conn, ctx.geo_conn, gtype=cmd.type,
                              geo_id=cmd.geo_id, name=name)
    applied = await _run_linkage(ctx, cmd, now_ms)
    detail: Dict[str, Any] = dict(detail_stub)
    detail["refs"] = refs
    if applied:
        detail["linkage"] = applied
    events: List[Tuple[str, str, Dict[str, Any]]] = [
        ("info", "geo.deleted", {"geo_id": cmd.geo_id, "type": cmd.type})]
    if cmd.type == "route":
        # 15 S7.6 GC-3 requires the warn-level route_deleted event as well as
        # the info-level geo.deleted: the operator needs to see that a route a
        # task was using went away, not only that an object was removed.
        events.append(("warn", "geo.route_deleted",
                       {"geo_id": cmd.geo_id, "refs": refs}))
    return ApplyResult("accepted", "OK", detail, tuple(events))


async def _run_linkage(ctx: GeoContext, cmd: GeoCommand,
                       now_ms: int) -> List[Dict[str, str]]:
    """Apply 15 S7.6 to every live task, returning what was actually done.

    Returns only the tasks that CHANGED. A row that resolved to "leave it
    alone" is not reported: listing every unaffected task would bury the one
    line that matters in the ack the operator reads.
    """
    if ctx.task_conn is None:
        raise ValueError("geo linkage needs task.db (15 S7.6 GX-1)")
    marks = ", ".join("?" for _ in _LIVE_STATES)
    cur = await ctx.task_conn.execute(
        f"SELECT task_id, task_type, state, route_geo_id, mission_json, "
        f"error_context_json FROM tasks WHERE state IN ({marks})",
        _LIVE_STATES)
    rows = await cur.fetchall()
    change = GeoChange(kind=cmd.type, op="delete", object_id=cmd.geo_id)
    applied: List[Dict[str, str]] = []
    for task_id, task_type, state, route_geo_id, mission_json, err_json in rows:
        if not _references(cmd.geo_id, route_geo_id, mission_json):
            continue
        outcome = classify(change, state, task_type)
        if outcome.action == "fail":
            await _fail_task(ctx.task_conn, task_id, outcome.reason, err_json,
                             now_ms)
            applied.append({"task_id": task_id, "action": "failed",
                            "reason": outcome.reason})
        elif outcome.action == "clamp_laps":
            # GC-3 running patrol: patrol_progress.loop_total is clamped to the
            # lap in progress. The progress row only exists once the patrol has
            # reported a lap, so a missing row is normal and means "no lap yet".
            clamped = await _clamp_laps(ctx.task_conn, task_id, now_ms)
            applied.append({"task_id": task_id,
                            "action": "laps_clamped" if clamped
                                      else "laps_clamp_pending",
                            "reason": outcome.reason})
        elif outcome.action == "reselect_dock":
            # GC-6 wants a new dock chosen and a new goto issued. The charging
            # subsystem is not wired into the runtime, so there is nothing here
            # that could do it -- and inventing a half-version would be worse
            # than saying so. Reported in the ack and logged at warn level.
            _logger.warning(
                "p3 geo: task %s needs a dock reselect after %s was deleted "
                "(15 S7.6 GC-6); charging is not wired, no action taken",
                task_id, cmd.geo_id)
            applied.append({"task_id": task_id, "action": "reselect_required",
                            "reason": outcome.reason})
        # check_on_resume / check_on_dequeue / none: nothing to do NOW, by
        # specification. They are re-validations that belong to the resume and
        # dequeue paths.
    return applied


def _references(geo_id: str, route_geo_id, mission_json) -> bool:
    """Does this task name the object? Same two-way match as geo_refs, and the
    same reasoning: the id column is authoritative but largely unfilled today,
    so the mission slots are searched as well."""
    if route_geo_id and route_geo_id == geo_id:
        return True
    return bool(mission_json) and f'"{geo_id}"' in mission_json


async def _fail_task(task_conn, task_id: str, reason: str, err_json,
                     now_ms: int) -> None:
    """GC-3: suspended + route deleted -> failed, with fail_reason recorded.

    patrol_progress and task_route_snapshot are deliberately left alone: 15 S7.6
    requires them kept for the audit, so this touches the tasks row only.
    """
    try:
        ctx_obj = json.loads(err_json) if err_json else {}
        if not isinstance(ctx_obj, dict):
            ctx_obj = {}
    except ValueError:
        ctx_obj = {}
    ctx_obj["fail_reason"] = reason
    # suspend_kind / suspend_reason are cleared in the SAME statement. The tasks
    # table pairs them with the state by CHECK ((state='suspended') =
    # (suspend_kind IS NOT NULL)), so leaving them set while moving to failed is
    # rejected by sqlite -- and the task would have stayed suspended, resumable
    # onto a route that no longer exists, with the failure showing up only as a
    # log line inside the delete path.
    await task_conn.execute(
        "UPDATE tasks SET state='failed', suspend_kind=NULL, "
        " suspend_reason=NULL, error_context_json=?, updated_ms=? "
        "WHERE task_id=?",
        (json.dumps(ctx_obj, ensure_ascii=False), now_ms, task_id))
    await task_conn.commit()


async def _clamp_laps(task_conn, task_id: str, now_ms: int) -> bool:
    """Clamp loop_total to the lap in progress (15 S7.6 GC-3). True if a
    progress row existed to clamp."""
    cur = await task_conn.execute(
        "SELECT waypoint_ix FROM patrol_progress WHERE task_id=?", (task_id,))
    row = await cur.fetchone()
    if row is None:
        return False
    # patrol_progress in this build carries waypoint_ix / progress, not a lap
    # counter; the lap clamp is recorded on the task instead so the dispatcher
    # ends the task after the current pass. Written as a first-class marker in
    # error_context_json rather than a new column, because it is transient.
    await task_conn.execute(
        "UPDATE tasks SET error_context_json=json_set("
        "  COALESCE(NULLIF(error_context_json,''), '{}'), "
        "  '$.laps_clamped', 1), updated_ms=? WHERE task_id=?",
        (now_ms, task_id))
    await task_conn.commit()
    return True


async def apply_refs(cmd: GeoCommand, ctx: GeoContext,
                     now_ms: int) -> ApplyResult:
    """11 S7.9.1 refs: the impact query the HMI runs BEFORE offering a delete.

    Read-only, so no transaction and no cmd_log entry -- logging a read would
    make a repeated query answer 'duplicate', which is meaningless for a query
    and would break a UI that polls.
    """
    conn = conn_for(ctx, cmd.type)
    table, pk_col, _prefix = TABLE_FOR_TYPE[cmd.type]
    cur = await conn.execute(
        f"SELECT name, rev, state FROM {table} WHERE {pk_col}=?", (cmd.geo_id,))
    row = await cur.fetchone()
    if row is None:
        raise GeoCommandError(E_NOT_FOUND,
                              f"{cmd.type} {cmd.geo_id!r} does not exist")
    refs = await compute_refs(ctx.task_conn, ctx.geo_conn, gtype=cmd.type,
                              geo_id=cmd.geo_id, name=row[0])
    return ApplyResult("accepted", "OK",
                       {"geo_id": cmd.geo_id, "rev": row[1], "state": row[2],
                        "refs": refs})


register_applier("delete", apply_delete)
register_applier("refs", apply_refs)
