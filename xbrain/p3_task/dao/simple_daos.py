"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: simple_daos.py
Brief: BIZ-P3-6 remaining DAOs (progress / geo_object / snapshot / pending_push / memory / fences / docks)

Description:
Seven single-table DAOs collected here because each is <30 lines
and there is no cross-cutting behaviour. Splitting them across seven
files would fragment review without adding structure. TasksDAO is
in its own file because it has more surface area (state transitions,
priority scan) and downstream code will grow it further.

Every method uses parameterised SQL. Reads return raw tuples so
callers can shape rows in the domain layer -- 15 S9 forbids the
DAO from performing schema translation, which is where drift enters.
"""

from __future__ import annotations


class PatrolProgressDAO:
    """15 S9.5 patrol_progress. One `active` row per task (idx_patrol_active).

    *** Rewritten 2026-09-28 with the table (docs/NEXT.md EX-6). The previous
    version wrote waypoint_ix / progress, two names that exist nowhere in
    15 S9.5 or in the PathProgress wire message (11 S3.5B); `progress` was a
    derived 0..1 fraction, which PP-3 forbids storing at all. See
    schema_task.DDL_PATROL_PROGRESS for the full reasoning.

    Boundary: this DAO decides nothing. WHEN to write is PP-1 (waypoint change
    or progress_flush_s, whichever comes first) and lives in
    state/path_progress.py; this only performs the write it is told to.
    """

    def __init__(self, conn) -> None:
        self._conn = conn

    async def start_run(self, *, task_id: str, route_name: str,
                        route_geo_id, route_rev: int, direction: str,
                        loop_mode: str, waypoint_total: int,
                        route_total_m: float, loop_total: int,
                        now_iso: str) -> None:
        """Open the `active` row for a dispatch (15 S9.3 ready -> running).

        Not an upsert: idx_patrol_active makes a second active row for the
        same task an INTEGRITY ERROR rather than a silent overwrite, and that
        is the point -- two active rows means two readings of "where is this
        task", and U07a would resume from whichever one the query returned.
        """
        await self._conn.execute(
            "INSERT INTO patrol_progress ("
            " task_id, route_name, route_geo_id, route_rev, direction,"
            " loop_mode, waypoint_total, route_total_m, loop_total,"
            " created_at, updated_at)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (task_id, route_name, route_geo_id, route_rev, direction,
             loop_mode, waypoint_total, route_total_m, loop_total,
             now_iso, now_iso))

    async def update_progress(self, task_id: str, *, waypoint_index: int,
                              waypoint_total: int, seg_done_m: float,
                              dist_done_m: float, odom_dist_m: float,
                              loop_index: int, loop_total: int,
                              skipped_m: float, dir_sign: int,
                              now_iso: str) -> int:
        """PP-1 flush of the active row. Returns rows affected (0 = no active
        row, which the caller must NOT read as success).

        `AND status='active'` is load-bearing and not defensive: without it a
        task that runs the same route twice has its FIRST, already-closed row
        overwritten by the second run's position. Nothing errors; the history
        is just wrong.
        """
        cur = await self._conn.execute(
            "UPDATE patrol_progress SET"
            " waypoint_index=?, waypoint_total=?, seg_done_m=?, dist_done_m=?,"
            " odom_dist_m=?, loop_index=?, loop_total=?, skipped_m=?,"
            " dir_sign=?, updated_at=?"
            " WHERE task_id=? AND status='active'",
            (waypoint_index, waypoint_total, seg_done_m, dist_done_m,
             odom_dist_m, loop_index, loop_total, skipped_m, dir_sign,
             now_iso, task_id))
        return cur.rowcount

    async def close_run(self, task_id: str, status: str,
                        now_iso: str) -> int:
        """Move the active row to a terminal status. `status` is checked here
        as well as by the DDL CHECK so the caller gets a Python error naming
        the closed set, not an opaque sqlite IntegrityError."""
        if status not in ("completed", "aborted"):
            raise ValueError(
                "patrol_progress status must be completed|aborted, got %r"
                % (status,))
        cur = await self._conn.execute(
            "UPDATE patrol_progress SET status=?, updated_at=?"
            " WHERE task_id=? AND status='active'",
            (status, now_iso, task_id))
        return cur.rowcount

    async def clamp_laps(self, task_id: str, now_iso: str) -> int:
        """15 S7.6 GC-3: the route was deleted under a running patrol -- finish
        the lap in progress and drop the rest, rather than stopping in the
        roadway. loop_total := loop_index + 1, which also terminates a
        loop_total = 0 standing patrol (the only clean way to end one that is
        not a cancel)."""
        cur = await self._conn.execute(
            "UPDATE patrol_progress SET loop_total = loop_index + 1,"
            " updated_at=? WHERE task_id=? AND status='active'",
            (now_iso, task_id))
        return cur.rowcount

    async def fetch_active(self, task_id: str):
        """The U07a breakpoint plus the S7.3A remap inputs, as a raw tuple in
        the declared order (15 S9 forbids the DAO reshaping rows)."""
        cur = await self._conn.execute(
            "SELECT route_geo_id, route_rev, direction, loop_mode,"
            " waypoint_index, waypoint_total, seg_done_m, dist_done_m,"
            " odom_dist_m, route_total_m, loop_index, loop_total, skipped_m,"
            " dir_sign FROM patrol_progress"
            " WHERE task_id=? AND status='active'", (task_id,))
        return await cur.fetchone()


class MemoryDAO:
    def __init__(self, conn) -> None:
        self._conn = conn

    async def put(self, key: str, value: bytes, updated_ms: int) -> None:
        await self._conn.execute(
            "INSERT INTO memory (key, value, updated_ms) VALUES (?, ?, ?) "
            "ON CONFLICT(key) DO UPDATE SET value=excluded.value, "
            " updated_ms=excluded.updated_ms", (key, value, updated_ms))

    async def get(self, key: str):
        cur = await self._conn.execute(
            "SELECT value FROM memory WHERE key=?", (key,))
        row = await cur.fetchone()
        return None if row is None else row[0]


class SnapshotDAO:
    def __init__(self, conn) -> None:
        self._conn = conn

    async def replace(self, task_id: str, waypoints) -> None:
        """Replace snapshot atomically: clear then append.
        Called inside an outer BEGIN IMMEDIATE by the caller."""
        await self._conn.execute(
            "DELETE FROM task_route_snapshot WHERE task_id=?", (task_id,))
        for seq, (x, y, heading) in enumerate(waypoints):
            await self._conn.execute(
                "INSERT INTO task_route_snapshot (task_id, seq, x_m, y_m, "
                " heading_rad) VALUES (?, ?, ?, ?, ?)",
                (task_id, seq, x, y, heading))

    async def fetch(self, task_id: str):
        cur = await self._conn.execute(
            "SELECT seq, x_m, y_m, heading_rad FROM task_route_snapshot "
            "WHERE task_id=? ORDER BY seq ASC", (task_id,))
        return await cur.fetchall()


class PendingPushDAO:
    def __init__(self, conn) -> None:
        self._conn = conn

    async def enqueue(self, kind: str, object_id: str, rev: int,
                       enqueued_ms: int) -> None:
        await self._conn.execute(
            "INSERT INTO geo_pending_push (kind, object_id, rev, enqueued_ms) "
            "VALUES (?, ?, ?, ?)",
            (kind, object_id, rev, enqueued_ms))

    async def drain(self, limit: int):
        cur = await self._conn.execute(
            "SELECT push_id, kind, object_id, rev FROM geo_pending_push "
            "ORDER BY push_id ASC LIMIT ?", (limit,))
        return await cur.fetchall()

    async def ack(self, push_id: int) -> None:
        await self._conn.execute(
            "DELETE FROM geo_pending_push WHERE push_id=?", (push_id,))


class GeoObjectDAO:
    """Shared shape helper for waypoints / routes / docks / fences.
    Each of those tables has (rev, content_hash, tombstone) triples;
    this class centralises rev-bump and tombstone helpers so the
    invariant is enforced in one place, not four."""

    def __init__(self, conn, table: str) -> None:
        allowed = {"waypoints", "routes", "docks", "fences"}
        if table not in allowed:
            raise ValueError(f"unknown table {table!r}")
        self._conn = conn
        self._table = table

    async def bump_rev(self, object_id: str, new_hash: str,
                        updated_ms: int) -> int:
        """Increment rev by exactly 1 and record the new hash.
        Returns the new rev. Idempotency: if content_hash matches,
        no-op and returns the current rev unchanged."""
        # v1.5 PLAN A: waypoints / routes / docks are keyed by the UNIQUE geo_id
        # now (id is an internal AUTOINCREMENT). Only fences keep a string PK.
        pk = "fence_id" if self._table == "fences" else "geo_id"
        cur = await self._conn.execute(
            f"SELECT rev, content_hash FROM {self._table} "
            f"WHERE {pk}=?", (object_id,))
        row = await cur.fetchone()
        if row is None:
            raise KeyError(object_id)
        rev, current_hash = row
        if current_hash == new_hash:
            return rev
        await self._conn.execute(
            f"UPDATE {self._table} SET rev=rev+1, content_hash=?, "
            f"updated_ms=? WHERE {pk}=?",
            (new_hash, updated_ms, object_id))
        return rev + 1


class FencesDAO:
    def __init__(self, conn) -> None:
        self._conn = conn

    async def insert(self, fence_id: str, role: str, geom_json: str,
                      content_hash: str, updated_ms: int, kind: str = "polygon",
                      name=None, soft_margin_m=None) -> None:
        # v1.5: role (11 S9A.2) replaces the old kind='zone'/zone_label overload;
        # warning never hard-enforces (S9A.2). The S9A.1A count invariants are the
        # fence.db triggers, not this DAO.
        hard_enforce = 0 if role == "warning" else 1
        await self._conn.execute(
            "INSERT INTO fences (fence_id, name, role, kind, geom_json, "
            " hard_enforce, soft_margin_m, content_hash, updated_ms) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (fence_id, name, role, kind, geom_json, hard_enforce, soft_margin_m,
             content_hash, updated_ms))

    async def list_active(self):
        # Ordered for a DETERMINISTIC FenceSet crc32 (11 S9A.2 concatenates
        # polygons in array order; a stable order keeps the crc32 reproducible).
        # state='active' is the operative filter, not tombstone alone: a fence
        # the operator recorded but has not activated (F15) is stored as draft
        # and must NOT be enforced -- broadcasting it would put a keep-out zone
        # into effect that nobody switched on. tombstone=0 stays as the second
        # half (a deleted row is also state='deleted', both are checked so the
        # filter still holds if one of the two is ever written alone).
        cur = await self._conn.execute(
            "SELECT fence_id, name, role, kind, geom_json, hard_enforce, rev "
            "FROM fences WHERE tombstone=0 AND state='active' "
            "ORDER BY fence_id")
        return await cur.fetchall()


class DocksDAO:
    def __init__(self, conn) -> None:
        self._conn = conn

    async def insert(self, geo_id: str, name: str, rtk_lat: float,
                      rtk_lon: float, dock_heading_rad: float,
                      handover_lat: float, handover_lon: float,
                      handover_heading_rad: float, content_hash: str,
                      updated_ms: int, num=None, rtk_alt=None,
                      handover_tol_m: float = 0.3, handover_tol_rad: float = 0.09,
                      on_route_json: str = "[]", enabled: int = 1,
                      occupied_by=None, description=None) -> None:
        # v1.5: full 15 S9.3 dock (WGS84 body + handover point), keyed by geo_id.
        await self._conn.execute(
            "INSERT INTO docks (geo_id, name, num, rtk_lat, rtk_lon, rtk_alt, "
            " dock_heading_rad, handover_lat, handover_lon, handover_heading_rad, "
            " handover_tol_m, handover_tol_rad, on_route_json, enabled, "
            " occupied_by, description, content_hash, updated_ms) "
            "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
            (geo_id, name, num, rtk_lat, rtk_lon, rtk_alt, dock_heading_rad,
             handover_lat, handover_lon, handover_heading_rad, handover_tol_m,
             handover_tol_rad, on_route_json, enabled, occupied_by, description,
             content_hash, updated_ms))

    async def list_active(self):
        cur = await self._conn.execute(
            "SELECT geo_id, name, num, rtk_lat, rtk_lon, dock_heading_rad, "
            " handover_lat, handover_lon, handover_heading_rad, enabled, rev "
            "FROM docks WHERE tombstone=0 AND state='active' ORDER BY geo_id")
        return await cur.fetchall()
