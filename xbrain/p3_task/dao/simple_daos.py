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
    def __init__(self, conn) -> None:
        self._conn = conn

    async def upsert(self, task_id: str, waypoint_ix: int,
                      progress: float, updated_ms: int) -> None:
        await self._conn.execute(
            "INSERT INTO patrol_progress (task_id, waypoint_ix, progress, "
            " updated_ms) VALUES (?, ?, ?, ?) "
            "ON CONFLICT(task_id) DO UPDATE SET waypoint_ix=excluded.waypoint_ix, "
            " progress=excluded.progress, updated_ms=excluded.updated_ms",
            (task_id, waypoint_ix, progress, updated_ms))

    async def fetch(self, task_id: str):
        cur = await self._conn.execute(
            "SELECT waypoint_ix, progress, updated_ms FROM patrol_progress "
            "WHERE task_id=?", (task_id,))
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
        pk = "waypoint_id" if self._table == "waypoints" else (
             "route_id"    if self._table == "routes"    else (
             "dock_id"     if self._table == "docks"     else "fence_id"))
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

    async def insert(self, fence_id: str, kind: str, geom_json: str,
                      zone_label: str, content_hash: str,
                      updated_ms: int) -> None:
        await self._conn.execute(
            "INSERT INTO fences (fence_id, kind, geom_json, zone_label, "
            " content_hash, updated_ms) VALUES (?, ?, ?, ?, ?, ?)",
            (fence_id, kind, geom_json, zone_label, content_hash,
             updated_ms))

    async def list_active(self):
        cur = await self._conn.execute(
            "SELECT fence_id, kind, geom_json, zone_label, rev "
            "FROM fences WHERE tombstone=0")
        return await cur.fetchall()


class DocksDAO:
    def __init__(self, conn) -> None:
        self._conn = conn

    async def insert(self, dock_id: str, x_m: float, y_m: float,
                      heading_rad: float, content_hash: str,
                      updated_ms: int) -> None:
        await self._conn.execute(
            "INSERT INTO docks (dock_id, x_m, y_m, heading_rad, "
            " content_hash, updated_ms) VALUES (?, ?, ?, ?, ?, ?)",
            (dock_id, x_m, y_m, heading_rad, content_hash, updated_ms))

    async def list_active(self):
        cur = await self._conn.execute(
            "SELECT dock_id, x_m, y_m, heading_rad FROM docks "
            "WHERE tombstone=0")
        return await cur.fetchall()
