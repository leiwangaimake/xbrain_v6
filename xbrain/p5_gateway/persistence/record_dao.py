"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: record_dao.py
Brief: record.db events DAO -- ch_seq allocation + dedup merge + need_ack + JSONL degrade

Description:
The only writer of the events table (15 S9.10). Every rule that must hold at
write time lives here, not in the caller, so no producer can create a gap or
skip durability:

  * ch_seq allocation + row insert happen in ONE BEGIN IMMEDIATE tx (SEQ-2): the
    next_ch_seq is read, used, and bumped inside the same exclusive lock, so a
    channel's ch_seq is dense with no gaps for the cloud to misread as loss.
  * alarm/fault rows route to the FULL writer (FS-d, 11 PWR-3); info/warn to the
    NORMAL writer. Routing is by SEV, not channel -- a fence.recovered (sev=info,
    channel=alarm) is durable-enough on NORMAL, but its ack is still forced by
    need_ack (the S3.3 union).
  * on a DB write failure the event is appended to a JSONL sidecar (15 S9.1 S-6),
    so nothing is lost; it replays into the DB on recovery. CRITICAL (S3.4): a
    fault event produced BY a write failure must NOT be fed back in here, or
    "write fails -> fault -> write -> fails" self-loops (V5 bug 9). This DAO only
    persists; the pipeline is responsible for not looping the degrade fault.

Dedup (S3.2): when dedup_key is set and a still-open row with that key exists
within its window, we merge (dedup_count += 1, last_ts = new ts) and consume NO
ch_seq and push nothing again -- flood suppression that keeps the first
occurrence's time, which is the load-bearing fact. Window comparison uses the
event ts; ts is display-only for ORDERING (DBF-3), but dedup is best-effort flood
suppression, not an ordering decision, so a slightly-off producer clock at worst
merges a little wrong -- never mis-orders or drops a distinct event.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional

from .base import PersistenceMisuse, RecordConn
from .schema_record import CHANNELS, SEVERITIES, advance_confirmed, need_ack


# The events columns written on a fresh insert, in a fixed order shared by the
# INSERT statement and the value tuple below. Kept as one list so the two can
# never drift (a mismatch would bind the wrong value to the wrong column).
_INSERT_COLUMNS = (
    "eid", "rid", "channel", "ch_seq", "sev", "cat", "title", "detail", "src",
    "trace_id", "task_id", "episode_id",
    "ts", "ts_sync", "detected_at", "created_at",
    "location_lat", "location_lon", "location_alt",
    "pose_x", "pose_y", "heading_deg", "heading_src", "fix_type",
    "dedup_key", "dedup_window_s", "dedup_count", "last_ts",
    "need_ack", "delivered", "deliver_tries",
    "media_json",
)


@dataclass(frozen=True)
class InsertResult:
    """What insert_event did. status is the single source of truth for the
    pipeline's next step: 'inserted' -> push cloud/HMI; 'merged' -> push nothing
    (S3.2); 'degraded' -> in JSONL not the DB, do NOT loop a fault back (S3.4)."""

    status: str            # 'inserted' | 'merged' | 'degraded'
    channel: Optional[str] = None
    ch_seq: Optional[int] = None


class RecordDao:
    """DAO over the three record.db connections (15 S9.1 S-2). The writers are
    aiosqlite connections opened by base.open_record_writer; the reader by
    open_record_reader. The DAO never opens its own connection."""

    def __init__(self, writer_normal: RecordConn, writer_full: RecordConn,
                 reader: RecordConn, jsonl_path: str) -> None:
        for w, want in ((writer_normal, "writer_normal"),
                        (writer_full, "writer_full"), (reader, "reader")):
            if w.role != want:
                raise PersistenceMisuse(
                    f"RecordDao: expected {want}, got role {w.role!r}")
        self._wn = writer_normal
        self._wf = writer_full
        self._rd = reader
        # JSONL degrade sidecar; appended to only when a DB write fails (S-6).
        self._jsonl_path = jsonl_path

    # -- startup --------------------------------------------------------------

    async def init_cursors_from_table(self) -> None:
        """SEQ-3: on startup, next_ch_seq = MAX(ch_seq)+1 per channel, read from
        the table, so ch_seq never resets across a restart. Runs on the NORMAL
        writer (a plain cursor read+update, no alarm durability needed)."""
        c = self._wn.conn
        for channel in sorted(CHANNELS):
            cur = await c.execute(
                "SELECT MAX(ch_seq) FROM events WHERE channel = ?", (channel,))
            row = await cur.fetchone()
            max_seq = row[0] if row and row[0] is not None else 0
            await c.execute(
                "UPDATE event_cursor SET next_ch_seq = ? WHERE channel = ?",
                (max_seq + 1, channel))
        await c.commit()

    # -- insert ---------------------------------------------------------------

    async def insert_event(self, ev: dict) -> InsertResult:
        """Persist one event. Validates the closed sets, computes need_ack, tries
        a dedup merge, else allocates ch_seq and inserts -- all under one
        BEGIN IMMEDIATE. On DB failure, degrades to JSONL and returns 'degraded'
        (never raises a DB error up into the pipeline, which must keep running)."""
        channel = ev.get("channel")
        sev = ev.get("sev")
        if channel not in CHANNELS:
            raise ValueError(f"channel not in {sorted(CHANNELS)}: {channel!r}")
        if sev not in SEVERITIES:
            raise ValueError(f"sev not in {sorted(SEVERITIES)}: {sev!r}")

        # FS-d: alarm/fault durability is non-negotiable -> FULL writer.
        writer = self._wf if sev in ("alarm", "fault") else self._wn
        c = writer.conn
        try:
            await c.execute("BEGIN IMMEDIATE")
            if ev.get("dedup_key"):
                merged = await self._try_merge(c, ev)
                if merged:
                    await c.commit()
                    return InsertResult(status="merged", channel=channel)
            ch_seq = await self._alloc_ch_seq(c, channel)
            await self._insert_row(c, ev, channel, sev, ch_seq)
            await c.commit()
            return InsertResult(status="inserted", channel=channel, ch_seq=ch_seq)
        except Exception:  # noqa: BLE001 -- any DB error degrades, never crashes p5
            try:
                await c.rollback()
            except Exception:  # noqa: BLE001
                pass
            self._degrade_to_jsonl(ev)
            return InsertResult(status="degraded", channel=channel)

    async def _try_merge(self, c, ev: dict) -> bool:
        """S3.2: if a still-open row with this dedup_key exists within its window,
        bump dedup_count + last_ts and return True (no new row, no ch_seq). 'Open'
        = delivered != -1 (not given up) and (ev.ts - last_ts) <= window."""
        key = ev["dedup_key"]
        window = ev.get("dedup_window_s")
        ts = ev["ts"]
        cur = await c.execute(
            "SELECT id, last_ts, dedup_window_s FROM events "
            "WHERE dedup_key = ? AND delivered != -1 "
            "ORDER BY last_ts DESC LIMIT 1", (key,))
        row = await cur.fetchone()
        if row is None:
            return False
        row_id, last_ts, row_window = row
        win = window if window is not None else row_window
        # No window -> the producer opted out of time-bounding; still merge onto
        # the open row (S3.2 "dedup_key 缺省 不去重" is handled by the caller not
        # setting a key -- here a key IS set, so merge).
        if win is not None and last_ts is not None and (ts - last_ts) > win:
            return False
        await c.execute(
            "UPDATE events SET dedup_count = dedup_count + 1, last_ts = ? "
            "WHERE id = ?", (ts, row_id))
        return True

    async def _alloc_ch_seq(self, c, channel: str) -> int:
        """SEQ-2: read next_ch_seq, hand it out, bump it -- all inside the caller's
        BEGIN IMMEDIATE, so the number is dense with no gap even under concurrent
        writers (the two writers serialise on the exclusive lock)."""
        cur = await c.execute(
            "SELECT next_ch_seq FROM event_cursor WHERE channel = ?", (channel,))
        row = await cur.fetchone()
        if row is None:
            raise PersistenceMisuse(f"event_cursor missing channel {channel!r}")
        ch_seq = row[0]
        await c.execute(
            "UPDATE event_cursor SET next_ch_seq = ? WHERE channel = ?",
            (ch_seq + 1, channel))
        return ch_seq

    async def _insert_row(self, c, ev: dict, channel: str, sev: str,
                          ch_seq: int) -> None:
        """Insert the fresh row. need_ack is computed here (S3.3 union) and frozen
        into the column; detail/media are JSON-serialised. Values are built in
        _INSERT_COLUMNS order so a column and its value cannot drift apart."""
        na = 1 if need_ack(channel, sev) else 0
        values = {
            "eid": ev["eid"], "rid": ev["rid"], "channel": channel,
            "ch_seq": ch_seq, "sev": sev, "cat": ev["cat"],
            "title": ev["title"], "detail": json.dumps(ev.get("detail", {})),
            "src": ev["src"],
            "trace_id": ev.get("trace_id"), "task_id": ev.get("task_id"),
            "episode_id": ev.get("episode_id"),
            "ts": ev["ts"], "ts_sync": 1 if ev.get("ts_sync") else 0,
            "detected_at": ev["detected_at"], "created_at": ev["created_at"],
            "location_lat": ev.get("location_lat"),
            "location_lon": ev.get("location_lon"),
            "location_alt": ev.get("location_alt"),
            "pose_x": ev.get("pose_x"), "pose_y": ev.get("pose_y"),
            "heading_deg": ev.get("heading_deg"),
            "heading_src": ev.get("heading_src"), "fix_type": ev.get("fix_type"),
            "dedup_key": ev.get("dedup_key"),
            "dedup_window_s": ev.get("dedup_window_s"),
            "dedup_count": 1, "last_ts": ev["ts"],
            "need_ack": na, "delivered": 0, "deliver_tries": 0,
            "media_json": json.dumps(ev["media"]) if ev.get("media") else None,
        }
        cols = ", ".join(_INSERT_COLUMNS)
        marks = ", ".join("?" for _ in _INSERT_COLUMNS)
        await c.execute(
            f"INSERT INTO events ({cols}) VALUES ({marks})",
            tuple(values[k] for k in _INSERT_COLUMNS))

    def _degrade_to_jsonl(self, ev: dict) -> None:
        """S-6 degrade: append the event as one JSON line so a DB write failure
        never loses it. Synchronous append is acceptable -- this path is the rare
        failure case, not the hot path, and losing the event is worse than a brief
        block. Best-effort: if even the JSONL write fails we swallow it, because
        raising here would take down the p5 event loop (P-2 forbids that)."""
        import os

        try:
            parent = os.path.dirname(self._jsonl_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(self._jsonl_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(ev, ensure_ascii=False) + "\n")
        except Exception:  # noqa: BLE001
            pass

    # -- backfill reads / delivery marks --------------------------------------

    async def backlog(self, channel: str, limit: int) -> list:
        """Undelivered rows for one channel, oldest ch_seq first -- the backfill
        scan (17 S3.5). Uses the partial index (delivered = 0), so a delivered row
        is already gone from the scan. Reader connection (query_only)."""
        if channel not in CHANNELS:
            raise ValueError(f"channel not in {sorted(CHANNELS)}: {channel!r}")
        cur = await self._rd.conn.execute(
            "SELECT ch_seq, eid, sev, cat, title, detail, need_ack "
            "FROM events WHERE channel = ? AND delivered = 0 "
            "ORDER BY ch_seq ASC LIMIT ?", (channel, limit))
        rows = await cur.fetchall()
        return [
            {"ch_seq": r[0], "eid": r[1], "sev": r[2], "cat": r[3],
             "title": r[4], "detail": json.loads(r[5]), "need_ack": bool(r[6])}
            for r in rows
        ]

    async def mark_delivered(self, eids: list, batch: Optional[str],
                             delivered_at: str) -> int:
        """Set delivered = 1 for the given eids (they leave the backfill index).
        Runs on the NORMAL writer -- a delivery mark is not itself an alarm event.
        Returns the number of rows updated."""
        if not eids:
            return 0
        c = self._wn.conn
        marks = ", ".join("?" for _ in eids)
        await c.execute("BEGIN IMMEDIATE")
        try:
            cur = await c.execute(
                f"UPDATE events SET delivered = 1, delivered_at = ?, "
                f"deliver_tries = deliver_tries + 1, backfill_batch = ? "
                f"WHERE eid IN ({marks}) AND delivered = 0",
                (delivered_at, batch, *eids))
            await c.commit()
            return cur.rowcount
        except Exception:  # noqa: BLE001
            await c.rollback()
            raise

    async def read_cursor(self, channel: str) -> dict:
        """Current cursor state for a channel (reader connection)."""
        if channel not in CHANNELS:
            raise ValueError(f"channel not in {sorted(CHANNELS)}: {channel!r}")
        cur = await self._rd.conn.execute(
            "SELECT next_ch_seq, confirmed_upto FROM event_cursor "
            "WHERE channel = ?", (channel,))
        row = await cur.fetchone()
        return {"next_ch_seq": row[0], "confirmed_upto": row[1]}

    async def advance_confirmed_upto(self, channel: str, acked_upto: int,
                                     backfill_at: str) -> int:
        """Move confirmed_upto to the cloud-acked high-water mark (SEQ-3 monotonic;
        a rewind raises). NORMAL writer."""
        if channel not in CHANNELS:
            raise ValueError(f"channel not in {sorted(CHANNELS)}: {channel!r}")
        c = self._wn.conn
        cur = await c.execute(
            "SELECT confirmed_upto FROM event_cursor WHERE channel = ?",
            (channel,))
        row = await cur.fetchone()
        new_upto = advance_confirmed(row[0], acked_upto)  # raises on rewind
        await c.execute("BEGIN IMMEDIATE")
        try:
            await c.execute(
                "UPDATE event_cursor SET confirmed_upto = ?, last_backfill_at = ? "
                "WHERE channel = ?", (new_upto, backfill_at, channel))
            await c.commit()
            return new_upto
        except Exception:  # noqa: BLE001
            await c.rollback()
            raise
