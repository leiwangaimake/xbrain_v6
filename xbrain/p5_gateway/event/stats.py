"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: stats.py
Brief: state/event/stats (11 S4.7) -- per-severity event counts of record.db for today / the last 7 days, and the latest alarm / fault
Description:
What problem this solves
------------------------
Field test 2026-10-29 / 10-30: "报警了几次", "最近都发生了什么", "上次警报"
answered "暂时查不到" -- the events live in record.db, which only p5 owns
(17 S3), and nothing published a summary of it. p4 answers voice queries from
state topics only (16 S8.2), so it had no source. User ruling 2026-10-30
(99 U103): p5 publishes an event-statistics state, p4 subscribes.

What is counted
---------------
  * rows of the events table (a merged recurrence, EVT-14, is ONE row: the
    count is what the HMI / cloud event list shows, not dedup_count);
  * per sev (11 S6.1 closed set info / warn / alarm / fault);
  * "today" = since local midnight of the site timezone; "last_7d" = today
    plus the six local days before it;
  * last_alarm / last_fault = the newest row of that sev in the whole table:
    its cat and when p5 recorded it.

The time used is detected_at -- p5's own UTC wall-clock stamp at insert
(main_wiring._normalise_event), the column idx_evt_time indexes. It is
display / statistics only, never an ordering key (15 S9.1 DBF-3): a day
boundary is a calendar question a person asks, not an age or a timeout.
The producer's ts is NOT used: an unsynced producer clock (ts_sync = 0) would
move events across days.

Rules
-----
  * the database is opened READ-ONLY (mode=ro): p5's EventSubsystem stays the
    only writer; WAL lets this reader run beside it;
  * any sqlite error -> read() returns None and the state is NOT published:
    p4 then answers "暂时读不到" (stale), never a zero that looks like "no
    alarms" (CLAUDE.md 3.2).

Why stdlib sqlite3 and not aiosqlite (CLAUDE.md 4.1 / CFG-DC-3)
-----------------------------------------------------------------
The ban exists because a synchronous call blocks an asyncio event loop. This
reader is called from p5's SYNCHRONOUS heartbeat loop (main_wiring, a plain
`while` with time.monotonic), after that beat's estop ping and state/link
have gone out, so nothing async waits on it. In WAL mode a reader does not
wait for the writer; BUSY_TIMEOUT_S caps the rare checkpoint wait.

Boundary: one read-only query set per call, returns the data dict. The p5
heartbeat loop calls it once a second and publishes the result.
"""
from __future__ import annotations  # postponed annotations

import sqlite3  # BUSINESS-IMPORT-OK(read-only-stats): runs in p5's synchronous heartbeat loop, not an asyncio loop; mode=ro, WAL readers never wait on the writer
import time  # the system wall clock
from datetime import datetime, timedelta, timezone  # local day boundaries
from typing import Dict, Optional  # signatures
from zoneinfo import ZoneInfo  # site timezone

SEVS = ("info", "warn", "alarm", "fault")  # 11 S6.1 sev closed set
STAMP_FMT = "%Y-%m-%d %H:%M:%S"  # detected_at format (UTC), see _normalise_event
DAYS_WINDOW = 7  # last_7d = today + 6 previous local days
BUSY_TIMEOUT_S = 0.2  # never hold the heartbeat loop on a writer lock


def wall_now() -> float:
    """The system wall clock, in one place so tests can replace it."""
    # WALL-CLOCK-OK(record): a calendar-day boundary for statistics; never an age or timeout.
    return time.time()  # what `date` prints


def day_start_utc(now_wall: float, tz_name: Optional[str], days_back: int = 0) -> str:
    """detected_at string (UTC) of local midnight `days_back` days before today."""
    tz = ZoneInfo(tz_name) if tz_name else None  # None = system local zone
    local = datetime.fromtimestamp(now_wall, tz=timezone.utc).astimezone(tz)  # now, local
    midnight = local.replace(hour=0, minute=0, second=0, microsecond=0) - timedelta(days=days_back)  # local 00:00
    return midnight.astimezone(timezone.utc).strftime(STAMP_FMT)  # as detected_at spells it


def _epoch(stamp: str) -> float:
    """detected_at (UTC text) -> epoch seconds."""
    return datetime.strptime(stamp, STAMP_FMT).replace(tzinfo=timezone.utc).timestamp()  # UTC


class EventStats:
    """Reads the 11 S4.7 statistics from record.db (read-only)."""

    def __init__(self, db_path: str, tz_name: Optional[str]) -> None:
        self._path = db_path  # record.db
        self._tz = tz_name  # site timezone (common.timezone)
        self._conn: Optional[sqlite3.Connection] = None  # opened on first read

    def _connect(self) -> sqlite3.Connection:
        """The read-only connection (opened lazily, reopened after an error)."""
        if self._conn is None:  # first read, or after a failure
            self._conn = sqlite3.connect("file:%s?mode=ro" % self._path, uri=True,  # never a writer
                                         timeout=BUSY_TIMEOUT_S, check_same_thread=False)  # short wait
        return self._conn  # cached

    def _counts(self, c: sqlite3.Connection, since: str) -> Dict[str, int]:
        """Rows per sev with detected_at >= since (every sev present, 0 when none)."""
        out = {s: 0 for s in SEVS}  # every sev reported
        for sev, n in c.execute("SELECT sev, COUNT(*) FROM events WHERE detected_at >= ? GROUP BY sev",
                                (since,)):  # indexed range (idx_evt_time)
            if sev in out:  # the CHECK constraint keeps it closed
                out[sev] = int(n)  # count
        return out  # per sev

    def _last(self, c: sqlite3.Connection, sev: str) -> Optional[dict]:
        """The newest row of one sev: its cat and when p5 recorded it, or None."""
        row = c.execute("SELECT cat, detected_at FROM events WHERE sev = ? ORDER BY id DESC LIMIT 1",
                        (sev,)).fetchone()  # id grows with every insert
        if row is None:  # never happened
            return None  # spoken as "no record"
        return {"cat": row[0], "at": _epoch(row[1])}  # cat + epoch seconds

    def read(self, now_wall: Optional[float] = None) -> Optional[dict]:
        """The state/event/stats data object, or None when record.db is unreadable."""
        now = wall_now() if now_wall is None else now_wall  # the calendar "now"
        try:  # any sqlite / parse failure -> no publish
            c = self._connect()  # read-only
            local_day = datetime.fromtimestamp(now, tz=timezone.utc).astimezone(  # today's date
                ZoneInfo(self._tz) if self._tz else None).strftime("%Y-%m-%d")  # local
            total = int(c.execute("SELECT COUNT(*) FROM events").fetchone()[0])  # all rows
            return {"tz": self._tz, "day": local_day, "total": total,  # header
                    "today": self._counts(c, day_start_utc(now, self._tz)),  # since 00:00
                    "last_7d": self._counts(c, day_start_utc(now, self._tz, DAYS_WINDOW - 1)),  # 7 days
                    "last_alarm": self._last(c, "alarm"),  # newest alarm
                    "last_fault": self._last(c, "fault")}  # newest fault
        except (sqlite3.Error, ValueError, TypeError):  # locked, missing, malformed
            self.close()  # reopen next time
            return None  # not a reading: do not publish

    def close(self) -> None:
        """Drop the connection (the next read reopens it)."""
        if self._conn is not None:  # something to close
            try:  # closing a broken handle may raise
                self._conn.close()  # release the file
            except sqlite3.Error:  # already broken
                pass  # nothing else to do
            self._conn = None  # reopen on next read


__all__ = ["EventStats", "day_start_utc", "wall_now", "SEVS", "DAYS_WINDOW"]
