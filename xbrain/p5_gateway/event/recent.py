"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: recent.py
Brief: the newest record.db events, read-only, to seed the HMI event list at p5 start-up

Description:
What problem this solves
------------------------
The HMI event list lived only in p5's memory: every p5 restart (and p5 is
restarted for every config change) emptied it, and an operator opening the page
afterwards saw "no events" with a low-battery alarm running. User ruling
2026-11-12 (recommendation 7): the list reads the newest entries from record.db
so it survives a restart. record.db is p5's own (17 S3); EventSubsystem is its
only writer.

What it reads
-------------
The newest ROW_LIMIT rows by id (id grows with every insert), returned oldest
first so event_ring.seed() replays them in arrival order and folds repeats the
same way the live path does. ROW_LIMIT is larger than the ring because the
newest rows are often one alarm repeated every 2 s (20 k of 31.5 k rows on
2026-11-12); merged, 2000 rows still fill the 200-entry ring with distinct
entries.

Each row -> {eid, sev, cat, title, detail (parsed), src, ts, pos}. pos comes
from location_lat / location_lon when both are set (none are today) -- never
0,0.

Rules
-----
  * READ-ONLY (mode=ro); WAL lets it run beside the writer;
  * any sqlite / JSON failure -> None: the ring starts empty, exactly as before
    this module -- the list is a convenience, p5 must start regardless;
  * a row whose detail is not valid JSON keeps detail None, the row survives.

Why stdlib sqlite3 and not aiosqlite: called once, synchronously, while p5
wires itself up, before any event loop or subscriber runs (same reasoning as
event/stats.py, CLAUDE.md 4.1).
"""
from __future__ import annotations  # postponed annotations

import json  # detail column is JSON text
import sqlite3  # BUSINESS-IMPORT-OK(read-only-seed): one synchronous read at p5 start-up, before any event loop; mode=ro, WAL readers never wait on the writer
from typing import Any, Dict, List, Optional  # signatures

ROW_LIMIT = 2000  # newest rows read; folded down to the 200-entry ring
BUSY_TIMEOUT_S = 0.5  # a checkpoint wait at start-up, never longer


def _row(r: sqlite3.Row) -> Dict[str, Any]:
    """One events row -> the body shape event_ring.ring_item reads."""
    try:  # the writer stores JSON; tolerate anything else
        detail = json.loads(r["detail"]) if r["detail"] else None  # parsed
    except (TypeError, ValueError):  # not JSON
        detail = None  # the row survives without it
    lat, lon = r["location_lat"], r["location_lon"]  # stamped place, if any
    pos = {"lat": lat, "lon": lon} if lat is not None and lon is not None else None  # never 0,0
    return {"eid": r["eid"], "sev": r["sev"], "cat": r["cat"], "title": r["title"],  # identity
            "detail": detail, "src": r["src"], "ts": r["ts"], "pos": pos}  # rest


def read_recent(db_path: str, limit: int = ROW_LIMIT) -> Optional[List[Dict[str, Any]]]:
    """The newest `limit` events, oldest first, or None when unreadable."""
    try:  # any failure -> None, p5 starts with an empty list
        conn = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True,  # never a writer
                               timeout=BUSY_TIMEOUT_S)  # short wait
        try:  # always close
            conn.row_factory = sqlite3.Row  # columns by name
            rows = conn.execute(  # newest by id, then flipped below
                "SELECT eid, sev, cat, title, detail, src, ts, location_lat, location_lon "
                "FROM events ORDER BY id DESC LIMIT ?", (int(limit),)).fetchall()  # bounded
        finally:  # read done
            conn.close()  # release the file
    except sqlite3.Error:  # missing file, locked, schema mismatch
        return None  # not a reading
    return [_row(r) for r in reversed(rows)]  # oldest first
