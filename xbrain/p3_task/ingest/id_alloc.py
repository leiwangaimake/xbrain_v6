"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: id_alloc.py
Brief: BIZ-P3-40 task_id (t-YYYYMMDD-NNN) + submit_seq allocation

Description:
15 S9.5 fixes the task_id FORM as 't-YYYYMMDD-NNN' (NNN a per-day sequence,
3-digit zero-padded) -- NOT a UUID, because operators name tasks by voice
('cancel 001') and read them on the HMI, and every 11 example uses this form.
Before this module nothing allocated an id or a submit_seq: the caller passed
them in and tests hard-coded 't-voice-1', so two real tasks on the same day
would collide or need an external counter.

Two allocators, both querying the tasks table so they survive a restart (an
in-memory counter would reset to 001 after a reboot and collide with the rows
already on disk):

  next_task_id(conn, date_str) -> 't-<date>-<NNN>'
      NNN = 1 + the max NNN already used for date_str. date_str is the
      YYYYMMDD the CALLER derived from the wall clock -- the id's date is a
      human-readable DISPLAY value (like the wall-clock audit columns), not a
      timing decision, so it is injected, never read from a clock here
      (keeps this pure/testable and off the CLK-C1 wall-clock ban).

  next_submit_seq(conn) -> int
      1 + max(submit_seq). The FIFO tiebreaker the scheduler orders on
      (priority DESC, submit_seq ASC); must be strictly increasing for the
      lifetime of the DB, hence max+1 from the table, not a process counter.

Concurrency: P3 has ONE db thread (15 S2.1), so these run serialised -- a
read-max-then-insert pair on that single writer cannot interleave with
another allocation. They are NOT safe to call from two writers.
"""
from __future__ import annotations

import re


# A well-formed daily sequence id: 't-' + 8 digits + '-' + digits. The capture
# is the NNN part, read back to find the current max for a day.
_TASK_ID_RE = re.compile(r"^t-(\d{8})-(\d+)$")


async def next_submit_seq(conn) -> int:
    """1 + max(submit_seq) over the tasks table (0-based start -> first is 1).
    max+1 (not a process counter) so a restart continues the sequence instead
    of restarting it and colliding with rows already persisted."""
    cur = await conn.execute("SELECT MAX(submit_seq) FROM tasks")
    row = await cur.fetchone()
    current = row[0] if row and row[0] is not None else 0
    return int(current) + 1


async def next_task_id(conn, date_str: str) -> str:
    """Return 't-<date_str>-<NNN>' with NNN = 1 + the max sequence already used
    for that date. date_str MUST be 8 digits (YYYYMMDD); the caller supplies it
    from the wall clock at creation time (a display value)."""
    if not (len(date_str) == 8 and date_str.isdigit()):
        raise ValueError(f"date_str must be YYYYMMDD, got {date_str!r}")
    # Only same-day ids can collide; scan them and take the max NNN. A LIKE on
    # the id prefix keeps the scan to one day's rows.
    cur = await conn.execute(
        "SELECT task_id FROM tasks WHERE task_id LIKE ?", (f"t-{date_str}-%",))
    rows = await cur.fetchall()
    max_n = 0
    for (task_id,) in rows:
        m = _TASK_ID_RE.match(task_id)
        # The LIKE can match a longer/odd id; only count well-formed ones for
        # THIS date (the regex re-checks the date so a stray prefix is ignored).
        if m and m.group(1) == date_str:
            max_n = max(max_n, int(m.group(2)))
    return f"t-{date_str}-{max_n + 1:03d}"
