"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_query.py
Brief: Project task.db rows into the HMI task-panel cards (17 S6.8.4, current/history)

Description:
The read side that backs the HMI 计划面板 (17 S6.8.4): the panel shows the same
five fields for both the CURRENT task and every HISTORY task -- task_id (field 1),
下发时间/created_at (field 2), 任务内容/command_text (field 3), 巡逻点 (field 4),
已巡逻/percent (field 5). This module turns tasks rows into those cards and splits
them into current vs history.

current = running + waiting to run (pending / ready / scheduled / blocked);
history = every other state -- done, failed, cancelled, the other terminal ones,
AND suspended -- newest first (user ruling 2026-11-10, 99 U156; see
CURRENT_STATES). ~~current = non-terminal states; history = terminal states~~
(superseded the same day: it filed suspended tasks under current).

Ordering + paging: submit_seq DESC. submit_seq is monotonic and never NULL, so
'newest first' is stable and keyset-pageable (before = the last seen submit_seq);
finished_at is a nullable wall string and must NOT be the sort key. The 30-day
history window is NOT a query filter -- retention GC (15 S8) keeps terminal rows
to 30 days, so "all history" IS what the table holds.

What it does NOT do: field 4 (巡逻点 names + per-point status) needs the keypoint
layer (F06 record_waypoint) which is not built yet, so `targets` is returned as
an empty list -- the SHAPE, so the frontend renders "no targets" rather than
fabricating points (3.1/3.2). It is wired to populate once that layer lands. This
module also never writes and never opens a connection -- the caller supplies a
live aiosqlite conn on P3's single db thread (15 S2.1).

Looks-right-but-wrong traps this guards:
  * percent from current_step/total_steps must be None when total_steps == 0
    (route not expanded, 17 S6.10.4 EX-1) -- never a fabricated 0 or 100.
  * the state IN-clause is built from the trusted CURRENT_STATES constants, not
    from any query input; scope is validated and limit/before are coerced to int,
    so the SQL carries no untrusted string.
"""

from __future__ import annotations

from typing import Any, Dict, List, Mapping, Optional, Sequence

from xbrain.p3_task.state.machine import TERMINAL_STATES

# The subset of `tasks` columns a card needs. Named explicitly so the SELECT is
# never `SELECT *` (column order then could not drift under us). submit_seq is
# the paging cursor -- carried through the row dict but NOT exposed on the card.
_CARD_COLUMNS = (
    "task_id", "created_at", "command_text", "state", "source",
    "task_type", "current_step", "total_steps", "submit_seq",
)

# CURRENT = what is running now plus what is waiting to run (queued or timed);
# HISTORY = everything else -- every task that has been executed, however it
# ended, INCLUDING a suspended one (user ruling 2026-11-10, 99 U156). A
# suspended task that is resumed goes back to `running` and so reappears under
# current by itself; nothing has to move it.
#   ~~History = terminal states; current = the complement.~~ -- superseded
#   2026-11-10: that put `suspended` under current, so tasks stopped hours ago by
#   an e-stop or an RTK loss sat in the current panel next to the one running.
# Written as a WHITELIST for current: a state added to the closed set later lands
# in history (it is "not running and not waiting") rather than silently joining
# the current panel. The values are trusted constants (11 S4.4 closed set), so
# interpolating them is not an injection surface -- query inputs never reach the
# SQL as strings.
CURRENT_STATES = ("running", "pending", "ready", "scheduled", "blocked")
assert all(s not in TERMINAL_STATES for s in CURRENT_STATES)  # a live state only
_CURRENT_IN = "(" + ",".join("'%s'" % s for s in sorted(CURRENT_STATES)) + ")"

_SCOPE_WHERE = {
    "current": "state IN %s" % _CURRENT_IN,
    "history": "state NOT IN %s" % _CURRENT_IN,
}


#: Queued states: nothing has moved yet, so the card shows 0 % (not "unknown").
_NOT_STARTED = ("pending", "scheduled", "ready", "blocked")


def card_percent(state: str, prog: Optional[Mapping[str, Any]]) -> Optional[float]:
    """Field 5 (已巡逻) of the task card: how far the robot has MOVED, by distance.

    User ruling 2026-11-12: a running task shows the live percent; a finished
    one 100 %; a suspended / failed / cancelled one the percent at that moment.
    15 S9.5A.2 ST-3 + S9.5B PP-3: the external percent is by distance only,
    pct = dist_done_m / (loop_total x route_total_m) x 100, computed on read
    from the task's patrol_progress row (P3 flushes it at every route point and
    at least every progress_flush_s). ~~current_step / total_steps~~ -- the step
    count is 1 for patrol / goto, so the card could only read 0 or 100 %.

      done                      -> 100.0 (even if dist_done_m ended a few cm short)
      queued (_NOT_STARTED)     -> 0.0
      a progress row            -> the PP-3 percent, clamped to 0..100 (dist_done_m
                                   overshoots route_total_m by centimetres)
      anything else             -> None: unknown, the card shows "--", never a guess
    """
    if state == "done":  # finished: the whole way, by definition
        return 100.0
    if state in _NOT_STARTED:  # still waiting: not moved
        return 0.0
    if not prog:  # no distance record (standby, a charge before its leg)
        return None
    total = (prog.get("route_total_m") or 0.0) * (prog.get("loop_total") or 0)  # whole job
    if total <= 0:  # PP-3a: a zero length would divide by zero
        return None
    pct = 100.0 * (prog.get("dist_done_m") or 0.0) / total  # PP-3
    return round(max(0.0, min(100.0, pct)), 1)  # clamp the centimetre overshoot


async def _progress_rows(conn, task_ids: Sequence[str]) -> Dict[str, Dict[str, Any]]:
    """The patrol_progress row of each task on the page (task_id is UNIQUE there:
    one distance record per task, kept through suspend / resume)."""
    if not task_ids:  # empty page
        return {}
    marks = ",".join("?" * len(task_ids))  # one placeholder per id
    cur = await conn.execute(  # one query for the whole page
        "SELECT task_id, dist_done_m, route_total_m, loop_total FROM patrol_progress "
        "WHERE task_id IN (%s)" % marks, list(task_ids))
    return {r[0]: {"dist_done_m": r[1], "route_total_m": r[2], "loop_total": r[3]}
            for r in await cur.fetchall()}  # task_id -> its distance record


def task_card_from_row(row: Mapping[str, Any],
                       prog: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    """Project one tasks row (a _CARD_COLUMNS mapping) into a 17 S6.8.4 card.

    '' -> None for the nullable TEXT display fields so the frontend gets an
    explicit null (render blank) rather than an empty string it must special-case.
    """
    return {
        "task_id": row["task_id"],                      # field 1
        "created_at": row["created_at"] or None,        # field 2: UTC ISO or None
        "command_text": row["command_text"] or None,    # field 3: raw command
        "state": row["state"],                          # badge + current/history
        "source": row["source"],                        # channel
        "task_type": row["task_type"],
        # field 4 (巡逻点): deferred to the keypoint layer (F06). Empty SHAPE, not
        # a fabricated list -- frontend renders "no targets" until it lands.
        "targets": [],
        "progress": {                                   # field 5
            "current_step": row["current_step"],
            "total_steps": row["total_steps"],
            # by distance (card_percent); the step fields stay for the record
            "percent": card_percent(row["state"], prog),
        },
    }


async def query_task_cards(conn, *, scope: str, limit: int,
                           before: Optional[int] = None) -> Dict[str, Any]:
    """One page of task-panel cards.

    scope: 'current' | 'history'. limit caps the page. before is a keyset cursor
    (a submit_seq) -- rows strictly older than it, for lazy-loading history. The
    reply is {tasks: [...], has_more: bool, next_before: int|None}; next_before
    is the cursor to pass for the next page (None when the page is the last).
    """
    where = _SCOPE_WHERE.get(scope)
    if where is None:
        raise ValueError(
            "scope must be 'current' or 'history', got %r" % (scope,))
    params: List[Any] = []
    if before is not None:
        where += " AND submit_seq < ?"
        params.append(int(before))
    # Fetch one MORE than asked: if it comes back, there is a further page, so
    # has_more is known without a second COUNT query.
    sql = ("SELECT %s FROM tasks WHERE %s ORDER BY submit_seq DESC LIMIT ?"
           % (", ".join(_CARD_COLUMNS), where))
    params.append(int(limit) + 1)
    cur = await conn.execute(sql, params)
    fetched = await cur.fetchall()
    has_more = len(fetched) > limit
    kept = fetched[:limit]
    # strict=True: r comes from SELECT <_CARD_COLUMNS>, same length by
    # construction. A silent truncation here would hand task_card_from_row a
    # dict with the tail keys missing -- a card short a field, not an error.
    dicts = [dict(zip(_CARD_COLUMNS, r, strict=True)) for r in kept]
    progress = await _progress_rows(conn, [d["task_id"] for d in dicts])  # one query
    cards = [task_card_from_row(d, progress.get(d["task_id"])) for d in dicts]
    # Cursor for the next page = the last kept row's submit_seq, only when there
    # IS a next page (else None so the client stops paging).
    next_before = dicts[-1]["submit_seq"] if (has_more and dicts) else None
    return {"tasks": cards, "has_more": has_more, "next_before": next_before}
