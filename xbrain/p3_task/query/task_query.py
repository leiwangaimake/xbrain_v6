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

current = non-terminal states (the tasks in play); history = terminal states,
newest first. The split is single-sourced from state.machine.TERMINAL_STATES so
it can never disagree with the state graph (a local literal is exactly how such
a split silently drifts when a state is added).

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
  * the state IN-clause is built from the trusted TERMINAL_STATES constants, not
    from any query input; scope is validated and limit/before are coerced to int,
    so the SQL carries no untrusted string.
"""

from __future__ import annotations

from typing import Any, Dict, List, Mapping, Optional

from xbrain.p3_task.state.machine import TERMINAL_STATES

# The subset of `tasks` columns a card needs. Named explicitly so the SELECT is
# never `SELECT *` (column order then could not drift under us). submit_seq is
# the paging cursor -- carried through the row dict but NOT exposed on the card.
_CARD_COLUMNS = (
    "task_id", "created_at", "command_text", "state", "source",
    "task_type", "current_step", "total_steps", "submit_seq",
)

# History = terminal states; current = the complement. sorted() gives a stable
# IN-clause. Values come from TERMINAL_STATES (trusted), so interpolating them is
# not an injection surface -- query inputs never reach the SQL as strings.
_TERMINAL_IN = "(" + ",".join("'%s'" % s for s in sorted(TERMINAL_STATES)) + ")"

_SCOPE_WHERE = {
    "current": "state NOT IN %s" % _TERMINAL_IN,
    "history": "state IN %s" % _TERMINAL_IN,
}


def _percent(current_step: int, total_steps: int) -> Optional[float]:
    """Progress percent for field 5, or None when total is unknown.

    total_steps is 0 until the route layer expands the task (17 S6.10.4 EX-1);
    percent MUST be None then, never a fabricated 0 or 100 (3.1/3.2 -- the same
    rule plan_group already applies to the 'done / total' fraction)."""
    if not total_steps or total_steps <= 0:
        return None
    return round(100.0 * current_step / total_steps, 1)


def task_card_from_row(row: Mapping[str, Any]) -> Dict[str, Any]:
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
            "percent": _percent(row["current_step"], row["total_steps"]),
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
    dicts = [dict(zip(_CARD_COLUMNS, r)) for r in kept]
    cards = [task_card_from_row(d) for d in dicts]
    # Cursor for the next page = the last kept row's submit_seq, only when there
    # IS a next page (else None so the client stops paging).
    next_before = dicts[-1]["submit_seq"] if (has_more and dicts) else None
    return {"tasks": cards, "has_more": has_more, "next_before": next_before}
