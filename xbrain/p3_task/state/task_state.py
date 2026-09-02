"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_state.py
Brief: 11 S4.4 TaskState -- the current/queue/suspended shape published on state/task

Description:
`state/task` is contracted as TaskState (11 S2.2.2 names the type and the cadence
"event + 1 Hz"; 11 S4.4 gives the shape) and its consumers are p4_agent, the HMI
and the CLOUD. P3 published a placeholder instead -- {schema, active_task:
{task_id, state, mono_ms}} -- three fields out of the contract's three lists. The
consequence reached the customer: the cloud task item (v2.0 S3.2) needs route_id
and started_ts, both of which TaskState carries and the placeholder did not, so Qt
showed a running task with no route and no start time. Reading them out of
query/tasks was not an option and not a shortcut worth taking: TaskCard (11 S12.2A
/ 17 S6.8.4) is the HMI panel's shape, and 11 is explicit that P5 does not read
P3's task.db, so TaskState is the only contracted path for this data.

Buckets, from the 12-value state closed set:
  current   = the running task (S4.4 shows one object, not a list)
  suspended = state 'suspended'
  queue     = the remaining non-terminal states (blocked/pending/ready/scheduled)
The non-terminal set is DERIVED from TASK_STATE minus TERMINAL_STATES rather than
written out, so a thirteenth state cannot quietly fall out of every bucket -- the
same failure the cloud projection already paid for once (a task in an unbucketed
state vanished from the snapshot while it was running).

What this does NOT fill, and why it must not:
  * `progress` is null while the route has not been expanded. Its 14-field table
    (11 S4.4) is 12-of-14 mandatory and anchored on task_route_snapshot
    (route_total_m is defined as identical to the snapshot's total_len_m); that
    table is empty because the executor is EX-4 gated. Fabricating the block would
    put pct=0.0 on the operator's screen, which is indistinguishable from "just
    started" -- v2.0 S3.2 says in as many words that an unknown percent must be
    null and never 0, and CLAUDE.md 3.1 calls the same move out as fail-silent.
  * `suspended_pos` is null: nothing records the position at suspend time. The
    key is present with a null so a reader can tell "not recorded" from "the
    field does not exist in this build".

This module never writes and never opens a connection: the caller supplies a live
aiosqlite conn on P3's single db thread (15 S2.1), same contract as task_query.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Dict, Iterable, List, Mapping, Optional

from xbrain.common.enums import TASK_STATE
from xbrain.p3_task.state.machine import TERMINAL_STATES


#: The `tasks` columns TaskState needs. Explicit so the SELECT is never
#: `SELECT *` -- column order could then drift under the zip below.
_STATE_COLUMNS = (
    "task_id", "task_type", "state", "priority", "source",
    "route_geo_id", "resume_policy", "started_at",
    "suspend_kind", "suspend_reason", "paused_at", "submit_seq",
)

#: Non-terminal = every state that is not terminal. Derived, not listed: a state
#: added to TASK_STATE without being added here would belong to no bucket and
#: disappear from the broadcast while the task is live.
NON_TERMINAL_STATES = frozenset(TASK_STATE) - frozenset(TERMINAL_STATES)

#: Ordering inside the queue list: the scheduler's own order (15 S6.1), so the
#: head of the broadcast queue is the task that would actually run next. A list
#: ordered differently from the scheduler reads as a bug report from operators.
_QUEUE_ORDER = "priority DESC, submit_seq ASC"

_IN_NON_TERMINAL = "(" + ",".join(
    "'%s'" % s for s in sorted(NON_TERMINAL_STATES)) + ")"


def wall_iso_to_epoch(value: Any) -> Optional[float]:
    """'2026-09-02T09:07:27Z' -> 1788339 ... .0, or None.

    started_ts / suspended_ts are wall-clock DISPLAY fields in 11 S4.4 and v2.0
    S3.2 gives them as numbers, while task.db stores the matching columns as UTC
    ISO strings (15 S9.5). This is a parse of a stored string, not a clock read --
    no age or timeout is ever computed from it (CLK-C1 keeps those on the
    monotonic anchors created_ms / updated_ms / started_mono).

    A malformed or absent string yields None rather than an exception: the column
    is nullable by design (a task that never started has no started_at), and one
    bad row must not take down the broadcast for every other task.
    """
    if not isinstance(value, str) or not value:
        return None
    text = value.strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.timestamp()


def current_item(row: Mapping[str, Any]) -> Dict[str, Any]:
    """The 11 S4.4 `current` object for the running task.

    `type` is the contract's field name for what task.db calls task_type; the
    rename happens here rather than in the reader so the SQL keeps naming real
    columns.
    """
    return {
        "task_id": row.get("task_id"),
        # S4.4 says `type`; the column is task_type. Renamed here rather than in
        # the SELECT so the SQL keeps naming columns that exist.
        "type": row.get("task_type"),
        # The internal 12-value state (15 S3.2). The v2.0 rename happens in P5's
        # projection, not here: this key is the contract's own vocabulary, and a
        # producer that pre-translated it would leave p4 and the HMI reading a
        # cloud-shaped value they have no mapping for.
        "state": row.get("state"),
        # Carried so a consumer can explain WHY this task is the running one
        # rather than one of the queued ones (15 S6.1 orders on it).
        "priority": row.get("priority"),
        # cloud | local | voice | charge ... -- the HMI badges it, and the cloud
        # uses it to tell its own dispatches from the robot's self-issued ones
        # (return_home shows up here as source=charge, not as a cloud task).
        "source": row.get("source"),
        # THE field this module exists for: v2.0 S3.2 marks route_id mandatory
        # and there was no other contracted path off the robot for it.
        "route_id": row.get("route_geo_id"),
        # continue | restart | abort | manual (15 S7.5). Belongs in the broadcast
        # because it decides what a resume will DO, and the operator confirming a
        # resume needs to know that before pressing it.
        "resume_policy": row.get("resume_policy"),
        # null until the route layer expands the task -- see the module docstring.
        "progress": None,
        # The other field this module exists for. Epoch, not the stored ISO
        # string: v2.0 S3.2 types it as a number.
        "started_ts": wall_iso_to_epoch(row.get("started_at")),
    }


def queue_item(row: Mapping[str, Any]) -> Dict[str, Any]:
    """One 11 S4.4 `queue` entry: four fields, no progress (it has not run)."""
    return {
        "task_id": row.get("task_id"),
        "type": row.get("task_type"),
        "state": row.get("state"),
        # Four fields only, per S4.4. Deliberately NOT route_id/started_ts: a
        # queued task has not loaded a route and has not started, so those would
        # be null for every entry -- keys that are always null read as "the data
        # is missing" rather than "the question does not apply yet".
        "priority": row.get("priority"),
    }


def suspended_item(row: Mapping[str, Any]) -> Dict[str, Any]:
    """One 11 S4.4 `suspended` entry.

    suspend_kind / suspend_reason are the v0.3 mandatory pair (the free-text
    reason was retired); they are written together by the DAO so a row cannot
    carry one without the other.
    """
    return {
        "task_id": row.get("task_id"),
        # type / route_id / started_ts are NOT in the S4.4 suspended example,
        # which lists only the suspend-specific fields. They are carried anyway
        # because v2.0 S3.2 makes all three mandatory on EVERY task item, this
        # list included, and a suspended task is one that RAN: it loaded a route
        # and it has a start time, so both are real values sitting in task.db.
        # Emitting null there is a false "unknown" -- and it is the list the
        # operator looks at when a task stops, so it is the worst place to be
        # blank. Adding fields takes nothing away from the S4.4 shape.
        "type": row.get("task_type"),
        "route_id": row.get("route_geo_id"),
        "started_ts": wall_iso_to_epoch(row.get("started_at")),
        "state": row.get("state"),
        # passive = something stopped it (estop, low battery, operator); yielding
        # = it stepped aside for a higher-priority task. The distinction drives
        # auto-resume: only yielding comes back on its own, so showing the pair
        # is what lets an operator tell "will resume itself" from "waiting for
        # me" without reading the scheduler.
        "suspend_kind": row.get("suspend_kind"),
        "suspend_reason": row.get("suspend_reason"),
        "resume_policy": row.get("resume_policy"),
        "progress": None,
        # paused_at is the wall-clock audit column written at the suspend
        # transition (15 S9.5); same epoch conversion as started_ts.
        "suspended_ts": wall_iso_to_epoch(row.get("paused_at")),
        # Not recorded anywhere today; present as a null so a reader can tell
        # "not captured" from "this build predates the field".
        "suspended_pos": None,
    }


def build_task_state(rows: Iterable[Mapping[str, Any]]) -> Dict[str, Any]:
    """Non-terminal task rows -> the 11 S4.4 TaskState body.

    Rows must already be filtered to non-terminal states and ordered by the
    scheduler's order; this function only buckets and shapes. A row whose state
    is terminal is dropped rather than bucketed -- it belongs in neither list,
    and putting it in `queue` would show operators a finished task as pending.
    """
    current: Optional[Dict[str, Any]] = None
    queue: List[Dict[str, Any]] = []
    suspended: List[Dict[str, Any]] = []
    for row in rows:
        state = row.get("state")
        if state == "running":
            # S4.4 `current` is one object. The scheduler admits one running
            # task, so a second would be a scheduler bug; keeping the first
            # (scheduler order = highest priority) is the honest reading and
            # never silently swaps which task the operator is watching.
            if current is None:
                current = current_item(row)
        elif state == "suspended":
            suspended.append(suspended_item(row))
        elif state in NON_TERMINAL_STATES:
            queue.append(queue_item(row))
    return {"schema": "task_state_v1", "current": current,
            "queue": queue, "suspended": suspended}


async def read_task_state(conn, *, limit: int = 512) -> Dict[str, Any]:
    """Read the live tasks and shape them into 11 S4.4 TaskState.

    limit is a guardrail against reading a runaway table into memory, not a
    business rule: it is far above any field's active-task count, and the state
    filter -- not the limit -- is what keeps terminal rows out (the same lesson
    list_by_priority's LIMIT 32 taught, where terminal rows crowded out every
    pending task).
    """
    sql = ("SELECT %s FROM tasks WHERE state IN %s ORDER BY %s LIMIT ?"
           % (", ".join(_STATE_COLUMNS), _IN_NON_TERMINAL, _QUEUE_ORDER))
    cur = await conn.execute(sql, (int(limit),))
    fetched = await cur.fetchall()
    return build_task_state(dict(zip(_STATE_COLUMNS, r)) for r in fetched)
