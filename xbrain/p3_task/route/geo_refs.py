"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_refs.py
Brief: Who references a geo object -- the CMD-31 impact set (11 S7.9.4 refs)

Description:
"Confirm deletion?" is not allowed to be a bare question. CMD-31 requires the
L2 confirmation to state the impact, and 11 S7.9.4 is explicit that the numbers
in it come from P3 and are NOT to be produced by the LLM: "this route has 64
points and 320 metres, and is referenced by 1 running task, 1 suspended task and
2 schedules -- confirm delete?"

This module computes that set. It is used twice: by the delete applier (so the
ack carries what was affected) and by action=refs, which is the query the HMI
runs BEFORE showing the dialog.

*** On the width of the match, which is the thing to understand here.

A task can name a geo object two ways:
  * tasks.route_geo_id, the first-class column -- exact, and the right answer;
  * inside mission_json, where a voice task keeps the SLOTS the operator spoke
    ({"route": "east gate route"}) -- a NAME, not an id.

The second is how the system actually works today: nothing writes route_geo_id
yet (the voice ingest path does not fill it), so an id-only match would report
"referenced by 0 tasks" for a route three tasks are about to run. That answer is
worse than useless -- it is a confident no.

So the name is matched too, as a substring of mission_json. That over-reports:
a route named "gate" matches a task whose slots mention any gate. The direction
is chosen deliberately. Over-reporting shows the operator MORE impact than
exists and they decline a delete they might have made; under-reporting deletes a
route out from under a running patrol having told them nothing. The confirmation
text is a human decision aid, and the failure it must not have is false calm.

The over-report is bounded: only tasks in a live state are counted at all
(terminal tasks are history), and the ack labels the count by state so the
operator sees WHICH tasks, not just how many.
"""
from __future__ import annotations

import json
from typing import Any, Dict, List, Optional

# 15 S3.2 groupings. Terminal tasks are excluded entirely -- a done patrol from
# last week is not an impact of deleting the route it used.
_RUNNING = ("running",)
_SUSPENDED = ("suspended",)
_QUEUED = ("pending", "scheduled", "blocked", "ready")


async def _matching_tasks(task_conn, states, geo_id: str,
                          name: Optional[str]) -> List[str]:
    """Task ids in `states` that reference geo_id (exact) or name (substring).

    The name is matched with a quoted-substring LIKE so a name that is a prefix
    of another word does not match: mission_json holds JSON, so "east gate"
    appears as "east gate" WITH its quotes. That still matches a longer slot
    value containing the whole quoted string, which cannot happen in valid JSON,
    and it does not match a slot whose value merely starts the same way.
    """
    marks = ", ".join("?" for _ in states)
    sql = (f"SELECT task_id, mission_json FROM tasks "
           f"WHERE state IN ({marks}) AND (route_geo_id = ? "
           f"      OR mission_json LIKE ?)")
    params = list(states) + [geo_id, f'%"{geo_id}"%']
    cur = await task_conn.execute(sql, params)
    rows = await cur.fetchall()
    out = [r[0] for r in rows]
    if name:
        cur = await task_conn.execute(
            f"SELECT task_id FROM tasks WHERE state IN ({marks}) "
            f"AND mission_json LIKE ?",
            list(states) + [f'%"{name}"%'])
        for (task_id,) in await cur.fetchall():
            if task_id not in out:
                out.append(task_id)
    return sorted(out)


async def _docks_on_route(geo_conn, route_id: str) -> List[str]:
    """Docks whose on_route list names this route (CHG-02). Deleting the route
    does not delete the dock, but it changes which docks are reachable from a
    patrol, which is exactly what the operator needs told."""
    cur = await geo_conn.execute(
        "SELECT geo_id, on_route_json FROM docks WHERE tombstone=0")
    out = []
    for geo_id, on_route_json in await cur.fetchall():
        try:
            routes = json.loads(on_route_json or "[]")
        except ValueError:
            # A corrupt JSON cell must not take the delete path down; the dock
            # simply does not contribute to the impact list, and the parse
            # failure is visible as a missing entry rather than as a crash.
            continue
        if route_id in routes:
            out.append(geo_id)
    return sorted(out)


async def compute_refs(task_conn, geo_conn, *, gtype: str, geo_id: str,
                       name: Optional[str] = None) -> Dict[str, Any]:
    """The 11 S7.9.4 refs block for one object.

    task_conn is required: refs computed without task.db would be an empty
    impact set that LOOKS like "nothing references this". If a caller has no
    task connection, that is a wiring error and it must surface here.
    """
    if task_conn is None:
        raise ValueError("compute_refs needs task.db -- an empty impact set "
                         "would read as 'nothing references this'")
    running = await _matching_tasks(task_conn, _RUNNING, geo_id, name)
    queued = await _matching_tasks(task_conn, _QUEUED, geo_id, name)
    suspended = await _matching_tasks(task_conn, _SUSPENDED, geo_id, name)
    # schedules is a COUNT in S7.9.4's example, not a list: it answers "will
    # this delete break something that fires later tonight".
    cur = await task_conn.execute(
        "SELECT COUNT(*) FROM tasks WHERE state='scheduled' AND "
        "(route_geo_id = ? OR mission_json LIKE ?)", (geo_id, f'%"{geo_id}"%'))
    schedules = (await cur.fetchone())[0]
    refs: Dict[str, Any] = {
        "running_task": running,
        "queued_task": queued,
        "suspended_task": suspended,
        "schedules": schedules,
    }
    if gtype == "route" and geo_conn is not None:
        refs["docks_on_route"] = await _docks_on_route(geo_conn, geo_id)
    return refs


def is_referenced(refs: Dict[str, Any]) -> bool:
    """True if any live task references the object. docks_on_route is NOT
    counted: a dock listing the route is a navigation hint, not a task."""
    return bool(refs.get("running_task") or refs.get("queued_task")
                or refs.get("suspended_task"))
