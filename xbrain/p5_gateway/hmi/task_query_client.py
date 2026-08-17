"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_query_client.py
Brief: P5 client for P3's query/tasks queryable -> GET /api/tasks (11 S12.2A)

Description:
P5 can NOT read P3's task.db directly (plane isolation, 15 four-DB model), so the
HMI task panel gets its current + history rows by doing a Zenoh get() against P3's
`query/tasks` queryable (11 S12.2A) and relaying the reply as REST GET /api/tasks.
This module is that client: build the selector, run the get(), pull the JSON out
of the first ok reply.

The selector is the load-bearing detail: zenoh query parameters are ';'-separated
(NOT '&' like an HTTP query string). Building it with '&' would collapse
scope/limit/before into one parameter and P3 would silently answer the default
(current, page 1). build_task_selector is the one place that joins with ';'.

session.get() is BLOCKING (it iterates a reply channel), so the FastAPI route must
call query_tasks via asyncio.to_thread -- never inline on the event loop. When P3
does not reply (queryable down, or it dropped a bad selector), parse_get_reply
returns an EMPTY page rather than raising, so the panel shows "no tasks" instead
of a 500 (a read for a display must never take the page down).
"""

from __future__ import annotations

import json
from typing import Any, Dict, Iterable, Optional

# The empty page returned when P3 sends no usable reply -- same shape as a real
# reply so the caller/frontend never special-cases "no reply".
_EMPTY_PAGE: Dict[str, Any] = {"tasks": [], "has_more": False, "next_before": None}


def build_task_selector(scope: str, limit: int,
                        before: Optional[int] = None) -> str:
    """Build the query/tasks selector. Params are ';'-separated (zenoh, NOT '&').

    scope/limit always present; before only when paging. The values are our own
    (validated upstream) ints/enum, so there is no user string to escape here.
    """
    parts = ["scope=%s" % scope, "limit=%d" % int(limit)]
    if before is not None:
        parts.append("before=%d" % int(before))
    return "query/tasks?" + ";".join(parts)


def parse_get_reply(replies: Iterable[Any]) -> Dict[str, Any]:
    """Pull the reply JSON from the first ok reply, or the empty page.

    `replies` is what session.get() yields (zenoh Reply objects, or fakes in a
    test). A Reply carries either .ok (a Sample with .payload) or an error; we
    take the first ok payload and decode it. No ok reply -> empty page (never
    raise -- a display read must not fail the request)."""
    for r in replies:
        ok = getattr(r, "ok", None)
        if ok is None:
            continue                       # error reply / no sample -> skip
        payload = getattr(ok, "payload", None)
        if payload is None:
            continue
        return json.loads(bytes(payload).decode("utf-8"))
    return dict(_EMPTY_PAGE)


def query_tasks(session: Any, *, scope: str, limit: int,
                before: Optional[int] = None) -> Dict[str, Any]:
    """get() P3's query/tasks and return the decoded reply page. BLOCKING --
    call via asyncio.to_thread from an async route (see module doc)."""
    selector = build_task_selector(scope, limit, before)
    return parse_get_reply(session.get(selector))
