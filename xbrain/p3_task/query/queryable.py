"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: queryable.py
Brief: Answer a query/tasks selector for the HMI task panel (11 S12.4, 17 S6.8.4)

Description:
The read side of P3's `xbrain/{rid}/query/tasks` Zenoh queryable -- the FIRST
queryable in the system (everything else is pub/sub + P5 REST). P5 can NOT read
P3's task.db directly (plane isolation, 15 four-DB model), so the HMI task panel
pulls current + history through this queryable and P5 relays it as REST.

This module is the pure/async HALF: parse the selector into (scope, limit,
before), run query_task_cards on P3's db, and serialise the reply. The Zenoh
transport (declare_queryable + reply) lives in main_wiring, which calls
answer_task_query from the queryable callback via run_coroutine_threadsafe so the
async db read happens on P3's single db loop (15 S2.1) while the reply is sent
back on the zenoh thread.

Selector grammar (zenoh: params are ';'-separated, NOT '&'):
    query/tasks?scope=history;limit=20;before=42
  * scope: 'current' (non-terminal) | 'history' (terminal); default 'current'
  * limit: page size, clamped to [1, MAX_LIMIT]; default DEFAULT_LIMIT
  * before: keyset cursor (a submit_seq); rows strictly older, for lazy history

A looks-right-but-wrong trap: zenoh selector params split on ';', so a caller
that writes '&' (HTTP habit) lands the whole tail in one param and scope/limit
silently take their defaults -- P5's client MUST build the selector with ';'.
"""

from __future__ import annotations

import json
from typing import Any, Optional, Tuple

from xbrain.p3_task.query.task_query import query_task_cards

# Paging defaults. NOT safety params (CLAUDE.md 3.1 is about spec/safety values);
# a query page size legitimately has a default and an upper clamp so one client
# cannot ask for the whole 30-day table in a single reply.
DEFAULT_LIMIT = 50
MAX_LIMIT = 500


def parse_query_params(params: Any) -> Tuple[str, int, Optional[int]]:
    """Parse a query/tasks selector into (scope, limit, before).

    `params` is a zenoh Parameters or a plain dict -- both expose .get(key). scope
    defaults to 'current'; limit is coerced and clamped to [1, MAX_LIMIT] (a bad
    or absent value falls back to DEFAULT_LIMIT rather than erroring -- a paging
    hint is not worth rejecting the whole query); before is an int cursor or None.
    scope is NOT validated here -- query_task_cards rejects an unknown scope, so
    the one authority on the closed set stays in one place.
    """
    scope = params.get("scope") or "current"
    limit = _coerce_int(params.get("limit"), DEFAULT_LIMIT)
    limit = max(1, min(limit, MAX_LIMIT))
    before = _coerce_int(params.get("before"), None)
    return scope, limit, before


def _coerce_int(value: Any, default: Optional[int]) -> Optional[int]:
    """int(value) or default -- a malformed paging param must not raise (it would
    take down the whole query); it just falls back."""
    if value in (None, ""):
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


async def answer_task_query(conn, params: Any) -> bytes:
    """Run the task-panel query for a selector and return the JSON reply bytes.

    Reply shape: {"tasks": [TaskCard...], "has_more": bool, "next_before": int|None}
    (query_task_cards). ensure_ascii=False so the Chinese command_text stays
    readable on the wire. Raises ValueError for an unknown scope -- the caller
    (the queryable callback) logs it and sends no reply rather than a wrong one.
    """
    scope, limit, before = parse_query_params(params)
    reply = await query_task_cards(conn, scope=scope, limit=limit, before=before)
    return json.dumps(reply, ensure_ascii=False).encode("utf-8")
