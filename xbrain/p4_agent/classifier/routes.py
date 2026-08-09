"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: routes.py
Brief: 16 §5.3 -- the 4-value route closed set

Description:
The set of possible routes an intent classifier can return. Sourced
from 16 §5.3 intents.yaml schema (each intent's `route` field) and
from _prompt_work/_triage.json (the master triage) which is the truth
source for per-intent route assignment.

Four values, no more:
  * bypass                -- do nothing (§4 safety-bypass estop is
                             a SEPARATE code path; bypass here is the
                             session-cancel / overheard route)
  * fastpath              -- resolved without LLM, direct dispatch
  * fastpath_then_llm     -- fastpath dispatch AND parallel LLM reply
  * llm                   -- open dialog / mission grammar via LLM

* CLAUDE.md 3.5 closed-set enforcement: adding a fifth route requires
a 16 doc change AND a change here. `validate_route` raises on any
value not in the set; the classifier's return type is a RouteDecision
whose __post_init__ calls validate_route.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet


# --- The 4-value closed set (16 §5.3 verbatim) ---------------------

ROUTE_BYPASS = "bypass"
ROUTE_FASTPATH = "fastpath"
ROUTE_FASTPATH_THEN_LLM = "fastpath_then_llm"
ROUTE_LLM = "llm"

ROUTES: FrozenSet[str] = frozenset({
    ROUTE_BYPASS,
    ROUTE_FASTPATH,
    ROUTE_FASTPATH_THEN_LLM,
    ROUTE_LLM,
})


def validate_route(route: str) -> None:
    """Raise ValueError if `route` is outside the 4-value closed set.

    Called by the RouteDecision constructor and by every code path
    that emits `route` into an event / audit log record. Keeps the
    closed-set gate in ONE place per CLAUDE.md 3.5."""
    if route not in ROUTES:
        raise ValueError(
            "unknown route %r; expected one of %s" %
            (route, sorted(ROUTES)))


@dataclass(frozen=True)
class RouteDecision:
    """The classifier's single-output value type.

    Fields:
        route          one of the 4-value closed set
        matched_intent intent id from 18 § (e.g. "A05", "B02", ...) or
                       '' for bypass / unknown
        reason         short human-readable classification reason
                       (mostly for events / audit)

    __post_init__ runs validate_route so a decision with a bad route
    cannot be constructed. Do NOT weaken this check to a runtime
    boolean gate -- the whole point of the frozen dataclass is that
    invalid instances do not exist.
    """
    route: str
    matched_intent: str = ""
    reason: str = ""

    def __post_init__(self) -> None:
        validate_route(self.route)
        if not self.reason:
            object.__setattr__(self, "reason", "route=" + self.route)
