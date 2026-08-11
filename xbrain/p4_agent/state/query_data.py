"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: query_data.py
Brief: GWY-P4-39 (32.G) -- G-query answers from LIVE state (stale -> unknown)

Description:
Bridges the freshness-aware StateCache (this package) to the query answer
templates (GWY-P4-34 render_reply). A G-class query answer is deterministic
code: read the REAL state field, pick the branch, fill the template
(16 S8.2.1 CMD-40). This module is where 'read the REAL state field'
happens -- and where a STALE reading becomes an 'unknown' answer instead of
a last-known value (16 S8.3 speed rule generalised to the QT shadow rule,
16 S8.2.1).

Boundary: this does NOT classify (that is the priority chain) and does NOT
speak (that is the TTS path). It maps (cache, now) -> a QueryAnswer the
orchestrator hands to TTS.

The low-battery threshold is INJECTED (low_soc_threshold), never a code
default: which SOC counts as 'low' is a deployment policy value, and
CLAUDE.md 3.1 forbids a safety-adjacent default baked into code. The
staleness bound (max_age_ms) is likewise injected from config.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from xbrain.p4_agent.state.cache import StateCache
from xbrain.p4_agent.templates.query_engine import render_reply


@dataclass(frozen=True)
class QueryAnswer:
    """A rendered G-query answer. known=False means the live state was
    stale or missing, so the text is the honest 'unknown' reply, not a
    filled template."""
    known: bool
    text: str


# Honest 'unknown' replies for a stale/missing source. Wording is
# hot-tunable (like the templates); the INVARIANT is that a stale reading
# yields one of these, never a filled last-known value.
_BATTERY_UNKNOWN = "电量信息暂时读不到,请稍后再问"


def battery_answer(
    cache: StateCache,
    templates: Mapping[str, Mapping[str, str]],
    now_mono_ms: int,
    *,
    max_age_ms: int,
    low_soc_threshold: int,
) -> QueryAnswer:
    """G02 query_battery answer from live state/power.

    state/power carries soc + range_km (16 S8.4 context table). If the
    reading is fresh, render the ok/low branch from the LIVE soc; if it is
    stale or never received, return the 'unknown' answer -- NEVER a stale
    soc (16 S8.2.1 QT shadow rule).

    The branch is chosen HERE (render_reply does not infer it): soc at or
    below low_soc_threshold -> 'low' (charge advice), else 'ok'.
    """
    power = cache.get_fresh("state/power", now_mono_ms, max_age_ms)
    if power is None:
        # Stale or missing -> unknown. A last-known soc read aloud would be
        # a silent-wrong answer (the operator acts on a number that may be
        # minutes old), which is exactly what the QT shadow rule forbids.
        return QueryAnswer(known=False, text=_BATTERY_UNKNOWN)
    soc = _require(power, "soc")
    if soc <= low_soc_threshold:
        text = render_reply(templates, "query_battery", "low", {"soc": soc})
    else:
        range_km = _require(power, "range_km")
        text = render_reply(templates, "query_battery", "ok",
                            {"soc": soc, "range_km": range_km})
    return QueryAnswer(known=True, text=text)


def _require(state: Mapping[str, Any], field: str) -> Any:
    """Pull a required field from a live state value; raise if absent.

    A missing field in a value the cache accepted is a producer contract
    break (11 S7.16), not something to paper over with a default -- raising
    surfaces it instead of speaking a fabricated number (CLAUDE.md 3.1)."""
    if field not in state:
        raise KeyError(
            "state value missing required field %r (keys: %s)"
            % (field, sorted(state)))
    return state[field]
