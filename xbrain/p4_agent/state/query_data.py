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


# --- 18-C G43-G47 RTK / heading status answers (F5 runtime wiring) ----------
# Reuse the C2 deterministic renders (sources_g43_g47); this module only adds the
# cache-freshness layer: a stale state/pose or state/clock -> 'unknown', never a
# last-known RTK reading spoken as current (16 S8.2.1 QT shadow rule).
from xbrain.p4_agent.query.sources_g43_g47 import (  # noqa: E402
    g43_render, g44_render, g45_render, g46_render, g47_render,
)

_POSE_UNKNOWN = "定位信息暂时读不到,请稍后再问"
_CLOCK_UNKNOWN = "授时状态暂时读不到,请稍后再问"


def _fresh_data(cache: StateCache, key: str, now_mono_ms: int, max_age_ms: int):
    """Fresh state value's DATA object, or None if stale/missing. Unwraps the
    3.0 envelope (p1 publishes state/pose enveloped: {..., data:{...}}); tolerates
    a flat value too so a producer that drops the envelope still reads."""
    v = cache.get_fresh(key, now_mono_ms, max_age_ms)
    if v is None:
        return None
    return v.get("data", v) if isinstance(v, Mapping) else None


def rtk_fix_answer(cache, now_mono_ms, *, max_age_ms) -> QueryAnswer:
    """G43 query_rtk_fix from live state/pose.fix_type."""
    d = _fresh_data(cache, "state/pose", now_mono_ms, max_age_ms)
    if d is None:
        return QueryAnswer(known=False, text=_POSE_UNKNOWN)
    return QueryAnswer(known=True, text=g43_render(d.get("fix_type")))


def satellites_answer(cache, now_mono_ms, *, max_age_ms) -> QueryAnswer:
    """G44 query_satellites from live state/pose.num_satellites."""
    d = _fresh_data(cache, "state/pose", now_mono_ms, max_age_ms)
    if d is None:
        return QueryAnswer(known=False, text=_POSE_UNKNOWN)
    return QueryAnswer(known=True, text=g44_render(d.get("num_satellites")))


def heading_status_answer(cache, now_mono_ms, *, max_age_ms) -> QueryAnswer:
    """G45 query_heading_status. H-1: heading_valid alone decides valid/invalid."""
    d = _fresh_data(cache, "state/pose", now_mono_ms, max_age_ms)
    if d is None:
        return QueryAnswer(known=False, text=_POSE_UNKNOWN)
    return QueryAnswer(known=True,
                       text=g45_render(d.get("heading_valid"), d.get("heading_level")))


def heading_source_answer(cache, now_mono_ms, *, max_age_ms) -> QueryAnswer:
    """G46 query_heading_source from live state/pose.heading_source."""
    d = _fresh_data(cache, "state/pose", now_mono_ms, max_age_ms)
    if d is None:
        return QueryAnswer(known=False, text=_POSE_UNKNOWN)
    return QueryAnswer(known=True, text=g46_render(d.get("heading_source")))


def clock_sync_answer(cache, now_mono_ms, *, max_age_ms) -> QueryAnswer:
    """G47 query_clock_sync from live state/clock (P1-13 mirror; CLK-A1)."""
    d = _fresh_data(cache, "state/clock", now_mono_ms, max_age_ms)
    if d is None:
        return QueryAnswer(known=False, text=_CLOCK_UNKNOWN)
    return QueryAnswer(known=True, text=g47_render(d.get("sync"), d.get("source")))


#: intent_id -> answer function, for the orchestrator query_fn dispatch.
RTK_QUERY_ANSWERS = {
    "G43": rtk_fix_answer,
    "G44": satellites_answer,
    "G45": heading_status_answer,
    "G46": heading_source_answer,
    "G47": clock_sync_answer,
}


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
