"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: llm_tier2_fn.py
Brief: build the live tier-2 classify function (mission -> grammar -> LLM)

Description:
Assembles the real tier-2 fallback the orchestrator calls when the six
deterministic layers all miss (16 S5.2 step 6). null_tier2 (the stub) always
declined; this composes the pieces that already existed but were never wired
together:

  select_mission(text)        -> one <=5-intent family, or None (decline)
  project_mission_intents +   -> the GBNF that lets the model emit ONLY that
    generate_grammar             mission's intents (AI-36 / GB-1)
  assemble system+mission+text-> the prompt (the user line is the ASR text)
  classify_unknown            -> the SINGLE GPU-gated LLM call (16 S9)
  llm_client.classify         -> the grammar-constrained request
  parse {"intent","slots"}    -> a Tier2Classification the orchestrator routes

Two honesty rules baked in:
  * No mission cue -> return None BEFORE any LLM call. The grammar forces the
    model to pick one of the mission's intents (there is no 'unknown' branch in
    the intent rule), so invoking it on out-of-scope text would MISLABEL it.
    select_mission returning None is the graceful decline, and it never burns
    the one GPU slot on chit-chat.
  * The parsed intent must be one of the mission's own intents (grammar
    guarantees it, but re-checked): a name outside the family is dropped, not
    dispatched, so a model glitch cannot route a stray id.

The LLM call is injected (llm_classify) so this is unit-testable without a
live server; main_wiring binds it to llm_client.classify with the base_url /
model / timeout. token_state is created ONCE by the caller and threaded in --
the breaker must persist across turns (16 S9).
"""
from __future__ import annotations

import json
from typing import Callable, Dict, Optional

from xbrain.p4_agent.classifier.mission_select import select_mission
from xbrain.p4_agent.gbnf.generator import (
    GbnfInvariantError, generate_grammar, project_mission_intents,
)
from xbrain.p4_agent.registry.missions import EXPECTED_EMISSIONS
from xbrain.p4_agent.runtime.llm_tier2 import Tier2Error, classify_unknown
from xbrain.p4_agent.runtime.turn_orchestrator import Tier2Classification


def _assemble_prompt(system_text: str, mission_text: str, text: str) -> str:
    """system persona + mission instructions + the user line (the ASR text).
    The mission prompts expect the utterance as the 'user' line; live
    per-mission context (preset catalogs, waypoint lists) is a later wiring --
    without it a context-needing mission's prompt returns its own miss branch,
    which is a graceful decline, not a wrong dispatch."""
    return "\n\n".join(p for p in (system_text, mission_text, text) if p)


def _parse_classification(raw: str, allowed) -> Optional[Tier2Classification]:
    """Parse the model's {"intent":..,"slots":..}. Returns None on unparseable
    JSON, a missing/foreign intent, or bad slot shape (all -> decline)."""
    try:
        obj = json.loads(raw)
    except (ValueError, TypeError):
        return None
    if not isinstance(obj, dict):
        return None
    name = obj.get("intent")
    if not isinstance(name, str) or name not in allowed:
        return None                       # foreign / missing -> drop, never route
    slots = obj.get("slots")
    if not isinstance(slots, dict):
        slots = {}
    return Tier2Classification(name=name, slots=slots)


def build_tier2_fn(
    registry,
    missions_text: Dict[str, str],
    system_text: str,
    *,
    base_url: str,
    model: str,
    timeout_s: float,
    token_state,
    llm_classify: Optional[Callable[[str, str], str]] = None,
):
    """Return a tier2_fn(text, session, now_mono_ms) -> Optional[
    Tier2Classification]. llm_classify(prompt, grammar) -> raw is injected for
    tests; when None it binds to llm_client.classify with the given endpoint."""
    from xbrain.p4_agent.ai_client import llm_client

    def _call(prompt: str, grammar: str) -> str:
        if llm_classify is not None:
            return llm_classify(prompt, grammar)
        return llm_client.classify(base_url, prompt, grammar,
                                   timeout_s=timeout_s, model=model)

    def tier2_fn(text: str, session, now_mono_ms: int
                 ) -> Optional[Tier2Classification]:
        mission = select_mission(text)
        if mission is None:
            return None                   # no confident family -> decline
        intents = EXPECTED_EMISSIONS.get(mission)
        if not intents:                   # M9/M10 dynamic families not wired here
            return None
        alternation, routes = project_mission_intents(registry, sorted(intents))
        try:
            grammar = generate_grammar(alternation, routes)
        except GbnfInvariantError:
            return None                   # a mission that cannot form a grammar
        prompt = _assemble_prompt(system_text, missions_text.get(mission, ""),
                                  text)
        try:
            result = classify_unknown(prompt, grammar, token_state,
                                      now_mono_ms, _call)
        except Tier2Error:
            return None                   # LLM failed after admission: drop turn
        if not result.admitted or result.raw is None:
            return None                   # gate denied / circuit open
        return _parse_classification(result.raw, set(alternation))

    return tier2_fn
