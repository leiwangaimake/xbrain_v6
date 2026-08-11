"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: orchestrator_turn.py
Brief: GWY-P4-41 (32.I) -- wire TurnOrchestrator into the voice loop

Description:
Glue between the six-step TurnOrchestrator (GWY-P4-38) and the outbound
cmd/* keys the voice loop publishes. It does two things:

  * build_orchestrator(): construct a TurnOrchestrator from the loaded
    registry + chitchat presets + a tier-2 callable, so main_wiring builds
    ONE orchestrator and reuses it (the keyword index + presets are built
    once).
  * decision_to_publishes(): map a TurnDecision to the list of (key,
    payload) the loop publishes. A bypass estop goes to cmd/estop (Tier1),
    a dispatched action to its cmd/* key, a reply/confirm/denial to
    cmd/audio/speak, and an overheard turn to NOTHING (16 S5.2.1 silence).

This replaces the V-2B naive_classify + dispatch path: main_wiring now
routes every turn through the orchestrator, so classification, the confirm
gate, and reply generation all follow 16 S5.2 instead of a demo keyword
map.

tier-2 wiring: null_tier2 is the safe default when the LLM classify path is
not enabled -- an unknown directed phrase then yields a 'did not catch
that' speak rather than a fabricated intent. make_llm_tier2 binds the real
GWY-P4-37 classify_unknown (GPU-gated, grammar-constrained) for the ORIN
run; it is kept out of the default so a missing LLM never blocks the loop.
"""
from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Tuple

from xbrain.p4_agent.registry.intents import IntentRegistry
from xbrain.p4_agent.runtime.intent_dispatch import CMD_AUDIO_SPEAK
from xbrain.p4_agent.session.chitchat import ChitchatResponder
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession, TurnDecision, TurnOrchestrator,
)


_logger = logging.getLogger("xbrain.p4.orch_turn")


# Bypass posture keys (16 S4). estop is its own safety key; prone/stand are
# posture motions.
CMD_ESTOP = "cmd/estop"
CMD_MOTION_INTENT = "cmd/motion/intent"

# Reply for a directed phrase we could not classify (no LLM / LLM denied).
_DID_NOT_CATCH = "没太听清,请再说一遍"


@dataclass(frozen=True)
class VoiceOrchestratorInputs:
    """Everything main_wiring needs to run the six-step orchestrator loop.

    Bundled into one object so run_voice_loop_wiring takes a single
    optional param instead of several -- and, deliberately, so the
    time/percent thresholds live as REQUIRED dataclass fields (no default)
    rather than function-parameter defaults. A `_ms`-suffixed parameter
    with a default trips the CLAUDE.md 3.1 safety-value scanner (any
    _s/_ms/_hz name is treated as a limit); a required field carries no
    default, so the discipline stays honest without renaming the unit away.
    __main__ builds this from the resolved config + static content files.
    """
    registry: IntentRegistry
    chitchat: ChitchatResponder
    l2_timeout_ms: int
    query_templates: Any
    query_max_age_ms: int
    query_low_soc_pct: int
    tier2_fn: Optional[Callable[[str, "OrchestratorSession", int],
                                Optional[str]]] = None


def null_tier2(text: str, session: OrchestratorSession,
               now_mono_ms: int) -> Optional[str]:
    """Default tier-2: no LLM classification wired -> return None (unknown).

    The orchestrator turns None into a 'denied' turn; decision_to_publishes
    speaks the 'did not catch that' line. This keeps the loop alive when the
    LLM is absent instead of blocking on it."""
    return None


def build_orchestrator(
    registry: IntentRegistry,
    chitchat: ChitchatResponder,
    *,
    l2_timeout_ms: int,
    tier2_fn: Optional[Callable[[str, OrchestratorSession, int],
                                Optional[str]]] = None,
    query_fn=None,
) -> TurnOrchestrator:
    """Construct the voice-loop orchestrator (built once, reused per turn).

    query_fn (optional) answers a G-class query from live state (GWY-P4-39);
    None leaves G queries to a normal dispatch."""
    return TurnOrchestrator(
        registry, chitchat=chitchat,
        tier2_fn=tier2_fn or null_tier2,
        l2_timeout_ms=l2_timeout_ms,
        query_fn=query_fn)


def make_battery_query_fn(cache, templates, *, max_age_ms: int,
                          low_soc_pct: int):
    """Return a query_fn that answers G02 query_battery from LIVE state.

    GWY-P4-39: reads state/power from the cache and renders the ok/low
    branch, or an 'unknown' reply when the reading is stale. Returns None
    for any other G id, so the orchestrator dispatches those normally (their
    data source is not wired here yet). Injected thresholds, not defaults
    (CLAUDE.md 3.1)."""
    from xbrain.p4_agent.state.query_data import battery_answer

    def _query(entry) -> Optional[str]:
        if entry.id != "G02":
            return None
        now_ms = int(time.monotonic() * 1000)
        ans = battery_answer(cache, templates, now_ms,
                             max_age_ms=max_age_ms,
                             low_soc_threshold=low_soc_pct)
        return ans.text

    return _query


def _speak(text: str) -> Tuple[str, dict]:
    payload = {
        "schema": "p4_speak_v1",
        "text": text,
        "mono_ms": int(time.monotonic() * 1000),
    }
    return (CMD_AUDIO_SPEAK, payload)


def decision_to_publishes(decision: TurnDecision) -> List[Tuple[str, dict]]:
    """Map one TurnDecision to the (key, payload) pairs to publish.

    Returns an empty list for a silent turn (overheard). Every speaking
    branch (reply / confirm prompt / denial / timeout / bypass-suppressed /
    did-not-catch) goes to cmd/audio/speak; a dispatched action goes to its
    own cmd/* key; a bypass goes to cmd/estop (estop) or cmd/motion/intent
    (posture)."""
    kind = decision.kind

    if kind == "overheard":
        # 16 S5.2.1: not addressed to the robot -> publish nothing.
        return []

    if kind == "bypass":
        action = decision.bypass_action
        if action == "estop":
            return [(CMD_ESTOP, {"schema": "p4_estop_v1",
                                 "action": "estop", "source": "voice",
                                 "mono_ms": int(time.monotonic() * 1000)})]
        # prone / stand -> posture motion.
        return [(CMD_MOTION_INTENT, {"schema": "p4_intent_v1",
                                     "action": action, "source": "voice",
                                     "mono_ms": int(time.monotonic() * 1000)})]

    if kind == "bypass_suppressed":
        # U45: voice estop suppressed during recording -> advise the handle.
        return [_speak(decision.tts_text)]

    if kind == "dispatch":
        dr = decision.dispatch_result
        return [(dr.key, dr.payload)]

    if kind == "reply":
        return [_speak(decision.reply_text or decision.tts_text or "")]

    if kind in ("await_confirm", "await_approval",
                "confirm_denied", "confirm_timeout"):
        return [_speak(decision.tts_text or "")]

    if kind == "denied":
        # LLM gate denied / classify failed. Speak the mandated message if
        # the gate provided one (open circuit), else the generic retry line.
        return [_speak(decision.tts_text or _DID_NOT_CATCH)]

    # Any unmodelled kind is a wiring bug, not a silent drop -- log it.
    _logger.warning("orch_turn: unhandled decision kind %r", kind)
    return []


def make_turn_handler(
    orchestrator: TurnOrchestrator,
    session: OrchestratorSession,
) -> Callable[[str], List[Tuple[str, bytes]]]:
    """Return a handler text -> [(key, payload_bytes)] for the turn loop.

    Binds ONE orchestrator + ONE session (per operator dialog). Uses the
    monotonic clock for the turn time (CLK-C1)."""
    def _handle(text: str) -> List[Tuple[str, bytes]]:
        now_ms = int(time.monotonic() * 1000)
        decision = orchestrator.handle_turn(text, session, now_ms)
        out: List[Tuple[str, bytes]] = []
        for key, payload in decision_to_publishes(decision):
            data = json.dumps(payload, ensure_ascii=False,
                              separators=(",", ":")).encode("utf-8")
            out.append((key, data))
        _logger.info("orch_turn: text=%r kind=%s intent=%s",
                     text, decision.kind, decision.intent_id)
        return out
    return _handle
