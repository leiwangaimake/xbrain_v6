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
from xbrain.p4_agent.runtime.intent_dispatch import (
    CMD_AUDIO_SPEAK, CMD_GEO, CMD_MODE, CMD_MOTION_INTENT, CMD_TASK, CMD_TEACH,
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession, TurnDecision, TurnOrchestrator,
)


_logger = logging.getLogger("xbrain.p4.orch_turn")


# Bypass posture keys (16 S4). estop is its own safety key; prone/stand are
# posture motions.
#: Keys whose frame IS a contract command, and the slot the orchestrator built
#: it into. On these three keys the payload must be the command ITSELF (11 S7.2
#: TaskCommand / S7.9 GeoCommand / S12A TeachCommand) -- P3 parses cmd_id and
#: action at the TOP LEVEL.
#:
#: *** Until 2026-08-20 the built command travelled NESTED inside p4_agent's
#: p4_intent_v1 envelope ({schema, intent_id, text, mono_ms, geo_command:{...}}),
#: because build_payload merges the orchestrator's slots as members. P3 saw a
#: frame with no top-level cmd_id and refused it as E_SCHEMA -- so the F-class
#: voice path (record a route, save it, delete it) could not have worked on the
#: wire, in either direction, however well each half tested on its own. Found
#: while migrating cmd/task to the contract shape; the same unwrap fixes all
#: three because they broke for one reason.
#:
#: Frames on these keys that carry NO built command (voice pause / cancel, which
#: S7.2 cannot express without a task_id) pass through untouched and P3's
#: receiver routes them by the absence of `action`, exactly as before.
_CONTRACT_FRAME_SLOT = {
    CMD_TASK: "task_command",
    CMD_GEO: "geo_command",
    CMD_TEACH: "teach_command",
    CMD_MODE: "mode_command",
    # A class. Unlike the other four this slot holds a full S3.0 ENVELOPE
    # ({v, src, data}), because P2's G-1 validates the envelope and does not
    # exempt it -- a relative move is a "loose" command and falling back to
    # "pass it through" on a parse failure is dangerous.
    CMD_MOTION_INTENT: "motion_intent",
}

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
    # Site display timezone (common.timezone, IANA name). The single source for
    # G24 query_time's local-time answer; a REQUIRED field with no default so a
    # deployment that forgets to set it fails loudly rather than silently
    # answering UTC (the module ships nowhere but a site, and a site always has
    # a timezone -- no code default is defensible here).
    site_timezone: str
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


def make_rtk_query_fn(cache, *, max_age_ms: int):
    """Return a query_fn that answers 18-C G43-G47 (RTK fix + heading + clock
    status) from live state/pose + state/clock (F5). Returns None for any other
    id so it composes with the other group query_fns. Injected max_age_ms."""
    from xbrain.p4_agent.state.query_data import RTK_QUERY_ANSWERS

    def _query(entry) -> Optional[str]:
        fn = RTK_QUERY_ANSWERS.get(entry.id)
        if fn is None:
            return None
        now_ms = int(time.monotonic() * 1000)
        return fn(cache, now_ms, max_age_ms=max_age_ms).text

    return _query


def make_time_query_fn(cache, tz_name, *, max_age_ms: int):
    """Return a query_fn that answers G24 query_time in the SITE timezone
    (common.timezone). Returns None for any other id. time_answer reconstructs the
    wall time from state/clock's (mono_ref, utc_ref) anchor via a monotonic delta,
    so NO wall clock is read here (CLK-C1); the unsync hard branch is in
    time_answer (18 S9.5)."""
    from xbrain.p4_agent.state.query_data import time_answer

    def _query(entry) -> Optional[str]:
        if entry.id != "G24":
            return None
        now_mono = int(time.monotonic() * 1000)
        return time_answer(cache, tz_name, now_mono, max_age_ms=max_age_ms).text

    return _query


def compose_query_fns(fns):
    """Chain group query_fns; the first to return non-None wins. Lets battery, RTK
    (and future groups) each own their own ids without one giant dispatcher."""
    def _query(entry) -> Optional[str]:
        for fn in fns:
            r = fn(entry)
            if r is not None:
                return r
        return None

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
        out = [(dr.key, _unwrap_contract_frame(dr.key, dr.payload))]
        # Speak a brief acknowledgment so a dispatched action (esp. motion,
        # which has no other audible feedback) is not silent. 2026-08-11
        # ORIN test: '原地待命' dispatched but the operator heard nothing.
        if decision.tts_text:
            out.append(_speak(decision.tts_text))
        return out

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


def _unwrap_contract_frame(key: str, payload: Any) -> Any:
    """The contract command itself for the three contract keys, else payload.

    See _CONTRACT_FRAME_SLOT for why this exists. The envelope's intent_id /
    text are not lost: the builders copy them into the command's params, which
    is where P3 reads provenance from (15 S9.5A.4 command_text).
    """
    slot = _CONTRACT_FRAME_SLOT.get(key)
    if slot and isinstance(payload, dict) and isinstance(payload.get(slot), dict):
        return payload[slot]
    return payload


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
