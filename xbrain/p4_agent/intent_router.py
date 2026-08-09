"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: intent_router.py
Brief: GWY-P4-02 partial -- minimal 4-way intent dispatcher (fastpath / bypass / llm / fastpath_then_llm)

Description:
Takes one recognized transcript and decides how to answer it. Four
routes per 16 S6.6:

  * fastpath        -- resolved by grammar match alone (no LLM roundtrip).
                       Examples: move_forward, ptz_move, set_light_bright.
                       Produces cmd/motion/intent OR a device command
                       directly; TTS reply optional.

  * llm             -- open dialog. Send transcript + system prompt to
                       llama-server, stream the reply, speak it.
                       Examples: general Q&A, chit-chat, planning talk.

  * bypass          -- do nothing. Recognizer misfired; VAD glitch;
                       explicit "算了" cancellation. Log and return.

  * fastpath_then_llm  -- try grammar first; if it matches, act on
                       intent AND speak an LLM-generated confirmation
                       AT THE SAME TIME (parallel dispatch). Used for
                       intents that benefit from a talking robot but
                       must not stall on LLM latency (e.g. M4_follow).

★ This is the MINIMAL router that unblocks the voice loop MVP. It
does NOT yet:
  * consult the intents.yaml registry (GWY-P4-07 will land the full
    grammar; today the classifier is keyword-driven)
  * enforce auth level (L0/L1a/L1b/L2 -- 16 S8.3A.2, GWY-P4-16)
  * dispatch via Zenoh cmd/motion/intent (GWY-P4-25, needs Zenoh session)
  * handle multi-turn context (16 S14 prompt.history, deliberately off)

What it DOES do (enough for MVP):
  * classifies transcript into one of four routes via keyword match
    (upgradeable to a full grammar-driven decision)
  * dispatches to the correct ai_client wrapper
  * returns a structured RouteDecision the caller can act on

Why keyword classifier as MVP: the full grammar generator (GWY-P4-08)
takes a compilation step and needs intents.yaml with values filled.
The keyword classifier can be replaced without changing the router's
interface; downstream code (turn_loop / __main__) is grammar-agnostic.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import List, Optional


_logger = logging.getLogger("xbrain.p4_agent.router")


# --- Route enum (string, not IntEnum) -----------------------------
# String constants because the 4-way decision is closed-set in 16 S6.6
# and shows up in logs / events; a string is what a human reads. The
# closed-set gate is via _ROUTES below.

ROUTE_FASTPATH = "fastpath"
ROUTE_LLM = "llm"
ROUTE_BYPASS = "bypass"
ROUTE_FASTPATH_THEN_LLM = "fastpath_then_llm"

_ROUTES = frozenset((
    ROUTE_FASTPATH, ROUTE_LLM, ROUTE_BYPASS, ROUTE_FASTPATH_THEN_LLM,
))


@dataclass(frozen=True)
class RouteDecision:
    """The router's output. One per transcript.

    Fields:
        route: which of the 4 routes to take (closed set validated)
        matched_intent: name of the intent that matched, or '' for
            LLM/BYPASS routes
        reason: short human-readable explanation for logs. Never '';
            defaults to route name if router had nothing better to say
    """
    route: str
    matched_intent: str = ""
    reason: str = ""

    def __post_init__(self) -> None:
        if self.route not in _ROUTES:
            raise ValueError(
                "route %r not in closed set %s" % (self.route, sorted(_ROUTES))
            )
        # dataclass frozen -- use object.__setattr__ for defaulting.
        if not self.reason:
            object.__setattr__(self, "reason", "route=" + self.route)


# --- Bypass triggers (empty-utterance or explicit cancel) ----------
# Bypass is the ONLY route with no downstream I/O; classifying as
# bypass is what avoids the LLM being asked "" (which would consume a
# GPU token slot for nothing).

_BYPASS_MARKERS = frozenset((
    "算了", "没事了", "取消", "cancel",
))


# --- Fastpath keyword table (MVP; will be replaced by GBNF grammar) -
# Each row: (keyword_substr, intent_name). First match wins; keywords
# are tested in insertion order. Chinese chars matched literally --
# recognizer output is already normalized to no-punctuation form.
#
# These 8 intents are the MI-1 relative-move set (11 S9.3.2A.4) plus
# the PTZ base (E01) and stop -- the minimum for a demonstrable MVP.
# The full 128-intent table lands with GWY-P4-07 + GWY-P4-08.

_FASTPATH_KEYWORDS: List[tuple] = [
    ("停下",     "stop"),
    ("停车",     "stop"),
    ("stop",     "stop"),
    ("前进",     "move_forward"),
    ("向前",     "move_forward"),
    ("后退",     "move_backward"),
    ("向后",     "move_backward"),
    ("左转",     "turn_left"),
    ("右转",     "turn_right"),
    ("向左",     "turn_left"),
    ("向右",     "turn_right"),
    ("云台向上", "ptz_move_up"),
    ("云台向下", "ptz_move_down"),
    ("云台向左", "ptz_move_left"),
    ("云台向右", "ptz_move_right"),
    ("开灯",     "set_light_on"),
    ("关灯",     "set_light_off"),
    ("拍照",     "take_photo"),
]


def _classify(transcript: str) -> RouteDecision:
    """MVP classifier. First MI-1 keyword match wins.

    Design: this function is intentionally boring. The full router
    will replace it with a GBNF grammar match + registry lookup, but
    every downstream consumer of RouteDecision stays the same.
    Swapping the classifier does not ripple."""
    text = transcript.strip()

    if not text:
        # Empty transcript = VAD false trigger; do nothing.
        return RouteDecision(
            route=ROUTE_BYPASS,
            reason="empty transcript (VAD false trigger)")

    # Explicit cancellation.
    for marker in _BYPASS_MARKERS:
        if marker in text:
            return RouteDecision(
                route=ROUTE_BYPASS,
                reason="cancel marker: " + marker)

    # Fastpath keyword match.
    for keyword, intent in _FASTPATH_KEYWORDS:
        if keyword in text:
            return RouteDecision(
                route=ROUTE_FASTPATH,
                matched_intent=intent,
                reason="fastpath keyword: " + keyword)

    # Fall through to LLM.
    return RouteDecision(
        route=ROUTE_LLM,
        reason="no fastpath match; sending to LLM")


# --- Public API ---------------------------------------------------

def classify(transcript: str) -> RouteDecision:
    """Public entry point. Take one transcript, decide one route.

    Args:
        transcript: recognized text from ASR. May be empty.

    Returns:
        RouteDecision -- always a valid one (never None). The caller
        acts on decision.route:
          * FASTPATH               -> dispatch cmd/motion/intent or device cmd
          * LLM                    -> call llm_client.complete + tts_client.speak
          * BYPASS                 -> log and return; keep listening
          * FASTPATH_THEN_LLM      -> both, in parallel

    This function is pure: given the same transcript it returns the
    same RouteDecision. No I/O, no clock. Testable with a parametrize.
    """
    decision = _classify(transcript)
    _logger.info(
        "classify: route=%s intent=%r reason=%s",
        decision.route, decision.matched_intent, decision.reason)
    return decision


# --- Closed-set enforcement (elemtestable) ------------------------

def validate_route(route: str) -> None:
    """Raise ValueError if `route` is outside the 4-value closed set.

    Called by the confirmation-level gate and by any future serializer
    that emits `route` into an event/log record; keeps the closed set
    in one place."""
    if route not in _ROUTES:
        raise ValueError(
            "unknown route %r; expected one of %s" % (route, sorted(_ROUTES))
        )
