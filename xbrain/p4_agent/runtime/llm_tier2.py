"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: llm_tier2.py
Brief: GWY-P4-37 (32.E) -- tier-2 LLM classify coordinator (gate + grammar)

Description:
16 S5.2 step 6: when the six-step chain misses every fastpath (the text is
'unknown'), P4 falls back to the LLM to CLASSIFY the utterance (pick an
intent + fill slots), NOT to free-generate a reply (that is chitchat, a
different, preset-only path -- 16 S11.5). This module is the coordinator
that runs that fallback correctly:

  1. the mission GBNF (generate_grammar, 16 S7) is supplied by the caller
  2. the 4-layer prompt (assembler.assemble, 16 S6) is supplied by the caller
  3. THIS module admits the single GPU token (16 S9 domain 6) BEFORE any
     LLM call, and releases it after -- success resets the breaker, a
     timeout advances it toward open
  4. the LLM is called WITH the grammar (llm_client.classify), never free

Why the gate is here and not optional: 16 S9 gives P4 exactly ONE
outstanding LLM call. A path that reached llm_client directly, skipping
try_admit, would let two turns hit the GPU at once and would never trip
the breaker on repeated timeouts -- the operator would sit through a dead
service with no 'temporarily unavailable' TTS. So the ONLY way this
coordinator calls the LLM is through an admitted token; a denied admission
returns WITHOUT calling the LLM (with the must_tts message on an open
circuit).

The actual LLM call is injected (llm_grammar_call) so this stays unit-
testable without a live server; the real wiring binds it to
llm_client.classify. The injected callable MUST receive the grammar --
this module passes it positionally and a caller cannot reach the LLM here
without it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from xbrain.p4_agent.gateway.gpu_token import (
    GpuTokenState, release, try_admit,
)


@dataclass(frozen=True)
class Tier2Result:
    """Outcome of a tier-2 classify attempt.

    admitted=False means the GPU gate denied the call (busy or circuit
    open); raw is None and no LLM call was made. On an open circuit
    must_tts/tts_text carry the mandated 'unavailable' announcement
    (16 S9: no silent circuit break)."""

    admitted: bool
    raw: Optional[str] = None
    reason: str = ""
    must_tts: bool = False
    tts_text: str = ""


class Tier2Error(RuntimeError):
    """The LLM call itself failed after admission (propagated so the
    orchestrator can drop the turn and keep listening)."""


# The injected LLM call: (prompt, grammar) -> raw model output. The grammar
# is the SECOND positional arg and is required -- there is no overload that
# omits it (16 S7 GB-1: an unconstrained classify defeats the closed set).
LlmGrammarCall = Callable[[str, str], str]


def classify_unknown(
    prompt: str,
    grammar: str,
    token_state: GpuTokenState,
    now_mono_ms: int,
    llm_grammar_call: LlmGrammarCall,
) -> Tier2Result:
    """Run the tier-2 grammar-constrained classify through the GPU gate.

    Returns a Tier2Result. The LLM (llm_grammar_call) is invoked ONLY when
    try_admit admits the token, and ALWAYS with the grammar. On admission
    denial the LLM is not called at all. After an admitted call the token
    is released with success/failure so the breaker advances correctly.

    Raises Tier2Error if the LLM call raises after admission (the token is
    still released first).
    """
    if not grammar:
        # A missing grammar must never reach the model (16 S7 GB-1). Fail
        # before admission so a bug cannot burn the GPU slot on it.
        raise Tier2Error(
            "tier-2 classify requires a grammar (16 S7 GB-1)")
    admission = try_admit(token_state, now_mono_ms)
    if not admission.admitted:
        # Denied: do NOT call the LLM. Carry the open-circuit TTS if any.
        return Tier2Result(
            admitted=False,
            reason=admission.reason,
            must_tts=admission.must_tts,
            tts_text=admission.tts_text,
        )
    success = False
    try:
        raw = llm_grammar_call(prompt, grammar)   # grammar ALWAYS passed
        success = True
    except Exception as exc:                       # noqa: BLE001 -- re-raised as Tier2Error
        raise Tier2Error("tier-2 LLM call failed: %s" % exc) from exc
    finally:
        # Release even on failure: success=False advances the breaker
        # toward open (16 S9), success=True resets it.
        release(token_state, success, now_mono_ms)
    return Tier2Result(admitted=True, raw=raw)
