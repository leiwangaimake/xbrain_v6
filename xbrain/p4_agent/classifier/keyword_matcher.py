"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: keyword_matcher.py
Brief: GWY-P4-33 (32.A) -- layer2 keyword match feeding priority_chain

Description:
16 S5.2 layer 2 is "long-phrase exact match, longest-first" (S484
verbatim: longest wins, so a short word does not substring-steal a
long phrase's match). This module builds a keyword -> intent index
from the IntentRegistry (keywords column filled by GWY-P4-33 step1)
and returns the intent whose LONGEST matched keyword is a substring of
the ASR text.

Feeding priority_chain: classify_after_bypass (priority_chain.py) is
the decision logic for layers 2-6; it takes long_phrase_match /
session_state_match / large_class_match as ALREADY-COMPUTED inputs.
This module computes long_phrase_match (layer 2).

* layer 4 (large-class A-J -> intra-class) is deliberately NOT
computed here and stays None. 16 S5.2 layer4 needs a large-class word
table (e.g. deng->D lights class, yuntai->E ptz class) to map a
broad cue to a class before intra-class selection. That table is NOT
defined anywhere today: intents.yaml carries INTENT-level exact
trigger words (GWY-P4-33), not CLASS-level cues. Inventing one here
would violate CLAUDE.md 3.1 (no fabricated data). Until a class-cue
table is a decided design artifact, a phrase that misses every exact
keyword falls through to layer 6 (LLM), which is the correct
fail-open per 16 S5.2 step 6. This is recorded, not silently skipped.

* Bypass (estop/prone/stand) is handled by safety_bypass BEFORE this
runs (priority_chain: "safety-bypass already handled by caller"). The
index below still contains their keywords; that is harmless because
bypass text never reaches here (the orchestrator, GWY-P4-38, runs
bypass first).
"""

from __future__ import annotations

from typing import Callable, Dict, Optional

from xbrain.p4_agent.classifier.priority_chain import (
    ClassifyResult, classify_after_bypass,
)
from xbrain.p4_agent.registry.intents import IntentRegistry


# Intents that must NOT enter the layer-2 keyword index, per 16 S5.2:
#   * route == "bypass": estop/prone/stand are matched at LAYER 1 by
#     safety_bypass, before this runs. Indexing "ting" (stop) here would
#     let a normal "ting" phrase resolve to estop at layer 2.
#   * I01 confirm / I02 deny / I06 clarify_answer are LAYER-3 session
#     responses -- they only mean "yes/no/answer" while a confirm or
#     recording flow is open. Their trigger words ("shi", "bu") are one
#     character and would substring-match almost any sentence at layer 2
#     ("tian qi bu cuo" -> deny). 16 S5.2 layer 3 is where they belong;
#     the session state machine (GWY-P4-38) supplies them as
#     session_state_match, not this index.
_LAYER2_EXCLUDE_ROUTES = frozenset({"bypass"})
_LAYER2_EXCLUDE_IDS = frozenset({"I01", "I02", "I06"})


class KeywordMatcher:
    """keyword -> intent-id index with longest-first substring match.

    Built once from a registry; the index is read-only after
    construction so a shared matcher cannot drift between callers."""

    __slots__ = ("_by_kw",)

    def __init__(self, registry: IntentRegistry) -> None:
        # keyword string -> intent id. On a duplicate keyword the FIRST
        # registration wins; 16 S6.6 trigger words are meant to be
        # unique across intents, and a first-wins rule makes a stray
        # duplicate a visible (deterministic) choice rather than an
        # order-dependent one. bypass + layer-3 session-response intents
        # are excluded (see _LAYER2_EXCLUDE_* above): they are handled at
        # layer 1 / layer 3, and their short trigger words would
        # substring-mis-fire at layer 2.
        self._by_kw: Dict[str, str] = {}
        for e in registry.entries:
            if e.route in _LAYER2_EXCLUDE_ROUTES:
                continue
            if e.id in _LAYER2_EXCLUDE_IDS:
                continue
            for kw in e.keywords:
                if kw and kw not in self._by_kw:
                    self._by_kw[kw] = e.id

    def longest_match(self, text: str) -> Optional[str]:
        """Return the intent id whose LONGEST keyword is a substring of
        text, or None. 16 S484: longest-first, so 'stop broadcasting'
        beats 'stop' when both keywords are present.

        Ties on length keep the first-encountered; deterministic given
        the dict build order (insertion order = registry entry order)."""
        best_kw: Optional[str] = None
        best_intent: Optional[str] = None
        text = text or ""
        for kw, intent in self._by_kw.items():
            if kw in text:
                if best_kw is None or len(kw) > len(best_kw):
                    best_kw = kw
                    best_intent = intent
        return best_intent


def classify_text(
    text: str,
    registry: IntentRegistry,
    matcher: Optional[KeywordMatcher] = None,
    session_state_match: Optional[str] = None,
    asr_confidence: Optional[float] = None,
    large_class_fn: Optional[Callable[[str], Optional[str]]] = None,
) -> ClassifyResult:
    """Run 16 S5.2 layers 2-6 on ASR text (bypass already handled).

    layer 2 (long_phrase) = KeywordMatcher.longest_match.
    layer 3 (session_state) = passed in by the caller (the session
      state machine supplies the legal in-flow response when recording
      / awaiting-confirm; None here, wired by GWY-P4-38 32.F).
    layer 4 (large_class) = large_class_fn(text) when supplied -- the
      deterministic device-family router (classifier.large_class), which
      rescues keyword MISSES for the PTZ (E) and payload (D) closed classes
      per the 16 S5.2 "大类 + 类内规则". It only runs when layer 2 missed
      (classify_after_bypass tries long_phrase first), so it never overrides
      an exact keyword. None when not supplied -> a miss falls to layer 6.

    matcher is built fresh if not supplied; a long-lived caller should
    build one KeywordMatcher and reuse it (index build is O(intents *
    keywords))."""
    if matcher is None:
        matcher = KeywordMatcher(registry)
    long_phrase = matcher.longest_match(text)
    # Layer 4 is only consulted when layer 2 missed. Computing it here is
    # cheap and classify_after_bypass ignores it when long_phrase fired, but
    # short-circuit anyway so the router is not run needlessly on every hit.
    large_class = None
    if long_phrase is None and large_class_fn is not None:
        large_class = large_class_fn(text)
    return classify_after_bypass(
        text=text,
        long_phrase_match=long_phrase,
        session_state_match=session_state_match,
        large_class_match=large_class,
        asr_confidence=asr_confidence,
    )
