"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: priority_chain.py
Brief: GWY-P4-06 -- 16 S5.2 six-layer priority chain + directional check

Description:
16 S5.2: after safety_bypass fires (or does not), the classifier
walks a SIX-LAYER priority chain:

  1. safety-bypass       (handled by safety_bypass/matcher.py; NOT here)
  2. long-phrase exact   ("停止喊话" beats "停止")
  3. session-state       (if in recording state, recording responses win)
  4. large-class + inner (A-J 10 classes -> intra-class intent)
  5. directional check   (is this even addressed at the robot? overheard filter)
  6. unknown             (falls to LLM)

* Layer 5 is CRITICAL: without it every ambient conversation reaches
the LLM (waste + noise pollution + operators can't stop the robot
talking).

* Directional signals (16 S5.2.1):
  * contains wake-word / robot address -> DIRECTED
  * second-person imperative -> DIRECTED
  * matched a known intent keyword (layer 4 fired) -> DIRECTED
  * third-person narration / partial sentence -> OVERHEARD
  * other person names / topic unrelated to robot -> OVERHEARD
  * ASR low confidence AND semantically broken -> OVERHEARD

* Asymmetric rule: when uncertain -> judge OVERHEARD.
False-suppress = operator repeats; false-reply = robot talks to air.

Only exception: safety-bypass words (急停 etc.) always win at layer 1,
regardless of directional judgment (anyone yelling stop -> stop).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import FrozenSet, List, Optional


class ChainLayer(str, Enum):
    LONG_PHRASE = "long_phrase"           # layer 2
    SESSION_STATE = "session_state"        # layer 3
    LARGE_CLASS = "large_class"            # layer 4
    OVERHEARD = "overheard"                # layer 5 negative
    UNKNOWN = "unknown"                    # layer 6 (to LLM)


@dataclass(frozen=True)
class ClassifyResult:
    """Output of the chain."""
    layer: str            # ChainLayer value
    intent: str = ""      # matched intent id (empty for OVERHEARD/UNKNOWN)
    fires_llm: bool = False


# Wake-word / address tokens (case-insensitive; substring test).
_WAKE_WORDS: FrozenSet[str] = frozenset({
    "机器人", "小泰", "泰莎", "喂", "嘿",
})

# Second-person imperative markers (rough Chinese cue set).
_IMPERATIVE_MARKERS: FrozenSet[str] = frozenset({
    "你去", "帮我", "请你", "去把", "把它",
})


def is_directed_at_robot(text: str,
                         matched_intent_kw: bool) -> bool:
    """Layer-5 directional check.

    matched_intent_kw: True if layer-4 (or earlier) matched a known
    intent keyword. That alone counts as 'directed'.
    """
    if matched_intent_kw:
        return True
    if any(w in text for w in _WAKE_WORDS):
        return True
    if any(m in text for m in _IMPERATIVE_MARKERS):
        return True
    return False


def is_semantically_broken(text: str,
                           asr_confidence: Optional[float]) -> bool:
    """Layer-5 negative signal: low confidence + broken text ->
    overheard. Kept conservative -- 'broken' is very short OR
    confidence is very low."""
    if asr_confidence is not None and asr_confidence < 0.4:
        return True
    if len(text.strip()) < 2:
        return True
    return False


def classify_after_bypass(
    text: str,
    long_phrase_match: Optional[str],
    session_state_match: Optional[str],
    large_class_match: Optional[str],
    asr_confidence: Optional[float] = None,
) -> ClassifyResult:
    """Run the 5 remaining layers (safety_bypass already handled
    by caller before this function).

    Args:
      text: post-processed ASR text.
      long_phrase_match: intent id if layer 2 fired, else None.
      session_state_match: intent id if layer 3 fired, else None.
      large_class_match: intent id if layer 4 fired, else None.
    """
    # Layer 2.
    if long_phrase_match is not None:
        return ClassifyResult(
            layer=ChainLayer.LONG_PHRASE.value,
            intent=long_phrase_match,
        )
    # Layer 3.
    if session_state_match is not None:
        return ClassifyResult(
            layer=ChainLayer.SESSION_STATE.value,
            intent=session_state_match,
        )
    # Layer 4.
    if large_class_match is not None:
        # Layer 5 directional check is bypassed here per 16 S5.2.1:
        # 'matched keyword -> DIRECTED' (rule 3 in the signal table).
        return ClassifyResult(
            layer=ChainLayer.LARGE_CLASS.value,
            intent=large_class_match,
        )
    # Layer 5 fires only when NO earlier layer matched.
    if not is_directed_at_robot(text, matched_intent_kw=False) \
            or is_semantically_broken(text, asr_confidence):
        return ClassifyResult(
            layer=ChainLayer.OVERHEARD.value,
            fires_llm=False,
        )
    # Layer 6: nothing matched; still directed -> LLM.
    return ClassifyResult(
        layer=ChainLayer.UNKNOWN.value,
        fires_llm=True,
    )
