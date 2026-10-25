"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: matcher.py
Brief: 16 §4 safety-bypass matcher -- estop / prone / stand

Description:
Pure function `match_bypass(text) -> Optional[BypassAction]`.
Called TWICE by the audio_rx wire (once on raw ASR text, once on
post-normalized text) per 16 §4 约束表. Either hit fires; either
miss falls through to the general classifier.

Match strategy per 16 §4 约束表 + §4.1 non-symmetric cost:
  * Substring containment (not anchored regex) -- 16 §4.1 note:
    field test showed anchored `^stop$` missed "现在立刻停下来".
    V6 uses ESTOP_CONTAINS approach from V5's classifier: any of the
    tokens ANYWHERE in the cleaned text triggers.
  * Wake-prefix tolerated: text may open with "哎/那个/嗯/喂/嘿/
    机器人/先" etc without defeating the match.
  * Trailing punctuation / colloquial particles tolerated.
  * 拼音 + 模糊: TODO (Phase 2 with a pinyin lib on-target); today
    we ship exhaustive Chinese variants harvested from V5's field
    test which covered 18+ operator panic phrasings.

Recording-state suppression (16 §4.2 U45):
  * Caller MUST check `is_recording` (P3 state) before invoking this
    matcher. If in geometry_recording, voice estop is suppressed.
  * This module does NOT read state itself -- it is a pure function.
    The caller (audio_rx) owns the state check and the suppression
    log + TTS "录制中,请用手柄急停" advisory.

* Why NOT reuse V5's regex directly here: V5's ESTOP_PATTERN uses a
big anchored regex; V6 spec explicitly ships the CONTAINS approach
(V5 also has ESTOP_CONTAINS which is the actually-used path). The
regex path is a legacy fallback in V5; V6 takes the more-permissive
contains-first path per non-symmetric cost.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Iterable, Optional

# --- Action closed set (16 §4 table) -----------------------------

BYPASS_ESTOP = "estop"
BYPASS_PRONE = "prone"
BYPASS_STAND = "stand"

BYPASS_ACTIONS: FrozenSet[str] = frozenset({
    BYPASS_ESTOP, BYPASS_PRONE, BYPASS_STAND,
})


@dataclass(frozen=True)
class BypassHit:
    """One bypass match. Returned by match_bypass; None if no match."""
    action: str                          # one of BYPASS_ACTIONS
    matched_token: str                   # the token that fired
    source: str                          # "raw" or "normalized"

    def __post_init__(self) -> None:
        if self.action not in BYPASS_ACTIONS:
            raise ValueError(
                "action %r not in %s" % (self.action, sorted(BYPASS_ACTIONS)))
        if self.source not in ("raw", "normalized"):
            raise ValueError(
                "source %r not in {'raw','normalized'}" % self.source)


# --- Token tables (16 §4 + V5 field-test harvest) ----------------
# * ORDERED tuples sorted LONGEST-FIRST -- this is the "长优先" rule
# from 16 §5.2 (long phrase beats short substring). For estop
# specifically, "立刻停" must win over "停下" when text is "现在立
# 刻停下来", so the fired token identifies the operator intent
# accurately for logs/audit. Substring match then finds the first
# (longest) hit and returns.

# * estop tokens: comprehensive because 漏 > 误 (16 §4.1).
# Sourced from 16 §4 verbatim yaml sample + V5's ESTOP_CONTAINS set
# + V5's ESTOP_PATTERN alternation.
_ESTOP_TOKENS_UNSORTED = (
    "急停", "紧急停", "紧急停止", "紧急", "急刹", "急刹车",
    "停下", "停下来", "停止", "停车", "停一下",
    "马上停", "立刻停", "立即停", "快停", "立刻急停",
    # Latin panic-mode.
    "stop", "STOP",
)
_ESTOP_TOKENS = tuple(sorted(_ESTOP_TOKENS_UNSORTED, key=len, reverse=True))

# * prone tokens (趴下 = go to lying-down / down posture).
# V5 field-test discovered ASR appends colloquial particles (趴下去
# / 趴下来); the substring match tolerates them naturally.
_PRONE_TOKENS = tuple(sorted(("趴下", "卧倒"), key=len, reverse=True))

# * stand tokens (站立 / 站起来 / 起立).
_STAND_TOKENS = tuple(sorted(
    ("站立", "站起来", "起立", "起来"), key=len, reverse=True))


def strip_yield_phrases(text: str, yield_phrases: Iterable[str]) -> str:
    """Remove each complete qualified stop phrase from text.

    18 boundary tables (the E08 one and the D12 one in S6.3A): a bare 停 /
    停止 / 停下 is ALWAYS A01, the bypass; E08 is the stop that carries a
    qualifier (扫 / 环视 / 云台 / 镜头), and E05 "停止跟踪" sits in the same
    PTZ stop family. Matching the estop tokens as substrings (16 S4.1,
    deliberately loose) gave the qualified phrases to A01 as well, so
    "停止扫描" stopped the chassis and never the PTZ (2026-10-24 / 10-25 on
    .1.26).

    The phrases are REMOVED, not used to skip the match: whatever is left is
    still matched, so "停止扫描然后停下" keeps its 停下 and still stops.
    Replaced by a space so the two sides of a removed phrase cannot join
    into a token that was never said. Longest first, so a phrase that
    contains another is removed whole.
    """
    for phrase in sorted(set(yield_phrases), key=len, reverse=True):
        if phrase:
            text = text.replace(phrase, " ")
    return text


def match_bypass(text: str,
                 yield_phrases: Iterable[str] = ()) -> Optional[BypassHit]:
    """Pure single-string bypass match.

    Args:
        text: cleaned ASR text OR raw ASR text. The caller decides
              which; this function is called TWICE per utterance
              (raw + normalized) per 16 §4 约束表.
        yield_phrases: complete qualified stop phrases of other intents
              (see strip_yield_phrases); empty = the pre-2026-10-25
              behaviour, every estop token anywhere fires.

    Returns:
        BypassHit if a bypass action matched, None otherwise. On
        ambiguity (a rare "急停站立" utterance) ESTOP wins per
        priority order below -- safety-first.

    Note the argument does NOT carry the `source` field. The caller
    sets it when wrapping the return in the pipeline:
        raw_hit = match_bypass(raw)
        if raw_hit: hit = BypassHit(raw_hit.action, raw_hit.matched_token, "raw")
    or simpler: the module also exports `match_raw` / `match_normalized`
    that call this and set the source, below.
    """
    if not text:
        return None
    if yield_phrases:
        text = strip_yield_phrases(text, yield_phrases)
    # Priority order: estop first, then prone, then stand. Reason:
    # estop is the highest-cost missed match and its non-symmetric
    # cost dominates. Prone/stand miss is recoverable (repeat the
    # word); estop miss is not (16 §4.1).
    for token in _ESTOP_TOKENS:
        if token in text:
            return BypassHit(BYPASS_ESTOP, token, "raw")
    for token in _PRONE_TOKENS:
        if token in text:
            return BypassHit(BYPASS_PRONE, token, "raw")
    for token in _STAND_TOKENS:
        if token in text:
            return BypassHit(BYPASS_STAND, token, "raw")
    return None


def is_estop_phrase(phrase: str) -> bool:
    """True iff the phrase alone would fire the estop bypass.

    Used to pick, from an intent's keywords, the ones the bypass would eat
    (only those need to yield) without a second copy of the token table.
    """
    hit = match_bypass(phrase)
    return hit is not None and hit.action == BYPASS_ESTOP


def match_raw(text: str,
              yield_phrases: Iterable[str] = ()) -> Optional[BypassHit]:
    """Match on RAW ASR text (before post-processing).

    16 §4 约束表 point 1: "在三层后处理之前先做一次原文匹配". This
    is the FIRST match point in the pipeline."""
    hit = match_bypass(text, yield_phrases)
    if hit is None:
        return None
    return BypassHit(hit.action, hit.matched_token, "raw")


def match_normalized(text: str,
                     yield_phrases: Iterable[str] = ()) -> Optional[BypassHit]:
    """Match on POST-normalized text (after 3-layer post-processing).

    16 §4 约束表 point 1: "之后再对纠正后文本匹配一次". This is the
    SECOND match point; it catches cases where L1/L2/L3 mangled the
    raw stop into a non-stop token but a different stop-family token
    survived the normalization."""
    hit = match_bypass(text, yield_phrases)
    if hit is None:
        return None
    return BypassHit(hit.action, hit.matched_token, "normalized")
