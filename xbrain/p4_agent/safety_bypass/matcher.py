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
from typing import FrozenSet, Iterable, Optional, Tuple

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

# * stand tokens (站立 / 站起来 / 起立) -- exactly the configured stand group
#   (common.audio.bypass_keywords.stand), nothing extra.
#   The bare "起来" was here until 2026-10-25 and is REMOVED by user ruling:
#   matched as a substring it fires on ordinary speech ("看起来", "说起来"),
#   and on .1.26 a misheard "云台向右看" came back "起来向右看" and triggered
#   the stand bypass (L0, no confirm). 16 S4.1's "a false trigger is cheap"
#   argument holds for the ESTOP (a false stop is one extra halt); a false
#   STAND is the robot starting a motion nobody asked for, so the loose
#   harvest word does not belong in this group. The union in
#   tokens_from_config can add words from config, so the robot still stands
#   on every configured stand word.
_STAND_TOKENS = tuple(sorted(
    ("站立", "站起来", "起立"), key=len, reverse=True))


def _longest_first(words: Iterable[str]) -> Tuple[str, ...]:
    """Distinct non-blank words, longest first (the 16 S5.2 long-wins rule).

    Ties are broken by the word itself: a set's order follows str hashing,
    which Python randomises per process, and the fired token is logged and
    audited -- the same utterance must name the same token every run.
    """
    return tuple(sorted({w for w in words if w and w.strip()},
                        key=lambda w: (-len(w), w)))


@dataclass(frozen=True)
class BypassTokens:
    """The three bypass word groups one matcher instance uses (16 S4).

    Built by tokens_from_config at p4 startup from the resolved
    bypass.estop / prone / stand keys, which reference the single source
    common.audio.bypass_keywords (10 S5.4.5). BUILTIN_TOKENS is the table
    above alone, for callers that have no config (unit tests, tools).
    """
    estop: Tuple[str, ...]
    prone: Tuple[str, ...]
    stand: Tuple[str, ...]


#: The tables above, unchanged: what every caller got before 2026-10-25.
BUILTIN_TOKENS = BypassTokens(_ESTOP_TOKENS, _PRONE_TOKENS, _STAND_TOKENS)


def tokens_from_config(estop: Iterable[str], prone: Iterable[str],
                       stand: Iterable[str]) -> BypassTokens:
    """The configured word groups UNIONED with the built-in tables.

    Why config at all (2026-10-25). common.audio.bypass_keywords is the one
    list (user ruling 2026-09-21, the union of 16 S4 and 18 A01, 10 S5.4.5
    "词表只此一份"), and it holds the bare 停 and 趴着. This module never
    read it: its hard-coded tables have neither, so "停" alone did not stop
    the robot and "趴着" did not lie it down. Measured 2026-10-25 on .1.26:
    match_bypass("停") -> None.

    Why a union and not the config alone. The built-in tables carry the V5
    field harvest (急刹 / 急刹车 / 紧急 / 停车 / 马上停 ...) that the config
    list does not repeat. Dropping them would lose coverage, and 16 S4.1
    puts a missed stop above everything else. The union can only ADD words:
    no config edit can remove a built-in stop. (Folding the harvest into
    common.yaml so there is truly one list is a change to a user-ruled safety
    list and is left to that ruling -- recorded in 16 S4.)

    An empty or missing group raises: a bypass with no estop words is the
    fail-silent form of no bypass (CLAUDE.md 3.1).
    """
    groups = {"estop": list(estop or ()), "prone": list(prone or ()),
              "stand": list(stand or ())}
    for name, words in groups.items():
        if not words or not all(isinstance(w, str) and w.strip() for w in words):
            raise ValueError("bypass.%s must be a non-empty list of words, got %r"
                             % (name, words))
    return BypassTokens(
        estop=_longest_first(groups["estop"] + list(_ESTOP_TOKENS)),
        prone=_longest_first(groups["prone"] + list(_PRONE_TOKENS)),
        stand=_longest_first(groups["stand"] + list(_STAND_TOKENS)))


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
                 yield_phrases: Iterable[str] = (),
                 tokens: Optional[BypassTokens] = None) -> Optional[BypassHit]:
    """Pure single-string bypass match.

    Args:
        text: cleaned ASR text OR raw ASR text. The caller decides
              which; this function is called TWICE per utterance
              (raw + normalized) per 16 §4 约束表.
        yield_phrases: complete qualified stop phrases of other intents
              (see strip_yield_phrases); empty = the pre-2026-10-25
              behaviour, every estop token anywhere fires.
        tokens: the word groups to match (tokens_from_config); None =
              BUILTIN_TOKENS.

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
    groups = tokens or BUILTIN_TOKENS
    # Priority order: estop first, then prone, then stand. Reason:
    # estop is the highest-cost missed match and its non-symmetric
    # cost dominates. Prone/stand miss is recoverable (repeat the
    # word); estop miss is not (16 §4.1).
    for token in groups.estop:
        if token in text:
            return BypassHit(BYPASS_ESTOP, token, "raw")
    for token in groups.prone:
        if token in text:
            return BypassHit(BYPASS_PRONE, token, "raw")
    for token in groups.stand:
        if token in text:
            return BypassHit(BYPASS_STAND, token, "raw")
    return None


def is_estop_phrase(phrase: str,
                    tokens: Optional[BypassTokens] = None) -> bool:
    """True iff the phrase alone would fire the estop bypass.

    Used to pick, from an intent's keywords, the ones the bypass would eat
    (only those need to yield) without a second copy of the token table.
    Pass the same tokens the matcher runs with: with the configured bare 停
    the set of phrases the bypass would eat is larger.
    """
    hit = match_bypass(phrase, (), tokens)
    return hit is not None and hit.action == BYPASS_ESTOP


def match_raw(text: str,
              yield_phrases: Iterable[str] = (),
              tokens: Optional[BypassTokens] = None) -> Optional[BypassHit]:
    """Match on RAW ASR text (before post-processing).

    16 §4 约束表 point 1: "在三层后处理之前先做一次原文匹配". This
    is the FIRST match point in the pipeline."""
    hit = match_bypass(text, yield_phrases, tokens)
    if hit is None:
        return None
    return BypassHit(hit.action, hit.matched_token, "raw")


def match_normalized(text: str,
                     yield_phrases: Iterable[str] = (),
                     tokens: Optional[BypassTokens] = None) -> Optional[BypassHit]:
    """Match on POST-normalized text (after 3-layer post-processing).

    16 §4 约束表 point 1: "之后再对纠正后文本匹配一次". This is the
    SECOND match point; it catches cases where L1/L2/L3 mangled the
    raw stop into a non-stop token but a different stop-family token
    survived the normalization."""
    hit = match_bypass(text, yield_phrases, tokens)
    if hit is None:
        return None
    return BypassHit(hit.action, hit.matched_token, "normalized")
