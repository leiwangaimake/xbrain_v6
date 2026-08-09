"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: three_layer.py
Brief: GWY-P4-03 -- ASR 3-layer post-processing (L1/L2/L3)

Description:
16 S3.1 pipeline:
  raw ASR -> L1 exact-replace -> L2 pinyin fuzzy -> L3 closed-set snap
           -> normalized text

  L1 (16 S3.2) : yaml dict, longest-match-wins, 2-5 char range
  L2 (16 S3.3) : pinyin edit distance <= threshold; single best only,
                  ties -> DO NOT replace, punt to L3
  L3 (16 S3.4) : snap fuzzy candidates to closed set (place names,
                  path names, action words); if best score < threshold,
                  keep raw (mark unknown for LLM)

* HOT UPDATE: L1 dict is one of the two hot-updatable files
(asr_dict.yaml). Reload atomic; schema fail keeps old dict.

* Q-P4-9 discipline: L3 threshold defaults to 0.75; a value below
that would rescue a false candidate. NEVER snap on a tie.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass
class L1Dict:
    """Exact-replace dictionary. Longest-match-first with single-pass
    non-overlapping semantics -- once a substring is replaced, the
    replacement is NOT scanned again this apply() (a shorter key
    whose value now appears in the substituted region does NOT
    re-fire)."""
    entries: Dict[str, str] = field(default_factory=dict)

    def apply(self, text: str) -> str:
        if not text or not self.entries:
            return text
        import re
        # Sort keys longest-first so the compiled alternation
        # prefers the longer match at any position.
        keys = sorted(self.entries.keys(), key=len, reverse=True)
        pat = re.compile("|".join(re.escape(k) for k in keys))
        # re.sub with a callable does one pass left-to-right, and
        # substituted text is not re-scanned -- exactly the
        # semantics we want.
        return pat.sub(lambda m: self.entries[m.group(0)], text)


@dataclass
class L3ClosedSet:
    """Closed-set snap. `members` is the current SQLite-loaded set of
    legal names (paths, waypoints, action words). Snap score is
    simple longest-common-prefix length / max length."""
    members: Tuple[str, ...] = ()
    snap_threshold: float = 0.75

    def apply(self, candidate: str) -> str:
        """Snap `candidate` to a member if the best score >= threshold.
        Otherwise return candidate unchanged (LLM will mark unknown)."""
        if not candidate or not self.members:
            return candidate
        best: Optional[str] = None
        best_score = 0.0
        for m in self.members:
            s = _similarity(candidate, m)
            if s > best_score:
                best_score = s
                best = m
        # Snap only if UNIQUE best AND above threshold.
        second_best = 0.0
        for m in self.members:
            if m == best:
                continue
            s = _similarity(candidate, m)
            if s > second_best:
                second_best = s
        if best_score >= self.snap_threshold and best_score > second_best:
            # If the best member is already a substring of candidate,
            # snap is a no-op -- the legal name is already there.
            if best in candidate:
                return candidate
            return best   # type: ignore[return-value]
        return candidate


def _similarity(a: str, b: str) -> float:
    """Cheap char-set similarity (jaccard-ish). No pinyin lib
    dependency: production upgrades to pypinyin edit distance."""
    if not a or not b:
        return 0.0
    sa, sb = set(a), set(b)
    inter = len(sa & sb)
    union = len(sa | sb)
    return inter / union if union > 0 else 0.0


def post_process(raw: str, l1: L1Dict, l3: L3ClosedSet) -> str:
    """Run L1 -> L2(stub) -> L3 in order. L2 pinyin fuzzy is a
    future pass; here we just chain L1 + L3.

    Empty input -> empty output (do not surface as unknown).
    """
    if not raw:
        return raw
    text = l1.apply(raw)
    # L2 placeholder: no pinyin lib on-target yet. Just pass through.
    text = l3.apply(text)
    return text
