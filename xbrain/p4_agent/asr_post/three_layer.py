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

import logging
import os
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

_logger = logging.getLogger("xbrain.p4.asr_post")


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


# --- 2026-10-24: L2 and the wired pipeline ---------------------------------
#
# The functions above were never called: turn_loop fed raw ASR text straight to
# the classifier, L2 was a pass-through and asr_dict.yaml was empty. What
# follows is what p4 now runs between ASR and the classifier (orchestrator_turn
# make_turn_handler). post_process() above is left as it was for its existing
# callers; it is NOT the production path (its L3 scores the whole sentence and
# would replace an utterance with a place name).

#: 16 S3.2 phrase length range. L1 keys and L2 windows both live in it.
PHRASE_MIN, PHRASE_MAX = 2, 5

#: Intent classes whose keywords L2 may correct TOWARDS (18 class letters).
#: D = payload (lights / siren / speaker), E = PTZ: the classes whose words were
#: being misheard on the robot. Every other class is excluded on purpose: a
#: first prototype that used all keywords turned "灯开到最大" into "灯开倒退大"
#: (A06 backward, a motion command), "关闭" into "关机" and "后态" into "后退".
#: A correction must never manufacture a motion or system command -- the same
#: non-symmetric cost 16 S4.1 states for the safety bypass. All classes are
#: still PROTECTED (a legal keyword of any class is never rewritten).
L2_TARGET_CLASSES = ("D", "E")

#: One correction: (layer, text before, text after, pinyin distance or 0).
Correction = Tuple[str, str, str, int]


def _is_cjk(text: str) -> bool:
    """True iff every character is a CJK unified ideograph."""
    return bool(text) and all("\u4e00" <= c <= "\u9fff" for c in text)


def _edit_distance(a: str, b: str) -> int:
    """Levenshtein distance over characters (the pinyin letters, 16 S3.3)."""
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(prev[j] + 1, cur[j - 1] + 1,
                           prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


class L2Pinyin:
    """16 S3.3 near-homophone correction by pinyin edit distance.

    * distance = Levenshtein over the letters of the toneless pinyin (16 S3.3:
      "不区分声调"; ignore_tone=False keeps tone digits). 16 S3.3 gives the
      threshold (1) but not the unit; letters is the conservative reading --
      "dao shan" vs "bao shan" is 1, "kai dao" vs "kai deng" is 3.
    * a two-character window must match at distance 0 (pure homophone). Two
      syllables leave no room: at distance 1 the prototype rewrote "关闭" to
      "关机". Three to five characters use max_distance.
    * only a UNIQUE best candidate replaces; a tie leaves the text (16 S3.3).
    * a window that is already a protected keyword is kept as is and skipped,
      longest first, so a correct command is never touched.
    """

    def __init__(self, targets: Iterable[str], protected: Iterable[str],
                 max_distance: int, ignore_tone: bool) -> None:
        # pypinyin is imported here, not at module load: L1 and the legacy
        # functions above must stay importable on a host without it.
        from pypinyin import Style, lazy_pinyin
        self._lazy = lazy_pinyin
        self._style = Style.NORMAL if ignore_tone else Style.TONE3
        self._max = int(max_distance)

        def ok(w: str) -> bool:
            return PHRASE_MIN <= len(w) <= PHRASE_MAX and _is_cjk(w)

        self._protected = frozenset(w for w in protected if ok(w))
        self._by_len: Dict[int, List[Tuple[str, str]]] = {}
        for w in sorted({w for w in targets if ok(w)}):
            self._by_len.setdefault(len(w), []).append((w, self._py(w)))
        self._lengths = sorted(
            {len(w) for w in self._protected} | set(self._by_len),
            reverse=True)

    def _py(self, text: str) -> str:
        """Joined pinyin of a pure-CJK word."""
        return "".join(self._lazy(text, style=self._style))

    def _per_char(self, text: str) -> List[Optional[str]]:
        """Pinyin per character, None for non-CJK.

        lazy_pinyin returns a whole run of non-CJK characters as ONE item, so
        calling it on the full text would misalign every index after a digit.
        It is called per CJK run instead (contextual within the run).
        """
        out: List[Optional[str]] = [None] * len(text)
        i = 0
        while i < len(text):
            if not _is_cjk(text[i]):
                i += 1
                continue
            j = i
            while j < len(text) and _is_cjk(text[j]):
                j += 1
            for k, syl in enumerate(self._lazy(text[i:j], style=self._style)):
                out[i + k] = syl
            i = j
        return out

    def apply(self, text: str) -> Tuple[str, List[Tuple[str, str, int]]]:
        """Return (corrected text, [(window, replacement, distance)])."""
        if not text or not self._by_len:
            return text, []
        syl = self._per_char(text)
        out: List[str] = []
        fixes: List[Tuple[str, str, int]] = []
        i = 0
        while i < len(text):
            step = 0
            for n in self._lengths:
                win = text[i:i + n]
                if len(win) < n or not _is_cjk(win):
                    continue
                if win in self._protected:
                    out.append(win)
                    step = n
                    break
                cands = self._by_len.get(n)
                if not cands:
                    continue
                wp = "".join(s or "" for s in syl[i:i + n])
                best, best_d, tie = None, 1 << 30, False
                for word, py in cands:
                    d = _edit_distance(wp, py)
                    if d < best_d:
                        best, best_d, tie = word, d, False
                    elif d == best_d:
                        tie = True
                limit = 0 if n == 2 else self._max
                if best is not None and best_d <= limit and not tie:
                    out.append(best)
                    fixes.append((win, best, best_d))
                    step = n
                    break
            if step == 0:
                out.append(text[i])
                step = 1
            i += step
        return "".join(out), fixes


def load_l1_entries(path: str) -> Dict[str, str]:
    """Read asr_dict.yaml `exact:` (16 S3.2). Raise ValueError on a bad shape.

    Keys are 2-5 character phrases (16 S3.2 length rule); a one-character key
    would rewrite that character everywhere it appears.
    """
    import yaml

    with open(path, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    if not isinstance(doc, dict):
        raise ValueError("asr_dict top level is not a mapping")
    exact = doc.get("exact") or {}
    if not isinstance(exact, dict):
        raise ValueError("asr_dict.exact is not a mapping")
    for k, v in exact.items():
        if not isinstance(k, str) or not (PHRASE_MIN <= len(k) <= PHRASE_MAX):
            raise ValueError("asr_dict.exact key %r is not a 2-5 char phrase" % k)
        if not isinstance(v, str) or not v:
            raise ValueError("asr_dict.exact value for %r is empty" % k)
    return {str(k): str(v) for k, v in exact.items()}


class AsrPostProcessor:
    """L1 (hot-reloaded dict) -> L2 (pinyin). What the turn handler calls.

    * L1 reloads when the dict file's mtime changes (16 S3.2 hot update,
      CFG-30); a reload that fails keeps the previous dict and logs once per
      change (a bad edit on site must not wipe the working dictionary).
    * L3 (16 S3.4 closed-set snap to place / route names) is not run: its legal
      set comes from the geo catalogue, and p4 has no subscription to it yet.
    * corrections are returned and, with log_corrections, logged at INFO --
      the 16 S3.5 asr_correction table has no schema yet (see 16 S3).
    """

    def __init__(self, dict_path: str, l2: Optional[L2Pinyin],
                 log_corrections: bool) -> None:
        self._path = dict_path
        self._l2 = l2
        self._log = bool(log_corrections)
        self._l1 = L1Dict()
        self._mtime: Optional[int] = None
        self._reload()

    def _reload(self) -> None:
        try:
            mtime = os.stat(self._path).st_mtime_ns
        except OSError as exc:
            if self._mtime != -1:
                _logger.error("asr_post: dict %s unreadable (%s); L1 keeps %d "
                              "entries", self._path, exc, len(self._l1.entries))
                self._mtime = -1
            return
        if mtime == self._mtime:
            return
        self._mtime = mtime
        try:
            entries = load_l1_entries(self._path)
        except Exception as exc:      # noqa: BLE001 -- any bad edit keeps the old dict
            _logger.error("asr_post: dict %s rejected (%s); L1 keeps %d "
                          "entries", self._path, exc, len(self._l1.entries))
            return
        self._l1 = L1Dict(entries=entries)
        _logger.info("asr_post: L1 dict loaded, %d entries", len(entries))

    def apply(self, raw: str) -> Tuple[str, List[Correction]]:
        """Return (corrected text, corrections). Empty in, empty out."""
        if not raw:
            return raw, []
        self._reload()
        corrections: List[Correction] = []
        text = self._l1.apply(raw)
        if text != raw:
            corrections.append(("L1", raw, text, 0))
        if self._l2 is not None:
            text, fixes = self._l2.apply(text)
            corrections.extend(("L2", a, b, d) for a, b, d in fixes)
        if corrections and self._log:
            _logger.info("asr_post: %r -> %r %s", raw, text, corrections)
        return text, corrections

