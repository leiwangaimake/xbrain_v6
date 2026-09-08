"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rns_todo_cover.py
Brief: M-2 RNS_TODO<->book-20 coverage (RNS_TODO M-2)

Description:
The machine proof that RNS_TODO.md leaves no book-20 decision unassigned. It
computes the two-way difference between:
  A = every RNS-<M|N|I|T>-<n> decision id and every A-<FAM> assertion family in
      docs/20-...md
  B = every such id/family referenced in RNS_TODO.md's coverage appendix
and requires A - B == empty (every decision maps to a task) and B - A == empty
(no task cites a decision book 20 does not have -- a typo guard).

This is RNS_TODO's "no omission" claim made falsifiable. It replaces the hand
reconciliation that CLAUDE.md 3.7 says always rots; the numbers live nowhere,
only the two-way-empty criterion.

Self-test (--self-test): drop one decision id from the appendix scan surface and
require a red result, proving the check can fail (a coverage checker that cannot
go red proves nothing -- the always-green shape, 3.2 form 1).
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BOOK20 = ROOT / "docs" / "20-RNS反应式导航软件系统详细设计.md"
TODO = ROOT / "docs" / "RNS_TODO.md"

# Decision ids: RNS-M/N/I/T-<n>. Assertion FAMILIES: A-<FAM> (the appendix maps
# families to tasks; individual assertion rows are M-1's job, not this one).
_DECISION_RE = re.compile(r"\bRNS-[MNIT]-\d+\b")
_FAMILY_RE = re.compile(r"\bA-[A-Z]+\b")
# Assertion families are collected ONLY from LIVE assertion-table rows
# ("| **A-...**"), not from prose. Book 20's obsolescence notes (S0.4) mention
# dead ids like A-DIS-1 (the retired scan-timeout branch); scanning the whole
# file would count those and force RNS_TODO to "cover" a deleted assertion --
# a scan surface too wide is CLAUDE.md 3.2 form 3.
_FAMILY_ROW_RE = re.compile(r"^\|\s*\*\*(A-[A-Z]+)-\d+", re.MULTILINE)


def _book_ids():
    txt = BOOK20.read_text(encoding="utf-8")
    decisions = set(_DECISION_RE.findall(txt))
    families = set(_FAMILY_ROW_RE.findall(txt))
    return decisions, families


def _todo_ids(todo_text=None):
    txt = todo_text if todo_text is not None else TODO.read_text(encoding="utf-8")
    return set(_DECISION_RE.findall(txt)), set(_FAMILY_RE.findall(txt))


def evaluate(todo_text=None):
    bd, bf = _book_ids()
    td, tf = _todo_ids(todo_text)
    return {
        "decisions_uncovered": sorted(bd - td),   # in book, not in TODO -> gap
        "decisions_bogus": sorted(td - bd),        # in TODO, not in book -> typo
        "families_uncovered": sorted(bf - tf),
        "families_bogus": sorted(tf - bf),
        "n_decisions": len(bd), "n_families": len(bf),
    }


def run(todo_text=None) -> bool:
    r = evaluate(todo_text)
    print("RNS-TODO-COVER  book decisions=%d families=%d"
          % (r["n_decisions"], r["n_families"]))
    ok = True
    for k in ("decisions_uncovered", "decisions_bogus",
              "families_uncovered", "families_bogus"):
        if r[k]:
            ok = False
            print("  FAIL %s: %s" % (k, r[k]))
    if ok:
        print("  PASS every book-20 decision & assertion family maps to a task")
    return ok


def self_test() -> bool:
    """Drop a known decision id from the TODO surface; the check must go red."""
    txt = TODO.read_text(encoding="utf-8")
    # remove the first occurrence of some decision id from the TODO text
    m = _DECISION_RE.search(txt)
    if not m:
        print("self-test FAIL: no decision id in TODO to remove")
        return False
    victim = m.group(0)
    mutated = txt.replace(victim, "RNS-X-999", 1)  # break exactly one reference
    # if that id still appears elsewhere in TODO it stays covered; find one that
    # appears exactly once so removal actually uncovers it.
    counts = {}
    for d in set(_DECISION_RE.findall(txt)):
        counts[d] = txt.count(d)
    unique = [d for d, c in counts.items() if c == 1]
    if not unique:
        print("self-test SKIP: no singly-referenced decision to uncover")
        return True
    victim = unique[0]
    mutated = txt.replace(victim, "RNS-X-999", 1)
    r = evaluate(mutated)
    caught = victim in r["decisions_uncovered"]
    print("self-test: removed %s -> %s"
          % (victim, "caught" if caught else "MISSED"))
    if not caught:
        print("self-test FAIL: the check cannot go red")
    return caught


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[5])
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return 0 if self_test() else 1
    return 0 if run() else 1


if __name__ == "__main__":
    sys.exit(main())
