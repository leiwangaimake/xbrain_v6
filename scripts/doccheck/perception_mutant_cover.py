#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: perception_mutant_cover.py
Brief: Assert every PRC-*/PSC-* rule in book 19 has at least one mutant row

Description:
Book 19 (perception detailed design) states every rule it introduces must be
paired with a mutant that makes it turn red -- see CLAUDE.md 3.3. A rule with
no mutant is indistinguishable from a rule that does not exist: it never enters
tests/perception/, no run ever fails because of it, yet readers assume it is
guarded. The 2026-08-05 adversarial review found more than half of book 19's
rules in exactly that state.

Criterion (single, falsifiable):
  A = every PRC-<n> / PSC-<n> id defined in sections 1..10 of book 19
  B = every PRC-<n> / PSC-<n> id appearing in the mutant table of section 11
  assert A - B == empty

Scope is declared explicitly and printed on every run -- a coverage number
computed over an undeclared surface is worthless (CLAUDE.md 3.2 form 6).
No counts are written back into any markdown file (CLAUDE.md 3.7).

Self-test (--self-test) plants a rule id that has no mutant row and requires
the check to go red; a checker that cannot go red has not been verified.
"""

import argparse
import re
import sys
from pathlib import Path

DOC = Path("/opt/xbrain_v6/docs/19-perception详细设计.md")

# Section 11 is the mutant table; section 12 starts right after. Anchors are
# verbatim, line-anchored and greppable (NUM-4: no line numbers).
#
# Original bug (2026-08-05, caught on the first live run): the start anchor was
# "| **PMT-1**", which also matches the "变异体见" cell of the PRC-1 row back in
# section 1.2. The scan surface silently collapsed to that early row and the
# check reported PASS over a single rule. A checker whose scan surface is wrong
# reports green for the same reason a do-nothing implementation does, so both
# anchors are now line-anchored section headers and asserted unique.
TABLE_START_ANCHOR = "\n## 11."
TABLE_END_ANCHOR = "\n## 12."

RULE_RE = re.compile(r"P(?:RC|SC)-\d+")

# Coverage counts ONLY real mutant rows, i.e. table lines whose first cell is a
# PMT id. Counting the whole of section 11 would let its own prose grant
# coverage: the criterion paragraph names PRC-71 while explaining the scan
# surface, so PRC-71 would stay "covered" even if its mutant row were deleted.
# That is CLAUDE.md 3.2 form 3 -- a criterion sentence sitting inside the
# surface it greps, which can never be violated by construction.
MUTANT_ROW_RE = re.compile(r"^\|\s*\*\*PMT-\d+\*\*", re.MULTILINE)


def sort_key(rule_id: str):
    prefix, num = rule_id.split("-")
    return (prefix, int(num))


def split_doc(text: str):
    """Return (rules_region, mutant_table_region).

    Both anchors must occur exactly once. Raising here rather than falling back
    to "scan the whole file" is deliberate: a mis-anchored scan reports full
    coverage over the wrong surface, which is the 'always green assertion'
    failure mode this script exists to prevent.

    Rules are collected from EVERYTHING outside the table, not just the text
    before it -- PRC-71 is introduced in section 14 (the config listing), so a
    "text before the table" surface would silently drop it.
    """
    for anchor in (TABLE_START_ANCHOR, TABLE_END_ANCHOR):
        hits = text.count(anchor)
        if hits != 1:
            raise SystemExit(f"FAIL: anchor {anchor!r} occurs {hits} times, expected 1")
    start = text.index(TABLE_START_ANCHOR)
    end = text.index(TABLE_END_ANCHOR, start)
    return text[:start] + text[end:], text[start:end]


def mutant_rows(table_region: str):
    """Yield only the lines that are actual mutant rows (first cell = PMT id)."""
    for line in table_region.split("\n"):
        if MUTANT_ROW_RE.match(line):
            yield line


def evaluate(text: str):
    rules_region, table_region = split_doc(text)
    defined = set(RULE_RE.findall(rules_region))
    covered = set()
    for row in mutant_rows(table_region):
        covered.update(RULE_RE.findall(row))
    return defined, covered, sorted(defined - covered, key=sort_key)


def run(text: str, label: str) -> bool:
    defined, covered, missing = evaluate(text)
    print(f"scan surface: {DOC}  [{label}]")
    print(f"  rules region : whole file MINUS the mutant table")
    print(f"  mutant table : {TABLE_START_ANCHOR!r} .. {TABLE_END_ANCHOR!r}")
    print(f"  rules defined={len(defined)}  covered={len(defined & covered)}")
    if missing:
        print("MUT-COVER  FAIL  rules with no mutant row:")
        for rule_id in missing:
            print(f"    {rule_id}")
        return False
    print("MUT-COVER  PASS  every PRC-*/PSC-* has at least one mutant row")
    return True


def self_test(text: str) -> bool:
    """Plant a rule id that no mutant row can guard; the check must go red."""
    # The planted rule must land in the RULES region (outside the table). An
    # earlier version planted it just before the table's END anchor, i.e. inside
    # the table -- it then counted as its own coverage and the self-test came
    # back MISSED. Keeping this note: the self-test caught a bug in the
    # self-test, which is the only evidence that it can go red at all.
    planted = "PRC-9901"
    mutated = text.replace(
        TABLE_START_ANCHOR,
        f"\n| **{planted}** | planted rule with no mutant row |\n" + TABLE_START_ANCHOR,
        1,
    )
    _, _, missing = evaluate(mutated)
    caught = planted in missing
    print(f"self-test: planted {planted} -> {'caught' if caught else 'MISSED'}")
    if not caught:
        print("self-test FAIL: the check cannot go red, so it verifies nothing")
    else:
        print("self-test PASS")
    return caught


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[5])
    parser.add_argument("--self-test", action="store_true",
                        help="inject a violating mutant and require a red result")
    args = parser.parse_args()

    text = DOC.read_text(encoding="utf-8")
    if args.self_test:
        return 0 if self_test(text) else 1
    return 0 if run(text, "live") else 1


if __name__ == "__main__":
    sys.exit(main())
