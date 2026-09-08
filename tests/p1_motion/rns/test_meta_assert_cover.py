"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_meta_assert_cover.py
Brief: M-1 assertion<->test coverage meta-test (RNS_TODO M-1 / 20 S13)

Description:
The machine judge for "no assertion in book 20 is unimplemented". Book 20 S13
lists every assertion (A-<FAM>-<n>); each must have a test in tests/p1_motion/rns/
that names it. This meta-test computes the two-way difference and requires the
gap to shrink to zero as phases land -- it is what makes RNS_TODO's completeness
claim machine-checked instead of hand-counted (CLAUDE.md 3.7: hand counts rot).

Progressive form: assertions land across P0..P6, so a full two-way-empty check
would be red for the whole build. Instead:
  - EVERY assertion id that appears in a test file MUST exist in book 20
    (a test citing a non-existent assertion is a typo -> hard fail, always).
  - The set of book-20 assertions with NO test yet is the OPEN set; it is
    allowed to shrink only. OPEN_BASELINE ratchets down each phase; a new
    unimplemented assertion (baseline would grow) fails.

Self-check equivalent (the mutant that must redden this): cite A-BOGUS-9 in any
rns test -> the "test cites unknown assertion" check fails. Add a book-20
assertion without ratcheting -> the baseline check fails.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
BOOK20 = ROOT / "docs" / "20-RNS反应式导航软件系统详细设计.md"
RNS_TESTS = ROOT / "tests" / "p1_motion" / "rns"

# An assertion id: A-<FAMILY>-<n> where n may carry a trailing letter (A-CLS-4,
# A-PROF-1b). Anchored to the table-row form so prose mentions do not count as
# definitions: a real assertion row starts "| **A-...**".
_ROW_RE = re.compile(r"^\|\s*\*\*(A-[A-Z]+-\d+[a-z]?)\*\*", re.MULTILINE)
_CITE_RE = re.compile(r"\bA-[A-Z]+-\d+[a-z]?\b")

# Assertions with no test yet (the OPEN set). Ratchets DOWN only; each phase that
# implements assertions removes ids here. P0 has landed A-ST-1 and the two config
# assertions; everything else is still open. This is a machine-enforced TODO.
#: 2026-09-08 P0.2/P0.4: only types + config assertions have tests -> 68.
#: 2026-09-08 P1.1/P1.2/P1.3: +A-RT-1/2/3/4 (route+mission) -> 62.
#: 2026-09-08 P1.4/P1.5/P1.7: +A-ALN-1/2/3, A-DEV-1, A-SPD-1/2/3 -> 55.
#: 2026-09-08 P2.1~P2.6: +A-FUS-1/2/3/4/5/6, A-MEM-3, A-RTK-1/2 -> 47. Ratchets
#:   down each phase; a phase that implements assertions removes ids from the open
#:   set and lowers this WITH a reason, so a regression (test deleted) reddens.
OPEN_BASELINE = 47


def _book_assertions():
    return set(_ROW_RE.findall(BOOK20.read_text(encoding="utf-8")))


def _cited_in_tests():
    cited = set()
    for f in RNS_TESTS.glob("test_*.py"):
        if f.name.startswith("test_meta"):
            continue  # meta-tests cite ids in prose, not as coverage
        cited |= set(_CITE_RE.findall(f.read_text(encoding="utf-8")))
    return cited


def test_every_cited_assertion_exists_in_book20():
    """A test citing an assertion book 20 does not define is a typo. Hard fail,
    no baseline -- this is always two-way-clean on the cite side."""
    book = _book_assertions()
    cited = _cited_in_tests()
    bogus = sorted(cited - book)
    assert not bogus, (
        "rns tests cite assertion ids not in book 20 S13 (typo?): %s" % bogus)


def test_open_assertion_set_only_shrinks():
    """The count of book-20 assertions with no test yet must not grow. A new
    unimplemented assertion, or a test removed, pushes this over the baseline."""
    book = _book_assertions()
    cited = _cited_in_tests()
    open_set = book - cited
    assert len(open_set) <= OPEN_BASELINE, (
        "unimplemented-assertion count rose to %d (baseline %d); +%d new "
        "assertions have no test. Either add the test or ratchet the baseline "
        "WITH a reason.\nopen: %s"
        % (len(open_set), OPEN_BASELINE, len(open_set) - OPEN_BASELINE,
           sorted(open_set)))


def test_book20_has_assertions_at_all():
    # guards against a broken regex silently making the scan surface empty
    # (CLAUDE.md 3.2 form: an always-green check over the wrong surface).
    assert len(_book_assertions()) >= 60
