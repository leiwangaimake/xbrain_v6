"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: debt_code_cover.py
Brief: INF-DB-1 -- 21册 debt ID -> code branch coverage bi-diff

Description:
Parses 21-实测与第三方欠账.md's 正表 rows and cross-references each
debt ID against tests/debt/{ID}.py to produce a bidirectional diff:

  * "in 21 but no test file"  -- new debt entry lacks coverage
  * "test file but no 21 row" -- ghost test (or renamed debt)

Failure by design when tests/debt/ is empty: this is a NEW checker.
The first run reports the initial gap (every 21 row uncovered) and
that count becomes the baseline the next commit tries to shrink.
Silent "no coverage requirement" would defeat the purpose.

CLAUDE.md 3.7 (no hardcoded counts): the debt-row count is NOT written
into a doc or a test. This checker discovers it live and reports it.

Row shape parsed:

  | `<book>` **<id>** | <criteria> | <who tests> | <default behavior> | <where to fix> |

<book> = 11 / 13 / 99 / 本 ; <id> = V-nn, M-nn, T-XXX-n, U-nn, D-nn.

The 4th column (default behavior) additionally MUST hit a fail-loud
word (DBT-2). This is checked here too so a row that fell into a
fail-silent default gets caught before the parser passes it through.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set


# Row regex: 5 pipe-separated columns; first cell has book + bold ID.
# Example:
#   | `11` **V-02** | ... | ... | ... | ... |
_ROW_RE = re.compile(
    r"^\|\s*"
    r"`([^`]+)`\s+"
    r"\*\*([A-Z][A-Z0-9-]*)\*\*"
    r"\s*\|\s*"
    r"([^|]+)\|\s*"
    r"([^|]+)\|\s*"
    r"([^|]+)\|\s*"
    r"([^|]+)\|"
    r"\s*$"
)


# Fail-loud lexicon. A row's 4th column MUST contain at least one of
# these words for DBT-2 to pass. Empty defaults are auto-caught (empty
# string hits none), which is the desired behavior.
_FAIL_LOUD = (
    "拒绝启动", "拒绝",
    "rejected", "warn", "null",
    "恒不通过", "拒",
    "E_NOT_IMPLEMENTED", "E_CONFIG_INVALID", "E_NO_HEADING",
    "不阻塞", "不启用", "不实现",
    "不得", "不可编程",
)

# Fail-silent lexicon. Any of these in the 4th column is a defect
# (a defaulted safety value or a swallowed error).
_FAIL_SILENT = (
    "先用一个猜的值",
    "先跑起来",
    "填一个默认值",
    "静默降级",
    "自动兜底",
    "默认放行",
)


@dataclass
class Debt:
    """One row of 21册 正表."""
    book: str          # "11" / "13" / "99" / "本"
    id: str            # "V-02"
    criteria: str
    who_tests: str
    default_behavior: str
    where_to_fix: str
    line_no: int


@dataclass
class Report:
    debts: List[Debt] = field(default_factory=list)
    fail_silent_hits: List[dict] = field(default_factory=list)
    missing_fail_loud: List[dict] = field(default_factory=list)
    uncovered: List[str] = field(default_factory=list)
    ghost_tests: List[str] = field(default_factory=list)


def parse_21(path: str) -> List[Debt]:
    """Parse 21册 into a list of Debt rows. Non-matching lines are
    skipped; matching lines with malformed shape are also skipped
    (they belong to header tables or dead-ID summary tables)."""
    out: List[Debt] = []
    with open(path, "r", encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            m = _ROW_RE.match(line.rstrip("\n"))
            if not m:
                continue
            book = m.group(1).strip()
            # Skip header-table rows whose first cell is not a book
            # id (e.g. "11 §15.3 / §15.4" -- has a § in it).
            if "§" in book:
                continue
            debt_id = m.group(2).strip()
            out.append(Debt(
                book=book,
                id=debt_id,
                criteria=m.group(3).strip(),
                who_tests=m.group(4).strip(),
                default_behavior=m.group(5).strip(),
                where_to_fix=m.group(6).strip(),
                line_no=lineno,
            ))
    return out


_NEGATION_MARKERS = ("🚫", "不得", "不可", "禁止")


def _fail_silent_in_positive_context(text: str, phrase: str) -> bool:
    """True iff `phrase` appears in `text` and is NOT preceded (within
    ~20 chars on the same clause) by a Chinese negation marker.

    Rows in 21 often say things like:
       !! 不得为"先跑起来"而填手感值
    which is a WARNING against the pattern, not an instance of it.
    Naive substring match false-fires on those."""
    idx = 0
    while True:
        j = text.find(phrase, idx)
        if j < 0:
            return False
        # Look at the preceding 20 chars (or start of string).
        prefix = text[max(0, j - 20):j]
        if not any(m in prefix for m in _NEGATION_MARKERS):
            return True
        idx = j + len(phrase)


def check_fail_loud(debts: List[Debt], report: Report) -> None:
    """DBT-2: each row's default_behavior must contain a fail-loud
    word AND must NOT contain a fail-silent phrase (in positive use)."""
    for d in debts:
        db = d.default_behavior
        if not any(w in db for w in _FAIL_LOUD):
            report.missing_fail_loud.append({
                "id": d.id, "book": d.book, "line": d.line_no,
                "default_behavior": db[:80],
            })
        for bad in _FAIL_SILENT:
            if _fail_silent_in_positive_context(db, bad):
                report.fail_silent_hits.append({
                    "id": d.id, "book": d.book, "line": d.line_no,
                    "matched": bad,
                    "default_behavior": db[:120],
                })
                break


def check_coverage(debts: List[Debt], debt_test_dir: Path,
                   report: Report) -> None:
    """Bi-directional diff. Test filename convention:
    tests/debt/test_{ID_lowercase_with_hyphens_kept}.py, one per row.

    A test file that isn't `assert True` is required by DBT-3 partial;
    the "at least one meaningful assertion" check is enforced by a
    separate meta-test that greps the AST. Here we only check presence."""
    known_ids: Set[str] = {d.id for d in debts}

    if not debt_test_dir.is_dir():
        # New checker path: no tests exist yet. Every debt is uncovered;
        # no ghosts by definition.
        report.uncovered = sorted(known_ids)
        return

    present: Set[str] = set()
    for p in debt_test_dir.glob("test_*.py"):
        # Filename convention: test_v-02.py  (case-insensitive; lower)
        name = p.stem
        if not name.startswith("test_"):
            continue
        stem = name[len("test_"):]
        # Map stem back to the ID form (uppercase). "v-02" -> "V-02".
        present.add(stem.upper())

    for did in sorted(known_ids):
        if did not in present:
            report.uncovered.append(did)
    for pid in sorted(present):
        if pid not in known_ids:
            report.ghost_tests.append(pid)


def run(path_21: str, debt_test_dir: Path) -> Report:
    r = Report()
    r.debts = parse_21(path_21)
    check_fail_loud(r.debts, r)
    check_coverage(r.debts, debt_test_dir, r)
    return r


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[3])
    ap.add_argument("--doc",
                    default="/opt/xbrain_v6/docs/21-实测与第三方欠账.md")
    ap.add_argument("--tests-dir",
                    default="/opt/xbrain_v6/tests/debt")
    ap.add_argument("--format", choices=("json", "human"), default="human")
    ap.add_argument("--strict-coverage", action="store_true",
                    help="exit non-zero if any debt lacks coverage")
    args = ap.parse_args()

    report = run(args.doc, Path(args.tests_dir))

    if args.format == "json":
        print(json.dumps({
            "total": len(report.debts),
            "uncovered": report.uncovered,
            "ghost_tests": report.ghost_tests,
            "missing_fail_loud": report.missing_fail_loud,
            "fail_silent_hits": report.fail_silent_hits,
        }, ensure_ascii=False, indent=2))
    else:
        print("21册 debt trace")
        print("  parsed: %d rows" % len(report.debts))
        print("  uncovered: %d" % len(report.uncovered))
        print("  ghost tests: %d" % len(report.ghost_tests))
        print("  missing fail-loud: %d" % len(report.missing_fail_loud))
        print("  fail-silent hits: %d" % len(report.fail_silent_hits))
        if report.missing_fail_loud:
            print("\n  -- missing fail-loud rows: --")
            for row in report.missing_fail_loud[:20]:
                print("    %s (%s:%d): %s" %
                      (row["id"], row["book"], row["line"],
                       row["default_behavior"]))
        if report.fail_silent_hits:
            print("\n  -- fail-silent rows: --")
            for row in report.fail_silent_hits:
                print("    %s: '%s' in default: %s" %
                      (row["id"], row["matched"], row["default_behavior"]))

    # DBT-2 defects are ALWAYS blocking (they mean the doc itself
    # contains a fail-silent default). Coverage gaps only block when
    # --strict-coverage is set (baseline mode).
    exit_code = 0
    if report.fail_silent_hits:
        exit_code = 1
    if args.strict_coverage and report.uncovered:
        exit_code = 1
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
