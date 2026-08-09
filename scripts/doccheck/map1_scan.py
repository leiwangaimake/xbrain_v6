#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: map1_scan.py
Brief: INF-QD-1 / CFG-DC-1 -- MAP-1 alignment table bidirectional diff

Description:
10 S3.3.6 has TWO tables that must agree:

  A. The 29-row failure enumeration ('逐条清单'): each row lists a
     failure item, its detection point, class (R/B/D/T), and an
     ecode.

  B. The alignment table ('与 11 S13.15 的对齐'): for each E_* code,
     lists which failure-row IDs use it.

MAP-1 asserts, for EACH code:
    forward  = rows-in-A-with-code(C) - rows-in-B-for-code(C)  (empty)
    reverse  = rows-in-B-for-code(C)  - rows-in-A-with-code(C) (empty)

Both must be empty; a non-empty forward means someone added a row
to A without updating B; a non-empty reverse means B points at a
row A doesn't have.

The doc itself (10 S3.3.6, 2026-08-05 record) explains this defect
class in first person: rows 7f/7g/7h/7i were added to A during S28
but the alignment table B was not updated, and MAP-1 (if it had
existed) would have caught it. This script IS that MAP-1.

Scope note (verbatim from the doc): both operands live INSIDE
10 S3.3, so this is a per-file check; it does NOT establish that
11 S13.15's own enumeration is aligned with anything. The 11 side
enforcement is 11's own concern.

Usage:
  python3 scripts/doccheck/map1_scan.py             # scan, exit 1 on diff
  python3 scripts/doccheck/map1_scan.py --self-test # inject known drift
  python3 scripts/doccheck/map1_scan.py -v          # verbose
"""

import argparse
import re
import sys
from pathlib import Path
from typing import Dict, FrozenSet, List, Tuple


# Default doc paths. Override via CLI.
_DEFAULT_DOC = "docs/10-顶层设计.md"

# Table A -- row parser for the failure enumeration.
# The row id is the first pipe cell (with optional leading decor).
# The ecode is the sixth pipe cell (0-indexed: after the fifth |).
# Regex is line-anchored to a table row that starts with an id token.
# Decorative BLACK STAR chars in the doc are consumed via unicode
# escape rather than the raw glyph so charset_lint stays clean.
_A_ROW = re.compile(
    r"^\|\s*"                          # row leading |
    r"(?:\u2605\s*)*"                  # optional star decor (BLACK STAR)
    r"(?:\*\*)?"                       # optional bold
    r"(?P<id>[0-9]+[a-z]?)"            # row id: '1' or '7f' etc.
    r"(?:\*\*)?"
    r"(?:<br>[^|]*)?"                  # optional multi-line prefix
    r"\s*\|"                           # closes cell 1
)
_A_CODE_TOKEN = re.compile(r"E_[A-Z_]+")

# Table B -- alignment table row parser.
# First cell = code (backticked, may have leading decor).
# Second cell = row-id list (middle-dot separated per doc convention).
_B_ROW = re.compile(
    r"^\|"                              # row leading |
    r"[^`|]*"                           # any decor before backtick
    r"`(?P<code>E_[A-Z_]+)`"            # backticked code (first only)
    r"[^|]*\|"                          # rest of cell 1
    r"\s*(?P<rows>[^|]*)"               # cell 2 (row list, greedy)
    r"\|"
)
# Row id tokens: 1..29 with optional single-letter suffix. Bounded
# to <= 3 digits so date fragments (2026, 08, 05) do not match.
# Word boundary via non-alphanumeric surround.
_B_ID_TOKEN = re.compile(
    r"(?<![A-Za-z0-9_-])"               # left bound
    r"([1-9][0-9]?[a-z]?)"              # 1..99 + opt letter
    r"(?![A-Za-z0-9_-])"                # right bound
)


def _find_a_and_b_sections(text: str) -> Tuple[str, str]:
    """Isolate table A (row enumeration) and table B (alignment).

    A starts at the enumeration table header ('# | 失败项 | 检出点 | 类 | 处置 | 错误码 | 依据 |')
    and ends at the AI heartbeat sub-section.

    B starts at the '与 11 S13.15 的对齐' subsection and ends at the
    MAP-1 subsection.
    """
    # Find enumeration table via its distinctive header row.
    a_hdr = text.find("| # | 失败项 |")
    if a_hdr < 0:
        raise ValueError("cannot locate table A header in doc")
    a_end = text.find("AI 探活次数 `N = 6`", a_hdr)
    if a_end < 0:
        # Fallback: read to the next major section marker.
        a_end = text.find("\n#### ", a_hdr + 10)
    section_a = text[a_hdr:a_end if a_end > 0 else len(text)]

    b_hdr = text.find("与 `11` §13.15 的对齐", a_hdr)
    if b_hdr < 0:
        raise ValueError("cannot locate alignment table B header")
    b_end = text.find("MAP-1", b_hdr)
    if b_end < 0:
        b_end = text.find("\n#### ", b_hdr + 10)
    section_b = text[b_hdr:b_end if b_end > 0 else len(text)]
    return section_a, section_b


def parse_table_a(section: str) -> Dict[str, str]:
    """Return {row_id: ecode} for every row that names an ecode.

    Rows whose ecode column is '-' or empty are skipped (some rows
    are pointers or 'not a failure' with no code).

    Handles the '同上' inheritance idiom (row N says '(same as
    above)' -- inherit the previous row's ecode). Without this,
    row 2 (migration failure with '同上') would be silently dropped.
    """
    result: Dict[str, str] = {}
    prev_code: str = ""
    for line in section.splitlines():
        m = _A_ROW.match(line)
        if not m:
            continue
        row_id = m.group("id")
        # Walk the whole line for any E_* token; take the first.
        codes = _A_CODE_TOKEN.findall(line)
        if codes:
            result[row_id] = codes[0]
            prev_code = codes[0]
        elif "同上" in line and prev_code:
            # Inherit previous row's ecode.
            result[row_id] = prev_code
    return result


def parse_table_b(section: str) -> Dict[str, FrozenSet[str]]:
    """Return {ecode: {row_id, ...}} for the alignment table."""
    result: Dict[str, FrozenSet[str]] = {}
    for line in section.splitlines():
        m = _B_ROW.match(line)
        if not m:
            continue
        code = m.group("code")
        rows_text = m.group("rows")
        # Extract id tokens from the row list. IDs are separated by
        # middle-dot (U+00B7) in the doc, but we tokenise loosely so
        # future style (comma, slash) still works.
        ids = frozenset(_B_ID_TOKEN.findall(rows_text))
        # Merge duplicates by union: if a code appears twice in
        # section B (shouldn't, but defensively), take the union.
        result[code] = result.get(code, frozenset()) | ids
    return result


def diff(table_a: Dict[str, str],
         table_b: Dict[str, FrozenSet[str]]) -> Dict[str, Dict[str, FrozenSet[str]]]:
    """Compute per-code forward + reverse diff.

    forward[C] = rows-in-A-with-code(C) - rows-in-B-for-code(C)
    reverse[C] = rows-in-B-for-code(C)  - rows-in-A-with-code(C)

    Returns {code: {'forward': set, 'reverse': set}} for codes that
    have non-empty diff on either side.
    """
    # Invert A: {ecode: {row_id, ...}}
    a_by_code: Dict[str, set] = {}
    for row_id, code in table_a.items():
        a_by_code.setdefault(code, set()).add(row_id)
    # Union of codes in either side.
    all_codes = set(a_by_code) | set(table_b)
    out: Dict[str, Dict[str, FrozenSet[str]]] = {}
    for code in sorted(all_codes):
        a_set = frozenset(a_by_code.get(code, set()))
        b_set = table_b.get(code, frozenset())
        fwd = a_set - b_set
        rev = b_set - a_set
        if fwd or rev:
            out[code] = {"forward": fwd, "reverse": rev}
    return out


def _self_test() -> int:
    """Inject a fake row into a synthetic A, verify forward diff fires.
    Inject a fake alignment id into a synthetic B, verify reverse fires."""
    # Minimal synthetic tables.
    a = {"1": "E_ONE", "2": "E_ONE", "3": "E_TWO"}
    b = {"E_ONE": frozenset({"1", "2"}), "E_TWO": frozenset({"3"})}
    d = diff(a, b)
    if d:
        print("self-test FAIL: baseline had diff %s" % d)
        return 1
    # Variant 1: A has row 4 -> E_ONE, B does not.
    a2 = dict(a); a2["4"] = "E_ONE"
    d2 = diff(a2, b)
    if not d2.get("E_ONE", {}).get("forward"):
        print("self-test FAIL: variant 1 (forward) did not fire")
        return 1
    # Variant 2: B points E_TWO at row 99, A does not.
    b2 = dict(b); b2["E_TWO"] = frozenset({"3", "99"})
    d3 = diff(a, b2)
    if not d3.get("E_TWO", {}).get("reverse"):
        print("self-test FAIL: variant 2 (reverse) did not fire")
        return 1
    print("self-test PASS: forward + reverse diffs both fire on injection")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("doc", nargs="?", default=_DEFAULT_DOC,
                    help="path to 10-顶层设计.md")
    args = ap.parse_args()
    if args.self_test:
        return _self_test()
    text = Path(args.doc).read_text()
    section_a, section_b = _find_a_and_b_sections(text)
    table_a = parse_table_a(section_a)
    table_b = parse_table_b(section_b)
    print("scan surface: %s (10 S3.3.6 enumeration + alignment)"
          % args.doc)
    print("  table A rows with ecode: %d" % len(table_a))
    print("  table B codes:           %d" % len(table_b))
    diffs = diff(table_a, table_b)
    if diffs:
        for code, d in diffs.items():
            if d["forward"]:
                print("  BAD (fwd) %s: rows in A not in B: %s"
                      % (code, sorted(d["forward"])))
            if d["reverse"]:
                print("  BAD (rev) %s: rows in B not in A: %s"
                      % (code, sorted(d["reverse"])))
    print("  codes with non-empty diff: %d" % len(diffs))
    print("criterion: diff == 0 per code, both directions")
    return 1 if diffs else 0


if __name__ == "__main__":
    sys.exit(main())
