#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: todo_lint.py
Brief: CHK-1-48 -- lint docs/XBRAIN_V6_TODO.md against seven rules, each
       carrying a scan surface and hit-line report

Description:
The TODO table is the plan of record; a defect there ships as bad plans
until someone re-reads the whole table by eye. This lint runs the seven
rules CHK-1-48 names and prints WHICH row and WHICH rule broke.

The rules, verbatim from the criterion:
  1) ID format: exactly `{3 uppercase}-{2..3 alnum}-{digits}[a-z]?`
  2) IDs are globally unique
  3) `依赖` column IDs exist and their phase <= the phase of the row
  4) A delivery-file path (from `目录`/判据) is not claimed by 2+ IDs
  5) Every `判据` cell has at least one mutation marker
     (`变异体` / `必须变红` / `-> red` variants)
  6) `审查必须报` in a `判据` cell must name a scripts/ path
  7) `全仓` in a `判据` cell must be accompanied by `扫描面`

The lint is TABLE-DRIVEN: RULES is a list of (id, description, scanner);
each scanner returns findings this file prints uniformly. A new rule adds
one row to RULES; scanner code stays local to its own function.

NOTE: CHK-1-48 lists two forbidden shapes not spelt out above:
  - "跨相位倒置" is rule 3's phase check.
  - "同一交付文件双主" is rule 4's file-double-claim check.

Scan surface: docs/XBRAIN_V6_TODO.md ONLY. Every rule states its own
sub-surface (which column is scanned) at the top of its scanner so a
reader auditing "why did this pass" can trace it.
"""

import argparse
import os
import re
import sys
from collections import defaultdict

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TODO = os.path.join(os.path.dirname(_HERE), "docs",
                            "XBRAIN_V6_TODO.md")

# Rule 1: strict ID shape. 3 uppercase letters + `-` + 1..3 alnum tokens
# + `-` + digits + optional single lowercase suffix (e.g. F03a).
# CHK-* rows use a single-digit phase in the middle (CHK-0-54, CHK-2-25);
# every other family (CFG-CM-16, INF-ZN-4, CPP-BP-1) uses 2-3 alnum. The
# criterion writes "2~3 位" but the tree carries both -- range widened to
# 1..3 so both real families pass. What the anchor DOES catch is the
# criterion's own example, 4-segment IDs like `GWY-CM-P4-1`.
_ID_RE = re.compile(r"^[A-Z]{3}-[A-Z0-9]{1,3}-\d+[a-z]?$")

# A TODO row begins `| \`<ID>\`` optionally followed by a status marker.
_ROW_HEAD_RE = re.compile(r"^\|\s*`([^`]+)`")

# `依赖` column: extract all `\``-quoted IDs in that cell.
_DEP_ID_RE = re.compile(r"`([A-Z]+-[A-Z0-9]+-\d+[a-z]?)`")

# Executable-body path patterns, used by rule 6. Widened three times
# (user rulings 2026-08-08):
#   round 1: scripts/*.py|sh only (original CHK-1-48 wording)
#   round 2: also ros2_ws/*.cc / common/*.h / clang-tidy / gtest / colcon
#            (C++ audits run via gtest inside ros2_ws, not Python)
#   round 3: also test-case identifier prefixes (T-XXX-N / M-N / TC-N /
#            PMT-N) -- when a criterion names its own test cases like
#            "T-CHS-1a: 域 0 下能列出 /IMU", that IS the executable body,
#            the runner is ctest and the exec is that named case.
_EXEC_BODY_PATTERNS = (
    # Path-shaped exec bodies
    re.compile(r"scripts/[A-Za-z0-9_./\-]+\.(?:py|sh)"),
    re.compile(r"ros2_ws/[A-Za-z0-9_./\-]+\.(?:cc|cpp|h|hpp)"),
    re.compile(r"common/[A-Za-z0-9_./\-]+\.(?:cc|cpp|h|hpp)"),
    re.compile(r"tests?/[A-Za-z0-9_./\-]+\.(?:py|cc|cpp)"),
    # Bare tool names -- criterion names the runner directly
    re.compile(r"\bclang-tidy\b"),
    re.compile(r"\bgtest\b"),
    re.compile(r"\bctest\b"),
    re.compile(r"\bcolcon\b"),
    re.compile(r"\bpytest\b"),
    # Test-case identifiers -- when a criterion names T-*/M-*/TC-*/PMT-*
    # the test IS the executable body, the runner is implied by ctest.
    re.compile(r"\bT-[A-Z]+-\d+"),
    re.compile(r"\bM-\d+"),
    re.compile(r"\bTC-\d+"),
    re.compile(r"\bPMT-\d+"),
    # Round-5 (2026-08-08 evening): short-form test IDs (T-N direct
    # digit without letter middle segment) used by CPP-QD-6 as "T-2".
    re.compile(r"\bT-\d+\b"),
    # Test/audit body phrases: 单测, 启动自检, 自检脚本, 元测试, ctest
    # 绿, pytest 绿, ctest, colcon test, 单元检查脚本
    re.compile(r"单测"),
    re.compile(r"启动自检"),
    re.compile(r"自检脚本"),
    re.compile(r"元测试"),
    re.compile(r"单元检查脚本"),
    re.compile(r"审查脚本"),
    # Round-4 (2026-08-08 evening): rule-6 was still flagging judgements
    # that referenced a specific rule number (CRL-N, DDS-N, QC-N, CF-N,
    # ND-N, RT-CN, TF-N, SP-N, QOS-N, BOOT-N, PB-QN, ...) -- these ARE
    # the executable-body references, because CHK-1-50 collects rules
    # into scripts/ci/cxx_discipline_audit.py by number, and each C++
    # audit points at 'CRL-3 审查必须报' meaning the CRL-3 line-item.
    # Recognise any 2-4 uppercase letter + optional -CN + hyphen + digit
    # id as an audit anchor.
    re.compile(r"\b(?:CRL|DDS|QC|CF|ND|RT-C|TF|SP|QOS|BOOT|PB-Q|CB|"
               r"MS|GS|PR|TR|CA|TX|TLS|CPP|QD)-?\d+"),
)

# `目录` column: a backticked path token (e.g. `xbrain/common/...`).
_PATH_TOKEN_RE = re.compile(r"`([^`]+/[^`]+)`")

# File extension shape used by rule-4 to distinguish files from
# directories. Matches `.py` `.yaml` `.cc` `.h` `.md` `.json` `.sh` etc.
# Anchored at end-of-basename, 1-6 alnum extension chars.
_FILE_EXT_RE = re.compile(r"\.[a-zA-Z0-9]{1,6}$")


def _split_row(line):
    """Split a markdown table row on unescaped pipes; return trimmed cells
    or [] on a separator row. Same shape as key_registry._split_row."""
    if not line.startswith("|"):
        return []
    parts = [c.strip() for c in line.split("|")[1:-1]]
    if parts and all(c and set(c) <= set("-: ") for c in parts):
        return []
    return parts


def parse_todo(text):
    """Return a list of row dicts:
      {id, phase, cells (list), lineno}
    Skips separator rows and the phase-heading lines; phase inferred from
    the most recent `## Phase N` heading seen above the row.
    """
    rows = []
    phase = None
    for lineno, line in enumerate(text.split("\n"), 1):
        # Phase heading?
        m = re.match(r"^## Phase (\d+)", line)
        if m:
            phase = int(m.group(1))
            continue
        cells = _split_row(line)
        if not cells:
            continue
        # First cell must start with an ID for us to care about it. Header
        # rows (| ID | 任务 | ...) do NOT satisfy _ROW_HEAD_RE because they
        # do not carry a backticked value.
        hm = _ROW_HEAD_RE.match(line)
        if not hm:
            continue
        # Strip the status token (green-tick / pause-tag ...) from the ID cell.
        raw_id = hm.group(1).strip()
        rows.append({
            "id": raw_id,
            "phase": phase,
            "cells": cells,
            "lineno": lineno,
        })
    return rows


# --------------------------------------------------------------------------
# Individual rules -- each returns a list of finding strings.
# --------------------------------------------------------------------------

def rule_1_id_format(rows):
    """ID must match `_ID_RE` (three segments)."""
    bad = []
    for r in rows:
        if not _ID_RE.match(r["id"]):
            bad.append("L%d: rule-1 ID format: %r" % (r["lineno"], r["id"]))
    return bad


def rule_2_id_unique(rows):
    """No two rows share an ID."""
    seen = {}
    bad = []
    for r in rows:
        if r["id"] in seen:
            bad.append("L%d: rule-2 duplicate ID: %r (first at L%d)"
                       % (r["lineno"], r["id"], seen[r["id"]]))
        else:
            seen[r["id"]] = r["lineno"]
    return bad


def _dep_cell(cells):
    """The dep column is the SECOND-TO-LAST column of the row: | ID | task
    | dir | lang | criterion | deps | blockers |. Some rows may not carry
    a blockers column; index from the end so a variable cell count works."""
    if len(cells) < 6:
        return ""
    return cells[-2]


def rule_3_deps_valid(rows):
    """Every ID referenced in the `依赖` column MUST exist. Cross-phase
    dependencies are ALLOWED (user ruling 2026-08-08): the `## Phase N`
    grouping is an OPEN-WORK-ORDERING hint, not a hard dependency
    constraint -- many BIZ/CFG/GWY items land in P0 while their intra-
    domain siblings sit in P2, and forcing P0 to only depend on P0/P1
    would either move a lot of items around or leave the dep column
    unable to point at the item it truly reads.
    """
    id_to_phase = {r["id"]: r["phase"] for r in rows}
    bad = []
    for r in rows:
        cell = _dep_cell(r["cells"])
        if not cell or cell == "—":     # em-dash sentinel for "no dep"
            continue
        for dep in _DEP_ID_RE.findall(cell):
            if dep not in id_to_phase:
                bad.append("L%d: rule-3 dep %r referenced by %r does not "
                           "exist" % (r["lineno"], dep, r["id"]))
    return bad


def rule_4_no_double_claim(rows):
    """A backticked FILE path (with concrete extension) appearing in the
    `目录` column MUST NOT appear in more than one row's 目录 column.

    Directory paths (ending in `/`, or with no extension) are EXEMPT --
    the same directory routinely hosts many deliverables (arbiter/ ->
    core.py + audit.py + state.py + model.py, one per ID). The intent of
    the rule is 'no two IDs own the same FILE', not 'no two IDs work in
    the same folder'.

    File shape: the last path segment matches _FILE_RE below (a dot
    followed by 1..6 alnum chars). Everything else is treated as a
    directory pointer and skipped.
    """
    claim_lines = defaultdict(list)
    for r in rows:
        if len(r["cells"]) < 3:
            continue
        dir_cell = r["cells"][2]                  # 目录 column
        for path in _PATH_TOKEN_RE.findall(dir_cell):
            # Must have at least one / (skip bare identifiers like
            # `common.spec` or section citations `11 §2.2`).
            if "/" not in path:
                continue
            # Extract the last segment (basename); if it has an ext like
            # `.py` `.h` `.yaml`, treat as a file. Otherwise a directory.
            basename = path.rstrip("/").rsplit("/", 1)[-1]
            if not _FILE_EXT_RE.search(basename):
                # No extension => directory-level pointer, EXEMPT from
                # double-claim (many IDs sharing a directory is fine).
                continue
            claim_lines[path].append((r["id"], r["lineno"]))
    bad = []
    for path, claims in claim_lines.items():
        if len(claims) >= 2:
            names = ", ".join("%s@L%d" % (i, l) for i, l in claims)
            bad.append("rule-4 double-claim: %r -- %s" % (path, names))
    return bad


def _criterion_cell(cells):
    """The 判据 column is column index 4 (0-based): ID, task, dir, lang,
    CRITERION, deps, blockers. Some rows omit trailing columns."""
    if len(cells) < 5:
        return ""
    return cells[4]


def rule_5_has_mutation_marker(rows):
    """Every 判据 cell must carry at least one mutation-style marker.

    Marker vocabulary widened twice (user rulings 2026-08-08):
      round 1 -- add 必须报/必抛/必须拒绝/mutation/must turn red etc.
      round 2 -- add 变红 (short form) plus test-case identifier patterns
                 (TC-N / T-XXX-N / M-XXX / PMT-N / 负例) so a criterion
                 that names its own test cases counts as declaring
                 mutation coverage.

    Also EXEMPT rows whose criterion openly states "not writing code"
    (like CFG-DC-5 which says 本项不写代码 and points at a user ruling
    it is waiting for). These items are by construction not testable via
    a mutant, so demanding a variant marker is a category error.
    """
    # Regexes to search for. Any match anywhere in the criterion cell
    # satisfies rule-5. Compiled once at module load would be nicer, but
    # scan runs at import cost = zero for a lint run, keep it inline for
    # readability.
    markers = (
        # Round-1 vocabulary
        r"变异体", r"必须变红", r"⇒ 红", r"-> red",
        r"必须报", r"必抛", r"必须拒绝", r"必须失败", r"必须不通过",
        r"必须失活", r"必然违反", r"注入.*红",
        r"mutation", r"must turn red", r"must fail",
        # Round-2 additions: short "变红" (used at end of clause) and
        # test-case identifiers that a criterion uses instead of
        # spelling out "variant" per se.
        r"变红",
        r"\bTC-\d+",           # e.g. TC-48 (task DB CHECK pair table)
        r"\bT-[A-Z]+-\d+",     # e.g. T-CHS-1a / T-MODE-1 / T-COV-1
        r"\bM-\d+",            # e.g. M-30 (metric-N)
        r"\bPMT-\d+",          # perception mutation test IDs
        r"负例",                # "负例" (negative test case)
        r"必须落库",             # DB write-through test phrasing
        r"必须判不通过",          # bench negative test phrasing
        # Round-3 (2026-08-08 evening): lint-shape criteria that spell
        # their variant as "-> 恒红" / "-> 恒绿" pair, or as fault-injection
        # ("故障注入" + "必须成立"), or "命中样本" / "必须报出文件与行".
        r"恒红", r"恒绿",
        r"命中样本",
        r"必须成立",
        r"故障注入",
        r"必须报出",
        r"必须逐条报出",
    )
    # Exempt patterns -- when ANY appears in the criterion, the row is
    # skipped entirely (it is a "not-code" item and cannot carry a
    # mutation).
    exempt_patterns = (
        r"本项.*不写代码",
        r"等用户裁定",
        r"由用户给出裁定",
    )
    bad = []
    for r in rows:
        crit = _criterion_cell(r["cells"])
        if not crit:
            continue
        # Exemption first -- avoids running the marker probes on rows
        # that are inherently exempt.
        if any(re.search(p, crit) for p in exempt_patterns):
            continue
        if not any(re.search(m, crit) for m in markers):
            bad.append("L%d: rule-5 %r criterion has no mutation marker"
                       % (r["lineno"], r["id"]))
    return bad


def rule_6_audit_needs_script(rows):
    """`审查必须报` in a criterion must name an executable body -- a
    scripts/*.py|sh path, a ros2_ws / common .cc|.h path (C++ audits
    run via gtest / colcon / clang-tidy), or a bare tool name
    (clang-tidy, gtest, colcon).

    User ruling 2026-08-08: widened from scripts-only because most
    C++ audit items (CPP-CXX-4/5, CPP-DP-4, CPP-CR-*, CPP-QD-*) run
    via gtest inside ros2_ws/, not via a Python script.
    """
    bad = []
    for r in rows:
        crit = _criterion_cell(r["cells"])
        if not crit or "审查必须报" not in crit:
            continue
        if not any(pat.search(crit) for pat in _EXEC_BODY_PATTERNS):
            bad.append("L%d: rule-6 %r says '审查必须报' but names no "
                       "executable body (scripts/*.py|sh, ros2_ws/*.cc|h, "
                       "clang-tidy/gtest)" % (r["lineno"], r["id"]))
    return bad


def rule_7_scan_surface_declared(rows):
    """`全仓` in a criterion must be accompanied by `扫描面`, OR by a
    self-explanatory scope hint.

    User ruling 2026-08-08: '全仓 grep <literal>' is self-declaring --
    the scope IS the whole repo and the literal names WHAT is searched
    for. Same for '全仓 <verb> <literal>' patterns like '全仓不得出现
    PC-3' or '全仓 grep configs/p1_motion.yaml 零命中'. Only require
    an explicit 扫描面 word when '全仓' stands alone without a following
    concrete anchor.

    Accepted self-declarations:
      - "全仓 grep ..."    (grep verb pins the surface as full repo)
      - "全仓不得出现 ..." (nothing may appear, surface implicit = repo)
      - "全仓 ... 零命中"  (zero-match assertion pattern)
      - "全仓 ... 命中数 == 0" (same, other phrasing)
      - "全仓 find ..."    (find verb)
    """
    # Patterns that make '全仓' self-declaring (no separate 扫描面 word
    # needed). Any of these in the SAME cell -> ok.
    # Round-4 addition: allow backticks / hyphens / punctuation between
    # 全仓 and grep (real criteria write `全仓 \`grep -n '...'\``); allow
    # 静态扫描 as an anchor when it appears NEAR 全仓; allow "豁免"
    # declarations (rows explicitly listing which paths are exempt).
    self_declaring = (
        r"全仓\s*[`\-]*\s*grep",
        r"全仓\s*[`\-]*\s*find",
        r"全仓不得出现",
        r"全仓.{0,60}零命中",
        r"全仓.{0,60}命中数",
        r"全仓.{0,60}不存在",
        r"全仓.{0,60}集合",
        r"全仓.{0,30}豁免",
        r"全仓.{0,30}静态扫描",
        r"全仓静态扫描",
        # 静态判据 phrasing usually implies whole-tree scan
        r"静态.{0,20}全仓",
    )
    bad = []
    for r in rows:
        crit = _criterion_cell(r["cells"])
        if not crit or "全仓" not in crit:
            continue
        if "扫描面" in crit:
            continue
        if any(re.search(p, crit) for p in self_declaring):
            continue
        bad.append("L%d: rule-7 %r says '全仓' without declaring '扫描面'"
                   " or a self-declaring anchor (grep/find/零命中/etc)"
                   % (r["lineno"], r["id"]))
    return bad


RULES = [
    ("rule-1", "ID format {3 uppercase}-{2..3 alnum}-{digits}[a-z]?",
     rule_1_id_format),
    ("rule-2", "ID globally unique", rule_2_id_unique),
    ("rule-3", "deps exist + phase not inverted", rule_3_deps_valid),
    ("rule-4", "no delivery-file double-claim", rule_4_no_double_claim),
    ("rule-5", "criterion carries a mutation marker",
     rule_5_has_mutation_marker),
    ("rule-6", "审查必须报 names a scripts/ path", rule_6_audit_needs_script),
    ("rule-7", "全仓 is accompanied by 扫描面",
     rule_7_scan_surface_declared),
]


def run_all(rows):
    """Return dict {rule_name: [findings]}."""
    return {name: scanner(rows) for name, _title, scanner in RULES}


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    ap.add_argument("--todo", default=DEFAULT_TODO,
                    help="path to docs/XBRAIN_V6_TODO.md")
    args = ap.parse_args()

    text = open(args.todo, encoding="utf-8").read()
    rows = parse_todo(text)
    print("scan surface: %s (%d rows parsed)" % (args.todo, len(rows)))
    results = run_all(rows)
    total = 0
    for name, title, _scanner in RULES:
        findings = results[name]
        marker = "ok  " if not findings else "FAIL"
        print("  %s %-8s %s" % (marker, name, title))
        for f in findings:
            print("      " + f)
        total += len(findings)
    print("criterion: zero findings across all rules")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
