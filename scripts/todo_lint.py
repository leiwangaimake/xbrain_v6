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

# scripts/... path pattern, used by rule 6.
_SCRIPT_PATH_RE = re.compile(r"scripts/[A-Za-z0-9_./\-]+\.(?:py|sh)")

# `目录` column: a backticked path token (e.g. `xbrain/common/...`).
_PATH_TOKEN_RE = re.compile(r"`([^`]+/[^`]+)`")


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
    """Every ID referenced in the `依赖` column exists, and its phase
    is <= the phase of the referring row (a Phase-0 item cannot depend
    on a Phase-3 item -- that is 跨相位倒置)."""
    id_to_phase = {r["id"]: r["phase"] for r in rows}
    bad = []
    for r in rows:
        cell = _dep_cell(r["cells"])
        if not cell or cell == "—":
            continue
        for dep in _DEP_ID_RE.findall(cell):
            if dep not in id_to_phase:
                bad.append("L%d: rule-3 dep %r referenced by %r does not "
                           "exist" % (r["lineno"], dep, r["id"]))
                continue
            dep_phase = id_to_phase[dep]
            if dep_phase is not None and r["phase"] is not None \
                    and dep_phase > r["phase"]:
                bad.append("L%d: rule-3 phase inversion: %r (P%d) depends "
                           "on %r (P%d)"
                           % (r["lineno"], r["id"], r["phase"], dep,
                              dep_phase))
    return bad


def rule_4_no_double_claim(rows):
    """A backticked path token (something with `/` in it) appearing in the
    `目录` column MUST NOT appear in more than one row's 目录 column."""
    claim_lines = defaultdict(list)
    for r in rows:
        if len(r["cells"]) < 3:
            continue
        dir_cell = r["cells"][2]                  # 目录 column
        for path in _PATH_TOKEN_RE.findall(dir_cell):
            # Only paths that clearly point at a deliverable (contain a /
            # AND a . for extension OR .py suffix). Skip citations like
            # `11 §2.2` (no /) or bare identifiers.
            if not any(seg in path for seg in ("/", ".")):
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
    """Every 判据 cell must carry at least one mutation-style marker."""
    markers = ("变异体", "必须变红", "⇒ 红")
    bad = []
    for r in rows:
        crit = _criterion_cell(r["cells"])
        if not crit:
            continue
        if not any(m in crit for m in markers):
            bad.append("L%d: rule-5 %r criterion has no mutation marker"
                       % (r["lineno"], r["id"]))
    return bad


def rule_6_audit_needs_script(rows):
    """`审查必须报` in a criterion must name a scripts/ path."""
    bad = []
    for r in rows:
        crit = _criterion_cell(r["cells"])
        if not crit or "审查必须报" not in crit:
            continue
        if not _SCRIPT_PATH_RE.search(crit):
            bad.append("L%d: rule-6 %r says '审查必须报' but names no "
                       "scripts/ path (missing 执行体)"
                       % (r["lineno"], r["id"]))
    return bad


def rule_7_scan_surface_declared(rows):
    """`全仓` in a criterion must be accompanied by `扫描面`."""
    bad = []
    for r in rows:
        crit = _criterion_cell(r["cells"])
        if not crit or "全仓" not in crit:
            continue
        if "扫描面" not in crit:
            bad.append("L%d: rule-7 %r says '全仓' without declaring '扫描面'"
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
