#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: debt21_scan.py
Brief: Executable body for DBT-1..DBT-6 -- the checks of 21 (measurement debt)

Description:
21-实测与第三方欠账.md exists for one reason: every "measure it later" and
"ask the vendor" item in the package must have a DEFINED default behaviour so it
stops blocking coding. The document is worthless if a row can be added without
that default, or if the default is a guessed number instead of a fail-loud
refusal. These checks are that guarantee, and they live here (not in the
markdown) because CLAUDE.md 3.7 forbids writing measured quantities into prose:
counts rot the moment a human copies them.

DBT-1  every row of the main table fills all four mandatory cells
DBT-2  the "default behaviour" cell is fail-loud, and carries no fail-silent verb
DBT-3  every id in the main table is addressed as <volume> <id> (GL-3 extended)
DBT-4  the "what changes on closure" cell names a volume AND a section anchor
DBT-5  inclusion completeness: every M/V/T id found on the declared scan surface
       appears either in the main table or in the explicit exclusion table
DBT-6  attribution: the volume a row names must really contain that bare id

SCAN SURFACE (declared, not implied -- CLAUDE.md 3.2 form 6):
  DBT-1..DBT-4  rows of 21's main tables whose FIRST CELL carries a bare M/V/T
                id. Prose, headers, separator rows and the exclusion table are
                NOT scanned; that is what keeps DBT-2 from self-harming, since
                the prose explaining the fail-silent word list necessarily
                contains those words. Selection is by BARE id and never by the
                volume-prefixed form DBT-3 checks -- see scanned_rows().
  DBT-5         every file in scan_manifest.members EXCEPT 21 itself.
  DBT-6         the same member set, read as the authority on what each volume
                actually contains.

Usage:
  debt21_scan.py                # run DBT-1..DBT-6
  debt21_scan.py --enumerate    # print the DBT-5 candidate set and exit
  debt21_scan.py --self-test    # inject one mutant per check, each must go red
"""

import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(HERE, "scan_manifest.json")
DEBT_DOC = "21-实测与第三方欠账.md"

# ---------------------------------------------------------------------------
# Id namespaces.
#
# Two digits for M/V is deliberate and is copied verbatim from the package's own
# practice: 13's numbering audit runs `grep -oE '\bV-[0-9]{2}\b'`. One-digit
# M-1/M-2 (10 §4.1 milestones) and V-1/V-2/V-3 (11 §14.6A implementation
# constraints) are DIFFERENT namespaces that happen to share a prefix; matching
# them would drag milestone ids into a measurement-debt registry.
# ---------------------------------------------------------------------------
# The family part must allow digits: T-TIER1-1/2/3 exist. The first draft of
# this regex was [A-Z]{2,6} and silently dropped all three -- an enumerator that
# under-counts is the "undeclared scan surface" failure wearing a script costume,
# and it would have made DBT-5 report a completeness it did not have.
ID_T = re.compile(r"\bT-[A-Z][A-Z0-9]{1,6}-[0-9]{1,2}[a-z]?\b")
ID_M = re.compile(r"\bM-[0-9]{2}\b")
ID_V = re.compile(r"\bV-[0-9]{2}\b")

# 2026-08-05 adversarial review. The three regexes above were the WHOLE id space
# of the first version, and that space silently excluded every family-scoped id:
#   M-PTZ-1/2, M-PAY-1..9, V-A1..V-A11, V-NET-1, V-T1/V-T2
# all exist upstream, all are open measurement debt, and none of them were
# visible to DBT-5 -- which therefore reported a completeness it did not have.
# This is the SAME defect as the T-TIER1 regex bug recorded above, one namespace
# over, and it survived the very review that wrote that note down. Keep both
# notes: a checker whose scan surface can be quietly narrowed is CLAUDE.md 3.2
# form 1 (permanently green) wearing form 6 (undeclared surface) as a costume.
ID_MF = re.compile(r"\bM-[A-Z][A-Z0-9]{1,6}-[0-9]{1,2}[a-z]?\b")
# V families are written both hyphenated (V-NET-1) and glued (V-A11, V-T2).
# \b before V keeps NAV-31 / DEV-02 style tokens out: the preceding character is
# a word char there, so there is no boundary.
ID_VF = re.compile(r"\bV-[A-Z][A-Z0-9]*-?[0-9]{1,2}\b")

ID_ALL = (ID_T, ID_M, ID_V, ID_MF, ID_VF)

# DBT-3's criterion: a scanned row's first cell must address the id as
# `NN` <id>. NOTE this is evaluated INSIDE the check only -- it must never be
# used to select rows, or the check can never fail. See scanned_rows().
VOLUME_PREFIX = re.compile(r"`\d{2}(?:-[AB])?`")

# DBT-2 vocabulary. Kept here, never in the markdown -- a criterion that quotes
# its own trigger strings into the scanned surface can never report zero
# (CLAUDE.md 3.2 form 3).
FAIL_LOUD = [
    "拒绝启动", "拒绝", "rejected", "恒不通过", "恒拒", "零速", "停车",
    "E_", "warn", "fatal", "null", "不启用", "不实现", "抛", "报错",
    "静态判", "按无覆盖", "不得", "禁", "降级", "只读", "不推", "保持现状",
]
FAIL_SILENT = [
    "暂定", "先用", "工程初值", "猜", "姑且", "先按", "临时取值", "TBD",
    "待定", "待确认", "?", "？",
]

# DBT-4: a volume number in backticks plus a section anchor. NUM-4 forbids line
# numbers, so a bare volume number is not enough -- it must carry a §.
CLOSURE_OK = re.compile(r"`\d{2}(?:-[AB])?`[^|]*?§|§[^|]*?`\d{2}(?:-[AB])?`|本册\s*§")

MANDATORY_CELLS = 4


def load_manifest():
    with open(MANIFEST, encoding="utf-8") as fh:
        return json.load(fh)


def split_row(line):
    """Markdown row -> cells, with the leading/trailing empty cell dropped."""
    cells = line.split("|")
    if cells and cells[0].strip() == "":
        cells = cells[1:]
    if cells and cells[-1].strip() == "":
        cells = cells[:-1]
    return [c.strip() for c in cells]


def scanned_rows(text):
    """The DBT-1..DBT-4 scan surface: main-table rows whose FIRST CELL holds an id.

    Returns (lineno, cells). Prose, headings, separator rows and the exclusion
    table are out of surface by construction -- that is what keeps DBT-2 from
    grepping its own explanation (CLAUDE.md 3.2 form 3).

    Selection is by BARE id, deliberately not by the volume-prefixed form. The first version of
    this function selected on that same prefixed form -- the very thing DBT-3 evaluates -- and
    --self-test proved DBT-3 could never go red: stripping the volume prefix
    dropped the row out of the surface instead of failing the check. That is a
    permanently-green assertion (CLAUDE.md 3.2 form 1), and it is the exact class
    of defect this suite exists to catch, so it is recorded here rather than
    quietly fixed. A criterion must never be its own row selector.
    """
    out = []
    in_excl = False
    for n, line in enumerate(text.split("\n"), 1):
        if "DBT-5-EXCLUDE-BEGIN" in line:
            in_excl = True
            continue
        if "DBT-5-EXCLUDE-END" in line:
            in_excl = False
            continue
        if in_excl or not line.startswith("|"):
            continue
        cells = split_row(line)
        if not cells or set(cells[0]) <= set("-: "):
            continue
        if any(rx.search(cells[0]) for rx in ID_ALL):
            out.append((n, cells))
    return out


def excluded_ids(text):
    """Ids parked in the exclusion table, each with a stated reason."""
    out = {}
    in_block = False
    for line in text.split("\n"):
        if "DBT-5-EXCLUDE-BEGIN" in line:
            in_block = True
            continue
        if "DBT-5-EXCLUDE-END" in line:
            in_block = False
            continue
        if not in_block or not line.startswith("|"):
            continue
        cells = split_row(line)
        if len(cells) < 2 or set(cells[0]) <= set("-: "):
            continue
        for rx in ID_ALL:
            for tok in rx.findall(cells[0]):
                if cells[1].strip():
                    out[tok] = cells[1].strip()
    return out


def enumerate_candidates(docs_dir, members):
    """DBT-5 candidate set: every M/V/T id on the declared surface."""
    found = {}
    for fname in members:
        if fname == DEBT_DOC:
            continue
        path = os.path.join(docs_dir, fname)
        if not os.path.exists(path):
            continue
        text = open(path, encoding="utf-8").read()
        for rx in ID_ALL:
            for tok in rx.findall(text):
                found.setdefault(tok, set()).add(fname)
    return found


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def check_dbt1(rows):
    bad = []
    for n, cells in rows:
        body = cells[1:1 + MANDATORY_CELLS]
        if len(body) < MANDATORY_CELLS:
            bad.append((n, cells[0], "missing cells"))
            continue
        for i, c in enumerate(body):
            stripped = re.sub(r"[\s*★⚠️🚫✅⌦]", "", c)
            if not stripped:
                bad.append((n, cells[0], "cell %d empty" % (i + 1)))
    return bad


def check_dbt2(rows):
    bad = []
    for n, cells in rows:
        if len(cells) < 4:
            continue
        cell = cells[3]  # column 4 of the row = "default while unclosed"
        if not any(w in cell for w in FAIL_LOUD):
            bad.append((n, cells[0], "no fail-loud verb"))
        hit = [w for w in FAIL_SILENT if w in cell]
        if hit:
            bad.append((n, cells[0], "fail-silent: " + "/".join(hit)))
    return bad


def check_dbt3(rows):
    bad = []
    for n, cells in rows:
        if not VOLUME_PREFIX.search(cells[0]):
            bad.append((n, cells[0], "id not addressed as <volume> <id>"))
    return bad


def check_dbt4(rows):
    bad = []
    for n, cells in rows:
        if len(cells) < 5:
            continue
        if not CLOSURE_OK.search(cells[4]):
            bad.append((n, cells[0], "closure target lacks volume+section"))
        if re.search(r":\d{3,}\b", cells[4]):
            bad.append((n, cells[0], "closure target cites a line number (NUM-4)"))
    return bad


def volume_texts(docs_dir, members):
    """volume number ('11', '18-B', ...) -> full text of that member file."""
    out = {}
    for fname in members:
        m = re.match(r"(\d{2}(?:-[AB])?)", fname)
        path = os.path.join(docs_dir, fname)
        if m and os.path.exists(path):
            out[m.group(1)] = open(path, encoding="utf-8").read()
    return out


def check_dbt6(rows, vols):
    """DBT-6: the volume a row names must really carry that id.

    Added 2026-08-05 by adversarial review. DBT-3 only proves a volume number is
    PRESENT; it cannot tell `11` T-PTZ-3 from `18` T-PTZ-3, which is exactly the
    hazard 0.5 exists for -- T-PTZ is allocated three times over with crossed
    meanings, so a wrong volume prefix reads as a correct citation. This check
    makes the attribution falsifiable instead of merely well-formed: the named
    volume must be a manifest member AND must contain the bare id.
    """
    bad = []
    for n, cells in rows:
        m = VOLUME_PREFIX.search(cells[0])
        if not m:
            continue          # DBT-3 owns that failure; do not double-report
        vol = m.group(0).strip("`")
        if vol not in vols:
            bad.append((n, cells[0], "volume %s is not a manifest member" % vol))
            continue
        ids = set()
        for rx in ID_ALL:
            ids.update(rx.findall(cells[0]))
        for tok in sorted(ids):
            if not re.search(r"\b%s\b" % re.escape(tok), vols[vol]):
                bad.append((n, cells[0], "%s does not occur in volume %s" % (tok, vol)))
    return bad


def check_dbt5(rows, excl, candidates):
    listed = set()
    for _, cells in rows:
        for rx in ID_ALL:
            listed.update(rx.findall(cells[0]))
    bad = []
    for tok in sorted(candidates):
        if tok not in listed and tok not in excl:
            bad.append((0, tok, "unaccounted (in %s)" % ",".join(sorted(candidates[tok]))))
    return bad


CHECKS = ["DBT-1", "DBT-2", "DBT-3", "DBT-4", "DBT-5", "DBT-6"]


def run(docs_dir, members, debt_text, quiet=False):
    rows = scanned_rows(debt_text)
    excl = excluded_ids(debt_text)
    cands = enumerate_candidates(docs_dir, members)
    vols = volume_texts(docs_dir, members)
    results = {
        "DBT-1": check_dbt1(rows),
        "DBT-2": check_dbt2(rows),
        "DBT-3": check_dbt3(rows),
        "DBT-4": check_dbt4(rows),
        "DBT-5": check_dbt5(rows, excl, cands),
        "DBT-6": check_dbt6(rows, vols),
    }
    if not quiet:
        print("scan surface: %d volumes for DBT-5; %d scanned rows in %s"
              % (len(members) - 1, len(rows), DEBT_DOC))
        print("")
        for k in CHECKS:
            v = results[k]
            print("%-6s violations %-4d %s" % (k, len(v), "PASS" if not v else "FAIL"))
            for n, ident, why in v[:12]:
                print("         line %-6s %-14s %s" % (n or "-", ident[:14], why))
        print("")
        print("criterion is violations == 0 per check; row and candidate counts "
              "drift with normal edits and are context only (GC-1d)")
    return results


# ---------------------------------------------------------------------------
# Self-test: one mutant per check. A check that has never been red has not been
# written (CLAUDE.md 3.3). Each mutant below is a minimal edit that MUST break
# exactly the check it targets.
# ---------------------------------------------------------------------------

MUTANTS = {
    # blank the "default behaviour" cell of the first scanned row
    "DBT-1": lambda t: _mutate_cell(t, 3, ""),
    # replace the default with a guessed value -- the exact thing DBT-2 exists for
    "DBT-2": lambda t: _mutate_cell(t, 3, "暂定 0.0，先用着"),
    # strip the volume prefix from the id cell
    "DBT-3": lambda t: _mutate_cell(t, 0, lambda c: re.sub(r"`\d{2}(?:-[AB])?`", "", c)),
    # point the closure target at a line number instead of a section
    "DBT-4": lambda t: _mutate_cell(t, 4, "11:20877"),
    # delete a whole row -- the id it carried becomes unaccounted
    "DBT-5": lambda t: _drop_first_row(t),
    # re-attribute the row to a volume that provably does not carry its id
    "DBT-6": lambda t: _misattribute_first_row(t),
}


def _first_row_lineno(text):
    # anchor on the surface DBT-1..DBT-4 actually use
    rows = scanned_rows(text)
    if not rows:
        raise SystemExit("self-test: no scanned row found")
    return rows[0][0] - 1   # scanned_rows is 1-indexed, list index is 0-based


def _mutate_cell(text, idx, new):
    lines = text.split("\n")
    n = _first_row_lineno(text)
    cells = lines[n].split("|")
    lead = 1 if cells[0].strip() == "" else 0
    target = lead + idx
    if target >= len(cells):
        raise SystemExit("self-test: row too short to mutate cell %d" % idx)
    cells[target] = (new(cells[target]) if callable(new) else " %s " % new)
    lines[n] = "|".join(cells)
    return "\n".join(lines)


def _misattribute_first_row(text):
    """DBT-6's mutant: re-address the first row to a volume that lacks its id.

    18-A is the smallest member and carries no M/V/T debt ids at all, so it is a
    volume every row is guaranteed to be wrong about. The mutant deliberately
    keeps the row WELL-FORMED (DBT-3 still passes) -- otherwise it would prove
    nothing about DBT-6.
    """
    lines = text.split("\n")
    n = _first_row_lineno(text)
    cells = lines[n].split("|")
    lead = 1 if cells[0].strip() == "" else 0
    cells[lead] = VOLUME_PREFIX.sub("`18-A`", cells[lead], count=1)
    lines[n] = "|".join(cells)
    return "\n".join(lines)


def _drop_first_row(text):
    lines = text.split("\n")
    del lines[_first_row_lineno(text)]
    return "\n".join(lines)


def self_test(docs_dir, members, debt_text):
    ok = True
    base = run(docs_dir, members, debt_text, quiet=True)
    for k in CHECKS:
        if base[k]:
            print("%-6s BASELINE ALREADY RED -- mutant proves nothing" % k)
            ok = False
    print("")
    for k in CHECKS:
        mutated = MUTANTS[k](debt_text)
        res = run(docs_dir, members, mutated, quiet=True)
        went_red = len(res[k]) > len(base[k])
        print("%-6s mutant -> %-4s %s" % (k, "RED" if went_red else "GREEN",
                                          "ok" if went_red else "MUTANT NOT CAUGHT"))
        if not went_red:
            ok = False
    print("")
    print("a check that its own mutant does not turn red is not written (3.3)")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--enumerate", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    man = load_manifest()
    docs_dir = man["docs_dir"]
    members = [m["file"] for m in man["members"]]

    if DEBT_DOC not in members:
        raise SystemExit(
            "%s is not in scan_manifest.members -- refusing to run.\n"
            "An unregistered document is invisible to SEC-12, and a checker that "
            "silently accepts that would be exactly the 'undeclared scan surface' "
            "failure this package keeps hitting." % DEBT_DOC)

    if args.enumerate:
        cands = enumerate_candidates(docs_dir, members)
        for tok in sorted(cands):
            print("%-14s %s" % (tok, ",".join(sorted(cands[tok]))))
        return 0

    debt_text = open(os.path.join(docs_dir, DEBT_DOC), encoding="utf-8").read()
    if args.self_test:
        return 0 if self_test(docs_dir, members, debt_text) else 1

    res = run(docs_dir, members, debt_text)
    return 1 if any(res.values()) else 0


if __name__ == "__main__":
    sys.exit(main())
