#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cov45_scan.py
Brief: T-COV-1 -- 13 S5's 45-row coverage matrix against the code

Description:
13 S11.1 T-COV-1 asks for "13 S5 的 45 行逐行对代码里的 not_implemented 与
dispatch 表比对, 计数必须一致: 45 = 23 + 11 + 11". Until this script existed
that item had no executable, and the two sides could drift in either direction:
a row re-marked in the doc with nothing changed in the code, or a refusal
removed from the code with the doc still claiming it.

What each check answers, and why it can fail:

  COV-1  The matrix still has 45 rows, across S5.1 (C-01..C-08), S5.2
         (M-01..M-10), S5.3 (G-01..G-05), S5.4 (six axes), S5.5 (S-01..S-12)
         and S5.6 (P-01..P-04). A row added or lost without S5.7 following is
         the drift T-COV-1 names first.

  COV-2  The per-marker totals match S5.7's own table AND its 45 = 23/11/11
         line. S5.7 is written by hand; v0.2 records that two rows were once
         classified there opposite to their own row marker, and that
         "T-COV-1 按原表必然判失败".

  COV-3  Every motion state marked forbidden in S5.2 is refusable in the code,
         and nothing else is. Refusable means EITHER absent from rt_parse's
         commandable table (structural) OR listed in
         configs/quadruped.yaml motion.not_implemented.motion_states.

  COV-4  The same for gaits (S5.3). Both mechanisms are in play here: G-02
         (stair_standard) is listed in not_implemented.gaits while G-03
         (platform) is simply absent from the commandable table, and 13 GS-3
         keeps stair_standard recognisable on the READ-BACK side regardless.

  COV-5  S5.1's forbidden control ASDU is in not_implemented.commands.
         illumination is deliberately NOT expected in the 45: 13 S5's boundary
         note says it is a field of 11 S9.4.1, not a row of the guide's ASDU
         table, so it is counted on the interface side instead.

What this script does NOT do: it does not judge whether a row's marker is
RIGHT. That is a reading of the vendor guide, and a script that re-derived it
would be comparing the document with itself (CLAUDE.md 3.2, "定义式冒充实测
结论"). It only holds the doc, the config and the parser tables to each other.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DOC = None
for name in os.listdir(os.path.join(ROOT, "docs")):
    if name.startswith("13-"):
        DOC = os.path.join(ROOT, "docs", name)
CFG = os.path.join(ROOT, "configs", "quadruped.yaml")
RT_PARSE = os.path.join(ROOT, "ros2_ws", "quadruped", "src", "rt_parse.cc")

# Escaped, not literal: CLAUDE.md 2.2 keeps these symbols out of source, and
# the doc they match is markdown where 2.2 explicitly allows them. The escape
# is the same codepoint the matrix uses.
OK = "\u2705"        # WHITE HEAVY CHECK MARK
WAIT = "⚠"
NO = "\U0001f6ab"


def read(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def section(text, start, end):
    """The slice between two headings, so a row cannot be counted twice."""
    i = text.index(start)
    j = text.index(end, i)
    return text[i:j]


def marker(cell):
    """The PRIMARY marker of a row's 我方 cell.

    S5's rule 1 is "按该行"我方"列的主标记归类", and the primary marker is
    the FIRST of the three that appears -- a cell reading "ok 部分" is an
    implemented row with a qualifier, not a half of each.
    """
    for ch in cell:
        if ch in (OK, WAIT, NO):
            return ch
    return None


def rows_with_ids(block):
    out = []
    for line in block.split("\n"):
        if not line.startswith("|"):
            continue
        m = re.match(r"\|\s*(C-\d+|M-\d+|G-\d+|S-\d+|P-\d+)\b", line)
        if not m:
            continue
        cells = [c.strip() for c in line.split("|")]
        out.append((m.group(1), cells))
    return out


def axis_rows(block):
    out = []
    for line in block.split("\n"):
        if not line.startswith("|"):
            continue
        m = re.match(r"\|\s*`(X|Y|Yaw|Z|Roll|Pitch)`", line)
        if not m:
            continue
        cells = [c.strip() for c in line.split("|")]
        out.append((m.group(1), cells))
    return out


def yaml_list(text, key_path):
    """The scalar entries of one nested yaml list, by indentation.

    A real parser is avoided on purpose: this file is the SOURCE config, which
    carries comments and inline flow maps that a loader would normalise away --
    and the thing being checked is what a human reading the file sees.
    """
    lines = text.split("\n")
    depth = 0
    want = key_path.split(".")
    idx = 0
    base_indent = -1
    for n, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        if idx < len(want) and stripped.split(":")[0].strip() == want[idx]:
            if idx == len(want) - 1:
                base_indent = indent
                start = n + 1
                break
            idx += 1
    if base_indent < 0:
        return None
    out = []
    for line in lines[start:]:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        if indent <= base_indent:
            break
        if stripped.startswith("- "):
            out.append(stripped[2:].strip().strip('"').strip("'"))
    return out


def yaml_flow_list(text, key):
    """One inline `key: ["a", "b"]` list. The source config writes these lists
    in flow style, and reading only block style silently returned an empty
    list -- which every membership check then passed."""
    m = re.search(r"^\s*%s:\s*\[([^\]]*)\]" % re.escape(key), text, re.M)
    if not m:
        return None
    return [v.strip().strip('"').strip("'") for v in m.group(1).split(",")
            if v.strip()]


# Which header names the classifying column, per section. NOT one name and not
# a position: S5.5's classifier is 状态 (its 我方发布 column holds a key name),
# and S5.2 / S5.3 carry a 可下发 column BEFORE 我方 that has markers of its own
# -- reading "the first marker on the row" makes M-01 look unimplemented when
# its 可下发 is no and its 我方 says 只读.
CLASS_COLUMN = {
    "S5.1": "我方", "S5.2": "我方", "S5.3": "我方",
    "S5.4": "我方", "S5.5": "状态", "S5.6": "我方",
}


def our_column(block, want):
    for line in block.split("\n"):
        if line.startswith("|") and want in line:
            cells = [c.strip() for c in line.split("|")]
            for n, c in enumerate(cells):
                if c == want:
                    return n
    return None


def parser_table(text, name):
    """The names in one rt_parse NameValue table."""
    i = text.index("constexpr NameValue %s[] = {" % name)
    j = text.index("};", i)
    return re.findall(r'\{"([a-z_0-9]+)"', text[i:j])


def run(doc, cfg, parse, mm, quiet=False):
    """Every check against four texts. Taking them as ARGUMENTS rather than
    reading them here is what lets --self-test inject a mutant without touching
    the repository: the previous shape copied the real files aside, mutated
    them in place and restored on EXIT, which is the same hazard the C++ runner
    carries -- and an interrupted run leaves a mutated file behind."""
    findings = []

    blocks = {
        "S5.1": section(doc, "## 5. 运动功能全集与覆盖矩阵", "### 5.2"),
        "S5.2": section(doc, "### 5.2", "### 5.3"),
        "S5.3": section(doc, "### 5.3", "### 5.4"),
        "S5.4": section(doc, "### 5.4", "### 5.5"),
        "S5.5": section(doc, "### 5.5", "### 5.6"),
        "S5.6": section(doc, "### 5.6", "### 5.7"),
    }

    # ---- COV-1: 45 rows, in the sections S5.7 counts them in ---------------
    counted = {}
    for sec, blk in blocks.items():
        if sec == "S5.4":
            counted[sec] = axis_rows(blk)
        else:
            counted[sec] = rows_with_ids(blk)
    total = sum(len(v) for v in counted.values())
    per_sec = ", ".join("%s=%d" % (k, len(counted[k])) for k in sorted(counted))
    if total != 45:
        findings.append(("COV-1", "the matrix has %d rows, not 45 (%s)"
                         % (total, per_sec)))

    # ---- COV-2: per-marker totals against S5.7 -----------------------------
    tally = {OK: 0, WAIT: 0, NO: 0}
    unmarked = []
    our_col = {sec: our_column(blk, CLASS_COLUMN[sec])
               for sec, blk in blocks.items()}
    for sec, blk in blocks.items():
        if our_col[sec] is None:
            findings.append(("COV-2", "%s has no %s column header"
                             % (sec, CLASS_COLUMN[sec])))
    for sec, rws in counted.items():
        col = our_col[sec]
        for rid, cells in rws:
            cell = cells[col] if col is not None and col < len(cells) else ""
            m = marker(cell)
            # A 我方 cell reading 只读 / 内部 with no marker is counted as
            # "待外部输入" -- S5.7's own parenthetical says so verbatim:
            # "4(M-01/M-02/M-04/M-05 只读或内部)" under the wait column. This
            # is transcription of the doc's stated mapping, not a rule invented
            # here; the rows that DO carry a marker are where the check bites.
            if m is None and ("只读" in cell or "内部" in cell):
                m = WAIT
            # RULE 2, verbatim: ""ok 部分"且其未决点已在本文单列 wait 者
            # (如 C-04 的 ActionParam) 计入 wait".
            #
            # Mechanised as: the primary marker is ok 部分 AND the row carries
            # a wait of its own somewhere else. That distinguishes C-04 (whose
            # row says "wait ActionParam 取值表未给 (V-45)") from C-03, which is
            # also ok 部分 but whose per-value rows are counted separately and
            # carries no wait.
            #
            # *** This rule is why the first draft of this script reported
            # 24/10/11 against S5.7's 23/11/11 and I took the DOC to be wrong.
            # It was not: rule 2 was written down and the script had only
            # implemented rule 1. Encoding every stated rule is what "口径写死,
            # 便于复核" asks for.
            if m == OK and "部分" in cell and WAIT in " ".join(cells):
                m = WAIT
            if m is None:
                unmarked.append("%s %s" % (sec, rid))
            else:
                tally[m] += 1
    if unmarked:
        findings.append(("COV-2", "rows with no 我方 marker: " + ", ".join(unmarked)))
    # NOT "---": a markdown table separator row contains it, so the slice
    # ended one line into the table and the 合计 row was never seen.
    s57 = section(doc, "### 5.7", "## 6.")
    m = re.search(r"\*\*合计\*\*\s*\|\s*\*\*45\*\*\s*\|[^|]*?(\d+)[^|]*\|[^|]*?(\d+)[^|]*\|[^|]*?(\d+)",
                  s57)
    if not m:
        findings.append(("COV-2", "S5.7's 合计 row could not be read; its shape changed"))
    else:
        want = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
        got = (tally[OK], tally[WAIT], tally[NO])
        if want != got:
            findings.append(("COV-2",
                             "row markers tally %s but S5.7 claims %s (ok/wait/no)"
                             % (got, want)))

    # ---- COV-3 / COV-4 / COV-5: the doc's refusals against the code --------
    ni_states = yaml_flow_list(cfg, "motion_states") or []
    ni_gaits = yaml_flow_list(cfg, "gaits") or []
    ni_cmds = yaml_flow_list(cfg, "commands") or []
    commandable_states = parser_table(parse, "kMotionStates")
    commandable_gaits = parser_table(parse, "kGaits")

    # Which doc rows are forbidden, and whether the CODE can refuse them.
    #
    # Matched on the row's numeric VALUE, not on its prose: the rows name the
    # states and gaits in Chinese while the code and config use identifiers,
    # and a Chinese-to-identifier table here would be a third place for that
    # mapping to live. The value is the one token both sides already share.
    def value_of(cells):
        # Cell by cell. Joining them first loses the boundaries, and the value
        # column of S5.2 is a bare number -- indistinguishable from any other
        # digit on the row once the pipes are gone.
        for c in cells:
            h = re.search(r"0x([0-9a-fA-F]{4})", c)
            if h:
                return int(h.group(1), 16)
        for c in cells:
            t = c.replace("*", "").replace("\u2212", "-").strip()
            if re.match(r"^-?\d+$", t):
                return int(t)
        return None

    def forbidden(sec):
        out = []
        col = our_col[sec]
        for rid, cells in counted[sec]:
            cell = cells[col] if col is not None and col < len(cells) else ""
            if marker(cell) == NO:
                out.append((rid, value_of(cells)))
        return out

    # rt_parse's commandable tables, resolved to VALUES through the constants
    # mode_machine.h names. Reading the constants rather than repeating the
    # numbers keeps this from becoming a second copy of 13 S6.2.
    consts = dict(re.findall(
        r"inline constexpr std::int64_t (k\w+) = (0x[0-9a-fA-F]+|\d+);", mm))

    def resolve(tbl_name):
        i = parse.index("constexpr NameValue %s[] = {" % tbl_name)
        j = parse.index("};", i)
        out = {}
        for nm, val in re.findall(r'\{"([a-z_0-9]+)",\s*([^}]+)\}', parse[i:j]):
            val = val.strip()
            if val in consts:
                out[nm] = int(consts[val], 0)
            elif re.match(r"^(0x[0-9a-fA-F]+|\d+)$", val):
                out[nm] = int(val, 0)
        return out

    cmd_states = resolve("kMotionStates")
    cmd_gaits = resolve("kGaits")

    for rid, val in forbidden("S5.2"):
        if val is None:
            findings.append(("COV-3", "%s is no but its row carries no value" % rid))
        elif val in cmd_states.values():
            findings.append(("COV-3",
                             "%s is no in S5.2 and %d is still commandable in "
                             "rt_parse" % (rid, val)))
    for n in ni_states:
        if n in cmd_states:
            findings.append(("COV-3",
                             "not_implemented lists %s while rt_parse still "
                             "accepts it as commandable" % n))

    for rid, val in forbidden("S5.3"):
        if val is None:
            findings.append(("COV-4", "%s is no but its row carries no value" % rid))
            continue
        # Refusable either structurally (absent from the commandable table) or
        # by name through not_implemented.gaits.
        by_name = [n for n in ni_gaits if cmd_gaits.get(n) == val]
        if val in cmd_gaits.values() and not by_name:
            findings.append(("COV-4",
                             "%s is no in S5.3 and 0x%04x is commandable in "
                             "rt_parse without being in not_implemented.gaits"
                             % (rid, val)))

    # ---- COV-5: the forbidden control ASDU, and the boundary row -----------
    #
    # C-05 (normalized_axis) is the one no in S5.1. illumination is NOT a row
    # of the 45 -- S5.7's rule 6 puts it on 11 S9.13's interface side, because
    # it is a FIELD of 11 S9.4.1 rather than a line of the guide's ASDU table.
    # The config is therefore its only record, which is why its absence is
    # reported here even though no matrix row would notice.
    #
    # *** These two checks were silently LOST in an edit that replaced the
    # block around them, and the COV-5 line kept printing PASS on an empty set
    # of findings -- a check that had stopped being a check. The mutation
    # script caught it: "not_implemented.commands loses C-05" SURVIVED.
    if "normalized_axis" not in ni_cmds:
        findings.append(("COV-5",
                         "C-05 is no in S5.1 but normalized_axis is not in "
                         "not_implemented.commands"))
    if "illumination" not in ni_cmds:
        findings.append(("COV-5",
                         "illumination left not_implemented.commands; 13 V-47 "
                         "refuses it and S5.7 rule 6 keeps it out of the 45, so "
                         "the config is its only record"))

    if not quiet:
        print("scan surface: %s" % per_sec)
        print("row markers: ok=%d wait=%d no=%d  total=%d"
              % (tally[OK], tally[WAIT], tally[NO], total))
        for cid in CHECKS:
            mine = [f for f in findings if f[0] == cid]
            print("%-6s violations %-4d %s"
                  % (cid, len(mine), "PASS" if not mine else "FAIL"))
            for _, msg in mine:
                print("         %s" % msg)
        print("criterion is violations == 0 per check; T-COV-1 (13 S11.1)")
    return findings


CHECKS = ("COV-1", "COV-2", "COV-3", "COV-4", "COV-5")

# One mutation per check, each of which must break exactly that check. A check
# that cannot go red is not a check (CLAUDE.md 3.3).
#
# *** This found a real hole on its first run: COV-5's two assertions had been
# lost in an edit that replaced the block around them, and the line kept
# printing PASS over an empty finding set. Everything else was green, and
# nothing else would have said so.
MUTANTS = {
    # A row vanishes from the matrix.
    "COV-1": lambda d, c, p, m: (
        d[:d.index("| P-04")] + d[d.index("\n", d.index("| P-04")) + 1:], c, p, m),
    # S5.7 claims a total the rows do not support.
    "COV-2": lambda d, c, p, m: (
        d.replace("| **合计** | **45** | \u2605 **23** |",
                  "| **合计** | **45** | \u2605 **24** |", 1), c, p, m),
    # A state 13 S5.2 forbids becomes commandable again.
    "COV-3": lambda d, c, p, m: (
        d, c,
        p.replace('    {"rl_control", kCommandMotionStateRlControl},',
                  '    {"rl_control", kCommandMotionStateRlControl},\n'
                  '    {"zero_cal", 5},', 1), m),
    # not_implemented.gaits loses the gait S5.3 forbids.
    "COV-4": lambda d, c, p, m: (
        d, c.replace('gaits: ["stair_standard"]', "gaits: []", 1), p, m),
    # not_implemented.commands loses C-05.
    "COV-5": lambda d, c, p, m: (
        d, c.replace('commands: ["normalized_axis", "illumination"]',
                     'commands: ["illumination"]', 1), p, m),
}


def self_test(doc, cfg, parse, mm):
    ok = True
    base = run(doc, cfg, parse, mm, quiet=True)
    for k in CHECKS:
        if [f for f in base if f[0] == k]:
            print("%-6s BASELINE ALREADY RED -- mutant proves nothing" % k)
            ok = False
    print("")
    for k in CHECKS:
        d2, c2, p2, m2 = MUTANTS[k](doc, cfg, parse, mm)
        res = run(d2, c2, p2, m2, quiet=True)
        before = len([f for f in base if f[0] == k])
        after = len([f for f in res if f[0] == k])
        red = after > before
        print("%-6s mutant -> %-5s %s"
              % (k, "RED" if red else "GREEN", "ok" if red else "MUTANT NOT CAUGHT"))
        if not red:
            ok = False
    print("")
    print("a check that its own mutant does not turn red is not written (3.3)")
    return ok


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    doc = read(DOC)
    cfg = read(CFG)
    parse = read(RT_PARSE)
    mm = read(os.path.join(ROOT, "ros2_ws", "quadruped", "include",
                           "quadruped", "mode_machine.h"))
    if args.self_test:
        return 0 if self_test(doc, cfg, parse, mm) else 1
    return 1 if run(doc, cfg, parse, mm) else 0


if __name__ == "__main__":
    sys.exit(main())
