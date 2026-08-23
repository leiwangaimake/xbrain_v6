#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: progress.py
Brief: Derive XBRAIN_V6 development progress from XBRAIN_V6_TODO.md, never from ticks

Description:
docs/XBRAIN_V6_TODO.md deliberately carries no checkboxes and no percentages. Hand-kept
progress rots: this project already has live proof of that ("14 has zero hits",
true the day it was written and false a week later; "QC-1 through QC-17", which a
rerun of its own command now contradicts). CLAUDE.md 3.7 forbids writing measured
numbers into documents for exactly this reason.

So progress is evaluated here, at run time, from what actually exists on disk.

*** 2026-08-23: the evidence map is now the primary source.

The criteria in the TODO were written BEFORE the code, so the test paths they
name are PREDICTED filenames, not implementations -- measured: 182 items name a
path and 123 of those files do not exist. Reading the prose therefore reported
almost everything as IN_PROGRESS ("DONE 2/250"), which measured filename
coincidence rather than progress.

docs/TODO_EVIDENCE.tsv maps a task id to the artifact that ACTUALLY verifies it,
one line per item, each line checked by a human and guarded by
tests/meta/test_todo_evidence.py (which fails if a row points at a file that no
longer exists). When an item is in that map this script runs the mapped
artifacts; the prose path below remains only for items not yet mapped.

An item with no evidence row is UNMAPPED -- a distinct state from UNEVALUABLE on
purpose, because the two call for different work: UNMAPPED means "nobody has
recorded what verifies this yet", UNEVALUABLE means "the criterion names nothing
runnable". Neither is ever counted as done.

The one rule that makes this honest: an item whose criterion cannot be evaluated
mechanically is reported as UNEVALUABLE and is NEVER counted as done. A checker
that silently treats "I could not tell" as "finished" would be the always-green
assertion this project keeps catching (CLAUDE.md 3.2 form 1), and it would report
100% on an empty tree.
"""

import os
import re
import subprocess
import sys

ROOT = "/opt/xbrain_v6"
TODO = os.path.join(ROOT, "docs", "XBRAIN_V6_TODO.md")
EVIDENCE = os.path.join(ROOT, "docs", "TODO_EVIDENCE.tsv")

# Table rows look like: | `CFG-CM-1` | title | `common/errors/` | Python | criterion | dep | block |
# IDs carry a lot prefix (CFG/MOT/BIZ/GWY/CPP/INF) because six parallel miners
# reused 20 bare numbers across 46 items -- see docs/XBRAIN_V6_TODO.md S0.3.
# The tail after the id may carry a STATUS MARK before the closing pipe
# (a done/partial/deferred glyph plus an evidence note). Tolerating it is not
# cosmetic: the old pattern demanded a pipe immediately after the backtick, so
# every row ever marked done was invisible to this script -- 252 of 448
# rows, i.e. the parser could see only the unfinished ones. A progress checker
# that structurally cannot see completed work will report near-zero forever,
# which is exactly what "DONE 2/250" was.
ROW = re.compile(r"^\|\s*`([A-Z]{3}-[A-Za-z0-9-]+)`[^|]*\|(.+)$")
# Note: one miner produced a two-segment number (CM-P4-1), so the tail must
# accept hyphens. The earlier stricter pattern silently dropped that one row --
# a parser that skips what it cannot match is a checker that under-reports.
PHASE = re.compile(r"^##\s+(Phase\s+\d+[^\n]*)")
# Paths the criterion names. Only tests/ and scripts/ are treated as runnable.
TEST_PATH = re.compile(r"(tests/[\w/.-]+\.py)")
SCRIPT_PATH = re.compile(r"(scripts/[\w/.-]+\.py)")


def parse():
    """(phase, id, title, dir, criterion) for every row, in document order."""
    rows, phase = [], "(未分相)"
    for line in open(TODO, encoding="utf-8"):
        m = PHASE.match(line)
        if m:
            phase = m.group(1).strip()
            continue
        m = ROW.match(line)
        if not m:
            continue
        cells = [c.strip() for c in m.group(2).split("|")]
        if len(cells) < 5:
            continue
        rows.append({
            "phase": phase,
            "id": m.group(1),
            "title": cells[0],
            "dir": cells[1].strip("` "),
            "criterion": cells[3],
        })
    return rows


def load_evidence():
    """{normalised task id: [artifact paths]} from docs/TODO_EVIDENCE.tsv.

    Ids are normalised to two-digit form because the TODO writes both
    `BIZ-P3-0` and `GWY-P5-01`; comparing raw strings would miss a third of the
    table. Orphan rows (ids not in the TODO) are loaded like any other -- they
    simply never get looked up, and test_todo_evidence.py is what keeps them
    declared rather than silent.

    A missing file is NOT an error here: this script must still run on a
    checkout that has not built the map yet, and it says so in the report
    instead of dying.
    """
    ev = {}
    if not os.path.exists(EVIDENCE):
        return ev
    for line in open(EVIDENCE, encoding="utf-8"):
        if line.startswith("#") or "\t" not in line:
            continue
        cols = line.rstrip("\n").split("\t")
        if len(cols) < 2:
            continue
        ev[norm_id(cols[0])] = [f for f in cols[1].split(";") if f]
    return ev


def norm_id(task_id):
    """BIZ-P3-7 and BIZ-P3-07 are the same item; make them the same key."""
    head, _, tail = task_id.rpartition("-")
    return "%s-%02d" % (head, int(tail)) if tail.isdigit() else task_id


def target_dir(d):
    """The directory an item lands in, with the annotated ones reduced to a path."""
    d = d.split("  ")[0].strip()
    return os.path.join(ROOT, d) if d and not d.startswith(("?", "⚠", "（")) else None


def evaluate(item, run_tests, evidence=None, cache=None):
    """NOT_STARTED / IN_PROGRESS / DONE / UNEVALUABLE / UNMAPPED + a reason.

    DONE requires an artifact that exists AND passes. Existence alone is
    IN_PROGRESS -- a test file that has been created but never run tells us the
    work was started, not that it works.

    The evidence map wins when the item is in it. That is the whole point: the
    prose criterion names a filename someone predicted before writing the code,
    while the map names what actually verifies the item today.
    """
    mapped = (evidence or {}).get(norm_id(item["id"]))
    if mapped:
        missing = [f for f in mapped if not os.path.exists(os.path.join(ROOT, f))]
        if missing:
            # test_todo_evidence.py should have caught this first; if it fires
            # here the map and the tree have drifted since that gate last ran.
            return "IN_PROGRESS", "证据产物缺失: %s" % ", ".join(missing[:2])
        if not run_tests:
            return "IN_PROGRESS", "证据产物齐备(未跑; 加 --run 实跑)"
        for f in mapped:
            if not _artifact_passes(f, cache):
                return "IN_PROGRESS", "证据未通过: %s" % f
        return "DONE", "证据全部通过: %s" % ", ".join(
            os.path.basename(f) for f in mapped[:2])

    tests = TEST_PATH.findall(item["criterion"])
    scripts = SCRIPT_PATH.findall(item["criterion"])
    d = target_dir(item["dir"])

    if not tests and not scripts:
        # Not in the evidence map AND the criterion names nothing runnable.
        # Reported as UNMAPPED rather than UNEVALUABLE when the map exists at
        # all, because then the actionable next step is "add an evidence row",
        # not "rewrite the criterion".
        if evidence:
            return "UNMAPPED", "尚未落证据映射(docs/TODO_EVIDENCE.tsv)"
        # No mechanically checkable artifact named. Say so; do not guess.
        if d and os.path.isdir(d) and os.listdir(d):
            return "UNEVALUABLE", f"目标目录已有内容但判据未点名可执行件：{item['dir']}"
        return "UNEVALUABLE", "判据未点名任何 tests/ 或 scripts/ 可执行件"

    missing = [p for p in tests + scripts if not os.path.exists(os.path.join(ROOT, p))]
    if len(missing) == len(tests + scripts):
        if d and os.path.isdir(d) and os.listdir(d):
            return "IN_PROGRESS", f"目录已建但判据文件全缺：{', '.join(missing[:2])}"
        return "NOT_STARTED", f"判据文件不存在：{', '.join(missing[:2])}"
    if missing:
        return "IN_PROGRESS", f"部分判据文件缺失：{', '.join(missing[:2])}"

    if not run_tests:
        return "IN_PROGRESS", "判据文件齐备（未跑；加 --run 实跑）"

    for p in tests:
        r = subprocess.run(["python3", "-m", "pytest", "-q", os.path.join(ROOT, p)],
                           capture_output=True, text=True, cwd=ROOT)
        if r.returncode != 0:
            return "IN_PROGRESS", f"判据未通过：{p}"
    for p in scripts:
        r = subprocess.run(["python3", os.path.join(ROOT, p)],
                           capture_output=True, text=True, cwd=ROOT)
        if r.returncode != 0:
            return "IN_PROGRESS", f"判据未通过：{p}"
    return "DONE", "判据全部通过"


def _artifact_passes(rel, cache):
    """Run one artifact once per invocation and remember the verdict.

    Cached because several items legitimately share one artifact (test_batch_c
    covers four BIZ-P3 items); without the cache the same suite would be run
    four times and the report would take minutes instead of seconds.
    """
    if cache is not None and rel in cache:
        return cache[rel]
    path = os.path.join(ROOT, rel)
    if rel.startswith("tests/"):
        r = subprocess.run(["python3", "-m", "pytest", "-q", path],
                           capture_output=True, text=True, cwd=ROOT)
    else:
        r = subprocess.run(["python3", path],
                           capture_output=True, text=True, cwd=ROOT)
    ok = r.returncode == 0
    if cache is not None:
        cache[rel] = ok
    return ok


def main():
    run_tests = "--run" in sys.argv
    verbose = "-v" in sys.argv or "--verbose" in sys.argv
    rows = parse()
    evidence = load_evidence()
    cache = {}

    print(f"scan surface: {TODO}")
    print(f"  解析到开发项 {len(rows)} 个"
          + ("（--run：实跑判据）" if run_tests else "（未加 --run：只查文件存在性）"))
    if evidence:
        mapped = sum(1 for it in rows if norm_id(it["id"]) in evidence)
        print(f"  证据映射 {EVIDENCE}: {len(evidence)} 行, 命中本表 {mapped} 项")
    else:
        print(f"  !! 未找到证据映射 {EVIDENCE}; 全部回落到散文判据 "
              f"-- 那条路只能量出文件名巧合率, 见本文件头注")

    by_phase, tally = {}, {}
    for it in rows:
        st, why = evaluate(it, run_tests, evidence, cache)
        it["status"], it["why"] = st, why
        by_phase.setdefault(it["phase"], []).append(it)
        tally[st] = tally.get(st, 0) + 1

    print()
    for ph, its in by_phase.items():
        c = {}
        for i in its:
            c[i["status"]] = c.get(i["status"], 0) + 1
        done, tot = c.get("DONE", 0), len(its)
        print(f"  {ph[:44]:46} {done:3}/{tot:3} DONE"
              f"  · 进行 {c.get('IN_PROGRESS', 0):3}"
              f"  · 未开始 {c.get('NOT_STARTED', 0):3}"
              f"  · 待映射 {c.get('UNMAPPED', 0):3}"
              f"  · ⚠️ 无法求值 {c.get('UNEVALUABLE', 0):3}")

    print("\n  " + " · ".join(f"{k} {v}" for k, v in sorted(tally.items())))

    if verbose:
        print("\n=== 逐项 ===")
        for ph, its in by_phase.items():
            print(f"\n-- {ph}")
            for i in its:
                print(f"  {i['status']:12} {i['id']:8} {i['title'][:44]:46} {i['why'][:50]}")

    print("\n★★★ 口径（🚫 不要改成对自己有利的读法）：")
    print("  · DONE 必须【判据文件存在 ＋ 实跑通过】—— 存在但没跑只算 IN_PROGRESS。")
    print("  · ⚠️ UNMAPPED / UNEVALUABLE 两者【都永远不计入 DONE】。")
    print("    UNMAPPED = 还没人记录【什么验证了这一项】（去补 docs/TODO_EVIDENCE.tsv 一行）；")
    print("    UNEVALUABLE = 判据本身没点名任何可执行件（要改的是判据）。两者要做的事不同，")
    print("    所以分开报；把任一算成完成，就是一条在空目录上也能报 100% 的恒绿判据（3.2 形态①）。")
    print("  · 🚫 本脚本不写任何数字回 docs/XBRAIN_V6_TODO.md（CLAUDE.md 3.7）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
