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

# Table rows look like: | `CFG-CM-1` | title | `common/errors/` | Python | criterion | dep | block |
# IDs carry a lot prefix (CFG/MOT/BIZ/GWY/CPP/INF) because six parallel miners
# reused 20 bare numbers across 46 items -- see docs/XBRAIN_V6_TODO.md S0.3.
ROW = re.compile(r"^\|\s*`([A-Z]{3}-[A-Za-z0-9-]+)`\s*\|(.+)$")
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


def target_dir(d):
    """The directory an item lands in, with the annotated ones reduced to a path."""
    d = d.split("  ")[0].strip()
    return os.path.join(ROOT, d) if d and not d.startswith(("?", "⚠", "（")) else None


def evaluate(item, run_tests):
    """NOT_STARTED / IN_PROGRESS / DONE / UNEVALUABLE, plus a one-line reason.

    DONE requires a named test file that exists AND passes. Existence alone is
    IN_PROGRESS -- a test file that has been created but never run tells us the
    work was started, not that it works.
    """
    tests = TEST_PATH.findall(item["criterion"])
    scripts = SCRIPT_PATH.findall(item["criterion"])
    d = target_dir(item["dir"])

    if not tests and not scripts:
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


def main():
    run_tests = "--run" in sys.argv
    verbose = "-v" in sys.argv or "--verbose" in sys.argv
    rows = parse()

    print(f"scan surface: {TODO}")
    print(f"  解析到开发项 {len(rows)} 个"
          + ("（--run：实跑判据）" if run_tests else "（未加 --run：只查文件存在性）"))

    by_phase, tally = {}, {}
    for it in rows:
        st, why = evaluate(it, run_tests)
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
    print("  · ⚠️ UNEVALUABLE【永远不计入 DONE】—— 判据没点名可执行件，机器没法判。")
    print("    把它算成完成，就是一条在空目录上也能报 100% 的恒绿判据（CLAUDE.md 3.2 形态①）。")
    print("  · 🚫 本脚本不写任何数字回 docs/XBRAIN_V6_TODO.md（CLAUDE.md 3.7）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
