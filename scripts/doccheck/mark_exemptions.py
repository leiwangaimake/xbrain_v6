#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: mark_exemptions.py
Brief: Add visible line-level SEC-12 exemption markers, one justified line at a time

Description:
After the invented implicit exemption was removed from sec12_scan.py on 2026-08-04,
44 real violations surfaced. Most are prose that quotes a superseded value in order
to say it is superseded ("原文：patrol 5 m/s ... U54 已改判") -- those are history,
not live transcription, and the package's own mechanism for them is a marker written
ON the line so any reader can see the line was excused.

This script adds those markers, but deliberately NOT in bulk:
  - it classifies each line by what the line itself says (沿革/订正 vs 墓碑/销账),
  - it writes the reason into the marker, so the excuse is auditable,
  - and it refuses to touch codes that are genuinely undecided.

E_STORAGE_CORRUPT and E_UNSUPPORTED are the second kind. They are not in the closed
set and there is no adjudication closing them (SEC-12 ③, EC-2u > 0, open since before
this session). Exempting them would be the exact move this whole exercise exists to
prevent, so they stay red until someone decides.

Usage:
  mark_exemptions.py --dry-run
  mark_exemptions.py --apply
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sec12_scan as S  # noqa: E402

# Codes with no closed-set home and no adjudication. Never exempt these.
UNDECIDED = ("E_STORAGE_CORRUPT", "E_UNSUPPORTED")

# What the line says about itself -> why it may be excused.
CATEGORIES = [
    (("原文", "原表", "原写", "原值", "原：", "旧值", "U33", "已作废", "作废", "不再成立"),
     "沿革句：引用的是已被取代的旧值，本行的作用正是声明它已作废"),
    (("订正", "改判", "重定标", "下调", "同批改", "已改"),
     "订正说明：本行记录的是一次改动，被引数值是改动前的值"),
    (("墓碑", "已关闭", "销账", "已裁", "差集", "回流"),
     "墓碑/销账记录：被引码名是被清点的对象，不是活体转录"),
    (("判据", "豁免", "扫描面", "自伤"),
     "判据句：本行定义或讨论检查规则，含被检字串是必然的"),
]


def classify(line):
    for keys, reason in CATEGORIES:
        if any(k in line for k in keys):
            return reason
    return None


def collect(docs_dir, files, markers):
    """Every live violation, keyed by (file, lineno), with the checks that fired."""
    hits = {}
    for name in ("SP-8", "EC-2", "UL-6"):
        _, live = S.CHECKS[name](docs_dir, files, markers)
        for f, ln, a, _b, _txt in live:
            hits.setdefault((f, ln), {"checks": set(), "tokens": set()})
            hits[(f, ln)]["checks"].add(name)
            hits[(f, ln)]["tokens"].add(str(a))
    return hits


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--apply", action="store_true")
    args = ap.parse_args()

    man = S.load_manifest()
    docs_dir = man["docs_dir"]
    files = [m["file"] for m in man["members"]]
    markers = man["exempt_markers"]

    hits = collect(docs_dir, files, markers)
    plan, refused, unclassified = [], [], []
    for (name, ln), info in sorted(hits.items()):
        line = open(os.path.join(docs_dir, name), encoding="utf-8").read().split("\n")[ln - 1]
        if any(u in info["tokens"] for u in UNDECIDED):
            refused.append((name, ln, sorted(info["tokens"])))
            continue
        reason = classify(line)
        if not reason:
            unclassified.append((name, ln, sorted(info["checks"]), line.strip()[:90]))
            continue
        tag = " · ".join(sorted(c + " 豁免" for c in info["checks"]))
        plan.append((name, ln, "（★ %s —— %s）" % (tag, reason)))

    print("live violations on %d distinct lines" % len(hits))
    print("  will mark    %d" % len(plan))
    print("  refuse       %d   (undecided codes -- must stay red)" % len(refused))
    print("  unclassified %d   (line does not say why it may be excused)" % len(unclassified))
    for name, ln, toks in refused:
        print("      REFUSE %s:%d  %s" % (name[:8], ln, ", ".join(toks)))
    for name, ln, checks, txt in unclassified:
        print("      MANUAL %s:%d  %s  %s" % (name[:8], ln, ",".join(checks), txt))

    if args.apply and plan:
        by_file = {}
        for name, ln, note in plan:
            by_file.setdefault(name, []).append((ln, note))
        for name, items in by_file.items():
            path = os.path.join(docs_dir, name)
            lines = open(path, encoding="utf-8").read().split("\n")
            for ln, note in items:
                lines[ln - 1] = lines[ln - 1].rstrip() + note
            open(path, "w", encoding="utf-8").write("\n".join(lines))
        print("\napplied %d markers across %d files" % (len(plan), len(by_file)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
