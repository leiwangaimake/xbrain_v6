#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: verify_merge_claims.py
Brief: Check whether "已合入" patch blocks really have a copy outside themselves

Description:
Volume 11 was used as a patch queue. Blocks left over from that period carry
headings like "已合入(合入记录,非现行规范)", which reads as "safe to delete".
Measured on 2026-08-04: that label is not reliable. The block titled
"[块 E]插入 ---- 11 新增 §9A.13 ... 已合入" spans 4655 lines and contains the ONLY
copy of §9A.13 (可疑判定的输入契约, PER-36) -- the text was accepted in place and
never moved into the numbered body, so deleting the block deletes live contract.

A naive check "does §9A.13 exist?" passes, because the match is found inside the
very block under test -- the criterion is satisfied by the thing it validates.
This script searches OUTSIDE the candidate span instead, and reports:

  REDUNDANT  target section exists elsewhere -> the block really is a record
  ONLY-COPY  target section exists only here -> deleting it loses spec
  NO-TARGET  heading names no target section -> needs a human read

Usage:
  verify_merge_claims.py
"""

import glob
import os
import re
import sys

DOCS_DIR = "/opt/xbrain_v6/docs"

DEAD_HEAD = re.compile(r"已合入|已撤销|非现行规范|合入记录|已被正文超越|本小节【作废】")
LIVE_HEAD = re.compile(r"yaml` 全文|全文（")
HEADING = re.compile(r"^(#{2,4}) .*$", re.M)
# Section numbers a heading claims to have merged into: §9A.13, §4.6, §10.4 ...
SECTION_REF = re.compile(r"§\s*([0-9]+[A-Z]?(?:\.[0-9]+)*[A-Z]?)")


def candidates(text):
    heads = [(m.start(), len(m.group(1)), m.group(0)) for m in HEADING.finditer(text)]
    out = []
    for i, (pos, lvl, head) in enumerate(heads):
        if not DEAD_HEAD.search(head) or LIVE_HEAD.search(head):
            continue
        end = len(text)
        for pos2, lvl2, _ in heads[i + 1:]:
            if lvl2 <= lvl:
                end = pos2
                break
        out.append((pos, end, head.strip()))
    return out


def heading_positions(text, number):
    """Character offsets of every heading that introduces the given section number."""
    pat = re.compile(r"(?m)^#{2,5}\s*\**\s*" + re.escape(number) + r"(?![0-9.])")
    return [m.start() for m in pat.finditer(text)]


def main():
    verdicts = {"REDUNDANT": 0, "ONLY-COPY": 0, "NO-TARGET": 0}
    only_copies = []
    for path in sorted(glob.glob(os.path.join(DOCS_DIR, "*.md"))):
        name = os.path.basename(path)
        text = open(path, encoding="utf-8").read()
        cands = candidates(text)
        if not cands:
            continue
        print("\n== %s ==" % name)
        for start, end, head in cands:
            refs = SECTION_REF.findall(head)
            size = end - start
            lines = text.count("\n", start, end)
            if not refs:
                verdicts["NO-TARGET"] += 1
                print("  NO-TARGET  %6dch %5dln  %s" % (size, lines, head[:70]))
                continue
            outside = {}
            for ref in refs:
                pos = heading_positions(text, ref)
                outside[ref] = [p for p in pos if not (start <= p < end)]
            missing = [r for r, p in outside.items() if not p]
            if missing:
                verdicts["ONLY-COPY"] += 1
                only_copies.append((name, head, size, lines, missing))
                print("  ONLY-COPY  %6dch %5dln  %s" % (size, lines, head[:70]))
                print("             no copy outside for: %s" % ", ".join("§" + m for m in missing))
            else:
                verdicts["REDUNDANT"] += 1
                print("  REDUNDANT  %6dch %5dln  %s" % (size, lines, head[:70]))

    print("\n== summary ==")
    for k, v in verdicts.items():
        print("  %-10s %d" % (k, v))
    if only_copies:
        print("\n== blocks that are mislabelled: the label says record, the content is spec ==")
        for name, head, size, lines, missing in only_copies:
            print("  [%s] %dch / %dln  missing outside: %s"
                  % (name, size, lines, ", ".join("§" + m for m in missing)))
            print("      %s" % head[:100])
    return 0


if __name__ == "__main__":
    sys.exit(main())
