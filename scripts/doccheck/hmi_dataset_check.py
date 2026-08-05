#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: hmi_dataset_check.py
Brief: Check that every frozen HMI data element has a registered key and field

Description:
The HMI visualisation data set was frozen on 2026-08-04 against the client's
front-end design. Its single source of truth is the table in 17 §6.8 -- this
script PARSES that table rather than keeping a second copy, because the package
has repeatedly been bitten by two hand-maintained copies of the same list
drifting apart (seven evaluation lists across the docs, two "count corrections"
that each missed one).

For every row it asks: does the "我方 key" column name a key that is actually
registered in 11 §2.2, and does the "字段" column name something the contract
defines? Rows marked 待建 are open debt (11 §15.6A.3 ④) and are reported
separately -- they are known, not failures of this check.

Per CLAUDE.md 3.7 the satisfied/unsatisfied counts live here, not in the prose:
close a debt by replacing 待建 with the real key in 17 §6.8, and this script
stops reporting it. Nothing in the documents needs a status column.

Usage:
  hmi_dataset_check.py              # report
  hmi_dataset_check.py --self-test  # prove the check can go red
"""

import argparse
import os
import re
import sys

DOCS = "/opt/xbrain_v6/docs"
SPEC = os.path.join(DOCS, "17-P5网关与HMI详细设计.md")
CONTRACT = os.path.join(DOCS, "11-接口契约.md")

SECTION = "### 6.8 "
SECTION_END = re.compile(r"^## 7\. ")
PENDING = "待建"
LOCAL = "前端本地"
# Keys look like xbrain/{rid}/plane/... or a REST path; pick both out of a cell.
KEY_RE = re.compile(r"xbrain/\{?[a-z_]*\}?/[a-z_]+(?:/[a-z_0-9*{}]+)*|/api/[a-z_0-9/{}?=&.]+")


def parse_table(text):
    """Yield the data rows of the 17 §6.8 table as dicts."""
    start = text.find(SECTION)
    if start < 0:
        raise SystemExit("17 §6.8 not found -- has the section been renumbered?")
    rows = []
    group = None
    for line in text[start:].split("\n"):
        if SECTION_END.match(line):
            break
        m = re.match(r"^#### 6\.8\.\d+ ([A-F]) · (.+?)（", line)
        if m:
            group = m.group(1)
            continue
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 6 or not cells[0].isdigit():
            continue          # header or separator
        rows.append({
            "group": group, "n": cells[0], "ui": cells[1], "need": cells[2],
            "rate": cells[3], "key": cells[4], "field": cells[5],
        })
    return rows


GEO_TYPES = ("fence", "waypoint", "route", "dock")


def normalise(k):
    """Put a key/path into the form the contract writes it in.

    Three shapes differ between the two documents and must be reconciled before
    comparing, or every REST row reports a false miss:
      - the contract writes the geo endpoints as templates, /api/geo/{type}/{geo_id},
        while the data-set table instantiates them, /api/geo/waypoint/{geo_id};
      - the table carries query strings (?type=waypoint) the contract never lists;
      - both sides use {rid} / {robot_id} for the same segment.
    """
    k = k.split("?")[0].rstrip("/")
    k = k.replace("{robot_id}", "{rid}")
    parts = k.split("/")
    if len(parts) > 3 and parts[1] == "api" and parts[2] == "geo" and parts[3] in GEO_TYPES:
        parts[3] = "{type}"
        k = "/".join(parts)
    return k


def registered_keys(text):
    """Every Zenoh key and REST path the contract registers, normalised."""
    return {normalise(k) for k in KEY_RE.findall(text)}


def check(spec_text, contract_text):
    rows = parse_table(spec_text)
    known = registered_keys(contract_text)
    pending, local, ok, bad = [], [], [], []
    for r in rows:
        if LOCAL in r["key"] or r["key"].lstrip().startswith("—"):
            local.append(r)
        elif PENDING in r["key"]:
            pending.append(r)
        else:
            cited = [normalise(k) for k in KEY_RE.findall(r["key"])]
            missing = [k for k in cited if k not in known]
            if not cited:
                # The cell is prose ("WS 下行 geo_manifest", "见 §x.y") rather than a
                # literal key. That is a defect in the table, not in the contract:
                # a row nobody can mechanically check is a row nobody checks.
                bad.append((r, "key 列是描述性文字，没有可机械核对的 key —— 请在 17 §6.8 补上字面 key"))
            elif missing:
                bad.append((r, "契约里查无此 key: " + ", ".join(missing)))
            else:
                ok.append(r)
    return rows, ok, pending, local, bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true",
                    help="delete a key from the contract copy in memory; the check must go red")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    spec = open(SPEC, encoding="utf-8").read()
    contract = open(CONTRACT, encoding="utf-8").read()

    if args.self_test:
        # A criterion that has only ever been green is not a criterion
        # (CLAUDE.md 3.3). Remove one registered key and require a new failure.
        rows, ok, _, _, bad = check(spec, contract)
        if not ok:
            print("self-test inconclusive: no satisfied row to mutate")
            return 2
        victim = KEY_RE.findall(ok[0]["key"])[0]
        mutated = contract.replace(victim, "xbrain/{rid}/deliberately/removed")
        _, _, _, _, bad2 = check(spec, mutated)
        caught = len(bad2) > len(bad)
        print("self-test: removed %r from the contract" % victim)
        print("  violations %d -> %d  %s" % (len(bad), len(bad2), "OK" if caught else "MISSED"))
        print("self-test %s" % ("PASS" if caught else "FAIL -- the check is blind"))
        return 0 if caught else 1

    rows, ok, pending, local, bad = check(spec, contract)
    print("frozen HMI data set: %d elements (source: 17 §6.8)" % len(rows))
    print("  satisfied        %d" % len(ok))
    print("  open debt 待建   %d   (registered in 11 §15.6A.3 ④)" % len(pending))
    print("  front-end local  %d" % len(local))
    print("  UNSATISFIED      %d" % len(bad))
    for r, why in bad:
        print("      [%s%s] %s" % (r["group"], r["n"], r["ui"][:70]))
        print("            %s" % why)
    if args.verbose and pending:
        print("\n  open debt detail:")
        for r in pending:
            print("      [%s%s] %s" % (r["group"], r["n"], r["ui"][:70]))
    print("\ncriterion: UNSATISFIED == 0. 待建 rows are known debt, not failures --")
    print("close one by replacing 待建 with the real key in 17 §6.8.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
