#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
Shanghai Hachist Intelligent Ship Technology Co., Ltd.
File: enum_once_scan.py
Brief: ENUM-1 -- assertion-letter execution order must be enumerated exactly once

Description:
Doc 10 S5.4.4 declares one authoritative line for the Stage 0c assertion
execution order and forbids restating that enumeration anywhere else.
A prohibition with no executable check rots: the previous state of the repo
had five assertions that would never be evaluated because three Stage 0c
sites still carried a stale enumeration.

Criterion ENUM-1:
  On the declared scan surface (scan_manifest.json members), no line may
  contain a chain of five or more single-letter IDs joined by arrows.
  Why five and not three: the repo has two unrelated short letter chains
  that are NOT this enumeration -- the P2 startup stage machine (doc 10,
  four letters) and a mode-switch example (doc 14, four letters).  A
  criterion that flags those gets widened by the next person until it
  catches nothing (CLAUDE.md S3.2 form 2 decaying into form 1), so the
  threshold is set above them on purpose.  Cost of that choice, stated
  rather than hidden: a restated enumeration of four or fewer letters
  escapes this criterion.
  The one authoritative subsection in doc 10 is the only exemption, and it
  is located by heading text, not by line number (line numbers drift).

Expected state on 2026-08-05: RED, and the reason is named -- doc 13 still
quotes the pre-2026-08-05 execution order verbatim in four places.  Doc 13
is not in this batch's write scope.  Closing condition: doc 13 owner
changes those four quotes to cite the anchor text instead.  A red run with
a named owner and a closing condition is not the same thing as a criterion
that is red forever (CLAUDE.md S3.2 form 2).

Self-harm check (CLAUDE.md S3.2 form 3):
  This script lives in scripts/, the scan surface is docs/*.md.  The regex
  and the planted mutants below are therefore OUTSIDE the surface they
  are matched against.  Run --self-test to see the criterion go red.

Note: this script prints counts, it does not store them.  Counts are
judgement values and must not be copied into any markdown (CLAUDE.md S3.7).
"""

import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(HERE, "scan_manifest.json")

# One "letter token": an uppercase A-Z, optionally primed, optionally wrapped
# in markdown emphasis or backticks.  Two tokens joined by an arrow = one link.
_TOK = r"[*`]{0,2}[A-Z][′'’]?[*`]{0,2}"
_ARROW = r"\s*(?:→|->|=>)\s*"
# Five or more letters => four or more links.  See threshold note above.
CHAIN_RE = re.compile(_TOK + "(?:" + _ARROW + _TOK + "){4,}")

# The single authoritative subsection, located by heading text.
AUTH_FILE = "10-顶层设计.md"
AUTH_HEADING = "执行顺序（唯一权威行）"


def load_surface():
    with open(MANIFEST, encoding="utf-8") as fh:
        man = json.load(fh)
    docs_dir = man["docs_dir"]
    files = [m["file"] for m in man["members"]]
    return docs_dir, files, man.get("version"), man.get("updated")


def exempt_range(path, lines):
    """Return (start, end) line indices of the authoritative subsection, or None.

    Located by heading text so it survives edits above it.  The range ends at
    the next markdown heading of depth <= the authoritative heading's depth.
    """
    if os.path.basename(path) != AUTH_FILE:
        return None
    start = None
    depth = None
    for i, line in enumerate(lines):
        if line.startswith("#") and AUTH_HEADING in line:
            start = i
            depth = len(line) - len(line.lstrip("#"))
            break
    if start is None:
        return None
    for j in range(start + 1, len(lines)):
        line = lines[j]
        if line.startswith("#"):
            d = len(line) - len(line.lstrip("#"))
            if 0 < d <= depth:
                return (start, j)
    return (start, len(lines))


def scan_file(path):
    with open(path, encoding="utf-8") as fh:
        lines = fh.read().split("\n")
    ex = exempt_range(path, lines)
    hits = []
    for i, line in enumerate(lines):
        if ex and ex[0] <= i < ex[1]:
            continue
        m = CHAIN_RE.search(line)
        if m:
            hits.append((i + 1, m.group(0)[:60], line.strip()[:90]))
    return hits, (ex is not None)


def run(docs_dir, files):
    total = 0
    exempted = 0
    for name in files:
        path = os.path.join(docs_dir, name)
        if not os.path.exists(path):
            print("  MISSING  %s" % name)
            continue
        hits, was_ex = scan_file(path)
        if was_ex:
            exempted += 1
        for ln, frag, ctx in hits:
            total += 1
            print("  %-40s line %-6d chain %-22s | %s" % (name, ln, frag, ctx))
    return total, exempted


def self_test():
    """Plant mutants and require the criterion to catch every one of them."""
    planted = [
        ("J → A → B → C → D → E", True),
        ("`J` → `A` → `M` → `B` → `C`", True),
        ("**J** -> **A** -> **B** -> **C** -> **D**", True),
        ("J -> A -> B -> C", False),    # four letters: below threshold on purpose
        ("J -> A", False),            # one link is not an enumeration
        ("J -> A -> ... -> I -> K", False),  # ellipsis breaks the chain
        ("see section A and section B", False),
    ]
    caught = 0
    expected = 0
    ok = True
    for text, should_hit in planted:
        hit = CHAIN_RE.search(text) is not None
        if should_hit:
            expected += 1
            if hit:
                caught += 1
            else:
                ok = False
                print("  MISS      %r" % text)
        elif hit:
            ok = False
            print("  FALSE HIT %r" % text)
    print("  ENUM-1  planted %d, caught %d" % (expected, caught))
    print("self-test %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()

    docs_dir, files, ver, upd = load_surface()
    print("scan surface: %d files (manifest v%s, updated %s)" % (len(files), ver, upd))
    print("exemption: doc 10 subsection located by heading text, not line number")
    total, exempted = run(docs_dir, files)
    print("")
    print("ENUM-1  violations %d  exempted subsections %d  %s"
          % (total, exempted, "PASS" if total == 0 else "FAIL"))
    print("criterion is violations == 0; every hit is a second copy of an "
          "enumeration whose only authority is doc 10 S5.4.4")
    return 0 if total == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
