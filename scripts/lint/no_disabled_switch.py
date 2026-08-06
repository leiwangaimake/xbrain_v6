#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: no_disabled_switch.py
Brief: CHK-2-26 -- fail if a safety-ordering-bypass switch (enforce_ordering)
       reappears in a config or in the runtime

Description:
15 S12 removed the enforce_ordering switch: a config knob that let a process skip
the task-ordering safety assertion. A removed switch with no guard comes back --
someone re-adds it "just to unblock a test" and it ships. This lint is that
guard. It fails if the token enforce_ordering appears anywhere a process could
read it: configs/**/*.yaml (a knob) or xbrain/**/*.py (code that would honour
one). CLAUDE.md 3.6 is the rule this enforces -- there is to be no switch that
turns a safety assertion off.

Scan surface, and why it is exactly this (CLAUDE.md 3.2 form 3, 判据自伤):
  * INCLUDES configs/ and xbrain/ -- the two places the switch could live.
  * EXCLUDES scripts/ (where this file lives) and docs/. The forbidden token is
    written in THIS file, on purpose, as the thing to look for; a scan that swept
    scripts/ would match itself and could never reach zero. Excluding docs/ keeps
    a design note that explains the removed switch from tripping it.

Usage:
  python3 scripts/lint/no_disabled_switch.py            # scan the tree, exit 1 on a hit
  python3 scripts/lint/no_disabled_switch.py --self-test # prove the scan can go red
"""

import os
import sys

# The repo root: this file is scripts/lint/<name>, so two dirnames up.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

#: The forbidden token. 15 S12's removed switch. A tuple so a second removed
#: switch could join it WITH ITS OWN removal decision -- not speculatively
#: (CLAUDE.md 9.3); today there is one.
FORBIDDEN = ("enforce_ordering",)

#: The scan surface, verbatim from the CHK-2-26 criterion. scripts/ and docs/ are
#: excluded on purpose (see the file header: this file holds the token, and a
#: design note may name it).
SCAN = (
    ("configs", (".yaml", ".yml")),     # a knob a process would read
    ("xbrain", (".py",)),               # code that would honour one
)


def _find_hits(root):
    """Every (relpath, lineno, token) where a forbidden token appears under root.

    Whole-file scan, not a diff: CLAUDE.md 3.2's "only scans the diff" failure is
    exactly how a token that was added in an untouched file survives. Reads as
    text so a token inside a comment is still a hit -- a commented-out
    enforce_ordering is one uncomment away from live.
    """
    hits = []
    for top, exts in SCAN:
        base = os.path.join(root, top)
        if not os.path.isdir(base):
            continue                    # tree may be absent in a partial checkout
        for dirpath, _dirs, files in os.walk(base):
            if "__pycache__" in dirpath:
                continue
            for name in files:
                if not name.endswith(exts):
                    continue
                path = os.path.join(dirpath, name)
                with open(path, encoding="utf-8", errors="replace") as fh:
                    for lineno, line in enumerate(fh, 1):
                        for token in FORBIDDEN:
                            if token in line:
                                hits.append(
                                    (os.path.relpath(path, root), lineno, token))
    return hits


def self_test():
    """Prove the scan can go red, so a green run means something.

    Writes a temp yaml carrying the forbidden token into a throwaway root, scans
    THAT (never the real tree), asserts it is found, then asserts a clean root is
    empty. A lint whose --self-test only checked the clean case would pass while
    unable to detect anything (CLAUDE.md 3.3).
    """
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        cfg = os.path.join(tmp, "configs")
        os.makedirs(cfg)
        # The mutation the criterion names: a config re-introduces the switch.
        with open(os.path.join(cfg, "p3_task.yaml"), "w") as fh:
            fh.write("retention:\n  enforce_ordering: true\n")
        hits = _find_hits(tmp)
        if not hits:
            print("SELF-TEST FAIL: the injected enforce_ordering was NOT detected")
            return 1
        # And a clean root must be empty, or the check is a constant red.
        os.remove(os.path.join(cfg, "p3_task.yaml"))
        with open(os.path.join(cfg, "p3_task.yaml"), "w") as fh:
            fh.write("retention:\n  max_rows: 100000\n")
        if _find_hits(tmp):
            print("SELF-TEST FAIL: a clean config was flagged")
            return 1
    print("SELF-TEST PASS: enforce_ordering is detected when present, not when absent")
    return 0


def main():
    if "--self-test" in sys.argv:
        return self_test()
    # State the surface and the criterion up front, so the reader knows what a
    # zero here does and does not cover (CLAUDE.md 3.2 form 6, scan-surface).
    print("scan surface: " + ", ".join("%s/**%s" % (t, e) for t, e in SCAN))
    hits = _find_hits(ROOT)
    for rel, lineno, token in hits:
        print("  %s:%d  %s (15 S12 removed switch; CLAUDE.md 3.6)" % (rel, lineno, token))
    print("criterion: enforce_ordering hits == 0")
    if hits:
        print("FAIL: %d occurrence(s) of a removed safety-bypass switch" % len(hits))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
