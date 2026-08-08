#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: no_business_imports.py
Brief: CFG-DC-3 -- ban rclpy / sqlite3 / requests in business modules

Description:
CLAUDE.md 4.1 fixes three hard-no imports for xbrain/:

  1. import rclpy         -- ROS lives only in ros2_ws/; xbrain/ is
                             the AI stack and MUST NOT link ROS.
  2. import sqlite3       -- xbrain/**/persistence/ must use aiosqlite,
                             synchronous sqlite3 blocks the event loop.
  3. import requests      -- business modules must go through
                             xbrain/p4_agent/ai_client/*_client.py,
                             which is the ONLY layer allowed to speak
                             to AI services directly.

Exemptions:
  * ai_client/*_client.py may import requests -- that's what those
    files are for. Mark with comment: BUSINESS-IMPORT-OK(ai-client).
  * persistence/ may import aiosqlite (not sqlite3); that is enforced
    by the check itself (only bare sqlite3 flagged).
  * Tests under tests/ are not scanned; test setup often needs
    stdlib sqlite3 for fixtures.

Scan surface: xbrain/**/*.py.

Meta test (self-test): plants each of the three imports in a temp
tree, verifies each is caught.

Usage:
  python3 scripts/lint/no_business_imports.py
  python3 scripts/lint/no_business_imports.py --self-test
"""

import argparse
import os
import re
import sys
from typing import List, Tuple


# Banned import patterns. Match both `import X` and `from X import Y`.
_PATTERNS = [
    ("rclpy",   re.compile(r"^\s*(?:import\s+rclpy(?:\s|$|\.|,)|from\s+rclpy\b)")),
    ("sqlite3", re.compile(r"^\s*(?:import\s+sqlite3(?:\s|$|\.|,)|from\s+sqlite3\b)")),
    ("requests", re.compile(r"^\s*(?:import\s+requests(?:\s|$|\.|,)|from\s+requests\b)")),
]

# Exemption marker: any line with this string is skipped. Provides
# an escape hatch for the ai_client modules that legitimately need
# requests.
_EXEMPT_MARKER = "BUSINESS-IMPORT-OK"


def _scan_file(path: str) -> List[Tuple[int, str, str]]:
    """Return list of (lineno, module, text) hits for one file."""
    hits: List[Tuple[int, str, str]] = []
    try:
        with open(path, encoding="utf-8") as fh:
            lines = fh.readlines()
    except (OSError, UnicodeDecodeError):
        return hits
    for i, line in enumerate(lines, 1):
        if _EXEMPT_MARKER in line:
            continue
        for module, pattern in _PATTERNS:
            if pattern.match(line):
                hits.append((i, module, line.rstrip()))
                break
    return hits


def _walk(root: str) -> List[str]:
    out: List[str] = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames
                       if not d.startswith(("__pycache__", "."))]
        for name in filenames:
            if name.endswith(".py"):
                out.append(os.path.join(dirpath, name))
    return sorted(out)


def _self_test() -> int:
    import tempfile
    samples = {
        "r.py": "import rclpy\n",
        "s.py": "import sqlite3\n",
        "q.py": "from requests import get\n",
    }
    with tempfile.TemporaryDirectory() as td:
        for n, txt in samples.items():
            with open(os.path.join(td, n), "w") as f:
                f.write(txt)
        got = set()
        for p in _walk(td):
            for _, module, _ in _scan_file(p):
                got.add(module)
    expected = {"rclpy", "sqlite3", "requests"}
    missing = expected - got
    if missing:
        print("self-test FAIL: %s" % sorted(missing))
        return 1
    print("self-test PASS: %s" % sorted(got))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("root", nargs="?", default="xbrain")
    args = ap.parse_args()
    if args.self_test:
        return _self_test()
    print("scan surface: %s (marker exemption: %s)"
          % (args.root, _EXEMPT_MARKER))
    total = 0
    for path in _walk(args.root):
        for lineno, module, text in _scan_file(path):
            print("  BAD  %s:%d  %s  %s"
                  % (path, lineno, module, text.strip()))
            total += 1
    print("  violations:        %d" % total)
    print("criterion: violations == 0")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
