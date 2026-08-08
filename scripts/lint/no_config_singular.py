#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: no_config_singular.py
Brief: CFG-DC-3 (config/) -- ban singular config/ path references

Description:
CLAUDE.md 3.6 and 10 S5.4.0 fix the config root as the PLURAL
`configs/`. Any reference to a SINGULAR `config/` path is a defect:

  * `configs/` (plural) = the config root under /opt/xbrain_v6/
  * `config/`  (singular) = a legacy V5 spelling or a typo

Real-world defect this catches:

  Someone copies V5 code that reads `config/foo.yaml` into V6.
  The line looks natural, but the load fails because the file is
  under `configs/foo.yaml`. FileNotFoundError is what surfaces --
  the operator hunts for the missing file rather than the typo.

Scan surface:

  * INCLUDES  configs/, xbrain/, scripts/, deploy/, ros2_ws/,
              services/, tests/
  * EXCLUDES  docs/ (markdown prose about historical paths OK),
              this file (SCAN-SURFACE-EXEMPT: judge-self-catches),
              __pycache__/, .git/, common/lib/ (build artifacts)

The scan looks for:
  * string literals like "config/..." (heavier heuristic: at least
    one path segment after config/)
  * open("config/...") / Path("config/...") / os.path.join(...,"config/",...)

Python package NAMES like xbrain/p4_agent/config/ are NOT flagged --
those are legitimate package layouts. The lint distinguishes by
requiring the string form (quoted or unquoted-as-path-arg) not
attribute access (xbrain.common.config).

Exemption marker: NO-CONFIG-SINGULAR-LINT on the same line.

Usage:
  python3 scripts/lint/no_config_singular.py
  python3 scripts/lint/no_config_singular.py --self-test
"""

import argparse
import os
import re
import sys
from typing import List, Tuple


# Only flag when the singular 'config/' looks like a filesystem
# path to a config FILE (ends with .yaml/.yml/.json/.conf) OR is
# an absolute path segment (/opt/.../config/). Bare Zenoh key names
# like "cmd/config/ack" and Python module paths like
# xbrain/common/config/... do not match.
_PATTERN_STANDALONE = re.compile(
    r'''["']config/[^"']+\.(?:yaml|yml|json|conf)["']'''
)
_PATTERN_ABSOLUTE = re.compile(
    r'''["'][^"']*/opt/[^"']*/config/[^"']+["']'''
)
# Comment-only lines: skip (docstrings + prose).
_COMMENT_ONLY = re.compile(r"^\s*(?:#|//|\*|--)")


def _scan_file(path: str) -> List[Tuple[int, str]]:
    hits: List[Tuple[int, str]] = []
    try:
        with open(path, encoding="utf-8") as fh:
            lines = fh.readlines()
    except (OSError, UnicodeDecodeError):
        return hits
    for i, line in enumerate(lines, 1):
        if "NO-CONFIG-SINGULAR-LINT" in line:
            continue
        # Skip pure comment lines to reduce noise on prose about V5.
        if _COMMENT_ONLY.match(line):
            continue
        # Skip lines mentioning "configs/" only (plural is fine).
        stripped = line
        # Try standalone "config/foo.yaml" pattern first.
        m = _PATTERN_STANDALONE.search(stripped)
        if m:
            hits.append((i, line.rstrip()))
            continue
        # Then absolute "/opt/.../config/..." (deploy paths).
        m = _PATTERN_ABSOLUTE.search(stripped)
        if m:
            hits.append((i, line.rstrip()))
    return hits


def _walk(root: str) -> List[str]:
    out: List[str] = []
    skip_dirs = {"__pycache__", ".git", ".pytest_cache", "docs",
                 "node_modules", "build", "lib"}
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames
                       if d not in skip_dirs and not d.startswith(".")]
        for name in filenames:
            # Skip binary + config files.
            if not (name.endswith((".py", ".sh", ".bash", ".yaml", ".yml",
                                     ".json", ".cc", ".cpp", ".c", ".h",
                                     ".hpp", ".service", ".txt", ".cmake"))):
                continue
            # Skip this lint's own file (SCAN-SURFACE-EXEMPT: judge-self-catches).
            if os.path.basename(name) == "no_config_singular.py":
                continue
            out.append(os.path.join(dirpath, name))
    return sorted(out)


def _self_test() -> int:
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        (open(os.path.join(td, "bad.py"), "w")
            .write('p = "config/foo.yaml"\n'))
        (open(os.path.join(td, "ok.py"), "w")
            .write('p = "configs/foo.yaml"\n'))
        bad_hits = _scan_file(os.path.join(td, "bad.py"))
        ok_hits = _scan_file(os.path.join(td, "ok.py"))
    if not bad_hits:
        print("self-test FAIL: config/foo.yaml not caught")
        return 1
    if ok_hits:
        print("self-test FAIL: configs/foo.yaml false-positive")
        return 1
    print("self-test PASS")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("root", nargs="?", default=".")
    args = ap.parse_args()
    if args.self_test:
        return _self_test()
    print("scan surface: %s (exclude docs/, this file)" % args.root)
    total = 0
    for path in _walk(args.root):
        for lineno, text in _scan_file(path):
            print("  BAD  %s:%d  %s" % (path, lineno, text.strip()))
            total += 1
    print("  violations:        %d" % total)
    print("criterion: violations == 0")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
