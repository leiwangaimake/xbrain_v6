#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: no_safety_default.py
Brief: CFG-CM-13 -- fail if a safety parameter has a code-side default

Description:
CLAUDE.md 3.1 rule: common.safety.* and common.spec.* MUST NOT carry
a code-side default. The three defect shapes this lint catches:

  1. @dataclass field with a default whose name matches a safety key
     Example:  max_decel_mps2: float = 2.5
     Rationale: dataclass defaults bypass the freeze assertion A
     (unassigned key) because the field is 'already assigned' from
     the class body.

  2. dict.get("common.safety.X", fallback) or cfg.get("common.spec.X", fb)
     Example:  a = cfg.get("brake.a_mps2", 2.5)
     Rationale: the fallback silently substitutes when the config is
     missing, defeating the whole 'unassigned = refuse to start'
     model.

  3. `value or fallback` idiom where value is a safety param
     Example:  a = cfg.brake.a_mps2 or 2.5
     Rationale: the same defect as dict.get, just spelled with
     Python's short-circuit or. Both '0' and None trigger the
     fallback, which is worse: 0 is a legal-but-catastrophic value
     that would be replaced with 2.5 silently.

Scan surface: xbrain/ only. Tests exist that DELIBERATELY exercise
these patterns (e.g. to verify the assertion catches them) and live
in tests/ where the lint does not run.

Meta test (CFG-CM-13 activity requirement): the mutation --self-test
proves the lint can go red. A lint that is silently green because
its scan matches nothing has the same failure mode as the defect it
guards -- CLAUDE.md 3.2 form 3 (判据自伤).

Usage:
  python3 scripts/lint/no_safety_default.py               # scan, exit 1 on hit
  python3 scripts/lint/no_safety_default.py --self-test   # prove it goes red
  python3 scripts/lint/no_safety_default.py -v            # verbose
"""

import argparse
import os
import re
import sys
from typing import List, Tuple


# Safety-key tokens the lint recognises as safety-relevant. Anything
# whose leaf key matches these keywords is treated as a safety param
# for the default check. Kept conservative: better a false positive
# an author can silence than a false negative that ships.
_SAFETY_KEYWORDS = {
    "safety", "spec", "brake", "d_safe_m", "t_lat_s", "margin_lat_m",
    "margin_base_m", "margin_rot_m", "d_stop", "max_decel_mps2",
    "max_accel_mps2", "max_vx_mps", "max_vy_mps", "max_wz_radps",
}

# Pattern 1: dataclass field with default. Matches
#   name: type = value
# where name contains a safety keyword AND value is a numeric literal
# (0, 1.5, -2.5, etc.) or bool. Excludes None (which is legit
# unassigned marker) and dict/list literals (which are containers,
# not values).
_PATTERN_DATACLASS = re.compile(
    r"^\s*(?P<name>[a-z_][a-z0-9_]*)\s*"
    r":\s*[A-Za-z_][A-Za-z0-9_\[\], ]*\s*=\s*"
    r"(?P<value>-?[0-9]+(?:\.[0-9]+)?|True|False)\s*(?:#|$)"
)

# Pattern 2: dict-like .get("key", default_expr) where key contains
# a safety keyword and default is anything non-None. Loose on the
# default (any second arg) to catch alternatives like:
#   cfg.get("brake.a_mps2", DEFAULT_A)
#   cfg["brake"].get("a_mps2", 2.5)
_PATTERN_GET_DEFAULT = re.compile(
    r"\.get\s*\(\s*['\"](?P<key>[^'\"]*(?:safety|spec|brake|d_safe|t_lat|margin_lat|margin_base|margin_rot|d_stop|max_decel|max_accel|max_vx|max_vy|max_wz)[^'\"]*)['\"]\s*,\s*(?!None)"
)

# Pattern 3: `x = <expr containing safety key> or <fallback>` idiom.
# Anchored to an assignment (or `return`) so comment / string prose
# occurrences do not fire. Requires the fallback to be a numeric
# literal or a bare identifier -- excludes 'or None' and 'or []'.
_PATTERN_OR_FALLBACK = re.compile(
    r"^\s*(?:return\s+|[A-Za-z_][A-Za-z0-9_.\[\]'\"]*\s*=\s*)"
    r"[^#\n]*"
    r"(?:safety|spec|brake|d_safe|t_lat|margin_|d_stop|max_decel|max_accel|max_vx|max_vy|max_wz|a_mps2|k_brake|throttle)"
    r"[A-Za-z0-9_]*\s+or\s+(?!None)(?![\[\{])"
    r"(?:-?[0-9]+(?:\.[0-9]+)?|[A-Z_][A-Z_0-9]*)"
)


def _scan_file(path: str) -> List[Tuple[int, str, str]]:
    """Return list of (lineno, kind, text) hits for one file."""
    hits: List[Tuple[int, str, str]] = []
    try:
        with open(path, encoding="utf-8") as fh:
            lines = fh.readlines()
    except (OSError, UnicodeDecodeError):
        return hits
    for i, line in enumerate(lines, 1):
        # Skip lines with a NO-LINT marker so intentional test
        # fixtures can be exempted with a comment.
        if "NO-SAFETY-DEFAULT-LINT" in line:
            continue
        m = _PATTERN_DATACLASS.match(line)
        if m and m.group("name") in _SAFETY_KEYWORDS or (
            m and any(k in m.group("name") for k in _SAFETY_KEYWORDS)
        ):
            hits.append((i, "dataclass_default", line.rstrip()))
            continue
        if _PATTERN_GET_DEFAULT.search(line):
            hits.append((i, "get_with_default", line.rstrip()))
            continue
        if _PATTERN_OR_FALLBACK.search(line):
            hits.append((i, "or_fallback", line.rstrip()))
    return hits


def _walk(root: str) -> List[str]:
    """Return every .py file under root."""
    out: List[str] = []
    for dirpath, dirnames, filenames in os.walk(root):
        # Skip __pycache__ and hidden dirs.
        dirnames[:] = [d for d in dirnames
                       if not d.startswith(("__pycache__", "."))]
        for name in filenames:
            if name.endswith(".py"):
                out.append(os.path.join(dirpath, name))
    return sorted(out)


def _self_test() -> int:
    """Prove the lint can turn red on each of the three patterns."""
    import tempfile
    samples = {
        "d.py": "from dataclasses import dataclass\n"
                "@dataclass\n"
                "class C:\n"
                "    max_decel_mps2: float = 2.5\n",
        "g.py": "def f(cfg):\n"
                "    return cfg.get('common.safety.brake.a_mps2', 2.5)\n",
        "o.py": "def g(cfg):\n"
                "    return cfg.brake.a_mps2 or 2.5\n",
    }
    with tempfile.TemporaryDirectory() as td:
        for n, txt in samples.items():
            with open(os.path.join(td, n), "w") as f:
                f.write(txt)
        got_kinds = set()
        for p in _walk(td):
            for _, kind, _ in _scan_file(p):
                got_kinds.add(kind)
    expected = {"dataclass_default", "get_with_default", "or_fallback"}
    missing = expected - got_kinds
    if missing:
        print("self-test FAIL: patterns not caught: %s" % sorted(missing))
        return 1
    print("self-test PASS: all three patterns fire (%s)" % sorted(got_kinds))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--self-test", action="store_true",
                    help="run the mutation self-test instead of scanning")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("root", nargs="?", default="xbrain",
                    help="scan root (default: xbrain)")
    args = ap.parse_args()
    if args.self_test:
        return _self_test()
    print("scan surface: %s" % args.root)
    total = 0
    for path in _walk(args.root):
        hits = _scan_file(path)
        if hits:
            for lineno, kind, text in hits:
                print("  BAD  %s:%d  %s  %s"
                      % (path, lineno, kind, text.strip()))
                total += 1
    print("  violations:        %d" % total)
    print("criterion: violations == 0")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
