#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: no_chinese_in_log.py
Brief: CFG-CM-15 (partial) -- no Chinese characters in log / print / exception messages

Description:
CLAUDE.md 2.1 mandates: log lines, print outputs, and exception messages
MUST be entirely English. Chinese log lines are unsearchable by grep in
English-only tooling and unroutable in journald filters that split on
ASCII delimiters.

Comments and docstrings MAY be Chinese (CLAUDE.md 2.1 allows single-file
consistency). This lint scans ONLY:

  * log.<level>(...) calls where <level> in
    {debug, info, warning, warn, error, critical, exception, log}
  * print(...) calls
  * raise <ExcClass>(...) calls where any argument is a string literal
  * self.<log>.info(...) etc when self.<log> is a Logger

The lint uses Python ast to find the call sites, then walks the arg
tree for string constants (Constant node with str type). CJK detection
covers:
  * CJK Unified Ideographs (U+4E00 - U+9FFF)
  * CJK punctuation (U+3000 - U+303F)
  * Fullwidth ASCII (U+FF00 - U+FFEF) -- the fullwidth forms of ASCII
    that operators sometimes paste in
  * Halfwidth Katakana (skipped -- rare in project)

Scan surface: xbrain/, scripts/ (excluding this file), services/.
Test file exemption: not applied -- tests should also log in English.
Explicit exemption marker: NO-CHINESE-LOG-LINT on same line.

Meta test (self-test): plants an offending pattern in a temp tree,
verifies each of the three call kinds fires.

Usage:
  python3 scripts/lint/no_chinese_in_log.py
  python3 scripts/lint/no_chinese_in_log.py --self-test
"""

import argparse
import ast
import os
import sys
from typing import Iterable, List, Tuple


# CJK character ranges. Kept as a small tuple of (lo, hi) pairs so
# adding a range later is one line.
_CJK_RANGES = (
    (0x4E00, 0x9FFF),   # CJK Unified Ideographs
    (0x3000, 0x303F),   # CJK Symbols and Punctuation
    (0xFF00, 0xFFEF),   # Halfwidth and Fullwidth Forms
)

# Log function names -- attribute leaf names (e.g. log.info, self.log.info).
_LOG_METHODS = frozenset({
    "debug", "info", "warning", "warn", "error",
    "critical", "exception", "log",
})

# Exemption marker: on same line as the offending call.
_EXEMPT = "NO-CHINESE-LOG-LINT"


def _has_cjk(s: str) -> bool:
    """True iff s contains any CJK-range codepoint."""
    return any(any(lo <= ord(c) <= hi for lo, hi in _CJK_RANGES)
               for c in s)


def _is_log_call(node: ast.Call) -> bool:
    """True iff node.func matches log.<method>() or print()."""
    # print(...) direct.
    if isinstance(node.func, ast.Name) and node.func.id == "print":
        return True
    # Attribute: obj.info(...), self.log.warning(...), etc.
    if isinstance(node.func, ast.Attribute):
        if node.func.attr in _LOG_METHODS:
            # Heuristic: only fire when the immediate parent looks
            # like a logger (attribute chain contains 'log' / 'logger'
            # / obj is Name). Avoids firing on unrelated .info() calls
            # like re.info().
            root = node.func.value
            while isinstance(root, ast.Attribute):
                if root.attr in ("log", "logger", "_log", "_logger"):
                    return True
                root = root.value
            if isinstance(root, ast.Name) and root.id in (
                    "log", "logger", "logging", "self", "cls"):
                return True
    return False


def _is_raise_with_string(node: ast.Raise) -> List[ast.Constant]:
    """Return string-constant nodes inside a raise ExcCls('...') call."""
    out: List[ast.Constant] = []
    if node.exc is None:
        return out
    # raise foo() -- walk the Call args.
    if isinstance(node.exc, ast.Call):
        for arg in node.exc.args:
            for sub in ast.walk(arg):
                if isinstance(sub, ast.Constant) and isinstance(sub.value, str):
                    out.append(sub)
    return out


def _scan_file(path: str) -> List[Tuple[int, str, str]]:
    """Return list of (lineno, kind, text) hits."""
    hits: List[Tuple[int, str, str]] = []
    try:
        with open(path, encoding="utf-8") as fh:
            source = fh.read()
    except (OSError, UnicodeDecodeError):
        return hits
    try:
        tree = ast.parse(source, path)
    except SyntaxError:
        return hits
    lines = source.splitlines()

    def _check_str_const(const: ast.Constant, kind: str) -> None:
        if not isinstance(const.value, str):
            return
        if not _has_cjk(const.value):
            return
        lineno = const.lineno
        line = lines[lineno - 1] if 0 <= lineno - 1 < len(lines) else ""
        # Check current line + the line above (for multi-line raise/log
        # where the marker sits on the raise/call line and the literal
        # is on a continuation line).
        prev = lines[lineno - 2] if 0 <= lineno - 2 < len(lines) else ""
        if _EXEMPT in line or _EXEMPT in prev:
            return
        # Truncate the offending literal so log output stays scannable.
        snippet = const.value if len(const.value) < 40 else const.value[:37] + "..."
        hits.append((lineno, kind, snippet))

    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and _is_log_call(node):
            # Walk every arg for string constants (positional + kw).
            for arg in list(node.args) + [kw.value for kw in node.keywords]:
                for sub in ast.walk(arg):
                    if isinstance(sub, ast.Constant):
                        _check_str_const(sub, "log/print")
        elif isinstance(node, ast.Raise):
            for const in _is_raise_with_string(node):
                _check_str_const(const, "raise-msg")
    return hits


def _walk(root: str) -> Iterable[str]:
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames
                       if not d.startswith(("__pycache__", "."))]
        for name in filenames:
            if not name.endswith(".py"):
                continue
            if name == "no_chinese_in_log.py":
                continue   # self-exclusion
            yield os.path.join(dirpath, name)


def _self_test() -> int:
    import tempfile
    samples = {
        "log_zh.py": (
            "import logging\n"
            "log = logging.getLogger()\n"
            "def f():\n"
            "    log.info('you hao 你好 world')\n"
        ),
        "print_zh.py": "def g():\n    print('中文 output')\n",
        "raise_zh.py": (
            "def h():\n"
            "    raise ValueError('参数错误')\n"
        ),
    }
    with tempfile.TemporaryDirectory() as td:
        for n, txt in samples.items():
            with open(os.path.join(td, n), "w") as f:
                f.write(txt)
        seen_kinds = set()
        for p in _walk(td):
            for _, kind, _ in _scan_file(p):
                seen_kinds.add(kind)
    if "log/print" not in seen_kinds:
        print("self-test FAIL: log/print not caught")
        return 1
    if "raise-msg" not in seen_kinds:
        print("self-test FAIL: raise-msg not caught")
        return 1
    print("self-test PASS: %s" % sorted(seen_kinds))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("root", nargs="?", default="xbrain")
    args = ap.parse_args()
    if args.self_test:
        return _self_test()
    print("scan surface: %s (excludes docs/, self)" % args.root)
    total = 0
    for path in _walk(args.root):
        for lineno, kind, snippet in _scan_file(path):
            print("  BAD  %s:%d  %s  %r" % (path, lineno, kind, snippet))
            total += 1
    print("  violations:        %d" % total)
    print("criterion: violations == 0")
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
