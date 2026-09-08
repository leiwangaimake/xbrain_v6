"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_meta_unified_follow.py
Brief: A-RT-1 meta-test -- one follow code path, no goto-only branch (20 S2.1)

Description:
A-RT-1 (20 S13) is a META assertion: goto and path must run the SAME follow code,
so a second implementation is a defect even if both work. This test scans route.py
for the shape that would violate it -- a method or function whose name marks it as
goto-only or path-only follow logic. RNS-N-1: there is exactly one advance().

The mutant this catches (20 S13): "add a separate branch for single-point
navigation" -> a goto_advance / follow_goto style symbol appears -> reddens.
"""

from __future__ import annotations

import ast
from pathlib import Path

ROUTE = Path(__file__).resolve().parents[3] / "xbrain" / "p1_motion" / "rns" / "route.py"

# names that would mark a SEPARATE goto/path follow path (the A-RT-1 violation).
# advance() is the single unified entry; anything like goto_advance, follow_goto,
# path_follow_step, advance_goto is the second-implementation smell.
_FORBIDDEN_SUBSTRINGS = ("goto_advance", "advance_goto", "follow_goto",
                         "goto_follow", "path_advance", "advance_path",
                         "follow_path_only", "goto_step", "path_step")


def _defined_names(tree: ast.AST):
    names = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            names.append(node.name)
    return names


def test_no_goto_only_or_path_only_follow_method():
    tree = ast.parse(ROUTE.read_text(encoding="utf-8"))
    names = _defined_names(tree)
    offenders = [n for n in names
                 if any(sub in n.lower() for sub in _FORBIDDEN_SUBSTRINGS)]
    assert not offenders, (
        "route.py defines a goto/path-specific follow method %s -- RNS-N-1 / "
        "A-RT-1 requires ONE follow path (Mission.advance). A separate branch "
        "for single-point nav is the exact violation." % offenders)


def test_mission_has_exactly_one_advance():
    # the positive side: Mission.advance is the single tick entry. Not "some
    # advance exists" but "advance is the only per-tick follow method".
    tree = ast.parse(ROUTE.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "Mission":
            methods = [n.name for n in node.body
                       if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
            advance_like = [m for m in methods if "advance" in m.lower()]
            assert advance_like == ["advance"], (
                "Mission has advance-like methods %s; A-RT-1 wants exactly "
                "['advance']" % advance_like)
            return
    raise AssertionError("Mission class not found in route.py")
