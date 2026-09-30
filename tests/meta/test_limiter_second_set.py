"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_limiter_second_set.py
Brief: meta door -- no module under xbrain/ may mint a second gate.limiter set

Description:
The defect this door exists for, measured 2026-09-30:
xbrain/p1_motion/gate/audit.py carried

    # 12 S6.7 14-value limiter enum (verbatim).
    _LIMITER_VALUES = ("none", "f_speed", "g_targets", "h_heading", "i_rtk",
                       "hard_upper", "fence_soft", "fence_hard",
                       "rotation_permit", "profile", "estop", "hes",
                       "cmd_timeout", "source_deactivated")

and three green tests in tests/p1_motion/test_batch_a.py pinned it. Every part
of that comment was false. 12 S6.7 is titled 降档迟滞 and holds no enum; the
limiter table is 12 S6.8, which states it is a MIRROR of 11 S3.4 / S9.6.5 with
11 as the sole authority; and of the fourteen names only none / profile / estop
are members of the contract set. The other eleven were minted on the spot.

WHY A DOOR AND NOT JUST A DELETION
  The module was deleted, so the repository is clean today. A deletion does
  not keep it clean: the same shape can come back in any of the five P
  processes, and it comes back GREEN, because a unit test that imports the
  invented tuple and asserts on its own members passes no matter what the
  contract says. That is CLAUDE.md S3.2 form 1 -- the assertion has no way to
  disagree with the implementation, because it was written from it. The
  contract set is 11's F-19 frozen surface and 12 S6A.8 OB-1 forbids adding a
  member without review, so "a second list of limiter names" is never a local
  choice.

WHAT IT CHECKS
  Every module-level collection under xbrain/ whose NAME mentions LIMITER and
  whose VALUE is a literal collection of strings must have all its members
  inside xbrain.common.enums GATE_LIMITER. Literal covers tuple / list / set /
  dict keys, plus the one-call wrappers set(...) / frozenset(...) / tuple(...)
  / list(...) that take such a literal -- without that last case
  frozenset({...}) would be a one-character bypass.

  Names are matched, not values, on purpose. A value-based scan would have to
  decide what "looks like a limiter name" means, and every answer either
  misses the invented ones (they look like anything) or fires on unrelated
  string tuples. The name is what the author already told us.

SCAN SURFACE (CLAUDE.md S3.2 form 6 -- an undeclared count is unreadable)
  xbrain/**/*.py, module level only (a name inside a function body is not a
  shared set). Derived forms are out of reach by construction and that is
  correct: nav/host_gate.py writes _ORDER = list(GATE_LIMITER), which cannot
  drift from the authority because it IS the authority.

BOUNDARIES
  It does not check the ORDER of the contract set (that is
  tests/common/test_closed_sets.py, and order carries meaning here: 11 S9.6.5
  order is the attribution priority). It does not check the Chinese broadcast
  map (tests/common/test_limiter_cn.py, RS-LIM). It says one thing only: no
  second copy of the value set.

  This file is NOT in its own scan surface (surface is xbrain/, this is
  tests/), so the literal names quoted above cannot make it fire on itself --
  CLAUDE.md S3.2 form 3, 判据自伤.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

from xbrain.common.enums import GATE_LIMITER

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
XBRAIN = ROOT / "xbrain"

#: The wrappers that still yield a literal collection. Anything else (a Call to
#: a helper, a Subscript such as _SETS["gate_limiter"]) is derived from the
#: authority at runtime and is deliberately out of surface.
_LITERAL_WRAPPERS = frozenset({"set", "frozenset", "tuple", "list"})


def _string_members(value: ast.AST):
    """The string members of a literal collection node, or None if it is not one.

    None and an empty list are different answers: None means "not a literal
    collection, nothing to say", while [] means "a literal collection that
    happens to hold no strings". Collapsing them would make a non-collection
    silently count as checked.
    """
    if isinstance(value, ast.Call):
        # set({...}) / frozenset({...}) / tuple((...)) / list([...]): unwrap the
        # single literal argument and fall through to the cases below.
        func = value.func
        if (isinstance(func, ast.Name) and func.id in _LITERAL_WRAPPERS
                and len(value.args) == 1 and not value.keywords):
            return _string_members(value.args[0])
        return None
    if isinstance(value, ast.Dict):
        # dict KEYS are the value set when a module maps limiter -> something
        # (LIMITER_CN is exactly this shape).
        return [k.value for k in value.keys
                if isinstance(k, ast.Constant) and isinstance(k.value, str)]
    if isinstance(value, (ast.Tuple, ast.List, ast.Set)):
        return [e.value for e in value.elts
                if isinstance(e, ast.Constant) and isinstance(e.value, str)]
    return None


def _limiter_collections(source: str, label: str):
    """(label, name, members) for each module-level LIMITER-named literal set."""
    found = []
    tree = ast.parse(source)
    for node in tree.body:                      # module level only, see header
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        else:
            continue
        names = [t.id for t in targets if isinstance(t, ast.Name)]
        if not any("LIMITER" in n.upper() for n in names):
            continue
        members = _string_members(node.value)
        if members is None:
            continue
        found.append((label, names[0], members))
    return found


def _scan_xbrain():
    """Every LIMITER-named literal collection in the production Python tree."""
    out = []
    for path in sorted(XBRAIN.rglob("*.py")):
        text = path.read_text(encoding="utf-8")
        out.extend(_limiter_collections(text, str(path.relative_to(ROOT))))
    return out


def test_no_module_mints_a_limiter_value_outside_the_contract_set():
    """11 S9.6.5 / enums/sets.yaml gate_limiter is the only value set.

    MUTATION: add
        _LIMITER_VALUES = ("none", "f_speed", "hard_upper")
    at module level in any xbrain/**/*.py -> red, naming the file and the two
    out-of-set members. That is the audit.py defect reproduced exactly.
    """
    allowed = set(GATE_LIMITER)
    offences = []
    for label, name, members in _scan_xbrain():
        stray = sorted(set(members) - allowed)
        if stray:
            offences.append("%s: %s mints %s" % (label, name, stray))
    assert offences == [], (
        "a second gate.limiter value set exists (11 S9.6.5 is the closed set, "
        "12 S6A.8 OB-1 bars adding members):\n  " + "\n  ".join(offences))


def test_the_scanner_itself_catches_the_deleted_shape():
    """The door above is vacuous if the scanner finds nothing, so prove it finds.

    Without this, deleting the scanner body (or narrowing it until it matches
    no node) leaves the door GREEN and the repository unguarded -- the "one
    forever-green assertion" of CLAUDE.md S3.2 form 1. The source below is the
    verbatim body of the deleted xbrain/p1_motion/gate/audit.py constant.

    MUTATION: make _string_members return None unconditionally, or drop the
    Call branch, or restrict the name match -> this test goes red first.
    """
    deleted = (
        '_LIMITER_VALUES = (\n'
        '    "none", "f_speed", "g_targets", "h_heading", "i_rtk",\n'
        '    "hard_upper", "fence_soft", "fence_hard", "rotation_permit",\n'
        '    "profile", "estop", "hes", "cmd_timeout", "source_deactivated",\n'
        ')\n'
    )
    found = _limiter_collections(deleted, "<deleted audit.py>")
    assert len(found) == 1, "scanner missed a plain tuple assignment"
    stray = sorted(set(found[0][2]) - set(GATE_LIMITER))
    # The eleven invented names, and only those: none / profile / estop are
    # real members, so a scanner that flagged all fourteen would be wrong too.
    assert stray == ["cmd_timeout", "f_speed", "fence_hard", "fence_soft",
                     "g_targets", "h_heading", "hard_upper", "hes", "i_rtk",
                     "rotation_permit", "source_deactivated"]
    # frozenset({...}) must not be a bypass: same members, wrapped.
    wrapped = _limiter_collections(
        '_MORE_LIMITERS = frozenset({"hard_upper", "hes"})\n', "<wrapped>")
    assert wrapped and sorted(wrapped[0][2]) == ["hard_upper", "hes"]


def test_the_contract_set_is_what_the_door_measures_against():
    """A door measuring against a set that drifted is worse than no door.

    Pinned here as the three members the deleted module DID share with the
    contract, plus the two 12 S6.8 v0.7.1 additions (heading / clock) whose
    absence was the defect that section records. Full membership and order are
    tests/common/test_closed_sets.py's job, not this file's.

    MUTATION: point the import at a local literal instead of
    xbrain.common.enums -> this test still passes, which is why the real guard
    is test_closed_sets.py; what THIS one catches is a set that lost members.
    """
    members = set(GATE_LIMITER)
    for expected in ("none", "profile", "estop", "heading", "clock"):
        assert expected in members, (
            "%r left gate_limiter -- re-read 11 S9.6.5 before touching this "
            "door" % expected)
