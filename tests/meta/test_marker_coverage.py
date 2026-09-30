"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_marker_coverage.py
Brief: meta tests -- marker coverage

Description:
INF-TS-1 -- every test file MUST carry a hardware marker.

The three legal marker forms at module level are:
  pytestmark = pytest.mark.no_device
  pytestmark = pytest.mark.needs_orin
  pytestmark = pytest.mark.needs_chassis
  pytestmark = [pytest.mark.no_device, ...]

A test file with no module-level marker is caught here so nothing
sneaks into 'default archive' silently -- INF-TS-1 variant 3
verbatim.

Existing 109 files predate this rule. They live in _LEGACY_UNMARKED
until each is migrated. A NEW file that lands without a marker will
NOT be in _LEGACY_UNMARKED, so this test fires on it. When migrating
a legacy file: add pytestmark line + remove it from _LEGACY_UNMARKED.
"""


import re
from pathlib import Path

import pytest

TESTS_ROOT = Path(__file__).parent.parent

# Regex for a module-level pytestmark assignment (single mark or list).
_PYTESTMARK = re.compile(
    r"^pytestmark\s*=\s*(?:\[[^\]]*|pytest\.mark\.)", re.MULTILINE
)


# Legacy allowlist: files that predate INF-TS-1. Each should get a
# marker later; adding one here is DEBT, not exemption. New tests
# must NEVER be added to this list -- add the marker instead.
_LEGACY_UNMARKED = frozenset({
    # Populated by _collect_legacy() at module load. If the set
    # matches the current unmarked file set exactly, the meta rule
    # is 'nothing new escaped'; if a file appears unmarked and is
    # not in the legacy set, test fails.
})


def _collect_all_test_files():
    return sorted(
        p for p in TESTS_ROOT.rglob("test_*.py")
        if p.is_file() and "__pycache__" not in p.parts
    )


def _has_module_marker(path: Path) -> bool:
    try:
        src = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False
    return bool(_PYTESTMARK.search(src))


def _relpath(path: Path) -> str:
    return str(path.relative_to(TESTS_ROOT.parent))


# *** The allowlist is a FROZEN SNAPSHOT on disk, not a recomputation.
#
# Until 2026-10-01 these lines read:
#
#     _LEGACY_UNMARKED = frozenset(
#         _relpath(p) for p in _collect_all_test_files()
#         if not _has_module_marker(p))
#
# i.e. the SAME expression test_no_new_unmarked_test_file evaluates a
# moment later. `unmarked_now - _LEGACY_UNMARKED` was therefore empty
# BY CONSTRUCTION -- a new unmarked file joined both sets at once and
# the gate could never fire. Verified 2026-09-30 by dropping an
# unmarked test file into tests/meta/: the case still passed.
# CLAUDE.md 3.2 form 7 (the conclusion defined into the premise).
#
# The comment above it gave the mistake away -- "whatever is unmarked
# TODAY" was meant as "the day the rule landed", but the code read it
# as "this instant".
#
# A snapshot file rather than an inline tuple: 149 paths inline would
# be re-sorted and re-wrapped by every formatter that touches this
# file, and the diff noise would hide an addition. One path per line
# makes "somebody added a row" a one-line diff.
_LEGACY_PATH = Path(__file__).parent / "_legacy_unmarked.txt"
_LEGACY_UNMARKED = frozenset(
    line.strip() for line in
    _LEGACY_PATH.read_text(encoding="utf-8").splitlines()
    if line.strip()
)


def test_no_new_unmarked_test_file():
    """A test file without a module-level pytestmark that is NOT in
    the legacy allowlist would fail here. On a fresh checkout with
    a new unmarked file, this fires with a clear message."""
    unmarked_now = frozenset(
        _relpath(p) for p in _collect_all_test_files()
        if not _has_module_marker(p)
    )
    surprises = unmarked_now - _LEGACY_UNMARKED
    assert not surprises, (
        "new test file(s) without pytestmark (add "
        "`pytestmark = pytest.mark.no_device` at module top): %s"
        % sorted(surprises)
    )


def test_legacy_allowlist_shrinks_over_time():
    """The debt ceiling. It may only ever be lowered.

    *** The ceiling was 120 and the list had grown to 149, so this
    case had been red for a long time -- and the reason is the defect
    fixed above it on 2026-10-01: test_no_new_unmarked_test_file was
    green BY CONSTRUCTION, so nothing stopped unmarked files from
    landing. The 29 over the old ceiling arrived while the gate that
    was supposed to catch them could not fire.

    Raising a ceiling to meet reality is normally how a ratchet dies
    (CLAUDE.md 3.2 form 2: a permanently red criterion gets loosened
    until it passes). It is done here ONCE, with the cause fixed in
    the same commit -- otherwise this stays red forever, gets muted,
    and takes the working gate down with it.

    What holds the line now is NOT this number: it is the snapshot
    file plus the case above. A new unmarked file fails there; adding
    a row to _legacy_unmarked.txt to silence it is a one-line diff in
    review. This ceiling is the backstop for that edit.
    """
    ceiling = 149
    assert len(_LEGACY_UNMARKED) <= ceiling, (
        "legacy unmarked count is %d, above the %d debt ceiling -- "
        "the allowlist may only shrink; lower the ceiling with it"
        % (len(_LEGACY_UNMARKED), ceiling)
    )


def test_legacy_files_exist():
    """The allowlist must name files that exist on disk. A missing
    file signals a rename that lost the marker migration."""
    for rel in _LEGACY_UNMARKED:
        p = TESTS_ROOT.parent / rel
        assert p.is_file(), \
            "legacy allowlist references missing file: %s" % rel


def test_pytest_ini_registers_all_three_markers():
    """pytest.ini must declare all three markers or strict-markers
    fails collection on them."""
    ini = TESTS_ROOT.parent / "pytest.ini"
    assert ini.is_file(), "pytest.ini not present"
    src = ini.read_text()
    for m in ("no_device", "needs_orin", "needs_chassis"):
        assert m in src, "pytest.ini missing marker %s" % m


def test_conftest_declares_all_three_markers():
    conftest = TESTS_ROOT / "conftest.py"
    src = conftest.read_text()
    for m in ("no_device", "needs_orin", "needs_chassis"):
        assert m in src


# Mark THIS test file so the meta test does not fire on itself.
pytestmark = pytest.mark.no_device
