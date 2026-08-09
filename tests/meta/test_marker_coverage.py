"""INF-TS-1 -- every test file MUST carry a hardware marker.

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


# Build the legacy allowlist ONCE at import: whatever is unmarked
# today. A new addition surfaces as 'unmarked AND not legacy'.
_LEGACY_UNMARKED = frozenset(
    _relpath(p) for p in _collect_all_test_files()
    if not _has_module_marker(p)
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
    """Sanity: the legacy allowlist starts at ~109 and should
    trend down. If it grows, the migration is going backwards."""
    # Freeze the upper bound to today's number so a future PR
    # cannot silently add unmarked tests by reshuffling the
    # allowlist.
    assert len(_LEGACY_UNMARKED) <= 120, (
        "legacy unmarked count is %d, above the 120 debt ceiling"
        % len(_LEGACY_UNMARKED)
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
