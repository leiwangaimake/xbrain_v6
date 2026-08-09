"""CFG-DC-1 / INF-QD-1 -- MAP-1 alignment-diff scanner tests."""

import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).parent.parent.parent / "scripts" / "doccheck" / "map1_scan.py"


def _run(*args):
    return subprocess.run([sys.executable, str(SCRIPT), *args],
                          capture_output=True, text=True)


def test_self_test_passes():
    """Injection self-test: forward + reverse diffs both fire."""
    r = _run("--self-test")
    assert r.returncode == 0, r.stdout + r.stderr


def test_scan_produces_deterministic_report():
    """The real scan must run without crash + name every code with drift.

    NOTE: this test does NOT require the repo to currently pass
    MAP-1. As of the CFG-DC-1 landing, 10 S3.3.6 alignment table
    is missing 7 codes (E_CONFIG_LOCKED, E_FENCE_INVALID, E_LOCKED,
    E_PROTO_VERSION, E_SAFETY_LINK_LOST, E_TIMEOUT, E_UNHEALTHY).
    That is exactly the drift MAP-1 was designed to expose; the
    remediation is a doc update, not a code change.
    """
    r = _run()
    # Non-zero exit expected today (doc has known drift).
    # We assert the report includes the specific known drifts so a
    # future doc fix that reduces the drift set surfaces here.
    for code in ("E_CONFIG_LOCKED", "E_FENCE_INVALID", "E_LOCKED",
                 "E_PROTO_VERSION", "E_SAFETY_LINK_LOST",
                 "E_TIMEOUT", "E_UNHEALTHY"):
        assert code in r.stdout, "expected %s in report" % code


def test_scan_recognises_alignment_table_codes():
    """B side must recognise the 3 codes actually listed in the doc."""
    r = _run()
    # 'table B codes: 3' should appear.
    assert "codes:           3" in r.stdout


def test_scan_row_count_from_a_is_29():
    """§3.3.6 has 29 failure rows (post 2026-08-05 additions)."""
    r = _run()
    assert "rows with ecode: 29" in r.stdout


def test_reports_scan_surface():
    """CHK-2-51 scan-surface requirement: the tool MUST print the surface."""
    r = _run()
    assert "scan surface:" in r.stdout


# Programmatic API tests (not through subprocess).

def test_parse_and_diff_direct():
    """Import the module directly and exercise parse + diff on the
    real doc without going through subprocess."""
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        import map1_scan as m
    finally:
        sys.path.pop(0)
    text = (Path(__file__).parent.parent.parent / "docs" / "10-顶层设计.md").read_text()
    a_sec, b_sec = m._find_a_and_b_sections(text)
    a = m.parse_table_a(a_sec)
    b = m.parse_table_b(b_sec)
    d = m.diff(a, b)
    assert len(a) == 29
    assert "E_CONFIG_INVALID" in b
    assert d, "expected non-empty diff today (known doc drift)"


def test_diff_empty_on_matched_tables():
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        import map1_scan as m
    finally:
        sys.path.pop(0)
    a = {"1": "E_ONE", "2": "E_TWO"}
    b = {"E_ONE": frozenset({"1"}), "E_TWO": frozenset({"2"})}
    assert m.diff(a, b) == {}


def test_diff_forward_fires_on_a_extra():
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        import map1_scan as m
    finally:
        sys.path.pop(0)
    a = {"1": "E_ONE", "2": "E_ONE"}       # both use E_ONE
    b = {"E_ONE": frozenset({"1"})}         # B forgot row 2
    d = m.diff(a, b)
    assert "2" in d["E_ONE"]["forward"]


def test_diff_reverse_fires_on_b_extra():
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        import map1_scan as m
    finally:
        sys.path.pop(0)
    a = {"1": "E_ONE"}
    b = {"E_ONE": frozenset({"1", "999"})}  # B points at non-existent 999
    d = m.diff(a, b)
    assert "999" in d["E_ONE"]["reverse"]


def test_a_row_inheritance_from_tongshang():
    """'同上' means 'same ecode as previous row' -- must inherit."""
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        import map1_scan as m
    finally:
        sys.path.pop(0)
    section = (
        "| 1 | X | Y | R | Z | E_STORAGE_CORRUPT | ref |\n"
        "| 2 | X | Y | R | Z | 同上 | ref |\n"
    )
    a = m.parse_table_a(section)
    assert a.get("1") == "E_STORAGE_CORRUPT"
    assert a.get("2") == "E_STORAGE_CORRUPT"
