"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_map1_scan.py
Brief: common tests -- map1 scan

Description:
CFG-DC-1 / INF-QD-1 -- MAP-1 alignment-diff scanner tests.
"""


import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).parent.parent.parent / "scripts" / "doccheck" / "map1_scan.py"


def _run(*args):
    return subprocess.run([sys.executable, str(SCRIPT), *args],
                          capture_output=True, text=True)


def _module():
    """Import the scanner directly, for the parse/diff API tests."""
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        import map1_scan
        return map1_scan
    finally:
        sys.path.pop(0)


def test_self_test_passes():
    """Injection self-test: forward + reverse diffs both fire."""
    r = _run("--self-test")
    assert r.returncode == 0, r.stdout + r.stderr


def test_scan_produces_deterministic_report():
    """The real scan must pass, and must still NAME what it did not diff.

    *** This test used to assert the opposite, and the note it carried
    was wrong. It read: "the alignment table is missing 7 codes
    (E_CONFIG_LOCKED, E_FENCE_INVALID, E_LOCKED, E_PROTO_VERSION,
    E_SAFETY_LINK_LOST, E_TIMEOUT, E_UNHEALTHY) ... the remediation is
    a doc update". It is not. Table B's first column is headed
    "11 S13.15 codes"; S13.15 is group L and its entire membership is
    E_QOS_VIOLATION / E_CONFIG_INVALID / E_STORAGE_CORRUPT
    (xbrain/common/errors/codes.yaml, group: L). The seven above are
    group B / F / G / K codes, so writing them into a table titled
    "S13.15 codes" would state something false. The scanner was keying
    its diff on the union of both tables instead of on table B, which
    is what MAP-1 says verbatim -- so the gate could not be cleared by
    any edit to either table.

    The seven must still appear, now as OUT-OF-SCOPE rather than as
    findings: a criterion that silently narrows its surface is how
    "nothing was reported" turns into "nothing was checked".
    """
    r = _run()
    assert r.returncode == 0, r.stdout + r.stderr
    for code in ("E_CONFIG_LOCKED", "E_FENCE_INVALID", "E_LOCKED",
                 "E_PROTO_VERSION", "E_SAFETY_LINK_LOST",
                 "E_TIMEOUT", "E_UNHEALTHY"):
        assert ("OUT-OF-SCOPE %s" % code) in r.stdout, (
            "%s must still be named, as out-of-scope" % code)
    assert "codes with non-empty diff: 0" in r.stdout


def test_a_code_only_table_a_names_is_out_of_scope_not_a_finding():
    """The narrowing, pinned: B is answerable only for codes it carries.

    Keyed on the union, E_TIMEOUT (group K) made table B answerable
    for a code about which it says nothing -- permanently red, and a
    permanently red criterion gets relaxed into a permanently green
    one (CLAUDE.md 3.2 shape 2).

    MUTATION: put `set(a_by_code) | set(table_b)` back as diff()'s key
    set -> this goes red, and so does
    test_scan_produces_deterministic_report.
    """
    m = _module()
    a = {"1": "E_ONE", "9": "E_OTHER_GROUP"}
    b = {"E_ONE": frozenset({"1"})}
    assert m.diff(a, b) == {}, "a code B has no row for is not a finding"
    assert m.out_of_scope(a, b) == {"E_OTHER_GROUP": frozenset({"9"})}


def test_out_of_scope_never_hides_a_code_table_b_does_carry():
    """The other half: carrying a row makes a code answerable.

    Without this, "out of scope" would be a place to put anything
    inconvenient -- which is the relaxation this whole change exists
    to avoid.
    """
    m = _module()
    a = {"1": "E_ONE", "2": "E_ONE"}
    b = {"E_ONE": frozenset({"1"})}          # B forgot row 2
    assert m.out_of_scope(a, b) == {}, "E_ONE is in B, so never skipped"
    assert m.diff(a, b)["E_ONE"]["forward"] == frozenset({"2"})


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
    # Empty, and it must be empty for a reason that can be stated: every
    # code table B carries a row for agrees with A row-for-row. The codes
    # it carries no row for are reported separately, never folded in here.
    assert d == {}, d
    assert set(m.out_of_scope(a, b)).isdisjoint(b)


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
