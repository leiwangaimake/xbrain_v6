"""CFG-FZ-13 bidirectional-diff meta test: four families + variant."""

import pytest

from xbrain.boot.freeze.meta import (
    DOC_QC, DOC_SP, DOC_S, DOC_AS,
    DEFERRED_SP, DEFERRED_S, DEFERRED_QC, DEFERRED_AS,
    EXEMPT_SP,
    bidirectional_diff, impl_sp, impl_s, impl_qc, impl_as,
)


def test_sp_bidirectional_diff_empty():
    fwd, rev = bidirectional_diff("SP")
    assert not fwd, "SP forward diff non-empty: %s" % sorted(fwd)
    assert not rev, "SP reverse diff non-empty: %s" % sorted(rev)


def test_s_bidirectional_diff_empty():
    fwd, rev = bidirectional_diff("S")
    assert not fwd, "S forward diff non-empty: %s" % sorted(fwd)
    assert not rev, "S reverse diff non-empty: %s" % sorted(rev)


def test_qc_bidirectional_diff_empty():
    """All 17 QC rules must be implemented and no extras."""
    fwd, rev = bidirectional_diff("QC")
    assert not fwd, "QC forward diff non-empty: %s" % sorted(fwd)
    assert not rev, "QC reverse diff non-empty: %s" % sorted(rev)


def test_as_bidirectional_diff_empty():
    fwd, rev = bidirectional_diff("AS")
    assert not fwd, "AS forward diff non-empty: %s" % sorted(fwd)
    assert not rev, "AS reverse diff non-empty: %s" % sorted(rev)


# CFG-FZ-13 variant: add QC-18 to doc without adding runner -> forward non-empty
def test_variant_qc18_added_to_doc_forward_diff_reports():
    """Simulate adding QC-18 to 13 S8.3 without updating the runner.
    Meta test must report the forward diff non-empty."""
    mutated_doc = DOC_QC | {"QC-18"}
    fwd, rev = bidirectional_diff("QC", doc_override=mutated_doc)
    assert "QC-18" in fwd
    assert not rev


def test_variant_reverse_diff_catches_spurious_impl():
    """A runner implementing a rule not in the doc surfaces on the
    reverse diff. Simulate by dropping QC-1 from the doc."""
    mutated_doc = DOC_QC - {"QC-1"}
    fwd, rev = bidirectional_diff("QC", doc_override=mutated_doc)
    assert "QC-1" in rev


def test_impl_qc_covers_all_17():
    assert impl_qc() == frozenset("QC-%d" % i for i in range(1, 18))


def test_impl_sp_matches_g_registry():
    impl = impl_sp()
    # Must be a subset of the doc.
    assert impl.issubset(DOC_SP)
    # Currently implemented set exactly: SP-1, SP-2, SP-5, SP-11.
    # This test locks the state so any silent addition/removal in
    # g_safety_range._REGISTRY surfaces here.
    assert impl == {"SP-1", "SP-2", "SP-5", "SP-11"}


def test_impl_as_only_as7():
    assert impl_as() == {"AS-7"}


def test_impl_s_empty_today():
    assert impl_s() == frozenset()


def test_deferred_sp_documented():
    """Every deferred rule is a real doc-side rule and NOT already implemented."""
    for r in DEFERRED_SP:
        assert r in DOC_SP, "deferred %r not in doc" % r
        assert r not in impl_sp(), \
            "%r is implemented but marked deferred" % r


def test_exempt_sp_documented():
    for r in EXEMPT_SP:
        assert r in DOC_SP


def test_unknown_family_raises():
    with pytest.raises(KeyError):
        bidirectional_diff("XX")
