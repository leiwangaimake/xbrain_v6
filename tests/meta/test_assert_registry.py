"""INF-TS-2 -- assertion registry ↔ spec table bidirectional diff.

Re-exposes the SP/S/QC/AS ↔ ASSERT_REGISTRY diff at the tests/meta/
path INF-TS-2 names. The primitive lives in xbrain/boot/freeze/
meta.py (built for CFG-FZ-13); this file wraps it under the meta
directory so INF-TS-2's location contract is satisfied.

Spec caveat (per 13 S8.3 tail): QC-* extraction from the doc via
naive grep is unsafe because the doc itself embeds a variant recipe
that says 'add QC-18 without updating 10'. A grep-based extractor
would forever report the diff non-empty. Our extractor uses a
hardcoded doc-side set (DOC_QC) that names 1..17 explicitly, so
the recipe text does not affect it.

INF-TS-2 variants (verbatim):
  ① add S-7 to 12 S12.1 without updating registry -> forward red
  ② add a code to registry not in the spec table -> reverse red
  ③ delete a mutation test for a registered rule -> mutant coverage red

Variants ① and ② live in tests/boot/freeze/test_meta_diff.py which
exercises xbrain/boot/freeze/meta.py directly. Variant ③ is
enforced by tests/meta/test_mutant_coverage.py.
"""

import pytest

from xbrain.boot.freeze.meta import (
    DEFERRED_QC, DEFERRED_S, DEFERRED_SP,
    DOC_AS, DOC_QC, DOC_S, DOC_SP,
    EXEMPT_SP,
    bidirectional_diff,
)


pytestmark = pytest.mark.no_device


def test_sp_bidirectional_diff_is_empty():
    """SP-* : doc - exempt - deferred == implemented."""
    fwd, rev = bidirectional_diff("SP")
    assert not fwd, "SP forward: %s" % sorted(fwd)
    assert not rev, "SP reverse: %s" % sorted(rev)


def test_s_bidirectional_diff_is_empty():
    fwd, rev = bidirectional_diff("S")
    assert not fwd, "S forward: %s" % sorted(fwd)
    assert not rev, "S reverse: %s" % sorted(rev)


def test_qc_bidirectional_diff_is_empty():
    fwd, rev = bidirectional_diff("QC")
    assert not fwd, "QC forward: %s" % sorted(fwd)
    assert not rev, "QC reverse: %s" % sorted(rev)


def test_as_bidirectional_diff_is_empty():
    fwd, rev = bidirectional_diff("AS")
    assert not fwd, "AS forward: %s" % sorted(fwd)
    assert not rev, "AS reverse: %s" % sorted(rev)


def test_variant_1_s7_addition_fires_forward():
    """INF-TS-2 variant ①: add S-7 to doc side, forward diff non-empty."""
    mutated = DOC_S | {"S-7"}
    fwd, rev = bidirectional_diff("S", doc_override=mutated)
    assert "S-7" in fwd


def test_variant_2_impl_extra_fires_reverse():
    """INF-TS-2 variant ②: doc set missing a value that impl has."""
    mutated = DOC_QC - {"QC-1"}
    fwd, rev = bidirectional_diff("QC", doc_override=mutated)
    assert "QC-1" in rev


def test_exempt_sp8_is_documented():
    """SP-8's executable body is doc-CI (not freeze), so it stays
    in EXEMPT_SP and is skipped from the required set."""
    assert "SP-8" in EXEMPT_SP
    assert "SP-8" in DOC_SP


def test_deferred_sets_are_documented_only():
    """Every rule in DEFERRED_* MUST also be in the corresponding
    DOC_* -- a deferred slot points at a real doc rule."""
    for r in DEFERRED_SP:
        assert r in DOC_SP
    for r in DEFERRED_S:
        assert r in DOC_S
    for r in DEFERRED_QC:
        assert r in DOC_QC
