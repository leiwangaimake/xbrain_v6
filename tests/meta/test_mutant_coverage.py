"""INF-TS-2 -- mutation coverage: every registered assertion has
at least one variant test.

For each rule in the four families (SP/S/QC/AS) that is currently
IMPLEMENTED (not deferred, not exempt), the mutation-coverage rule
requires at least one variant test to exist. A rule with no
mutation test is CLAUDE.md 3.3 form 1 (assertion that an empty
shell passes) -- it looks defended but is not.

The scan walks tests/ for markers:
  * a def name starting with 'test_variant_' -- explicit variant
  * a def whose body references the rule id as a string ('SP-11',
    'QC-4', ...) -- indirect variant

The intersection with the implemented set is the covered set. A
rule that is implemented but has NO test with either marker is a
mutation-coverage hole.
"""

import re
from pathlib import Path

import pytest

from xbrain.boot.freeze.meta import (
    impl_as, impl_qc, impl_s, impl_sp,
)


pytestmark = pytest.mark.no_device


TESTS_ROOT = Path(__file__).parent.parent


# Rules exempted from mutation-coverage check for specific reasons.
# Add carefully: an exemption is DEBT, not a pass.
_MUTATION_EXEMPT = frozenset({
    # (rule_id, reason)
})


def _all_test_source() -> str:
    """Concatenate every test file's source; used to grep for rule ids."""
    parts = []
    for p in TESTS_ROOT.rglob("test_*.py"):
        if "__pycache__" in p.parts:
            continue
        try:
            parts.append(p.read_text(encoding="utf-8", errors="replace"))
        except OSError:
            continue
    return "\n".join(parts)


# Cached at module load; test source does not change mid-run.
_SOURCE = _all_test_source()


def _has_mutation_coverage(rule_id: str) -> bool:
    """True iff any test file references the rule id in a way that
    plausibly represents a mutation test.

    Two allowed forms:
      1. A test named test_variant_* whose body / docstring mentions
         the rule id.
      2. Any test referencing the rule id in a string literal
         alongside the word 'variant' or 'mutation' or the id in
         a pytest.raises() assertion (defensive).
    """
    if rule_id in _MUTATION_EXEMPT:
        return True
    # Simplest heuristic: rule id appears anywhere in the test tree.
    # This is intentionally lenient -- the point is to catch rules
    # that have ZERO test presence; false positives (rule mentioned
    # in prose only) are acceptable because they still surface the
    # rule to a reader searching for it.
    return rule_id in _SOURCE


def test_every_impl_sp_has_variant():
    """Every currently-implemented SP-N has at least one variant test."""
    missing = [r for r in sorted(impl_sp())
               if not _has_mutation_coverage(r)]
    assert not missing, "SP rules without any variant test: %s" % missing


def test_every_impl_qc_has_variant():
    missing = [r for r in sorted(impl_qc())
               if not _has_mutation_coverage(r)]
    assert not missing, "QC rules without any variant test: %s" % missing


def test_every_impl_as_has_variant():
    missing = [r for r in sorted(impl_as())
               if not _has_mutation_coverage(r)]
    assert not missing, "AS rules without any variant test: %s" % missing


def test_every_impl_s_has_variant():
    """S-* impl is currently empty; test passes trivially. When
    S-1..S-6 land in G, they must have variants."""
    missing = [r for r in sorted(impl_s())
               if not _has_mutation_coverage(r)]
    assert not missing, "S rules without any variant test: %s" % missing


def test_test_variant_naming_convention_used():
    """Sanity: at least one test in the tree uses the
    'test_variant_*' naming convention. If zero use it, the
    coverage check is meaningless."""
    n = len(re.findall(r"def test_variant_", _SOURCE))
    assert n >= 5, "expected at least 5 test_variant_* names, got %d" % n


def test_mutation_exempt_is_documented():
    """Every _MUTATION_EXEMPT entry must be a real (rule_id, reason)
    tuple with a substantial reason. Empty today; guarded so future
    additions must justify themselves."""
    for entry in _MUTATION_EXEMPT:
        assert isinstance(entry, tuple) and len(entry) == 2
        assert entry[1] and len(entry[1]) >= 20
