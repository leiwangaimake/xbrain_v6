"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: meta.py
Brief: CFG-FZ-13 meta test primitives -- doc <-> registry bidirectional diff

Description:
CFG-FZ-13 requires that the set of assertion rule names actually
IMPLEMENTED in the freeze pipeline matches the set of rule names
DECLARED in the four canonical spec tables:

  SP-*  11 S9.6                (with SP-8 explicitly exempt: doc-CI's job)
  S-*   12 S12.1               (only S-1 ~ S-6 exist)
  QC-*  13 S8.3                (17 rules, no exemption)
  AS-*  11 S8.13.1 (bounds)    (only AS-7 is a STARTUP assertion; the
                                other AS-* are RUNTIME contracts and
                                are out of scope for freeze assertion G)

Bidirectional diff = both:
  forward  = (doc_rules - exempt - deferred) - impl_rules   (must be empty)
  reverse  = impl_rules - doc_rules                          (must be empty)

Any surviving element in either diff is a defect:
  * forward: a rule the doc declares that no assertion implements
  * reverse: an assertion that implements a rule not in the doc
             (spurious / hallucinated rule name)

DEFERRED set carries every rule the doc declares but the CURRENT
codebase has not yet implemented. Each entry MUST carry a reason
so a future contributor sees why the exemption exists and whether
it should still be exempt. The DEFERRED set is intended to shrink
over time as CFG-FZ-N items land.

EXEMPT set carries rules the doc EXPLICITLY marks as out of scope
for the freeze assertion executor (SP-8 is the canonical example:
its executable body is doc-CI, not freeze). EXEMPT is expected to
stay small and rarely change.

Contract:
  DOC_SP / DOC_S / DOC_QC / DOC_AS   frozensets of every rule name
                                     the doc declares for that family
  EXEMPT_*                           subset of DOC_* explicitly out
                                     of scope for the freeze runner
  DEFERRED_*                         subset of DOC_* awaiting a
                                     future CFG-FZ-N item; each
                                     annotated with a reason
  impl_*()                           extract the currently-registered
                                     set from the implementation

Public entry point:
  bidirectional_diff(family)         returns (forward, reverse) diff
                                     tuples for the named family.
                                     Empty diffs = the meta test
                                     would pass for that family.

CFG-FZ-13 variant (verbatim):
  Add QC-18 to 13 S8.3 without adding a matching runner in
  k_quadruped_qc.py -> forward diff must be non-empty.

The variant maps to a test that mutates DOC_QC (add QC-18) and
verifies bidirectional_diff("QC") returns a non-empty forward set.
"""

# frozenset for immutable rule sets.
from typing import FrozenSet, Tuple

# Import the implementation-side registries so impl_*() functions
# stay in step with what actually runs at freeze time.
from xbrain.boot.freeze.assertions.g_safety_range import _REGISTRY as _G_REGISTRY
# k_quadruped_qc doesn't have a _REGISTRY tuple -- its QC-N checks
# are named _check_qcN. Enumerate them dynamically so a new QC row
# added there is picked up automatically.
from xbrain.boot.freeze.assertions import k_quadruped_qc as _k_module


# ---------------------------------------------------------------------------
# Doc-side sets (hardcoded per CFG-FZ-13 verbatim)
# ---------------------------------------------------------------------------

# 11 S9.6: SP-1 .. SP-11 (SP-8 marked doc-CI, other gaps to be filled).
# The full 1..11 range is the closed set the doc declares regardless
# of whether the current codebase implements them all.
DOC_SP: FrozenSet[str] = frozenset(
    "SP-%d" % i for i in range(1, 12)
)

# 12 S12.1: S-1 .. S-6 exactly.
DOC_S: FrozenSet[str] = frozenset(
    "S-%d" % i for i in range(1, 7)
)

# 13 S8.3: QC-1 .. QC-17 exactly.
DOC_QC: FrozenSet[str] = frozenset(
    "QC-%d" % i for i in range(1, 18)
)

# 11 S8.13.1 upper-bound table: only AS-7 is a startup assertion.
# The other AS-* rows (AS-1 .. AS-6, AS-8, AS-9) are runtime
# service contracts (see 11 S8.13.1 verbatim). They are NOT in the
# closed set the meta test checks against.
DOC_AS: FrozenSet[str] = frozenset({"AS-7"})


# ---------------------------------------------------------------------------
# Exempt sets (doc-declared but out of scope for the freeze runner)
# ---------------------------------------------------------------------------

# SP-8: 11 S9.6 verbatim says its executable body is doc-CI, not
# freeze. Documented here so the meta test does not require an
# assertion runner for SP-8.
EXEMPT_SP: FrozenSet[str] = frozenset({"SP-8"})

# No other family has EXEMPT rules today.
EXEMPT_S: FrozenSet[str] = frozenset()
EXEMPT_QC: FrozenSet[str] = frozenset()
EXEMPT_AS: FrozenSet[str] = frozenset()


# ---------------------------------------------------------------------------
# Deferred sets (doc-declared but awaiting a future CFG-FZ-N item)
# ---------------------------------------------------------------------------

# SP-* deferrals. Each with a one-line reason so a contributor
# considering removal sees the actual blocker.
DEFERRED_SP: FrozenSet[str] = frozenset({
    # SP-3: needs provenance tracking (which layer defined the key);
    #       provenance is present in build_overlay but not exposed
    #       through a check surface for SP-3 specifically.
    "SP-3",
    # SP-4: needs event bus + warn-level path (SP-4 does NOT reject
    #       startup; it lands a warn event). Freeze pipeline emits
    #       no events today.
    "SP-4",
    # SP-6: needs the speed-gate f() function to be available at
    #       freeze time; currently only implemented in P1 runtime.
    "SP-6",
    # SP-7: cruise/transit string ban -- currently caught by C's
    #       deprecated-string check, not a stand-alone SP-7 runner.
    "SP-7",
    # SP-9 / SP-10: need specific config keys not yet in the tree
    #       (gait_limits, per-gait v_max). To be added when the
    #       corresponding config skeleton lands.
    "SP-9", "SP-10",
})

# S-* deferrals: all six. The S-* family is designed to live under
# G's runner (per 10 S5.4.4 "断言 G 的求值范围逐字写着 S-1 ~ S-6")
# but the current G runner covers only the SP subset; S rules will
# be added in a follow-up CFG-FZ-N item.
DEFERRED_S: FrozenSet[str] = frozenset({
    "S-1", "S-2", "S-3", "S-4", "S-5", "S-6",
})

# QC-* deferrals: none. All 17 QC rules are implemented in
# k_quadruped_qc.py under CFG-FZ-12.
DEFERRED_QC: FrozenSet[str] = frozenset()

# AS-* deferrals: none. Only AS-7 is required and it is implemented.
DEFERRED_AS: FrozenSet[str] = frozenset()


# ---------------------------------------------------------------------------
# Implementation-side extractors
# ---------------------------------------------------------------------------

def impl_sp() -> FrozenSet[str]:
    """Return the set of SP-N rules the current G registry runs.

    Reads _G_REGISTRY from g_safety_range so the meta test stays
    in step with the actual runner list -- adding a row there is
    picked up here automatically.
    """
    # _G_REGISTRY carries _CheckRow tuples; row.name is the rule ID.
    return frozenset(row.rule for row in _G_REGISTRY
                     if row.rule.startswith("SP-"))


def impl_s() -> FrozenSet[str]:
    """Return the set of S-N rules the current G registry runs.
    Empty today; expands when S-1..S-6 land in G."""
    return frozenset(row.rule for row in _G_REGISTRY
                     if row.rule.startswith("S-"))


def impl_qc() -> FrozenSet[str]:
    """Return the set of QC-N rules k_quadruped_qc.py exposes.

    Enumeration is dynamic: any function _check_qcN in the module
    counts as an implementation. This keeps the meta test in step
    with the module without a second hardcoded list.
    """
    # Introspect the module's function names for _check_qc<N>.
    # A pattern _check_qcN maps to rule QC-N.
    names = set()
    for name in dir(_k_module):
        if name.startswith("_check_qc"):
            # Extract the numeric tail; skip malformed names.
            tail = name[len("_check_qc"):]
            if tail.isdigit():
                names.add("QC-%s" % tail)
    return frozenset(names)


def impl_as() -> FrozenSet[str]:
    """Return the set of AS-N rules the current G registry runs."""
    return frozenset(row.rule for row in _G_REGISTRY
                     if row.rule.startswith("AS-"))


# ---------------------------------------------------------------------------
# Public entry: bidirectional diff
# ---------------------------------------------------------------------------

# Family -> (doc, exempt, deferred, impl_fn) lookup. Keeps
# bidirectional_diff generic across the four families.
_FAMILIES = {
    "SP": (DOC_SP, EXEMPT_SP, DEFERRED_SP, impl_sp),
    "S":  (DOC_S,  EXEMPT_S,  DEFERRED_S,  impl_s),
    "QC": (DOC_QC, EXEMPT_QC, DEFERRED_QC, impl_qc),
    "AS": (DOC_AS, EXEMPT_AS, DEFERRED_AS, impl_as),
}


def bidirectional_diff(family: str,
                       doc_override=None) -> Tuple[FrozenSet[str],
                                                    FrozenSet[str]]:
    """Return (forward, reverse) diff for the named family.

    forward = (doc - exempt - deferred) - impl
              rules the doc requires but no runner implements
    reverse = impl - doc
              rules implemented that have no doc counterpart

    Both must be empty for the meta test to pass (with the
    exempt / deferred escape hatch documented above).

    doc_override: for the CFG-FZ-13 variant test, injecting a
    modified doc set (e.g. add QC-18) verifies the forward
    direction fires.
    """
    if family not in _FAMILIES:
        raise KeyError("unknown family %r; expected one of %s"
                       % (family, sorted(_FAMILIES)))
    doc, exempt, deferred, impl_fn = _FAMILIES[family]
    # Doc override wins for tests; otherwise use the canonical set.
    if doc_override is not None:
        doc = frozenset(doc_override)
    impl = impl_fn()
    # Effective doc set = doc - exempt - deferred. Anything left is
    # what MUST be implemented right now.
    required = doc - exempt - deferred
    forward = required - impl
    reverse = impl - doc
    return forward, reverse
