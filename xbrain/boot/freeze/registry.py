"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: registry.py
Brief: ASSERT_REGISTRY -- one row per startup assertion, driving both the
       execution order (ORD-1) and MANIFEST.assertions bidirectional-diff

Description:
CFG-FZ-1's core primitive. Every startup assertion (10 S5.4.4) is registered
here as an AssertSpec, and the pipeline (pipeline.py) iterates the registry
in DECLARATION order to produce MANIFEST.assertions. Two properties depend
on the registry being the single source of truth:

  ORD-1 (ordering)          -- assertions run in the declared order. J
                               (config file reachability) must precede A
                               (reference completeness) because A opens the
                               files J vouched for; A must precede M
                               (required-key completeness) because M walks
                               the fully-resolved tree A produced; etc.
                               A registry entry inserted out of order fires
                               the ORD-1 mutation ③ in the tests.

  MANIFEST symmetry         -- MANIFEST.assertions is exactly the set of
                               names in this registry. A row here that no
                               MANIFEST reflects (mutation ①) OR a
                               MANIFEST field that no registry row backs
                               (mutation ②) is a bidirectional-diff error.

*** What each subsequent CFG-FZ-N item does. This file is the FRAMEWORK
only; the individual assertions land per item:
  CFG-FZ-2  -> assertion J (config root + file reachability)
  CFG-FZ-3  -> assertions A (references complete) + M (required keys)
  CFG-FZ-4  -> assertions B (no dup) + C (cross-file) + D (identity)
  CFG-FZ-5  -> assertion E (safety-namespace hot-update disjoint)
  CFG-FZ-6..15 -> the rest (G / I / K / L / FV-ORG / C-6+MR-1 / ...)
Each item REPLACES the corresponding row's runner_stub with a real callable;
none of them change the registry SHAPE. Never inline an assertion body
outside of AssertSpec.runner -- an assertion whose body is a private if in
the pipeline is invisible to the registry, and the bidirectional-diff test
that keeps MANIFEST honest would silently pass.
"""

# dataclass(frozen=True) so an AssertSpec instance cannot be mutated at
# runtime; the ordered tuple below is likewise immutable. Both properties
# defend ORD-1 -- a caller cannot swap a runner or reorder the registry
# without a monkeypatch (tests use exactly that; production code cannot).
from dataclasses import dataclass, field
# Typing imports for AssertSpec (Any/Callable/Optional in the runner sig,
# Tuple for the immutable registry, Mapping stays around for future runners
# that need read-only dict-like ctx).
from typing import Any, Callable, Mapping, Optional, Tuple


# Real bodies live under xbrain/boot/freeze/assertions/. Import them here
# so the registry rows can point at the callable directly. Stubs (below)
# stand in for rows whose CFG-FZ-N item has not yet landed a real body.
from xbrain.boot.freeze.assertions.a_references import run as _a_run
from xbrain.boot.freeze.assertions.b_no_duplicates import run as _b_run
from xbrain.boot.freeze.assertions.c_cross_file import run as _c_run
from xbrain.boot.freeze.assertions.d_identity import run as _d_run
from xbrain.boot.freeze.assertions.c6_mr1_margins import run as _c6mr1_run
from xbrain.boot.freeze.assertions.e_hot_update_disjoint import run as _e_run
from xbrain.boot.freeze.assertions.f_qos_and_port import run as _f_run
from xbrain.boot.freeze.assertions.fv_org_enu import run as _fv_org_run
from xbrain.boot.freeze.assertions.g_safety_range import run as _g_run
from xbrain.boot.freeze.assertions.i_model_and_engine import run as _i_run
from xbrain.boot.freeze.assertions.j_config_root import run as _j_run
from xbrain.boot.freeze.assertions.l_bit_exemption import run as _l_run
from xbrain.boot.freeze.assertions.m_required import run as _m_run
from xbrain.boot.freeze.assertions.n_o_identities import (
    run_n as _n_run, run_o as _o_run,
)

# Every assertion body lives in a CFG-FZ-N item's own file eventually. Until
# that item lands, the stub below stands in -- visible, not silent. The stub
# vs real distinction is DELIBERATELY carried in the result dict's status
# field so an operator watching journalctl or a MANIFEST diff sees at a
# glance which assertions still need bodies.
def _runner_stub(name: str) -> Callable[[dict], dict]:
    """Return a placeholder runner used until the assertion's own CFG-FZ-N
    lands its real body. The stub is DELIBERATELY visible: it produces a
    result dict flagging itself unimplemented, so MANIFEST.assertions[name]
    shows `{"status": "stub", "impl_item": ...}` at bring-up on the current
    tree -- an operator sees at a glance which assertions are still stubs,
    and a green-through-the-stub deployment is impossible (real assertions
    return `status: pass` and mismatches produce `status: fail`)."""
    def _run(ctx: dict) -> dict:
        # Fixed shape: {"status": "stub", "assertion": name}. Real runners
        # return status in {"pass", "fail", "skip"} plus any assertion-
        # specific fields (assertion J adds "config_root", etc).
        return {"status": "stub", "assertion": name}
    # Rename the inner function so a traceback names the specific stub, not
    # just "_run". Dashes are not valid identifiers, hence the replace.
    _run.__name__ = "stub_" + name.replace("-", "_")
    return _run


@dataclass(frozen=True)
class AssertSpec:
    """One row of ASSERT_REGISTRY.

    Frozen so the ordered tuple below cannot be mutated in place: a caller
    that re-ordered ASSERT_REGISTRY at runtime would silently defeat ORD-1.
    The `runner` field is a callable stub by default; each CFG-FZ-N item
    replaces the callable with the real assertion body.
    """

    # Short letter code (A / J / M ...) or dotted composite for補入 assertions
    # like C-6+MR-1. Must appear as a key in MANIFEST.assertions verbatim.
    name: str
    # One-line human-readable description; shown in journalctl status lines
    # and in error messages so an operator sees WHAT failed, not just its code.
    title: str
    # ORD-1: names of assertions that must complete BEFORE this one runs.
    # validate_topology enforces every entry here appears earlier in the
    # tuple; a forward or missing dep raises AssertionError at import.
    depends_on: Tuple[str, ...] = field(default_factory=tuple)
    # Callable(ctx: dict) -> dict. None means "row without a runner", which
    # run_assertions treats as a construction defect (never silent).
    runner: Optional[Callable[[dict], dict]] = None
    # Which CFG-FZ-N item ships this row's real body (empty string until
    # that item lands). Human-readable, only appears in log messages.
    impl_item: str = ""


#: The registry. Order MATTERS -- it IS ORD-1.
#: The rows below are the assertions 10 S5.4.4 names + the CFG-FZ-14/15 補入
#: (FV-ORG for enu_origin, C-6/MR-1 for margins). Every entry starts as a
#: stub whose CFG-FZ-N replaces it. The order encodes dependency: an
#: assertion never runs before something it reads.
ASSERT_REGISTRY: Tuple[AssertSpec, ...] = (
    # ---------------------------------------------------------------------
    # J -- root and file reachability. Runs FIRST because every other
    # assertion opens files under the config root.
    # ---------------------------------------------------------------------
    AssertSpec("J", "config root + file reachability",
               depends_on=(), runner=_j_run,
               impl_item="CFG-FZ-2"),
    # ---------------------------------------------------------------------
    # A -- reference completeness. Runs SECOND, so M sees a fully-resolved
    # tree without unresolved ${common.*}.
    # ---------------------------------------------------------------------
    AssertSpec("A", "reference completeness (no unresolved ${common.*})",
               depends_on=("J",), runner=_a_run,
               impl_item="CFG-FZ-3"),
    # ---------------------------------------------------------------------
    # M -- required-key completeness. Runs after A because a required key
    # may hide behind a resolved reference.
    # ---------------------------------------------------------------------
    AssertSpec("M", "required-key completeness (walk resolved tree)",
               depends_on=("A",), runner=_m_run,
               impl_item="CFG-FZ-3"),
    # ---------------------------------------------------------------------
    # B / C / D -- structural. B (no duplicates) BEFORE C (cross-file
    # relations) BEFORE D (identity consistency); C would misreport a
    # duplicate as a cross-file conflict, and D compares canonicalised
    # values C already established are unique.
    # ---------------------------------------------------------------------
    AssertSpec("B", "no duplicates + alias-blacklist",
               depends_on=("M",), runner=_b_run,
               impl_item="CFG-FZ-4"),
    AssertSpec("C", "cross-file relations",
               depends_on=("B",), runner=_c_run,
               impl_item="CFG-FZ-4"),
    AssertSpec("D", "identity consistency",
               depends_on=("C",), runner=_d_run,
               impl_item="CFG-FZ-4"),
    # ---------------------------------------------------------------------
    # E -- safety namespaces vs hot-update whitelist must be disjoint.
    # No structural dep on B/C/D; kept after D for report legibility.
    # ---------------------------------------------------------------------
    AssertSpec("E", "safety namespace vs hot-update whitelist disjoint",
               depends_on=("D",), runner=_e_run,
               impl_item="CFG-FZ-5"),
    # ---------------------------------------------------------------------
    # F + F' -- QoS static (A-2~A-7 + depth=0) and port identity (GATE-5).
    # Depends on E because the QoS check reads common.qos which E's
    # namespace guard also touches. F' is folded into the same runner.
    # ---------------------------------------------------------------------
    AssertSpec("F", "QoS static (A-2~A-7 + depth=0) + port identity",
               depends_on=("E",), runner=_f_run,
               impl_item="CFG-FZ-6"),
    # ---------------------------------------------------------------------
    # G -- brake-derived speed-gate consistency (t_lat_s / decel / factor).
    # Depends on M: needs every key present.
    # ---------------------------------------------------------------------
    AssertSpec("G", "brake -> speed-gate consistency (t_lat_s / decel)",
               depends_on=("M",), runner=_g_run,
               impl_item="CFG-FZ-7"),
    # ---------------------------------------------------------------------
    # N -- safety constant identity (margin_base_m == d_safe_m). Depends
    # on G because G's range checks establish that both operands are
    # sane numbers; N then checks their equality.
    # ---------------------------------------------------------------------
    AssertSpec("N", "safety constant identity (margin_base_m == d_safe_m)",
               depends_on=("G",), runner=_n_run,
               impl_item="CFG-FZ-8"),
    # ---------------------------------------------------------------------
    # O -- cloud teleop priority identity. Depends on N as a sibling in
    # the "identity family"; ordering is arbitrary between N and O.
    # ---------------------------------------------------------------------
    AssertSpec("O", "cloud teleop priority identity",
               depends_on=("N",), runner=_o_run,
               impl_item="CFG-FZ-8"),
    # ---------------------------------------------------------------------
    # I -- TRT engine / model sha256 + build_env agreement.
    # ---------------------------------------------------------------------
    AssertSpec("I", "TRT engine sha256 + build_env",
               depends_on=("J",), runner=_i_run,
               impl_item="CFG-FZ-10"),
    # ---------------------------------------------------------------------
    # K -- quadruped private config QC-1..QC-17.
    # ---------------------------------------------------------------------
    AssertSpec("K", "quadruped private QC-1..QC-17",
               depends_on=("J",), runner=_runner_stub("K"),
               impl_item="CFG-FZ-12"),
    # ---------------------------------------------------------------------
    # L -- BIT exemption surface guard.
    # ---------------------------------------------------------------------
    AssertSpec("L", "BIT exemption surface guard",
               depends_on=("K",), runner=_l_run,
               impl_item="CFG-FZ-11"),
    # ---------------------------------------------------------------------
    # FV-ORG -- enu_origin startup validation (CFG-FZ-14 補).
    # ---------------------------------------------------------------------
    AssertSpec("FV-ORG", "enu_origin startup validation",
               depends_on=("M",), runner=_fv_org_run,
               impl_item="CFG-FZ-14"),
    # ---------------------------------------------------------------------
    # C-6 + MR-1 -- corridor lateral margin + rotation margin.
    # ---------------------------------------------------------------------
    AssertSpec("C-6+MR-1", "lateral clearance + rotation margin identities",
               depends_on=("M",), runner=_c6mr1_run,
               impl_item="CFG-FZ-15"),
)


def ordered_assertion_names() -> Tuple[str, ...]:
    """The declared execution order (ORD-1). Kept a tuple, not a set, and
    kept as its own function rather than a top-level tuple so a caller that
    depends on ORDER (not just membership) reads through this name.

    Convention: callers checking sequence import this; callers checking
    presence import registry_names(). Two names for two truths so a diff
    error names what actually broke.
    """
    return tuple(spec.name for spec in ASSERT_REGISTRY)


def registry_names() -> frozenset:
    """The membership set, for bidirectional-diff comparisons with
    MANIFEST.assertions. frozenset so a caller cannot mutate the truth.

    Deliberately not memoised: the registry is a compile-time constant,
    building the frozenset is cheap, and a cache would age (test mutations
    swap ASSERT_REGISTRY via monkeypatch and expect fresh reads).
    """
    return frozenset(spec.name for spec in ASSERT_REGISTRY)


def validate_topology() -> None:
    """Verify that every AssertSpec.depends_on references an assertion that
    APPEARS EARLIER in the registry. A forward dependency or a missing name
    is a bug in this file; raise loudly so bring-up cannot start.

    This is what mutation ③ (swap A and M) trips: M declares depends_on=(A,)
    but A now sits AFTER M, so seen_before does not contain A when M is
    inspected.

    Deliberately linear (single pass through registry) rather than a
    topological sort: the registry IS the topological order; we only
    verify no edge goes backward. A full sort would let the pipeline
    silently reshuffle if two rows disagreed with declaration order,
    which is the exact drift ORD-1 exists to prevent.

    Called from pipeline.run_assertions() BEFORE the first runner fires;
    if this raises, no runner has side-effected anything, so an operator
    fixes the registry and reruns without cleanup.
    """
    seen_before = set()
    for spec in ASSERT_REGISTRY:
        for dep in spec.depends_on:
            if dep not in seen_before:
                raise AssertionError(
                    "ORD-1 violation: %r depends on %r but %r is not "
                    "declared earlier in the registry"
                    % (spec.name, dep, dep)
                )
        # Add AFTER the deps check, not before -- a self-dep would
        # otherwise "see itself" in seen_before and pass. There are no
        # self-deps today, but the guard costs nothing.
        seen_before.add(spec.name)
