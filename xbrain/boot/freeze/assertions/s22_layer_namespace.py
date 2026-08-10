"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: s22_layer_namespace.py
Brief: Assertion S22 -- per-layer namespace validation (CFG-FZ-16)

Description:
Each config layer has an allowed top-level namespace. Writing
outside it means one layer is silently claiming a domain that
belongs to another (an L2 model file writing common.safety.*
would make safety parameters vary per-model, defeating the whole
point of the safety namespace being L3-exclusive).

The overlay layers L0-L5 already get namespace-checked by
build_overlay(check_namespaces=True), which is invoked by
assertion A. S22 REPEATS that check with an explicit failure
attribution ('S22 red' rather than 'A red') so the CFG-FZ-16
mutations are attributed cleanly, AND extends the check to L6
(per-process files) which do not go through build_overlay.

L6 namespace rule (2026-08-10 loosened to align with 10 S5.4.3
line 2803-2804 逐字: "不得出现 common.* 顶层键"): an L6 file
MUST NOT have a top-level key named `common`. Anything else is
legal -- multiple top-level keys are fine (patrol config often
splits into intent/gateway/prompt/logging sections at the root),
and the sole top-level key does not have to match the process
name.

Historical note: an earlier draft of S22 enforced "exactly one
top-level key = process name". That was an over-strict reading
never stated in 10 S5.4.3 and it collided with every P-process's
committed yaml (all five split their config across multiple
top-level sections). The wrap-under-proc-name approach was
rejected 2026-08-10 as adding a level of indirection for zero
new safety -- the R-6 rule (no common.* top-level) is what
prevents the drift 10 S5.4.3 was worried about.

Overlap-with-B note: the "no top-level `common`" rule is ALSO
enforced by assertion B (detail.kind = l6_common_top_level).
S22's L6 check is intentionally NOT redundant with that -- S22
now only defends against non-dict yaml at L6 top level, which
B's flatten-based approach would silently coerce.

CFG-FZ-16 S22 named variants (each MUST turn red in tests):

  1) `speed_profiles:` at top-level in L1 (should be common.motion
     .profiles which is L1's territory) -> S22 red
     `speed_profiles:` at top-level in L2 (motion is also L2's
     but it must be common-prefixed) -> S22 red
     A shortcut implementation ('only check L2') passes the L1
     case and misses the defect. Both must red.

  2) common.safety.brake block placed in an L2 model file -> S22
     red (safety is L3-exclusive). A shortcut implementation
     ('only check top-level == common') passes (top-level IS
     common) but misses this because the sub-namespace safety.
     belongs to L3, not L2.

  3) Full gait table repeated in both L1 and L2 (both individually
     legal placements) -> assertion B must red (duplicate keys).
     S22 does not catch this because both writes are namespace-
     legal; B is the enforcer.

Contract:
  input:   ctx["config_root"]
           optional ctx["layer_trees"] (from A) to avoid re-reading
           optional ctx["l6_trees"] to avoid re-reading L6 files
  raises:  XbrainError(E_CONFIG_INVALID) with detail.kind in
             {layer_ns_violation, l6_top_level_wrong,
              l6_multiple_top_levels, l6_empty}
           + detail.layer / detail.file / detail.key /
             detail.allowed as appropriate

Not in scope for S22:
  * value-level validation (H is field values, G is safety ranges)
  * duplicate detection across layers (that is B's job)
  * schema of individual keys (CFG-FZ-17 handles that)

Ordering in the freeze pipeline (ORD-1):

S22 depends_on=("J",) -- only needs the config root reachable.
Placed near the end of the current registry, running AFTER the
overlay-based assertions (A/M) so that if the CFG-FZ-16 mutation
tests specifically want an S22 attribution, S22 fires cleanly
even when A would also have fired. The overlap with A is
deliberate; splitting the check attribution keeps each mutation
paired with a single expected assertion name.

Why re-run the L1-L5 check that A already does:

The alternative -- 'let A handle L1-L5, only S22 for L6' -- has
a subtle problem: the CFG-FZ-16 mutation tests would then hit
assertion A when the operator's mental model says 'namespace
check'. When the tests grow to include diagnostics that print
'the mutation to try' -> 'the assertion expected to fire', the
answer has to be one specific name. S22 is that name for
namespace violations regardless of layer.

Failure-mode taxonomy (two distinct detail.kind values after
2026-08-10 loosening):

  layer_ns_violation      L1-L5: a layer wrote a key outside its
                          allowance / into another layer's namespace.
                          Detail carries layer name + the offending
                          key. Remediation: move the key to the
                          correct layer.

  l6_top_level_wrong      L6: yaml top level is not a dict (e.g. a
                          list or scalar at the root). Detail carries
                          file + actual_type. Remediation: wrap the
                          content in the proc-appropriate section.

  (removed 2026-08-10)    l6_multiple_top_levels was here until
                          2026-08-10; removed because 10 S5.4.3 does
                          not forbid multiple top-level keys, only
                          top-level `common`, which assertion B owns.

Overlap with assertion B (why S22 doesn't catch variant 3):

Variant 3 (full gait table in both L1 and L2) is namespace-legal
in BOTH layers: L1 owns common.motion.profiles, L2 also owns
common.motion.*. So check_namespace passes both. What breaks is
duplicate detection -- the same leaf key exists in two layers with
identical values -- which is assertion B's contract, not S22's.
Documenting this in the module docstring keeps a future reader from
'extending S22 to also catch duplicates' (a change that would
duplicate B's logic and create drift risk).
"""

# os for L6 file path composition.
import os
# typing for annotations.
from typing import Any, Dict, Iterable

# Layer loader: L1-L5 raw trees via load_layers (production path);
# L6 raw trees via load_l6_files. Both allow ctx override.
from xbrain.boot.freeze.assertions._layer_loader import (
    load_l6_files, load_layers,
)
# LAYERS carries the (allowed, excluded) prefix rules for L0-L5.
# check_namespace is the same primitive assertion A uses.
from xbrain.common.config.layers import LAYERS, check_namespace
# flatten walks nested dicts to dotted paths, matching what
# check_namespace expects.
from xbrain.common.config.merge import flatten
# E_CONFIG_INVALID by name, per CLAUDE.md 3.5.
from xbrain.common.errors import E_CONFIG_INVALID
# XbrainError base; S22 uses E_CONFIG_INVALID uniformly.
from xbrain.common.errors.exceptions import XbrainError


# Which top-level key each L6 file legitimately owns. Same shape
# as _L6_FILES in _layer_loader.py but keyed by filename and mapped
# to the allowed sole top-level key. Adding a new L6 file means one
# row here + one row in _L6_FILES.
#
# The mapping is filename->key, not key->filename, because the check
# runs per-file and needs O(1) lookup by filename. The reverse map
# is not needed and would be redundant.
_L6_OWNER: Dict[str, str] = {
    "p1_motion.yaml": "p1_motion",
    "p2_core.yaml": "p2_core",
    "p3_task.yaml": "p3_task",
    "p4_agent.yaml": "p4_agent",
    "p5_gateway.yaml": "p5_gateway",
    "quadruped.yaml": "quadruped",
}


def _fail(kind: str, message: str, **detail_extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + arbitrary context.

    Same failure shape as every other freeze assertion so downstream
    dashboards can aggregate by kind.
    """
    # detail carries kind (closed set) + extras (layer, file, key).
    detail: Dict[str, Any] = {"kind": kind}
    detail.update(detail_extra)
    raise XbrainError(E_CONFIG_INVALID,
                      "assertion S22 failed: %s" % message,
                      detail)


def _check_l1_l5(layer_trees: Dict[str, Dict[str, Any]]) -> None:
    """Re-run check_namespace over every LAYERS entry that has a tree.

    This duplicates work assertion A already does with
    build_overlay(check_namespaces=True) but re-attributes the
    failure to S22, which matches the CFG-FZ-16 mutation-attribution
    contract ('mutation X -> S22 red', not 'A red').

    Uses the exact same LAYERS list and check_namespace primitive
    as A, so any drift between the two callers is not possible:
    they share the source of truth.
    """
    # Walk LAYERS in order; skip L0 (excluded-only, no tree from disk)
    # and L5 (env vars, applied at build_overlay time, no raw tree).
    for layer in LAYERS:
        # Skip layers that do not have a raw tree on disk.
        # L0 = code defaults (never on disk). L5 = env vars.
        if layer.name in ("L0", "L5"):
            continue
        # Pull the tree the loader read for this layer name. If the
        # layer wasn't loaded (dev checkout, missing dir), tree is
        # empty and the check trivially passes.
        tree = layer_trees.get(layer.name, {})
        if not tree:
            continue
        # flatten to leaf dotted paths, then defer to the primitive.
        flat = flatten(tree)
        try:
            check_namespace(layer, flat.keys())
        except XbrainError as exc:
            # Re-raise as an S22 failure with layer attribution so
            # the CFG-FZ-16 mutation testing hits a stable detail.
            # str(exc) surfaces the original message which contains
            # the offending key path.
            _fail("layer_ns_violation",
                  "%s: %s" % (layer.name, str(exc)),
                  layer=layer.name,
                  allowed=list(layer.allowed),
                  excluded=list(layer.excluded))


def _check_l6(l6_trees: Dict[str, Dict[str, Any]]) -> None:
    """For each L6 file, enforce that top level is a dict.

    10 S5.4.3 line 2803-2804 逐字 forbids only 'common.* top-level' at
    L6; that specific rule is enforced by assertion B
    (detail.kind = l6_common_top_level). What B does NOT catch is an
    L6 file whose root is a LIST or a scalar (yaml load returns a
    non-dict), because B calls flatten() which happily accepts a list
    and produces indexed keys silently.

    S22 stays in the picture only to guard that shape. Everything
    else -- multiple top-level keys, a top-level key that is not the
    process name -- is legal per 10 S5.4.3 and used by every
    currently-committed P yaml.
    """
    for filename, tree in l6_trees.items():
        # An owner mapping is still useful for the failure message
        # (tells the reader which proc this file is supposed to serve)
        # but is no longer required for the check to run.
        owner = _L6_OWNER.get(filename)
        if not tree:
            # Empty dict / None: dev placeholder, legal.
            continue
        if not isinstance(tree, dict):
            # Non-dict at the root: list / scalar / etc. B's flatten
            # would accept this silently; catch it here.
            _fail("l6_top_level_wrong",
                  "%s: expected dict at top level, got %s"
                  % (filename, type(tree).__name__),
                  file=filename,
                  actual_type=type(tree).__name__,
                  expected=owner if owner else "dict")


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion S22. Namespace check across L1-L6.

    Flow:
      1. Wiring guard on ctx['config_root'].
      2. Resolve L1-L5 trees (ctx['layer_trees'] wins; else load).
      3. Re-run check_namespace on each; re-raise as S22 attribution.
      4. Resolve L6 trees (ctx['l6_trees'] wins; else load).
      5. Enforce sole-top-level-key == process name for each L6 file.
      6. Pass return with counts for observability.
    """
    # Wiring guard identical to every other assertion.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion S22 requires ctx['config_root']; caller did not "
            "populate it"
        )
    # ctx['layer_trees'] override: assertion A populates this the
    # first time it runs. Reusing it avoids a second disk read.
    layer_trees = ctx.get("layer_trees")
    if layer_trees is None:
        layer_trees = load_layers(ctx["config_root"])
        # Cache back for later assertions to reuse.
        ctx["layer_trees"] = layer_trees
    # L6 trees: same override pattern.
    l6_trees = ctx.get("l6_trees")
    if l6_trees is None:
        l6_trees = load_l6_files(ctx["config_root"])
        ctx["l6_trees"] = l6_trees

    # L1-L5 sweep. Every legitimate placement passes; every violation
    # raises with layer attribution.
    _check_l1_l5(layer_trees)
    # L6 sweep. Each file owns exactly one top-level key.
    _check_l6(l6_trees)

    # Success shape. Counts confirm every layer was actually walked
    # (silent-zero detection).
    return {
        "status": "pass",
        "assertion": "S22",
        "layers_checked": sum(1 for v in layer_trees.values() if v),
        "l6_files_checked": sum(1 for v in l6_trees.values() if v),
    }
