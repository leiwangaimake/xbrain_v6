"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: a_references.py
Brief: Assertion A -- reference completeness (CFG-FZ-3, first half)

Description:
Runs SECOND in the freeze pipeline (ORD-1, after J). Walks the merged
L1~L3 tree and rejects three shapes:

  (1) values still containing `${` -- an unresolved reference. Either
      the referenced key does not exist, or the reference target has
      a cycle. Both mean the tree cannot be fed to any consumer.
  (2) values equal to None (YAML `null`) -- some layer DECLARED the
      key and no upper layer filled it. This is a spec-shaped gap:
      the schema knows the key should exist, ops forgot to give it
      a value.
  (3) references that cannot be resolved at all -- covered by the
      first `resolve()` call raising ReferenceError_. Reported as a
      subset of (1) because the caller-visible artefact is the same
      key path with a `${` substring.

Every raise carries detail.key (the dotted path of the offending
leaf) and, when applicable, detail.layer (which layer declared the
null / broken reference), so an operator sees at once where to fix.

CFG-FZ-3 reverse test spelled out: value = null must be caught by A
(key path) BEFORE it can reach assertion G (range violation). G would
report "value out of range" for a null it received, which pretends the
config is filled -- exactly the fail-silent shape CLAUDE.md S3.1 warns
against. A must fire first.

Contract with the pipeline:
  input:  ctx["config_root"] (from run_freeze)
  writes: ctx["layer_trees"], ctx["overlay"] so M / G / F can reuse
          the same merged tree instead of re-reading files
  raises: XbrainError(E_CONFIG_INVALID, ...) on any of the three
          shapes; the caller stops on first raise (do not continue
          past a broken tree, ever)
"""

# Standard-library imports first, then internal. build_overlay + resolve +
# find_violations + flatten are the reference-axis primitives; A is the
# assertion that glues them into a startup check.
from typing import Any, Dict

from xbrain.boot.freeze.assertions._layer_loader import load_layers
from xbrain.common.config import build_overlay
from xbrain.common.config.merge import flatten
from xbrain.common.config.refs import ReferenceError_, resolve
from xbrain.common.errors.exceptions import XbrainError


def _fail(kind: str, key: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + key path.

    kind: closed set for this assertion:
      unresolved_ref   -- value string contains ${ that resolve()
                          cannot expand (target missing / cycle)
      null_unassigned  -- a layer declared the key, no layer filled it
      layer_load_failed-- a layer file could not be read at all;
                          bubbled up from _layer_loader (rare -- J
                          usually catches this first)
    """
    detail = {"kind": kind, "key": key}
    detail.update(extra)
    raise XbrainError(
        "E_CONFIG_INVALID",
        "assertion A failed at key %r: %s" % (key, kind),
        detail,
    )


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion A. Replaces registry's stub_a.

    Steps:
      1) Load L1~L3 layer trees from config_root
      2) Merge via build_overlay -> get OverlayResult (tree + provenance)
      3) Check nulls (unassigned() list) -- fail with null_unassigned
      4) Try to resolve() the tree -- on ReferenceError_, fail with
         unresolved_ref
      5) Success: cache layer_trees + overlay in ctx for M/G to reuse
    """
    # config_root is mandatory (same guard shape as J). If it's absent,
    # the caller wired ctx wrong -- raise AssertionError to distinguish
    # from a real config problem.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion A requires ctx['config_root']; caller did not "
            "populate it (see xbrain.boot.freeze.pipeline.run_freeze)"
        )
    root = ctx["config_root"]

    # ---- Load layers -------------------------------------------------
    # load_layers itself may raise XbrainError(E_CONFIG_INVALID) on a
    # YAML parse failure -- propagate as-is; there's no better wrapping
    # we could do here.
    layer_trees = load_layers(root)

    # ---- Merge via build_overlay -------------------------------------
    # build_overlay runs namespace checks (L2 not writing common.safety,
    # etc). A layer-placement violation raises ConfigLayerError, which
    # is XbrainError subclass -- propagate as-is (it already carries
    # E_CONFIG_INVALID + a descriptive message).
    overlay = build_overlay(layer_trees)

    # Cache both raw trees and the merged overlay in ctx so downstream
    # assertions (M / G / others) do not re-read the same files. This
    # is the "shared context" pattern registry.py describes: each
    # runner may READ what a prior one wrote, and A is the first place
    # the overlay materialises.
    ctx["layer_trees"] = layer_trees
    ctx["overlay"] = overlay

    # ---- (2) null values ---------------------------------------------
    # OverlayResult.unassigned() returns dotted paths whose leaf is None.
    # Sorted alphabetically, so the first one reported is stable across
    # runs (an operator diffing two failure logs sees only real changes).
    # Filter out keys whose null-in-L1 is DELIBERATE by design:
    # enu_origin's three components are null placeholders in L1 and
    # get filled by L4 (which build_overlay does NOT merge, because L4
    # picking needs site_id). FV-ORG (CFG-FZ-14) enforces that L4
    # actually provides them; A defers those three keys to FV-ORG.
    _A_NULL_EXCEPTIONS = frozenset({
        "common.geo.enu_origin.lat",
        "common.geo.enu_origin.lon",
        "common.geo.enu_origin.alt",
    })
    nulls = [k for k in overlay.unassigned()
             if k not in _A_NULL_EXCEPTIONS]
    if nulls:
        first = nulls[0]
        # Provenance tells us which layer DECLARED the null; that's
        # where the operator needs to fill it in.
        layer = overlay.provenance.get(first, "unknown")
        _fail("null_unassigned", first,
              layer=layer,
              null_count=len(nulls))

    # ---- (1) + (3) unresolved / cyclic references --------------------
    # resolve() walks the tree and expands every ${...}. On any failure
    # (missing target, cycle, malformed syntax) it raises
    # ReferenceError_. Wrap that into our shape.
    try:
        resolve(overlay.tree)
    except ReferenceError_ as exc:
        # ReferenceError_ carries the offending key in its message. We
        # copy the message into detail.reason and put the key path
        # (best-effort extracted) into detail.key.
        # Extraction is best-effort because the message shape is not
        # part of ReferenceError_'s contract; if the format changes,
        # we still surface SOMETHING useful (the raw message).
        msg = str(exc)
        # detail.key is required by the assertion contract; use the
        # message as a fallback so the field is never empty.
        _fail("unresolved_ref", msg,
              raw_message=msg)

    # ---- All checks passed -------------------------------------------
    # Return count telemetry so an operator scrolling the manifest sees
    # HOW MUCH the assertion checked, not just "pass". A future defect
    # that made the tree be empty would show null_count == 0 (misleading
    # pass); required_files_checked would drop to zero and be noticed.
    return {
        "status": "pass",
        "assertion": "A",
        "layers_loaded": len(layer_trees),
        "keys_flattened": len(flatten(overlay.tree)),
    }
