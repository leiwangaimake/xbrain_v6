"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: n_o_identities.py
Brief: Assertion N (safety constant identity) + Assertion O (cloud teleop
       priority identity) -- CFG-FZ-8

Description:
Two independent identity assertions, both landed in this one module.
Each proves that two values held in DIFFERENT keys must be equal, and
each fires the moment the two drift apart.

  N -- p1_motion.corridor.margin_base_m == common.safety.d_safe_m
       History: 10 S5.4.4 records that in v0.4 these two shared a
       YAML anchor (margin_base_m: *d_safe), but 10 S5.4.2 R-7
       forbids anchors ("they don't cross files, and give a false
       sense that shared state is guaranteed"). v0.5 removed the
       anchor and left the two values coupled only by human
       discipline; N is the mechanism that replaces the anchor.
       U67 (2026-08-06) settled d_safe_m at 1.00 m (L3 safety).

  O -- p1_motion.teleop.cloud.priority ==
       p1_motion.arbitration.priorities.teleop_cloud
       History: 12 S12 wrote 550 in two places (teleop.cloud sub-tree
       and arbitration.priorities), and 12 S12.1's S-1..S-6 range
       assertions did not guard the equality. 10 S5.4.4 records the
       decision to house this check in this table rather than as S-7
       (that number is already taken by 15 S9.1 for versioned
       migrations); housing it here also puts it on assertion G's
       execution chain, avoiding the SP-9/SP-10 "declared assertion
       that never runs" trap.

Both checks compare values living in DIFFERENT trees:
  N reads margin_base_m from L6 (p1_motion.yaml) and d_safe_m from
    the merged overlay (L3 safety originates the value).
  O reads BOTH operands from L6 (p1_motion.yaml).

Contract:
  input:  ctx["overlay"] + ctx["config_root"] (for L6 files)
  raises: XbrainError(E_CONFIG_INVALID) with detail.kind=
          identity_broken + detail.rule (N or O) + detail.lhs +
          detail.rhs + detail.lhs_key + detail.rhs_key

CFG-FZ-8 variants verbatim:
  (1) margin_base_m = 1.1 -> N red (d_safe_m stays 1.0)
  (2) teleop.cloud.priority = 560 -> O red (arb sub-tree stays 550)
"""

# typing for Any/Dict annotations. Both run_n and run_o share the
# same ctx contract shape, so no need for extra Callable types here.
# Dict + Any for the tree/ctx annotations.
from typing import Any, Dict

# _layer_loader gives both load_l6_files (for the L6 side of N/O) and
# load_layers (for the fresh-overlay path when ctx has no overlay yet).
from xbrain.boot.freeze.assertions._layer_loader import (
    load_l6_files, load_layers,
)
# build_overlay used in the isolated-caller fallback path (unit tests
# that skip A). Production ORD-1 has A -> ... -> G -> N/O, so overlay
# is normally present.
from xbrain.common.config import build_overlay
from xbrain.common.errors.exceptions import XbrainError


def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Dotted-path walker -- same shape as C/D/F/G.

    Kept local (not shared) so N/O stays self-contained. Same 5-line
    helper as elsewhere; dedup would save nothing but coupling.
    """
    # Walk segment-by-segment; any missing / non-dict node returns
    # the caller-supplied default (None by convention).
    # Walk the tree segment-by-segment; any missing segment aborts
    # with the default. isinstance guard covers the case where
    # part-way we land on a scalar (list, str, number).
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _fail(rule: str, lhs_key: str, lhs: Any,
          rhs_key: str, rhs: Any) -> None:
    """Raise E_CONFIG_INVALID with detail naming both sides.

    rule:    'N' or 'O' (which identity broke).
    lhs_key: dotted path of left side.
    lhs:     value at lhs_key.
    rhs_key: dotted path of right side.
    rhs:     value at rhs_key.

    Same fail-shape for both N and O so a downstream error handler
    can dispatch on detail.rule without a schema fork. Detail also
    carries lhs and rhs so the operator sees the mismatch amount
    without opening either config file.
    """
    # Detail carries both sides so an operator sees which pair
    # broke and by what amount without opening either file.
    # detail.kind = "identity_broken" is common across N and O; the
    # rule field distinguishes them.
    # detail includes both keys AND both values so an operator has
    # everything needed to fix the config in one message.
    detail = {
        "kind": "identity_broken",
        "rule": rule,
        "lhs_key": lhs_key, "lhs": lhs,
        "rhs_key": rhs_key, "rhs": rhs,
    }
    # Message interpolates both sides so a journalctl reader without
    # decoder scripts sees the full mismatch inline.
    # Format: 'assertion N failed: <lhs_key> = <lhs_val> != <rhs_key>
    # = <rhs_val>' -- reads cleanly at glance.
    raise XbrainError(
        "E_CONFIG_INVALID",
        "assertion %s failed: %s = %r != %s = %r"
        % (rule, lhs_key, lhs, rhs_key, rhs),
        detail,
    )


def _prepare(ctx: Dict[str, Any]):
    """Common ctx-preparation shared by run_n and run_o.

    Loads overlay + L6 trees on demand; caches under ctx keys so a
    later assertion in the same pass reuses them. Returns
    (overlay_tree, l6_trees). Raises AssertionError on missing
    ctx['config_root'] (wiring bug, not config bug).

    Factored into one helper because N and O share the exact same
    preparation shape; keeping them separate would duplicate the
    ctx-wiring guard and the overlay-load pattern.
    """
    # Same wiring guard as J/A/M/B/C/D/E/F/G -- missing ctx key is a
    # caller-side bug, not a config bug (AssertionError not XbrainError).
    # Distinguishing the two error types is important: operator triage
    # ends at "check bring-up code" vs "check yaml".
    if "config_root" not in ctx:
        # Same shape as the other assertions' guard.
        raise AssertionError(
            "assertion N/O requires ctx['config_root']; caller did "
            "not populate it"
        )
    # ctx.get("overlay") returns None if A hasn't run yet.
    overlay = ctx.get("overlay")
    if overlay is None:
        # Fresh load path for isolated callers who invoke N or O
        # without first invoking A. Production ORD-1 has A -> ... -> G
        # -> N -> O so overlay is normally present already.
        # Local import matches the pattern in C/D/F/G -- avoids top-
        # level cycle risk if _layer_loader ever imports back through us.
        layer_trees = load_layers(ctx["config_root"])
        overlay = build_overlay(layer_trees)
        # Populate ctx so a subsequent assertion (O after N) doesn't
        # re-load.
        ctx["overlay"] = overlay
        ctx["layer_trees"] = layer_trees
    # L6 trees loaded fresh (cheap) each pass. Not cached in ctx
    # because callers rarely need them twice.
    # We could cache under ctx['l6_trees'] to save the second read
    # between N and O runs; deferred until profiling shows it matters.
    # Two reads at ~1ms each is invisible next to the rest of freeze.
    l6 = load_l6_files(ctx["config_root"])
    # Return both so callers pick what they need without another
    # helper call. run_n uses both; run_o uses only l6.
    return overlay.tree, l6


def run_n(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """N: p1_motion.corridor.margin_base_m == common.safety.d_safe_m.

    Skips silently when either operand is absent (dev checkout).

    The two operands live in DIFFERENT files (LHS in L6 p1_motion.yaml,
    RHS in the L3 safety overlay), so a read from ctx["overlay"] alone
    is not enough -- we also load L6 through _prepare.

    Return dict distinguishes 'checked=True' (equality verified) from
    'checked=False' (skipped due to missing operand) so a MANIFEST
    reader can tell green-because-passed from green-because-skipped.
    """
    # _prepare loads overlay + L6 trees and populates ctx caches.
    # Both operands come out of the return in one call.
    overlay_tree, l6 = _prepare(ctx)
    # Left side: p1_motion L6 file.
    # LHS is a P1 config parameter that ultimately feeds speed_gate
    # margin logic; equality with d_safe_m is the anchor U67 pinned
    # after removing the YAML anchor.
    # p1 may be {} if p1_motion.yaml is empty; _get then returns None.
    # Missing file gets the same {} default via load_l6_files' skip.
    # Same defensive default used by run_n above.
    p1 = l6.get("p1_motion.yaml", {})
    lhs = _get(p1, "corridor.margin_base_m")
    # Right side: overlay-resolved common.safety.d_safe_m.
    # RHS lives in the L3 safety layer and gets merged into the
    # overlay tree; reading through overlay picks up L3's value (or
    # a later override if any).
    # Deliberately overlay-side read: any layer above L3 could have
    # rewritten it, and the OVERLAY value is what runs. Reading L3
    # directly would miss such an override.
    rhs = _get(overlay_tree, "common.safety.d_safe_m")
    # Skip if either is absent: A/M already covered required-ness;
    # N is an equality-only check on values that DO exist.
    # The skip is a legit outcome for dev checkouts (either config
    # file may be empty), not a defect this assertion should raise.
    # A production tree would have M refuse missing d_safe_m earlier.
    if lhs is None or rhs is None:
        # Skip result carries checked=False so a MANIFEST reader
        # sees why this assertion passed.
        return {
            "status": "pass",
            "assertion": "N",
            "checked": False,
            "reason": "one side missing; skipped",
        }
    # Strict inequality. No epsilon: both sides are configured
    # literals (not computed), so a difference is a config typo.
    # Even a 1e-9 difference would indicate someone typed different
    # numbers, and the two must be identical (not "close enough").
    # Contrast with C's fence_close_tol check which uses 1e-9
    # epsilon because one side is DERIVED (2 * min_dist_m).
    if lhs != rhs:
        _fail("N",
              "p1_motion.corridor.margin_base_m", lhs,
              "common.safety.d_safe_m", rhs)
    # Success: report the common value so a MANIFEST diff surfaces
    # a change even when equality still holds.
    # A change of d_safe_m from 1.0 to 1.2 (with margin_base_m
    # updated in lockstep) still shows up in the manifest via this
    # value field.
    return {
        "status": "pass",
        "assertion": "N",
        "checked": True,
        "value": lhs,
    }


def run_o(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """O: p1_motion.teleop.cloud.priority ==
       p1_motion.arbitration.priorities.teleop_cloud.

    Both sides live in the same L6 file (p1_motion.yaml), so we do
    not need the merged overlay for this check -- unlike N which
    reads across L6 + L3. _prepare still runs (to seed ctx and to
    keep the two run_* functions symmetric), we just ignore its
    overlay_tree return.
    """
    # underscore prefix on _overlay_tree signals "unused" so a reader
    # doesn't wonder why we grabbed it. _prepare still runs to keep
    # the ctx contract symmetric with run_n and to seed ctx caches.
    _overlay_tree, l6 = _prepare(ctx)
    # Both operands from the same L6 file. Reads through the raw L6
    # tree because arbitration.priorities is a P1-internal key that
    # would not appear in the merged overlay (L6 isn't part of
    # build_overlay's layer set).
    # Same p1 = {} defensive default as run_n.
    p1 = l6.get("p1_motion.yaml", {})
    # LHS: teleop.cloud.priority under the teleop sub-tree.
    # RHS: arbitration.priorities.teleop_cloud under a different
    # sub-tree; the two describe the same priority number.
    # v0.4 shared them via YAML anchor; anchors removed per R-7,
    # so equality is enforced by this assertion instead.
    lhs = _get(p1, "teleop.cloud.priority")
    rhs = _get(p1, "arbitration.priorities.teleop_cloud")
    # Same skip semantics as N -- both operands optional in dev
    # checkouts; equality only fires on values that exist.
    # A production tree would have M refuse missing priorities
    # earlier if that becomes required.
    if lhs is None or rhs is None:
        # checked=False for MANIFEST readability -- same shape as N.
        return {
            "status": "pass",
            "assertion": "O",
            "checked": False,
            "reason": "one side missing; skipped",
        }
    # Strict inequality -- same rationale as N (integer priorities,
    # not floats, so no epsilon question here). == on ints is exact.
    if lhs != rhs:
        _fail("O",
              "p1_motion.teleop.cloud.priority", lhs,
              "p1_motion.arbitration.priorities.teleop_cloud", rhs)
    # Success: report the common value.
    # Same reasoning as N -- MANIFEST diff can spot a priority
    # change even when the identity still holds.
    return {
        "status": "pass",
        "assertion": "O",
        "checked": True,
        "value": lhs,
    }
