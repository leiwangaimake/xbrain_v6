"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: c6_mr1_margins.py
Brief: Assertions C-6 (lateral margin >= base) + MR-1 (rotation margin
       identity) -- CFG-FZ-15

Description:
Two margin assertions landed together. Both live on the corridor /
rotation-clearance boundary between P1 and common.safety:

  C-6  p1_motion.corridor.margin_lat_m >= p1_motion.corridor.margin_base_m
       Rationale: the lateral margin (RNS-side, per-lane) must not
       be tighter than the base margin (fence-side, static). A
       tighter lateral margin would let RNS authorise a path the
       fence layer would refuse.
       Judged as >= (not ==) because lateral may legitimately be
       wider than base for slower profiles.

  MR-1 p1_motion.rotation_clearance.margin_rot_m ==
       common.safety.d_safe_m
       Rationale: rotation clearance and the fence safety distance
       are the SAME PHYSICAL QUANTITY (11 S10.3.2). Since the two
       used to share a YAML anchor (10 S5.4.2 R-7) and anchors are
       now forbidden, an equality assertion replaces the anchor.
       Judged as == (STRICT identity, not >=). Because 11 R-7 forbids
       any drift, even a "safer" value like margin_rot=1.5 while
       d_safe=1.0 is a defect: the two are meant to co-vary.

CFG-FZ-15 variants verbatim:
  M-MR1-a  margin_rot = 0.30 (d_safe stays 1.0)  -> MR-1 red
  M-MR1-b  margin_rot = 1.50 (bigger than base)  -> MR-1 red
           (distinguishes '==' from '>='. An implementation that
           only checks >= passes a; strict == is needed for b.)
  M-MR1-c  margin_rot = 0.30 AND d_safe = 0.30   -> MR-1 GREEN
           (both equal; MR-1's job is identity, not the value's
           range -- that's 12 S12.1 S-4's territory)

Contract:
  input:  ctx["config_root"] + ctx["overlay"] (from A)
  raises: XbrainError(E_CONFIG_INVALID) with detail.rule (C-6 / MR-1)
          + detail.lhs / lhs_key / rhs / rhs_key
"""

# typing for Any/Dict annotations.
from typing import Any, Dict

# load_l6_files for the p1_motion.yaml side of both checks; load_layers
# for the isolated-fallback path building overlay.
from xbrain.boot.freeze.assertions._layer_loader import (
    load_l6_files, load_layers,
)
# build_overlay for the fallback path only; production ORD-1 has A
# already provided overlay in ctx.
from xbrain.common.config import build_overlay
# XbrainError base -- both C-6 and MR-1 raise E_CONFIG_INVALID
# uniformly.
# E_CONFIG_INVALID (or E_QOS_VIOLATION / E_CONFIG_LOCKED)
# imported by name from xbrain.common.errors instead of
# spelled as a string literal. CLAUDE.md 3.5 forbids literal
# E_* strings anywhere outside common/errors/; scripts/lint/
# no_literal_ecode.py enforces it (both the whole-word literal
# and the substring form).
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError


# Helper: dotted-path walker, identical shape to other assertions.
def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Dotted-path walker -- same shape as other assertions.

    Kept local (not shared with C/D/F/G) so this module stays
    self-contained. Same 5-line pattern; dedup saves nothing but
    coupling.
    """
    # Walk segment-by-segment; missing/non-dict returns default.
    # Deliberate default=None so callers test `is not None`.
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


# Helper: uniform raise for both C-6 and MR-1.
def _fail(rule: str, lhs_key: str, lhs: Any,
          rhs_key: str, rhs: Any, relation: str) -> None:
    """Raise E_CONFIG_INVALID with both operands named.

    rule:     'C-6' or 'MR-1'
    relation: '==' (MR-1) or '>=' (C-6) -- named so the failure
              message reads naturally at a glance.

    Same shape as N/O but adds a 'relation' field so the same
    fail helper serves both == and >= rules; a reader sees which
    kind of inequality broke without decoding.
    """
    # Detail carries both sides so an operator sees the mismatch
    # amount without opening the config files.
    # detail.kind = 'margin_broken' is common; detail.rule
    # distinguishes C-6 from MR-1.
    detail = {
        "kind": "margin_broken",
        "rule": rule,
        "lhs_key": lhs_key, "lhs": lhs,
        "rhs_key": rhs_key, "rhs": rhs,
        "relation": relation,
    }
    # Message: 'assertion C-6 failed: <lhs_key> = <lhs> not >= <rhs_key>
    # = <rhs>' -- inline for journalctl readability.
    # Format string interpolates all five parts so the failure line
    # is self-explanatory without any decoder.
    raise XbrainError(
        E_CONFIG_INVALID,
        "assertion %s failed: %s = %r not %s %s = %r"
        % (rule, lhs_key, lhs, relation, rhs_key, rhs),
        detail,
    )


def _prepare(ctx: Dict[str, Any]):
    """Shared ctx-preparation: overlay + L6 trees.

    Same pattern as N/O -- both C-6 and MR-1 read the same trees, so
    load once. Returns (overlay_tree, l6_trees).

    Kept in one helper because the two check bodies share the exact
    setup shape; splitting the prep would duplicate the wiring guard
    and the fallback path.
    """
    # Wiring guard identical to other assertions -- ctx['config_root']
    # missing = caller-side bug, not config bug (AssertionError).
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion C-6/MR-1 requires ctx['config_root']; caller "
            "did not populate it"
        )
    # Prefer A's cached overlay; fall back for isolated callers.
    # Production ORD-1 has A -> ... -> N -> O -> C-6+MR-1 so overlay
    # is populated by then.
    overlay = ctx.get("overlay")
    if overlay is None:
        # Fresh load path (unit-test convenience).
        # Local import matches C/D/F/G/N/O/FV-ORG pattern.
        layer_trees = load_layers(ctx["config_root"])
        overlay = build_overlay(layer_trees)
        # Populate ctx for downstream assertions in the same pass.
        ctx["overlay"] = overlay
        ctx["layer_trees"] = layer_trees
    # L6 fresh -- both operands of C-6 live in p1_motion.yaml (L6).
    # Cheap read (~1ms); not cached in ctx because callers rarely
    # need it twice in the same pass.
    l6 = load_l6_files(ctx["config_root"])
    return overlay.tree, l6


def _check_c6(overlay_tree: Dict[str, Any],
              l6: Dict[str, Dict[str, Any]]) -> None:
    """C-6: p1_motion.corridor.margin_lat_m >= corridor.margin_base_m.

    Skips if either operand missing. Uses >= (not ==) because lateral
    is legitimately wider for slower profiles.

    Physical meaning: lateral margin is the horizontal clearance
    around the robot when following a corridor; base margin is the
    minimum static distance from any obstacle. If lat < base, the
    corridor could put the robot inside the base-safe zone, which
    the fence layer would separately refuse.

    overlay_tree parameter is accepted for signature symmetry with
    _check_mr1; C-6 only reads L6.
    """
    # p1 default {} tolerates missing file; _get then returns None
    # and we skip below.
    p1 = l6.get("p1_motion.yaml", {})
    # LHS: lateral margin (RNS-side, per-lane).
    # margin_lat_m is what the runtime uses for corridor half-width.
    lat = _get(p1, "corridor.margin_lat_m")
    # RHS: base margin (fence-side, static, matches d_safe by MR-1
    # transitivity but MR-1's key path is different).
    # margin_base_m == margin_rot_m == d_safe_m by MR-1 identity.
    base = _get(p1, "corridor.margin_base_m")
    # Skip on missing -- A/M handle required-ness elsewhere.
    # Dev checkouts may not have corridor block populated yet.
    # Skip is deliberate: C-6 is a comparison check on values that
    # exist; nothing to compare = nothing to fail.
    if lat is None or base is None:
        return
    # Type gate -- both numeric so we don't raise TypeError on cmp.
    # A non-numeric here is a shape defect for another check.
    # Both must pass -- one non-numeric would still crash < below.
    if not isinstance(lat, (int, float)) or not isinstance(base, (int, float)):
        return
    # Strict < triggers >= violation.
    # (Equality is legit -- C-6 is >= not >.)
    if lat < base:
        # relation='>=' so message reads "not >=", the correct English.
        # Both values reported for one-shot triage.
        _fail("C-6",
              "p1_motion.corridor.margin_lat_m", lat,
              "p1_motion.corridor.margin_base_m", base,
              ">=")


def _check_mr1(overlay_tree: Dict[str, Any],
               l6: Dict[str, Dict[str, Any]]) -> None:
    """MR-1: p1_motion.rotation_clearance.margin_rot_m == d_safe_m.

    STRICT equality. Even a "safer" value like margin_rot=1.5 while
    d_safe=1.0 is a defect -- the two are meant to co-vary.

    Rationale for STRICT: the two used to share a YAML anchor
    (10 S5.4.2 R-7) and anchors are now forbidden. An equality
    assertion replaces the anchor. A >= check would let the two
    drift silently in the "safer" direction -- but drift is drift,
    and re-reading them as separate values later would surface the
    difference as a live bug rather than a locked constraint.
    """
    # Same p1 = {} default as _check_c6 for missing-file tolerance.
    p1 = l6.get("p1_motion.yaml", {})
    # LHS: rotation-clearance margin in P1.
    # P1-side location matches 11 S10.3.2 wording.
    rot = _get(p1, "rotation_clearance.margin_rot_m")
    # RHS: common.safety.d_safe_m from the merged overlay (L3).
    # L3 safety-side location; U67 pinned value 1.0m.
    # Overlay-side read (not L3 raw) so any layer's override wins.
    d_safe = _get(overlay_tree, "common.safety.d_safe_m")
    # Skip if either missing.
    # Same optional-key policy as C-6: dev checkouts tolerated.
    # M/A refuse before we get here if either is required-and-missing.
    if rot is None or d_safe is None:
        return
    # Type gate.
    # Same rationale as _check_c6: avoid TypeError on !=.
    if not isinstance(rot, (int, float)) or not isinstance(d_safe, (int, float)):
        return
    # Strict inequality. MR-1 is IDENTITY -- no epsilon for the same
    # reason as N (both operands are configured literals, not
    # computed values). A 1e-9 difference indicates a typo.
    # This is where CFG-FZ-15 variant M-MR1-b (rot=1.5, d_safe=1.0)
    # catches -- a >= implementation would let it pass.
    if rot != d_safe:
        # Both operands + relation='==' for at-a-glance triage.
        _fail("MR-1",
              "p1_motion.rotation_clearance.margin_rot_m", rot,
              "common.safety.d_safe_m", d_safe,
              "==")


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for C-6 + MR-1. Replaces registry's stub for
    'C-6+MR-1'.

    Two sub-checks run sequentially; first failure raises. Order
    (C-6 then MR-1) is arbitrary -- checks are independent -- and
    fixed for reproducible failure logs.
    Single runner (not two) matches the registry row: 'C-6+MR-1' is
    the assertion name; the sub-rules are its two halves.
    """
    # Shared prep (overlay + L6).
    # _prepare handles the wiring guard + overlay fallback + L6 read.
    # Both sub-checks pull from the same two trees.
    overlay_tree, l6 = _prepare(ctx)

    # C-6 first (lateral margin >= base).
    # Position first because C-6 uses >= (looser check) which is
    # cheaper conceptually; the MR-1 == check follows.
    # Both check functions accept (overlay_tree, l6) for symmetry
    # even though C-6 only uses l6.
    _check_c6(overlay_tree, l6)
    # MR-1 second (rotation margin identity).
    # Reads d_safe_m from overlay_tree (L3-merged); rot_m from L6.
    _check_mr1(overlay_tree, l6)

    # Success: report both checks ran.
    # checks_run stays a constant 2 today; kept as a field so a
    # future variant can spot a silently-dropped check.
    # No individual pass/fail per rule -- assertion contract is
    # single "did it run" bit for the whole runner.
    return {
        "status": "pass",
        "assertion": "C-6+MR-1",
        "checks_run": 2,
    }
