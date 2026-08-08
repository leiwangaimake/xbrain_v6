"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: fv_org_enu.py
Brief: Assertion FV-ORG-1 / FV-ORG-2 / FV-ORG-3 -- enu_origin startup
       self-check (CFG-FZ-14)

Description:
Runs LATE in the freeze pipeline (ORD-1 position TBD; sensibly after
G because it depends on the full merged overlay). Three sub-checks
on common.geo.enu_origin:

  FV-ORG-1  lat / lon / alt each non-null (per-component check).
            Object presence alone is not enough -- an object with
            alt=null passes an "exists" test but fails the intent.
            Reported with the key path of the FIRST null component.

  FV-ORG-2  Value must come from L4 (sites/*.yaml), NOT from L0
            (dataclass defaults). L0 would silently backfill a value
            nobody chose, and the "declared but unassigned" report
            never fires.

  FV-ORG-3  Effective value must come from L4. L1 (common.yaml) may
            hold three null placeholders only -- writing a REAL value
            in L1 is a defect (site data misplaced into shared layer).
            L2 / L4b / L6 must not carry the key at all.

Four variants (CFG-FZ-14 verbatim, one of them is a MUST-PASS):
  M-ORG-a  {0.0, 0.0, 0.0}                    -> must PASS
           (0.0 is a legal coordinate; rejecting it is a
           "definition-masquerading-as-observation" defect)
  M-ORG-b  lat/lon filled, alt null           -> FV-ORG-1 red
           (distinguishes per-component check from object-exists)
  M-ORG-c  L1 filled, L4 absent               -> FV-ORG-3 red
  M-ORG-d  L1 null placeholders, L4 filled    -> must PASS
           (this IS the normal deployment shape per 10 S5.4.3)

Contract:
  input:  ctx["config_root"]
  reads:  L1 (common.yaml) + L2 (models/) + L3 (safety/) + L4
          (sites/{site_id}.yaml) + L4b (calib/{robot_id}.yaml) +
          L6 (per-process yaml)
  raises: XbrainError(E_CONFIG_INVALID) with detail.rule (FV-ORG-1/
          -2/-3) + detail.key + detail.layer (where the offender is)
"""

# os for path check (sites/{site_id}.yaml existence); typing for
# Any/Dict/Optional annotations. yaml unused here (each loader calls
# it internally). All standard-library where possible; freeze runs
# early and third-party imports at freeze time are risky.
import os
from typing import Any, Dict, Optional

# _read_yaml + load_l6_files + load_layers reused from the shared
# loader; keeps this module thin.
from xbrain.boot.freeze.assertions._layer_loader import (
    _read_yaml, load_l6_files, load_layers,
)
# build_overlay for the isolated-caller fallback path when ctx has no
# overlay yet (production: A populates it).
from xbrain.common.config import build_overlay
# Base exception for every deliberate raise; FV-ORG uses
# E_CONFIG_INVALID for all three sub-rules.
# E_CONFIG_INVALID (or E_QOS_VIOLATION / E_CONFIG_LOCKED)
# imported by name from xbrain.common.errors instead of
# spelled as a string literal. CLAUDE.md 3.5 forbids literal
# E_* strings anywhere outside common/errors/; scripts/lint/
# no_literal_ecode.py enforces it (both the whole-word literal
# and the substring form).
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError

# The three sub-components of enu_origin. All required non-null by
# FV-ORG-1. Tuple order determines which component is reported first
# on a violation.
# Deliberate lat/lon/alt order: matches the natural WGS84 tuple that
# a reader from geodesy will recognise instantly.
_ENU_COMPONENTS = ("lat", "lon", "alt")

# Root key path for enu_origin. Kept as a module constant so a rename
# lands in one place; also used in detail.key for failure messages.
# Same dotted path referenced by 10 S5.4.5 and 11 S9A verbatim.
_ENU_ROOT = "common.geo.enu_origin"


# Helper: uniform raise for all three FV-ORG sub-rules.
def _fail(rule: str, key: str, layer: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.rule + key + layer.

    rule:  'FV-ORG-1' / 'FV-ORG-2' / 'FV-ORG-3'
    key:   dotted path of the offending value.
    layer: which layer holds the offender ('L1', 'L2', 'L4',
           'L4b', 'L6:<file>', or 'merged' for post-overlay).
    **extra kwargs attach failure-specific context (component,
    site_id, actual value, ...).

    Common detail.kind = 'enu_origin_bad' -- one kind, three rules
    via detail.rule.
    """
    detail = {"kind": "enu_origin_bad", "rule": rule,
              "key": key, "layer": layer}
    detail.update(extra)
    # Message string carries rule + key + layer for at-a-glance
    # triage without decoding detail.
    raise XbrainError(
        E_CONFIG_INVALID,
        "assertion %s failed: %s at layer %s" % (rule, key, layer),
        detail,
    )


# Helper: dotted-path walker, same signature as other assertions.
def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Dotted-path walker -- same shape as other assertions.

    Kept local (not shared) so FV-ORG stays self-contained. Same
    5-line pattern as C/D/F/G.
    """
    # Walk segment-by-segment; any missing / non-dict node returns
    # the caller-supplied default (None by convention).
    # Walk the tree segment-by-segment; missing / non-dict returns default.
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


# Helper: pull the enu_origin OBJECT from a layer's raw tree.
def _get_layer_enu(tree: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Return the enu_origin OBJECT from a layer tree, or None.

    Returns the whole {lat, lon, alt} dict, not a single component.
    Callers check individual components via .get() on the returned
    dict.
    None (from _get) means the layer never even declared the key.
    """
    return _get(tree, _ENU_ROOT)


# Helper: read L4 (site) tree; empty dict when site_id or file absent.
def _load_l4_tree(config_root: str, site_id: Optional[str]) -> Dict[str, Any]:
    """Read sites/{site_id}.yaml if it exists; else empty.

    site_id absent OR file missing -> {} (L4 layer effectively empty).
    Callers distinguish "layer empty" from "layer missing enu_origin"
    by testing _get_layer_enu(result) after the load.

    Local to this assertion because the L4 file choice depends on
    site_id, which is a value INSIDE the tree being loaded --
    circular for the shared load_layers helper to handle. Each
    consumer of L4 loads it after resolving site_id.
    (Two-step load: read L1 -> get site_id -> read L4.)
    """
    if not site_id:
        # No site_id set (dev checkout, or L1 didn't set it) -> no
        # L4 selection possible. Return empty tree; callers check
        # emptiness and fire their own error if L4 was required.
        # This is the "site_id is None" branch, kept distinct from
        # "site_id set but file missing" below.
        return {}
    path = os.path.join(config_root, "sites", "%s.yaml" % site_id)
    if not os.path.isfile(path):
        # site_id names a file that isn't there -- a defect that
        # would deserve its own error, but we let downstream checks
        # (FV-ORG-2/-3) fire naturally because they see empty L4.
        # Not raising here also keeps this helper reusable by future
        # non-FV-ORG callers who may tolerate a missing file.
        return {}
    return _read_yaml(path)


# Helper: read L4b (calib) tree; empty dict when robot_id or file absent.
def _load_l4b_tree(config_root: str, robot_id: Optional[str]) -> Dict[str, Any]:
    """Same as L4 loader but for calib/{robot_id}.yaml.

    Same skip semantics: absent robot_id or missing file -> {}.
    L4b is per-robot calibration; it never carries enu_origin (that
    is site-level), so its role in FV-ORG is only "must not carry
    enu_origin at all" (FV-ORG-3).
    Loaded even when we know it can't hold enu_origin so that the
    "must-not-carry" test can catch someone violating the rule.
    """
    # Same skip pattern as _load_l4_tree.
    if not robot_id:
        return {}
    path = os.path.join(config_root, "calib", "%s.yaml" % robot_id)
    if not os.path.isfile(path):
        # Missing calib file returns {}; FV-ORG-3's must-not-carry
        # check treats {} as "layer clean".
        return {}
    return _read_yaml(path)


# Helper: is the L1 enu_origin value legitimate null-placeholders?
def _l1_placeholder_shape_ok(l1_enu: Any) -> bool:
    """L1 enu_origin is legal only when it is EXACTLY three null
    placeholders {lat: null, lon: null, alt: null}. Any real value
    at L1 = FV-ORG-3 violation.

    Returns True if the shape is legit-null-placeholder (all values
    are None, missing components are fine); False otherwise.
    Caller uses this to distinguish "L1 is doing the right thing"
    from "L1 wrote a real value".
    """
    if not isinstance(l1_enu, dict):
        # L1 can also legally omit the key entirely; caller handles that.
        # Non-dict at this path means schema defect, not our concern.
        return False
    # Every declared component must be None; missing components pass
    # too (L1 might declare only some keys). But any non-None is bad.
    # Iterating .items() rather than checking specific keys: adds
    # forward-compat if a fourth component (e.g. epoch) is added.
    # Any non-None value in ANY component = real value = bad.
    # Loop stops on first non-None; no need to inspect all.
    for k, v in l1_enu.items():
        if v is not None:
            return False
    # All components are None (or the dict is empty) = ok placeholder.
    return True


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for FV-ORG. Replaces registry's stub.

    Three sub-checks in one runner: FV-ORG-1 (component non-null),
    FV-ORG-2 (L4 supplies value, not L0 tail), FV-ORG-3 (L1 shape
    + L2/L4b/L6 must-not-carry). Order chosen for triage clarity:
    FV-ORG-3 layer-placement first (catches most defects), then
    FV-ORG-2 L4-must-supply, then FV-ORG-1 component-completeness.
    """
    # Same wiring guard as other assertions -- ctx['config_root']
    # missing = caller-side bug, not config bug, so AssertionError.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion FV-ORG requires ctx['config_root']; caller did "
            "not populate it"
        )
    # root is used by every _load_l*_tree call below.
    root = ctx["config_root"]

    # Merged overlay for FV-ORG-1 (three-component non-null check).
    # Falls back to fresh load for isolated callers who skip A.
    # Production: A runs first, overlay is in ctx already.
    overlay = ctx.get("overlay")
    if overlay is None:
        # Fresh load path -- same pattern as C/D/F/G/N/O.
        layer_trees = load_layers(root)
        overlay = build_overlay(layer_trees)
        ctx["overlay"] = overlay
        ctx["layer_trees"] = layer_trees
    else:
        # A cached layer_trees; still need to re-read for L4/L4b/L6
        # below. If for some reason layer_trees isn't in ctx (would be
        # a caller-shape defect), fall back to empty dict.
        layer_trees = ctx.get("layer_trees", {})

    # ---- Per-layer scan for FV-ORG-2 / -3 ----------------------------
    # Each layer's raw tree gets its own read; NO merged tree here
    # because FV-ORG-3 needs to know WHICH layer wrote what.
    # Reload L1 raw (layer_trees["L1"] is the raw L1 tree from
    # load_layers), plus L2/L3/L4/L4b/L6.
    # NOTE: build_overlay's merge covers L1/L2/L3 only (L4/L4b need
    # site_id/robot_id first). So we can't trust overlay.tree for
    # FV-ORG-1; we read L4 raw and check components there.
    # This is why FV-ORG-1 sits AFTER FV-ORG-2 in this function --
    # FV-ORG-2 ensures L4 is present + non-empty first.
    # L1 (common.yaml) + L2 (models/) + L3 (safety/) trees pulled
    # from the layer_trees dict A populated. Missing key -> {} so
    # downstream _get_layer_enu tolerates absence.
    # {} default here is defensive: A always populates, but the
    # isolated fallback path may not (test-only concern).
    l1_tree = layer_trees.get("L1", {})
    l2_tree = layer_trees.get("L2", {})
    l3_tree = layer_trees.get("L3", {})

    # L4 needs site_id from the merged overlay (any layer's value wins).
    # site_id from L1 is normal; L4 override of site_id is not
    # standard but supported by build_overlay if it happens.
    # Reading overlay.tree (not raw L1) so any layer's override is
    # honoured; whichever value wins is what freeze commits.
    site_id = _get(overlay.tree, "common.site_id")
    # None if site_id is unset -- _load_l4_tree returns {} in that
    # case, and FV-ORG-3 fires below.
    l4_tree = _load_l4_tree(root, site_id)

    # L4b needs robot_id. Same source-of-truth pattern -- overlay's
    # merged robot_id wins whatever layer set it.
    robot_id = _get(overlay.tree, "common.robot_id")
    # L4b tree loaded only for FV-ORG-3's must-not-carry check; the
    # tree contents themselves don't matter beyond that.
    l4b_tree = _load_l4b_tree(root, robot_id)

    # L6 loaded through the shared helper. All per-process yaml files
    # in one dict {basename: tree}.
    # Same reader B and other assertions use -- ensures consistent
    # missing-file behaviour.
    l6_trees = load_l6_files(root)

    # ---- FV-ORG-3: L1 shape + L2/L4b/L6 must-not-carry --------------
    # L1: enu_origin can only be null placeholders; ANY real value = red.
    # This is the "site data misplaced into shared layer" defect.
    # L1's role is to declare the key positions, not to hold real
    # site data -- CFG-FZ-14 variant M-ORG-c targets this violation.
    l1_enu = _get_layer_enu(l1_tree)
    if l1_enu is not None and not _l1_placeholder_shape_ok(l1_enu):
        # Report actual so the operator sees what they wrote wrong.
        # 'not is None' + 'not shape_ok' = the dict exists AND
        # contains at least one non-null value.
        _fail("FV-ORG-3", _ENU_ROOT, "L1",
              actual=l1_enu,
              reason="L1 may only carry null placeholders for enu_origin")

    # L2 / L4b / L6: forbidden to carry the key at all.
    # Only L1 (null placeholders) + L4 (real value) are legit sources.
    # Rationale: L2 is model spec (per-model, not per-site); L4b is
    # calib (per-robot); L6 is process-scoped (P1/P2/... not geo-owning).
    # site geometry belongs strictly in L4.
    # (layer_name, layer_tree) rows. L2 = models, L3 = safety, L4b
    # = calib. Adding a new forbidden layer = one row here.
    forbidden_layers = [
        ("L2", l2_tree),
        ("L3", l3_tree),        # 10 S5.4.5: L3 is safety, not geo; forbidden.
        ("L4b", l4b_tree),
    ]
    # Walk the forbidden list; first offender raises.
    # Order = list-declaration order (L2, L3, L4b). Deterministic
    # first-offender-wins pattern for reproducible failure logs.
    for layer_name, tree in forbidden_layers:
        if _get_layer_enu(tree) is not None:
            # Layer's tree carries enu_origin under common.geo -- that
            # layer is not allowed. Report layer name + reason.
            _fail("FV-ORG-3", _ENU_ROOT, layer_name,
                  reason="only L1 (null placeholders) and L4 may carry "
                         "this key")
    # L6 forbidden across every process file. Iterate all L6 files
    # so a single misplaced key surfaces the name of the offender.
    # sorted() so failure output is stable across runs when multiple
    # L6 files misplace the key.
    for fname, tree in sorted(l6_trees.items()):
        if _get_layer_enu(tree) is not None:
            # detail.layer includes file name for direct navigation.
            # 'L6:p1_motion.yaml' style makes the log line self-
            # explanatory without decoding detail.
            _fail("FV-ORG-3", _ENU_ROOT, "L6:%s" % fname,
                  file=fname,
                  reason="L6 process configs may not carry enu_origin")

    # ---- FV-ORG-2: value must come from L4, not from L0 tail --------
    # We do NOT read L0 (dataclass defaults) here because build_overlay
    # excludes L0 from providing common.safety etc; enu_origin lives
    # under common.geo which L0's exclusion list does NOT cover today
    # (dev checkouts may still have a default). We enforce the intent
    # by requiring L4 to provide the key.
    # Reading _get_layer_enu on L4 gets us the whole {lat,lon,alt}
    # object; None means "L4 file absent OR L4 doesn't set the key".
    # Either case (file missing / key absent) is FV-ORG-3 territory.
    l4_enu = _get_layer_enu(l4_tree)
    if l4_enu is None:
        # FV-ORG-3 explicit for this path too: no L4 = missing origin.
        # Includes site_id + site_file in detail so an operator knows
        # WHICH file they should be editing.
        # Conditional site_file: if site_id itself is None, we can't
        # even name a target file -- report None so the operator
        # sees "no site_id set" as distinct from "site_id set but
        # file missing".
        _fail("FV-ORG-3", _ENU_ROOT, "L4",
              reason="L4 (sites/) must provide enu_origin; not present",
              site_id=site_id,
              site_file="sites/%s.yaml" % site_id if site_id else None)
    # FV-ORG-2: reject L0 fallback -- L4 must have supplied a real
    # value (not just placeholders).
    # "At least one non-null component" is a weaker check than
    # FV-ORG-1's "all three non-null"; kept as a distinct guard so
    # the fail message can name L0-fallback specifically.
    # any() over the three components: pass if AT LEAST one is
    # non-null. This is deliberately lax (not "all three") because
    # FV-ORG-1 handles the stricter all-three check separately; the
    # split lets each fail with a specific detail.rule.
    if not any(
        _get(l4_tree, "%s.%s" % (_ENU_ROOT, c)) is not None
        for c in _ENU_COMPONENTS
    ):
        _fail("FV-ORG-2", _ENU_ROOT, "L4",
              reason="L4 must provide non-null enu_origin values; "
                     "having only placeholders would let L0 tail in")

    # ---- FV-ORG-1: three components non-null in L4 ------------------
    # After FV-ORG-2 above, we know L4 has enu_origin with at least
    # ONE non-null component. FV-ORG-1 checks that EVERY component is
    # non-null (per-component check, not object-existence).
    # M-ORG-a (variant): {0.0, 0.0, 0.0} passes here because 0.0 is
    # not None. Explicitly separates "value 0.0" from "no value".
    # Read from L4 raw because build_overlay does not include L4/L4b.
    # Walk _ENU_COMPONENTS in tuple order so the first None gets
    # reported deterministically.
    # THIS is variant M-ORG-b's target: lat/lon filled, alt null =>
    # object exists but component is None; per-component check catches.
    # Component iteration in _ENU_COMPONENTS order for determinism.
    # Same reproducibility rationale as elsewhere: same input, same
    # first-failing component reported across runs.
    for comp in _ENU_COMPONENTS:
        val = l4_enu.get(comp)
        if val is None:
            # First None component wins the raise -- others deferred
            # (they would produce the same operator action: fill it).
            _fail("FV-ORG-1",
                  "%s.%s" % (_ENU_ROOT, comp),
                  "L4",
                  component=comp,
                  reason="all three of lat/lon/alt must be non-null")

    # Success: report site_id + component count for MANIFEST audit.
    # site_id in the result lets a MANIFEST diff show which site the
    # freeze ran under (different runs on different sites diff
    # visibly). components_verified stays constant today but is a
    # field for future growth (a 4th component if epoch were added).
    return {
        "status": "pass",
        "assertion": "FV-ORG",
        "site_id": site_id,
        "components_verified": len(_ENU_COMPONENTS),
    }
