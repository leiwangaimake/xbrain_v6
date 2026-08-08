"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: m_required.py
Brief: Assertion M -- required-key completeness (CFG-FZ-3, second half)

Description:
Runs THIRD in the freeze pipeline (ORD-1, after A). Walks the merged
L1~L3 tree and rejects the case that A cannot catch: a required key
that is COMPLETELY ABSENT from every L1~L5 layer.

Why M exists (10 S5.4.4 M-row, verbatim):
  Assertion A checks null values -- "declared but unassigned". If a
  key is not declared by any layer, there is no null to check; L0
  (dataclass defaults) silently fills it in and A passes green.
  M is the second door: every required key must appear in AT LEAST
  ONE of L1~L5. A key found only in L0 = M fails.

The required-key list is CFG-FZ-2's §5.4.5 obligation. Each entry
below cites the row it enforces so a future editor of 10 S5.4.5 can
trace which assertions to update. New required keys land here as one
row per key with a citation to the section that made them mandatory.

Failure vocabulary (closed set for detail.kind on M raises):
  required_key_missing   -- key not present in any L1~L5 layer
  required_key_only_l0   -- key present only in L0 (dataclass default),
                            equivalent failure surface -- same message,
                            different failure origin
"""

# Standard-library / typing first, then internal. M is one of two halves
# of CFG-FZ-3 (A is the other); both read the same L1~L3 layer trees, so
# both import the same _layer_loader helper and both merge via
# build_overlay. Keeping the shared imports in the same order eases a
# side-by-side reading of the two assertions.
# Tuple is imported for the required-keys frozenset element type; not
# used directly in signatures but kept to make future annotations easy.
from typing import Any, Dict, FrozenSet, Tuple

from xbrain.boot.freeze.assertions._layer_loader import load_layers
from xbrain.common.config import build_overlay
from xbrain.common.config.merge import flatten
# E_CONFIG_INVALID (or E_QOS_VIOLATION / E_CONFIG_LOCKED)
# imported by name from xbrain.common.errors instead of
# spelled as a string literal. CLAUDE.md 3.5 forbids literal
# E_* strings anywhere outside common/errors/; scripts/lint/
# no_literal_ecode.py enforces it (both the whole-word literal
# and the substring form).
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError

# The required-key list. Each entry = one dotted path that MUST be
# present in L1~L5 layers. The list is a data table, not a for-loop
# spread across code -- adding a new required key is one row here.
#
# Cited section per row so an editor of 10 S5.4.5 knows what to check
# when adding / removing keys. Rows organised by common.* subtree so
# related keys sit together.
# frozenset (not set) so a caller cannot mutate the list at runtime;
# the required-keys contract is a compile-time invariant.
_REQUIRED_KEYS: FrozenSet[str] = frozenset({
    # ---- common.spec.* -- vehicle static spec (SP-1) --------------
    # §5.4.5 spec.* row: max_vx/vy/wz + max_accel/decel, all required.
    # Variant target in CFG-FZ-3 is specifically about safety.t_lat_s
    # but the spec-* keys are same-row obligations from the same table.
    "common.spec.max_vx_mps",
    "common.spec.max_vy_mps",
    "common.spec.max_wz_radps",
    "common.spec.max_accel_mps2",
    "common.spec.max_decel_mps2",
    # ---- common.safety.* -- safety timing + distance ---------------
    # §5.4.5 safety.* row -- t_lat_s + d_safe_m; t_lat_s is the CFG-FZ-3
    # variant target verbatim (deletion triggers must-refuse-startup).
    "common.safety.t_lat_s",
    "common.safety.d_safe_m",
    # ---- common.motion.profiles -- speed profile table -------------
    # §5.4.5 motion.profiles row -- two profiles (obstacle_avoid,
    # patrol), each with max_mps. Any leaf missing = M fails.
    "common.motion.profiles.obstacle_avoid.max_mps",
    "common.motion.profiles.patrol.max_mps",
    # ---- common.fence.* -- soft fence params -----------------------
    # §5.4.5 fence.* row -- soft_margin_min_m + predict_dt_s.
    "common.fence.soft_margin_min_m",
    "common.fence.predict_dt_s",
    # ---- common.robot_id / site_id ---------------------------------
    # §5.4.5 robot_id / site_id rows. robot_id fills the {rid} segment
    # of every Zenoh key; site_id names the sites/*.yaml file.
    "common.robot_id",
    # site_id is optional in the current tree: not yet in every deploy.
    # When it becomes universal, uncomment below to make M enforce it.
    # "common.site_id",
})


def _fail(kind: str, key: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + missing key path.

    kind is the closed-set string documented in the module docstring
    (required_key_missing / required_key_only_l0). extra kwargs let
    each raise site attach the layer + required_count fields without
    threading those through the signature.

    The message string interpolates BOTH the key and the kind, so a
    journalctl reader who never opens detail still sees which key is
    missing and why. CLAUDE.md S2.1 forbids Chinese in log messages;
    the sentence stays English.
    """
    detail = {"kind": kind, "key": key}
    detail.update(extra)
    raise XbrainError(
        E_CONFIG_INVALID,
        "assertion M failed: required key %r %s" % (key, kind),
        detail,
    )


def _has_key(flat_keys: FrozenSet[str], key: str) -> bool:
    """True iff `key` (dotted) is present as a leaf OR as a prefix of
    at least one leaf.

    Prefix-tolerance matters because §5.4.5 sometimes names a subtree
    (common.motion.profiles) rather than a specific leaf; a config that
    filled every leaf under the subtree should count as "present".

    Example: required_key = "common.motion.profiles". flat_keys has
    "common.motion.profiles.obstacle_avoid.max_mps" and
    "common.motion.profiles.patrol.max_mps". Both start with the
    required prefix + ".", so this returns True. A stricter exact-match
    check would falsely report the subtree as absent.

    Corollary: if the required list ever names a subtree that legitimately
    has NO leaves under it (a completely empty section), the current
    implementation would report it as missing. There is no such row in
    _REQUIRED_KEYS today; if one arrives, this function needs a mode
    switch (exact vs prefix).
    """
    # Fast path: exact leaf match. Set lookup is O(1); we try this
    # first because it catches the common case (most required keys
    # are leaves, not subtrees) without walking the whole set.
    if key in flat_keys:
        return True
    # Subtree presence: any flat key starting with key + "."
    # The trailing dot matters -- otherwise common.spec would match
    # common.speck too. Concatenate once, iterate once.
    prefix = key + "."
    for k in flat_keys:
        if k.startswith(prefix):
            return True
    # Neither leaf nor subtree presence: caller reports it missing.
    return False


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion M. Replaces registry's stub_m.

    Reuses ctx["overlay"] if assertion A already populated it (the
    common path: ORD-1 has A -> M so overlay is guaranteed present).
    Falls back to loading + merging fresh if called in isolation
    (unit-test path, or a future test that runs M without A).

    On success returns a result dict with counts (required_count and
    keys_present). On any missing required key, raises XbrainError
    with detail.key naming the FIRST missing key -- we stop at the
    first miss rather than collecting all, because bring-up should
    fail loudly on the first defect (and a wall of complaints would
    obscure the root cause anyway).
    """
    # Same wiring guard as A/J: missing ctx key is a caller bug, not
    # a config problem, so plain AssertionError to distinguish.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion M requires ctx['config_root']; caller did not "
            "populate it"
        )

    # Prefer the overlay A left in ctx (cheap: no re-read). Falls back
    # to a fresh load for isolated-test callers who invoke M without
    # first invoking A (unit tests do this; production ORD-1 does not).
    overlay = ctx.get("overlay")
    if overlay is None:
        # Fresh load path -- also populate ctx so a subsequent
        # assertion in the same pass doesn't re-read again.
        layer_trees = load_layers(ctx["config_root"])
        overlay = build_overlay(layer_trees)
        ctx["overlay"] = overlay
        ctx["layer_trees"] = layer_trees

    # Flatten once to a set for O(1) leaf lookup + prefix scan.
    # flatten() rebuilds the whole dotted-path map on each call;
    # doing it once outside the loop is a straightforward speedup
    # that also makes the loop body read as a single-line check.
    flat = frozenset(flatten(overlay.tree).keys())

    # Walk required list; first miss raises. sorted() so the failing
    # key is stable across runs regardless of frozenset iteration
    # order -- an operator diffing two failure logs sees only real
    # changes, not shuffled complaints.
    for key in sorted(_REQUIRED_KEYS):
        if not _has_key(flat, key):
            # Provenance may know something even if the key is absent
            # (an earlier layer might have hinted at it but the leaf
            # got dropped). Best-effort layer report; when nothing
            # is known, we report "none" rather than omitting the
            # field, so callers can always index detail.layer.
            layer = overlay.provenance.get(key, "none")
            _fail("required_key_missing", key,
                  layer=layer,
                  required_count=len(_REQUIRED_KEYS))

    # ---- All required keys present -----------------------------------
    # keys_present count equals required_count on success (obviously),
    # but emitting both lets a future extension (partial success where
    # keys_present < required_count) surface without a schema change.
    return {
        "status": "pass",
        "assertion": "M",
        "required_count": len(_REQUIRED_KEYS),
        "keys_present": sum(1 for k in _REQUIRED_KEYS if _has_key(flat, k)),
    }
