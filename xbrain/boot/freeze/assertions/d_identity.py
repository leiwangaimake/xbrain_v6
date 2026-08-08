"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: d_identity.py
Brief: Assertion D -- identity consistency (CFG-FZ-4)

Description:
Runs SIXTH in the freeze pipeline (ORD-1, after C). Enforces that the
values used to build IDENTITIES across the system stay consistent:

  D-1 robot_id matches `[a-z0-9_-]{1,32}` -- also enforced by
      11 S2.2.11 which uses this value as the {rid} segment of every
      Zenoh key. A mismatch here means the process's key names would
      not match its `common.robot_id` value, and the closed-set
      startup self-check would reject them; catching it at D makes
      the failure point the ONE key path, not thirty publish-side
      errors.
  D-2 site_id (if present) matches an L4 site file. Historically,
      sites/{site_id}.yaml is the file layout, so common.site_id
      must equal a basename that exists. Skipped when site_id is
      null or when no sites/ directory exists (dev checkouts).
  D-3 Zenoh key templates NOT sourced from config -- 11 S2.1 requires
      key patterns be code-defined; a config-driven pattern would let
      the whitelist and the pattern drift apart. Simple check: no
      `common.zenoh.key_template` or `common.zenoh.key_pattern` keys.

Contract:
  input:  ctx["config_root"] + ctx["overlay"] (from A)
  raises: XbrainError(E_CONFIG_INVALID) with detail.kind in
          rid_pattern_bad / site_id_no_l4_file / zenoh_key_from_config
"""

# os for path checks (site_id -> file), re for robot_id shape.
# typing gives Dict/Any annotations. All standard-library; freeze runs
# before third-party imports are guaranteed to be installed.
import os
import re
from typing import Any, Dict

from xbrain.common.config import build_overlay
from xbrain.common.errors.exceptions import XbrainError

# robot_id shape -- mirrors 11 S2.2.11 verbatim.
# Lowercase alphanumeric + underscore + hyphen, 1-32 chars.
# This same pattern is enforced at the Zenoh key layer; keeping it in
# sync here catches misconfigured robot_id at freeze time rather than
# at first publish (a much later failure point).
# Compiled at module load -- one pattern per process, reused across
# every freeze pass.
_RID_RE = re.compile(r"^[a-z0-9_-]{1,32}$")

# Config keys that must NOT exist -- Zenoh key templates are code, not
# config. Any of these present under common.zenoh.* is a defect.
# 11 S2.1 requires key patterns be code-defined; a config-driven pattern
# would let the whitelist and the pattern drift apart. Adding a new
# forbidden template key = one entry here.
_FORBIDDEN_ZENOH_KEYS = (
    # The literal key_template attribute at common.zenoh.
    "common.zenoh.key_template",
    # An older name that shows up in some drafts.
    "common.zenoh.key_pattern",
    # A nested-form variant.
    "common.zenoh.keys.template",
)


def _fail(kind: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + assertion-specific fields.

    D's closed set of detail.kind values: rid_pattern_bad /
    site_id_no_l4_file / zenoh_key_from_config. Same message shape
    across all three so a journalctl reader sees a consistent format.
    """
    # Kind stays required; extra kwargs let each raise site attach the
    # values the operator needs to act on (robot_id, site_id, etc).
    # Message stays English (CLAUDE.md S2.1) and interpolates kind so
    # a journalctl reader sees the failure class without decoding detail.
    detail = {"kind": kind}
    detail.update(extra)
    raise XbrainError(
        "E_CONFIG_INVALID",
        "assertion D failed: %s" % kind,
        detail,
    )


def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Dotted-path walker; identical shape to C's helper.

    Kept as a local helper (rather than importing C's) so D stays
    self-contained -- a future refactor that splits assertions into
    separate wheels won't force a shared-helper package.
    """
    # Walk segment-by-segment; any missing / non-dict node returns
    # the caller-supplied default (None by convention). Deliberate
    # None default: callers test `is not None` explicitly to keep
    # optional-key skipping shape unambiguous.
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _check_robot_id(tree: Dict[str, Any]) -> None:
    """D-1: robot_id must match _RID_RE. Missing (M's job) -> skip.

    The pattern rejects uppercase (Zenoh keys are case-sensitive and
    the {rid} segment expects a canonical lowercase form), spaces,
    slashes (would break key segmentation), and length > 32 (arbitrary
    upper bound to keep keys legible in logs). A robot_id like
    'GJ-001' passes M (it's a string, non-null) but fails D here --
    catches the case-mismatch at freeze rather than at first
    Zenoh publish.
    """
    # Pull robot_id from the merged overlay -- any layer wins.
    rid = _get(tree, "common.robot_id")
    if rid is None:
        # M already refused if this key is on the required list; if
        # M passed, this must be an optional deploy where the key is
        # explicitly absent -- skip.
        # D and M are complementary: M enforces existence, D enforces
        # shape given existence.
        return
    # isinstance guard: yaml can parse a bare number as int; a numeric
    # rid would not match the regex either way, but the explicit check
    # gives a cleaner error message than "int has no attribute match".
    # Reporting expected_pattern so an operator sees the shape they
    # need to conform to, not just "invalid".
    if not isinstance(rid, str) or not _RID_RE.match(rid):
        _fail("rid_pattern_bad", robot_id=rid, expected_pattern=_RID_RE.pattern)


def _check_site_id_has_l4(tree: Dict[str, Any], config_root: str) -> None:
    """D-2: site_id must correspond to a sites/{site_id}.yaml file.

    Filesystem check: sites/ layout uses the file basename as the
    identifier. A site_id like 'shanghai_main' MUST have a
    sites/shanghai_main.yaml file, else the L4 loader will silently
    skip loading site-specific config and every site-dependent key
    (enu_origin, retention overrides) falls back to L1 defaults --
    which are usually not what the site wants.

    Skipped when site_id is None (dev deploys) OR when the sites/
    directory is missing (fresh checkout that hasn't populated the
    tree yet).
    """
    # Pull site_id from the merged overlay (any layer wins).
    # site_id typically comes from L4 sites/*.yaml, but a common.yaml
    # or env-overlay default is also honoured.
    site_id = _get(tree, "common.site_id")
    if site_id is None:
        # Optional in the current tree; skip when not yet configured.
        # When site_id becomes required, M will refuse before we get here.
        return
    # sites/ directory holds L4 files -- one per site.
    sites_dir = os.path.join(config_root, "sites")
    if not os.path.isdir(sites_dir):
        # Dev checkout without sites -- skip. When sites/ becomes
        # required, an earlier assertion should catch this.
        # Skip is deliberate: refusing here would break dev deploys
        # that don't yet have site config.
        return
    # Convention: sites/{site_id}.yaml. The site_id value directly
    # names the file (no prefix / suffix hack).
    expected_file = "%s.yaml" % site_id
    if not os.path.isfile(os.path.join(sites_dir, expected_file)):
        # List what IS there so the operator sees the mismatch instantly.
        # sorted() so the list is stable between runs; only .yaml
        # files so a stray README does not clutter the report.
        # This is the single most helpful diagnostic in D -- a typo
        # in site_id is common, and showing the operator "here's what
        # exists" is faster than "the file is missing" for correction.
        present = sorted(
            n for n in os.listdir(sites_dir) if n.endswith(".yaml")
        )
        _fail("site_id_no_l4_file", site_id=site_id,
              expected_file=expected_file, present=present)


def _check_no_zenoh_key_from_config(tree: Dict[str, Any]) -> None:
    """D-3: no key-template config entries.

    Key patterns / templates must be code-defined (11 S2.1). If a
    config file provides `common.zenoh.key_template`, an operator
    could change the template at runtime and the code-defined
    whitelist would still reject the new pattern -- the two would
    drift apart. Rejecting the config path at startup is the only way
    to keep them coupled.
    """
    # Walk each forbidden path independently; first hit raises. The
    # order in _FORBIDDEN_ZENOH_KEYS is arbitrary but stable across
    # runs -- an operator diffing two failure logs sees only real
    # changes, not iteration-order shuffles.
    for forbidden in _FORBIDDEN_ZENOH_KEYS:
        # Inline walk (not _get) so we can detect "path exists" vs
        # "path returns None" -- a config-set None on a template key
        # is still a defect (someone wrote the key intending to set
        # it, just didn't fill the value).
        # ok flag tracks whether we made it all the way to the leaf.
        # False means "some intermediate segment missing" = safe.
        # True means "leaf reached" = defect, raise below.
        # node walks down the tree; ok tells us whether we reached leaf.
        node: Any = tree
        ok = True
        for part in forbidden.split("."):
            if not isinstance(node, dict) or part not in node:
                ok = False
                break
            node = node[part]
        if ok:
            # Reached the leaf -- key exists in the config. Report the
            # value's type so the operator sees what kind of drift
            # they created (string template, list of patterns, etc).
            # type().__name__ gives 'str' / 'list' / 'dict' etc.
            _fail("zenoh_key_from_config",
                  forbidden_key=forbidden,
                  found_value_type=type(node).__name__)


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion D. Replaces registry's stub_d.

    Reuses ctx["overlay"] from assertion A when available. Falls back
    to a fresh load + build_overlay when called in isolation.
    """
    # Same wiring guard as J / A / M / B / C -- ctx missing
    # config_root is a caller-side bug, not a config problem, and
    # AssertionError distinguishes it from a runtime XbrainError.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion D requires ctx['config_root']; caller did not "
            "populate it"
        )
    # config_root is filesystem input for D-2 (site_id -> file check).
    root = ctx["config_root"]
    # Prefer A's cached overlay; fall back for isolated callers who
    # invoke D without first invoking A (unit tests do this;
    # production ORD-1 has A -> B -> C -> D so overlay is always present).
    overlay = ctx.get("overlay")
    if overlay is None:
        # Local import avoids top-level cycle; same pattern as C.
        # Fresh load + build_overlay for the isolated call path;
        # the unit-test convenience path exercises this.
        from xbrain.boot.freeze.assertions._layer_loader import load_layers
        layer_trees = load_layers(root)
        overlay = build_overlay(layer_trees)
        # Populate ctx for downstream assertions in the same pass.
        ctx["overlay"] = overlay
        ctx["layer_trees"] = layer_trees
    # tree = overlay.tree pulls the merged dict once; all _check_*
    # helpers walk this same dict.
    tree = overlay.tree

    # Run the three checks. Independent, no order dependency; grouped
    # from cheapest (regex on one string) to most expensive (fs stat).
    # Order chosen for perf, not correctness -- any permutation gives
    # the same green/red result on the same tree.
    _check_robot_id(tree)                    # D-1: regex only
    _check_site_id_has_l4(tree, root)        # D-2: fs stat
    _check_no_zenoh_key_from_config(tree)    # D-3: dotted walk x N

    # Success return: three checks all passed. checks_run stays as a
    # field so a future variant that adds check D-4 can bump the
    # count visibly without needing a schema change.
    return {"status": "pass", "assertion": "D", "checks_run": 3}
