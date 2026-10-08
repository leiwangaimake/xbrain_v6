"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: _layer_loader.py
Brief: Read L1~L3 layer YAML files from config root; helper for A/M/G assertions

Description:
Assertions A and M both need the same input: L1~L4b layer trees read
from the config root, ready to feed build_overlay(). Rather than each
assertion reimplement the read + parse, this module owns the read side
once. Callers can either invoke load_layers(root, variant=ctx.get("config_variant")) themselves, or read
ctx["layer_trees"] (populated by assertion A the first time it runs).

Scope for CFG-FZ-3 (assertions A + M):
  L1  common.yaml      -- single top-level file, unconditional
  L2  models/*.yaml    -- iterate the directory, deep-merge in order
  L3  safety/*.yaml    -- same
  L4  sites/*.yaml     -- BLOCKED here: needs common.site_id to pick
                          the file, and site_id may itself be null; the
                          only clean way to iterate L4 is inside the
                          full pipeline (post-A resolution). Deferred
                          to a later CFG-FZ-N when site_id has landed.
  L4b calib/*.yaml     -- same reason as L4 (needs common.robot_id)

Consumers of this loader (assertion A, assertion M) must treat missing
L4/L4b as "layer present but empty" so the M-required check still fires
on keys that belong to L4/L4b -- the fact that we can't load L4 is
information: the row is missing, and M reports it as such.

The read here is INTENTIONALLY thin -- it does not check namespaces
(that is what build_overlay() does with check_namespaces=True), does
not resolve references (that is refs.resolve()), does not detect
duplicates (that is duplicates.detect_duplicates()). One responsibility
per module, so a bug in one is localised.
"""

# --------------------------------------------------------------------
# Implementation notes
# --------------------------------------------------------------------
#
# This module is one step in the freeze pipeline (ORD-1). Every step
# in that pipeline is written to the same shape:
#   * pure functions where possible (helpers with no side effects)
#   * a single run(ctx) entry point that raises XbrainError on any
#     failure and returns a small dict on success
#   * dependencies loaded from ctx (populated by earlier steps) with
#     a fresh-load fallback for isolated callers (unit tests)
#   * no imports of rclpy / requests / synchronous sqlite3 (CLAUDE.md
#     S4.1 hard rules); every heavy dependency stays behind a clear
#     factory so a test can substitute a fake
#
# Ordering guarantee: this file is imported by xbrain/boot/freeze/
# registry.py which builds the ASSERT_REGISTRY tuple. The tuple
# order is what the pipeline runner walks; the depends_on field on
# each row is a topology check, not the executor -- the executor
# trusts the registry's writer to have already put rows in a
# topologically valid order. Adding a new row means (1) adding the
# runner import here, (2) inserting the AssertSpec at the correct
# ORD-1 position, and (3) updating the CFG-FZ-N item in the TODO
# table to point at this file.
#
# Failure attribution: every raise carries detail.kind (or a
# structured rule name like QC-N / SP-N / AS-N) so a downstream
# dashboard can categorise without parsing the message string.
# Message strings are for humans; detail is for machines.

# yaml is required at runtime for parsing; loaded eagerly here so a
# missing PyYAML surfaces at import time (early in bring-up) rather
# than inside load_layers where an unrelated read error would mask it.
import os
import re
from typing import Any, Dict, List, Optional, Tuple

import yaml

from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError

# Layer name to (kind, path_frag) mapping. path_frag is the relative
# path under config_root; kind = "file" | "dir".
# L1 is a single file; L2/L3 are directories of *.yaml files.
# Tuple of (name, kind, path_frag) rows. Adding a new layer = one row
# here + a handler branch in load_layers (currently file / dir only).
# L4/L4b intentionally omitted -- their picking needs site_id/robot_id
# which live inside the tree we're loading, so they land in a later
# CFG-FZ-N when those values have been resolved.
_LAYER_SOURCES = (
    ("L1", "file", "common.yaml"),
    ("L2", "dir", "models"),
    ("L3", "dir", "safety"),
)

# ---- config variants (10 S5.4.7, user ruling 2026-09-12) ---------------------
#: Closed set of variant names. A file X_<variant>.yaml beside X.yaml is an
#: OVERLAY on X.yaml, applied only when the freeze runs with that variant
#: selected (--variant / XBRAIN_CONFIG_VARIANT); never in production. Only
#: "sim" exists today ("test" is added the day a test needs values that
#: differ from sim -- not before, CLAUDE.md 9.3).
VARIANTS: Tuple[str, ...] = ("sim",)
#: L3 safety never takes a variant (10 S5.4.6 ENV-2: the safety layer is
#: same-source on every machine, in every run).
_NO_VARIANT_LAYERS = frozenset({"L3"})
_VARIANT_RE = re.compile(r"^(?P<base>.+)_(?P<variant>%s)\.yaml$" % "|".join(VARIANTS))


def variant_of(filename: str) -> Optional[str]:
    """'m20s_sim.yaml' -> 'sim'; 'm20s.yaml' -> None. Basename only."""
    m = _VARIANT_RE.match(os.path.basename(filename))
    return m.group("variant") if m else None


def variant_sibling(path: str, variant: str) -> str:
    """'.../m20s.yaml' + 'sim' -> '.../m20s_sim.yaml'. An unknown variant
    name is a closed-set violation (CLAUDE.md 3.5), never a silent no-op."""
    if variant not in VARIANTS:
        raise XbrainError(E_CONFIG_INVALID,
                          "config variant %r not in %s" % (variant, list(VARIANTS)),
                          {"kind": "config_variant_unknown", "variant": variant})
    if not path.endswith(".yaml"):
        raise AssertionError("variant_sibling needs a .yaml path, got %r" % path)
    return path[:-len(".yaml")] + "_%s.yaml" % variant


def _with_variant(base_path: str, tree: Dict[str, Any],
                  variant: Optional[str]) -> Dict[str, Any]:
    """Overlay X_<variant>.yaml (if present) on the tree read from X.yaml.
    deep_merge, the variant winning per leaf -- the same rule the layers use.
    mutant: return `tree` untouched -> the sim overlay never applies and the
    dev freeze refuses on the nulls -> tests/configs/test_config_variants red."""
    if variant is None:
        return tree
    sib = variant_sibling(base_path, variant)
    if not os.path.isfile(sib):
        return tree
    from xbrain.common.config.merge import deep_merge  # local to avoid cycle
    return deep_merge(tree, _read_yaml(sib))



def _read_yaml(path: str) -> Dict[str, Any]:
    """Read a single YAML file into a dict; empty file -> {}.

    Raises XbrainError(E_CONFIG_INVALID) on parse failure -- a broken
    YAML at bring-up is a config problem, not a code bug. Absent file
    is NOT a failure here; the caller decides whether that layer's
    absence is legal (L2/L3 dirs may be empty in dev checkouts).
    """
    if not os.path.exists(path):
        # Caller-visible signal: empty dict. Callers distinguish
        # "layer exists but empty" from "layer missing" by looking
        # at the disk-level path themselves; here, both collapse to {}.
        return {}
    try:
        with open(path, encoding="utf-8") as fh:
            loaded = yaml.safe_load(fh)
    except yaml.YAMLError as exc:
        # Message string carries the path so a journalctl reader can
        # jump straight to the offending file. Detail.kind stays inside
        # the CFG-FZ-2 five-value closed set (config_file_missing is
        # the closest fit -- "we cannot read it").
        raise XbrainError(
            E_CONFIG_INVALID,
            "YAML parse failed at %s: %s" % (path, exc),
            {"kind": "config_file_missing", "path": os.path.abspath(path),
             "parse_error": str(exc)},
        ) from exc
    # YAML "---" empty document parses to None. Normalise to {} so
    # callers do not have to guard.
    return loaded if isinstance(loaded, dict) else {}


def _read_dir(dir_path: str, variant: Optional[str] = None,
              allow_variant: bool = True) -> Dict[str, Any]:
    """Read every BASE *.yaml in dir_path, deep-merge them in name order.
    Variant-suffixed files (X_sim.yaml) are never part of the plain walk;
    with `variant` set they overlay their base X.yaml (10 S5.4.7), and
    with allow_variant False (the safety layer) their presence is refused.

    Returns {} if the directory is missing or empty. Order is
    lexicographic on filename -- this is arbitrary but stable, and
    since L2/L3 are additive (later wins in deep_merge) a filename
    change would silently reorder precedence. Deterministic order
    makes that noticeable in review.
    """
    if not os.path.isdir(dir_path):
        return {}
    from xbrain.common.config.merge import deep_merge  # local to avoid cycle

    merged: Dict[str, Any] = {}
    for name in sorted(os.listdir(dir_path)):
        # Only *.yaml -- README / schema / _skeleton are common noise.
        if not name.endswith(".yaml"):
            continue
        # Skip _skeleton*.yaml -- convention in this repo: files whose
        # name starts with underscore are placeholders, not real data.
        # A skeleton file may contain 'null' values as documentation of
        # what SHOULD be filled in; merging it in would defeat the M
        # assertion by adding those nulls to the tree.
        if name.startswith("_"):
            continue
        if variant_of(name) is not None:
            if not allow_variant:
                # a variant file in safety/ is a defect, not a silent skip:
                # someone tried to give the safety layer a sim face (ENV-2).
                raise XbrainError(
                    E_CONFIG_INVALID,
                    "variant file %s is not allowed in %s (10 S5.4.7: the "
                    "safety layer takes no variant)" % (name, dir_path),
                    {"kind": "config_variant_in_safety",
                     "path": os.path.join(dir_path, name)})
            continue
        full = os.path.join(dir_path, name)
        one = _read_yaml(full)
        if allow_variant:
            one = _with_variant(full, one, variant)
        merged = deep_merge(merged, one)
    return merged


def load_layers(config_root: str,
                variant: Optional[str] = None) -> Dict[str, Dict[str, Any]]:
    """Read L1~L3 layer files from config_root; return {name: tree}.
    `variant` (10 S5.4.7) overlays X_<variant>.yaml on each base file of L1
    and L2; L3 (safety) never (ENV-2). None = production: variant files on
    disk are ignored entirely.

    L4 IS included, by a second pass (CFG-A6 / NEXT T18 "B12 二次合并 L4").
    sites/{site_id}.yaml cannot be named in _LAYER_SOURCES because its
    path depends on common.site_id, which is a value inside the tree
    being loaded. So the L1~L3 loop runs first, site_id is read out of
    the L1 tree it produced, and only then is L4 picked -- the "two-step
    load" shape that fv_org_enu and h_calib already use for L4 / L4b.
    When site_id is absent (production today: no supply point for it,
    that is CFG-A2) or names a file that is not there, L4 stays {} --
    exactly the shape every caller already handles.

    L4b is still NOT included: calib/{robot_id}.yaml has its own
    consumer (assertion H) which does its own two-step load and needs
    to distinguish "no calib file" from "calib file with holes" -- a
    distinction this loader would erase by merging it into the overlay.
    CFG-A6 names L4 only.

    Why the second pass is NOT "the loader coupling itself to a value":
    the coupling is one-directional and already unavoidable. L4 selection
    needs site_id no matter who does it; before this change every
    consumer that wanted L4 re-implemented the same two-step read
    (fv_org_enu._load_l4_tree, h_calib for L4b), and build_overlay --
    which already lists L4 in LAYERS with the namespace
    ("common.geo.", "common.site.", "common.retention.") -- silently
    merged an empty tree for it. That silent empty was the defect:
    a site file could supply every key it owns and the overlay would
    still report them unassigned.

    Every entry in the returned dict is present, even if empty ({}) --
    downstream build_overlay iterates the LAYERS constant, not the
    dict keys, so a missing entry would be silently skipped and its
    absence would not surface as a report. Returning {} keeps the
    layer visible in provenance as "layer present, wrote no keys".

    Order-preserving dict (Python 3.7+ guarantee): the returned mapping
    iterates in _LAYER_SOURCES declaration order, so a caller that
    prints the dict for debugging sees layers L1, L2, L3 top-to-bottom
    rather than in some hash order.

    The read here does NOT check for YAML anchors (& / *). That is
    R-7's job (see 10 S5.4.2). If a file uses anchors, this loader
    happily reads them into the tree; the anchor-check downstream is
    where the violation surfaces.
    """
    trees: Dict[str, Dict[str, Any]] = {}
    for name, kind, frag in _LAYER_SOURCES:
        # frag is a relative path; join under config_root to get absolute.
        # config_root itself may be relative when called from a test
        # (tmp_path is absolute anyway, but keep the code shape general).
        full = os.path.join(config_root, frag)
        allow = name not in _NO_VARIANT_LAYERS
        if kind == "file":
            # File case: single YAML doc -> a top-level dict (+ variant overlay).
            tree = _read_yaml(full)
            trees[name] = _with_variant(full, tree, variant) if allow else tree
        elif kind == "dir":
            # Dir case: multiple YAML files, deep-merged (variants per base file).
            trees[name] = _read_dir(full, variant=variant, allow_variant=allow)
        else:
            # Defensive: unexpected kind = construction bug in this
            # module, not a runtime config issue -- so plain AssertionError.
            # There is no third valid kind today; this branch exists to
            # localise a future edit that adds one but forgets a handler.
            raise AssertionError("unknown layer kind %r for %s"
                                 % (kind, name))

    # --- Second pass: L4 (CFG-A6) -------------------------------------
    # site_id comes from the L1 tree we just built, variant overlay
    # already applied -- so `--variant sim` picks up common_sim.yaml's
    # site_id and therefore the sim site file, while a production run
    # (no variant) sees whatever L1 itself declares.
    #
    # L5 (XBRAIN_SITE_ID) is deliberately NOT consulted here. L5 is
    # applied by build_overlay, downstream of this function, and reading
    # the environment here would make this loader's output depend on a
    # variable build_overlay has not yet merged -- two different site
    # selections for one freeze run, which is the exact failure
    # 10 S5.4.1 exists to prevent. If site_id ends up being an
    # L5-supplied key (one of the three options CFG-A2 is waiting on),
    # THIS is the line that has to move, not build_overlay.
    trees["L4"] = _load_site_layer(
        config_root,
        resolve_site_id(config_root, variant=variant,
                        l1_tree=trees.get("L1") or {}),
        variant)
    return trees


def resolve_site_id(config_root: str,
                    variant: Optional[str] = None,
                    env: Optional[Dict[str, str]] = None,
                    l1_tree: Optional[Dict[str, Any]] = None
                    ) -> Optional[str]:
    """The ONE place that decides which site file the freeze reads.

    Order is L1 then L5, matching build_overlay's layer precedence
    (LAYERS lists L5 after L4, and ENV_KEY_MAP maps XBRAIN_SITE_ID to
    common.site_id). Consulting L5 here is not optional: if the loader
    picked sites/<L1 value>.yaml while build_overlay then overwrote
    common.site_id with the L5 value, the merged tree would name one
    site and carry another site's keys, and nothing would say so.

    Why this function exists rather than each caller reading site_id:
    there are two callers that MUST agree -- load_layers (which reads
    the file) and inventory.layer_rows (which writes the file name into
    MANIFEST.layers[]). They disagreed the moment load_layers started
    picking L4 on its own: layer_rows took site_id as a parameter and
    the inventory test, which does not pass one, caught the mismatch
    (MANIFEST naming no site file while the overlay had merged one).
    A MANIFEST that names the wrong file is worse than one that names
    none -- layer_rows' own docstring says so.

    Returns None for "no site layer", which both callers already treat
    as "layer present, wrote no keys". A non-str or empty value is None
    too: a YAML `site_id: 1` must not become the path sites/1.yaml.

    `l1_tree` lets load_layers hand over the tree it has already read
    (with its variant overlay applied) instead of paying for a second
    YAML parse of the same file. Callers without a tree get an
    independent read of L1 + its variant -- same file, same helpers,
    so the same value.
    """
    if l1_tree is None:
        # Read L1 the same way load_layers does: base file, then the
        # variant sibling. Hard-coding "common.yaml" here would be a
        # second spelling of the L1 path, so take it from the table.
        l1_frag = next(frag for name, _kind, frag in _LAYER_SOURCES
                       if name == "L1")
        full = os.path.join(config_root, l1_frag)
        l1_tree = _with_variant(full, _read_yaml(full), variant)
    # L1's value first.
    site_id: Any = None
    common = l1_tree.get("common")
    if isinstance(common, dict):
        site_id = common.get("site_id")
    # Local import for the same reason deep_merge above is local:
    # xbrain.common.config imports nothing from assertions today, but
    # a future refactor that makes it do so would turn a module-level
    # import here into a cycle at interpreter start rather than a
    # failure in one function.
    from xbrain.common.config.layers import env_overlay
    # L5 wins when it supplies the key (env_overlay enforces the
    # whitelist and raises on an unknown XBRAIN_* name, so a typo does
    # not silently fall through to L1's value).
    env_flat = env_overlay(env)
    if "common.site_id" in env_flat:
        site_id = env_flat["common.site_id"]
    if not isinstance(site_id, str) or not site_id:
        return None
    return site_id


def _load_site_layer(config_root: str, site_id: Optional[str],
                     variant: Optional[str]) -> Dict[str, Any]:
    """Read sites/{site_id}.yaml for an already-resolved site_id.

    Returns {} for all three "no site layer" cases, which callers treat
    identically (layer present, wrote no keys):
      - common.site_id absent or null  -> nothing to pick
      - site_id set, file missing       -> J-2 is the assertion that
        reports a required layer file being absent; duplicating that
        judgement here would give two different verdicts for one fact
      - file present but empty          -> _read_yaml already gives {}

    The site_id value is NOT validated here (no path-segment check): a
    site_id containing a slash would reach os.path.join and could name
    a file outside sites/. That check belongs to the schema layer
    (S-10) which already types common.site_id, and to assertion D
    (10 S5.4.4 assertion D requires site_id to equal the L4 file name).
    What this function must not do is accept a tree it did not get from
    sites/, so the join is guarded below rather than trusted.
    """
    # resolve_site_id has already rejected non-str / empty / absent.
    if site_id is None:
        return {}
    # Containment guard: the joined path must stay directly inside
    # sites/. basename != site_id means site_id carried a separator.
    sites_dir = os.path.join(config_root, "sites")
    full = os.path.join(sites_dir, "%s.yaml" % site_id)
    if os.path.dirname(os.path.abspath(full)) != os.path.abspath(sites_dir):
        # Not raising: a malformed site_id is assertion D's finding and
        # it reports the mismatch with the file name in the detail.
        # Returning {} here makes D's check fire on an empty L4, which
        # is the same verdict by a clearer route.
        return {}
    if not os.path.exists(full):
        return {}
    # Same variant treatment as L1/L2 (L3 is the only no-variant layer,
    # ENV-2). A sites/{site_id}_sim.yaml is not expected to exist today;
    # treating L4 as variant-capable keeps one rule for the whole stack
    # instead of a second rule that says "except L4".
    return _with_variant(full, _read_yaml(full), variant)


# Public helper: expose the layer-name list without exposing the full
# _LAYER_SOURCES tuple (which carries private kind + path info that
# callers should not couple to).
# L6 (per-process configs) are read on demand by assertion B, not by
# load_layers -- they are not part of the overlay axis (each L6 file
# is a separate consumer's own tree, not merged into common). Kept
# here so tests + B share the same list without duplicating filenames.
_L6_FILES: Tuple[str, ...] = (
    "p1_motion.yaml",
    "p2_core.yaml",
    "p3_task.yaml",
    "p4_agent.yaml",
    "p5_gateway.yaml",
    "quadruped.yaml",
    "rtk_driver.yaml",
    # rns.yaml: the RNS module config (12 S12.0A verbatim "冻结线与各进程文件
    # 同列解析, 运行期 p1_motion 读 /run/xbrain/resolved/rns.yaml"); consumed by
    # xbrain/p1_motion/rns/config.py. perception.yaml: the perception process
    # config (19 S8.2), PSC-1 reads its resolved snapshot. Both were on disk but
    # never materialised (20 #20-26) -- p1 could only reach rns.yaml by reading
    # the SOURCE, which 10 S5.4.1 forbids; closed 2026-09-12 (P7.2).
    "rns.yaml",
    "perception.yaml",
)


def load_l6_files(config_root: str,
                  variant: Optional[str] = None) -> Dict[str, Dict[str, Any]]:
    """Read each L6 process config (+ its X_<variant>.yaml overlay when a
    variant is selected, 10 S5.4.7); return {basename: tree}. Missing
    files are silently skipped (assertion J already checked stat-able
    reachability, so a missing file here would mean J passed a partial
    tree -- caller decides how to handle that)."""
    trees: Dict[str, Dict[str, Any]] = {}
    for name in _L6_FILES:
        full = os.path.join(config_root, name)
        if os.path.isfile(full):
            trees[name] = _with_variant(full, _read_yaml(full), variant)
    return trees


def loaded_layer_names() -> List[str]:
    """The layer names load_layers actually returns. Callers who need
    to distinguish 'layer name we tried' vs 'layer name that exists in
    the LAYERS constant' use this to avoid hardcoding the set."""
    return [name for name, _kind, _frag in _LAYER_SOURCES]
