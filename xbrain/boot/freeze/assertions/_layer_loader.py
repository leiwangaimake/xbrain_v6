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
once. Callers can either invoke load_layers(root) themselves, or read
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
from typing import Any, Dict, List, Tuple

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
        )
    # YAML "---" empty document parses to None. Normalise to {} so
    # callers do not have to guard.
    return loaded if isinstance(loaded, dict) else {}


def _read_dir(dir_path: str) -> Dict[str, Any]:
    """Read every *.yaml in dir_path, deep-merge them in name order.

    Returns {} if the directory is missing or empty. Order is
    lexicographic on filename -- this is arbitrary but stable, and
    since L2/L3 are additive (later wins in deep_merge) a filename
    change would silently reorder precedence. Deterministic order
    makes that noticeable in review.
    """
    if not os.path.isdir(dir_path):
        return {}
    from xbrain.common.config.merge import deep_merge   # local to avoid cycle

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
        one = _read_yaml(os.path.join(dir_path, name))
        merged = deep_merge(merged, one)
    return merged


def load_layers(config_root: str) -> Dict[str, Dict[str, Any]]:
    """Read L1~L3 layer files from config_root; return {name: tree}.

    L4/L4b are NOT included -- they require site_id/robot_id which are
    values inside the tree we're loading, and picking them here would
    couple this loader to those values. See module docstring.

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
        if kind == "file":
            # File case: single YAML doc -> a top-level dict.
            trees[name] = _read_yaml(full)
        elif kind == "dir":
            # Dir case: multiple YAML files, deep-merged.
            trees[name] = _read_dir(full)
        else:
            # Defensive: unexpected kind = construction bug in this
            # module, not a runtime config issue -- so plain AssertionError.
            # There is no third valid kind today; this branch exists to
            # localise a future edit that adds one but forgets a handler.
            raise AssertionError("unknown layer kind %r for %s"
                                 % (kind, name))
    return trees


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
)


def load_l6_files(config_root: str) -> Dict[str, Dict[str, Any]]:
    """Read each L6 process config; return {basename: tree}. Missing
    files are silently skipped (assertion J already checked stat-able
    reachability, so a missing file here would mean J passed a partial
    tree -- caller decides how to handle that)."""
    trees: Dict[str, Dict[str, Any]] = {}
    for name in _L6_FILES:
        full = os.path.join(config_root, name)
        if os.path.isfile(full):
            trees[name] = _read_yaml(full)
    return trees


def loaded_layer_names() -> List[str]:
    """The layer names load_layers actually returns. Callers who need
    to distinguish 'layer name we tried' vs 'layer name that exists in
    the LAYERS constant' use this to avoid hardcoding the set."""
    return [name for name, _kind, _frag in _LAYER_SOURCES]
