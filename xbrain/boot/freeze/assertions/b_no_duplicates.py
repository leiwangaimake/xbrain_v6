"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: b_no_duplicates.py
Brief: Assertion B -- no-duplicates + alias blacklist (CFG-FZ-4)

Description:
Runs FOURTH in the freeze pipeline (ORD-1, after M). Walks EACH L6
process config as its OWN tree and rejects three shapes:

  (1) L6 file carries a `common` top-level key -- shared state must
      be referenced with `${common.*}`, never redefined per-process.
      What makes this failure invisible after the fact: each process
      gets its own resolved artifact, and no downstream comparator
      checks that two artifacts hold the same value for a shared key.
      Two processes silently disagreeing is exactly what this door
      exists to prevent.
  (2) L6 file uses a key NAME that is on the 10 S5.4.5 alias
      blacklist -- e.g. `dedup_min_dist_m` when the shared name is
      `common.recording.min_dist_m`. Same drift-over-time failure
      as (1); the check runs on the raw L6 tree, before any merge.
  (3) A dotted path (leaf) on the blacklist too. Some blacklist
      entries name a specific location, not just a bare leaf name.

Check runs PER FILE, not on the merged tree. The whole point is to
catch a private redefinition before it merges up; running on the
merged tree would mask the case where L6 wrote `common.foo = ...`
and L1 also had `common.foo = ...` -- deep_merge would have picked
one, and the other's value would silently vanish.

CFG-FZ-4 variant (1) verbatim: p4_agent.yaml carries
`point_min_dist_m: 0.5` -> B must fire on that private key because
`point_min_dist_m` is in BLACKLIST.

Contract with the pipeline:
  input:  ctx["config_root"]
  reads:  L6 files (via _layer_loader.load_l6_files)
  raises: XbrainError(E_CONFIG_INVALID) with detail.kind = one of
          l6_common_top_level / l6_alias_name / l6_alias_path
"""

# Standard-library first (typing), then internal. _alias_table owns the
# blacklist; _layer_loader owns the L6 file reader. B is the glue that
# runs the blacklist check across every L6 tree.
from typing import Any, Dict

from xbrain.boot.freeze.assertions._alias_table import BLACKLIST
from xbrain.boot.freeze.assertions._layer_loader import load_l6_files
from xbrain.common.config.merge import flatten
# E_CONFIG_INVALID (or E_QOS_VIOLATION / E_CONFIG_LOCKED)
# imported by name from xbrain.common.errors instead of
# spelled as a string literal. CLAUDE.md 3.5 forbids literal
# E_* strings anywhere outside common/errors/; scripts/lint/
# no_literal_ecode.py enforces it (both the whole-word literal
# and the substring form).
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError


def _fail(kind: str, file_name: str, key: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + file + key path.

    All three closed-set detail.kind values (l6_common_top_level /
    l6_alias_name / l6_alias_path) flow through this single raise site
    so a future edit that adds a fourth kind writes the message shape
    only once. Message string interpolates BOTH file and key so a
    journalctl reader without decoder scripts can act on the failure.
    """
    detail = {"kind": kind, "file": file_name, "key": key}
    detail.update(extra)
    raise XbrainError(
        E_CONFIG_INVALID,
        "assertion B failed in %s: %s at %r" % (file_name, kind, key),
        detail,
    )


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion B. Replaces registry's stub_b.

    B is unusual among freeze assertions: it does NOT read the merged
    overlay (A / M / C / D do). B needs the RAW per-file L6 trees
    because the check catches redefinition BEFORE merge; walking the
    merged tree would show only whichever L6 file's value survived
    deep_merge and hide the other one.
    """
    # Same wiring guard as J / A / M -- ctx missing config_root is
    # a caller-side bug, not a config problem, so AssertionError.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion B requires ctx['config_root']; caller did not "
            "populate it"
        )
    root = ctx["config_root"]

    # Load each L6 file independently. Assertion B runs on the RAW
    # per-process tree, not the merged tree -- see module docstring.
    # load_l6_files returns {basename: tree} for whichever files exist
    # under root; missing ones are silently skipped (J already verified
    # reachability, so a missing L6 here means J passed a partial
    # tree, which is the caller's concern, not B's).
    l6_trees = load_l6_files(root)

    # Track files checked so success payload can name the count -- an
    # empty tree (no L6 files present, unusual but possible on a fresh
    # checkout) would otherwise give a silently green pass with
    # checked_files == 0. A monitor watching the manifest for that
    # value can detect the "no L6 present" edge case immediately.
    checked_files = 0

    # sorted() so the failing file is stable across runs -- if two L6
    # files both violate, the alphabetical-earlier one wins the raise.
    # This matters for reproducibility of failure logs: an operator
    # bisecting a config change should see the same first-failing file
    # regardless of dict iteration order.
    for file_name, tree in sorted(l6_trees.items()):
        # Skip completely empty trees -- they cannot violate anything.
        # This is NOT a fail-silent: J already checked that the file
        # exists and is stat-able; an empty file passes M anyway because
        # M reads the merged overlay, not L6 raw.
        if not tree:
            continue
        checked_files += 1

        # (1) L6 top-level `common` key -- forbidden.
        # This is the classic "redefine shared state" mistake: an L6
        # file writes `common: { foo: 1 }`, deep_merge picks one value
        # per key, and the other process silently gets a different one.
        # Check the top-level dict key rather than any deeper path;
        # a `common` nested somewhere else (e.g. `foo.common.bar`) is
        # a name collision that we do not want to false-flag.
        if "common" in tree:
            _fail("l6_common_top_level", file_name, "common",
                  reason="L6 must reference shared state via ${common.*}, "
                         "not redefine it")

        # (2) + (3) alias-blacklist check. Walk the flat tree and match
        # BOTH bare leaf names and dotted paths against BLACKLIST.
        # flatten() gives dotted leaves so both kinds are one iteration.
        # Iteration order is dict-insertion (Python 3.7+ guarantee)
        # which is YAML declaration order, so the first hit's file
        # location is roughly what an operator scrolling the file
        # would find first.
        flat = flatten(tree)
        for key in flat:
            # (2) leaf-name check: last dotted component.
            # Catches `asr.point_min_dist_m` since the leaf is
            # point_min_dist_m (in BLACKLIST) regardless of the parent.
            leaf = key.split(".")[-1]
            if leaf in BLACKLIST:
                _fail("l6_alias_name", file_name, key,
                      leaf=leaf,
                      alias_of="see 10 S5.4.5 for canonical common.* name")
            # (3) full-path check: the whole dotted key.
            # Catches entries in BLACKLIST that name a specific dotted
            # location rather than a bare leaf (event.db_path style).
            if key in BLACKLIST:
                _fail("l6_alias_path", file_name, key,
                      alias_of="see 10 S5.4.5 for canonical common.* name")

    # blacklist_size in the return so a MANIFEST reader can spot-check
    # the BLACKLIST count didn't silently shrink between runs; a shrunk
    # blacklist would silently accept newly-added aliases. Reporting
    # the count doesn't defend against a shrink, but a MANIFEST diff
    # from run to run will surface it.
    # Success payload: report checked count + blacklist size so the
    # MANIFEST journal makes both facts observable across freeze runs.
    return {
        "status": "pass",
        "assertion": "B",
        "l6_files_checked": checked_files,
        "blacklist_size": len(BLACKLIST),
    }
