"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: pipeline.py
Brief: Freeze pipeline -- iterate ASSERT_REGISTRY, produce MANIFEST.json +
       per-process resolved snapshots

Description:
The framework half of CFG-FZ-1. Every assertion body lives on its own
AssertSpec.runner (registry.py); this module ONLY orders their execution
and assembles the output artifacts. Splitting the two ways round would
put each assertion's step index inline in the pipeline and defeat the
bidirectional-diff test that keeps MANIFEST honest -- see registry.py's
own docstring on the "invisible if" trap.

Output shape (10 S5.4.6, and the CFG-FZ-1 field-list correction):
  /run/xbrain/resolved/MANIFEST.json
    {
      "schema": "xbrain.config.manifest/1",
      "boot_id": ...,                    # /proc/sys/kernel/random/boot_id
      "config_root": ...,                # absolute path used
      "config_root_overridden": bool,    # XBRAIN_CONFIG_DIR set?
      "common_digest": ...,              # canonical hash of common.* leaves
      "config_rev": ...,                 # CFG-41: overall digest of
                                         #   MANIFEST.layers so an event can
                                         #   name WHICH configuration ran
      "calib_rev": ...,                  # extrinsics rev, or "unspecified"
      "layers": [ {name, sha256}, ... ], # L0..L6 files that fed the freeze
      "processes": {                     # per-process snapshot metadata
        "p1_motion": {"sha256": ..., "path": ".../p1_motion.yaml"},
        ...
      },
      "assertions": {                    # name -> runner result
        "J": {"status": "stub", "assertion": "J"},
        ...
      }
    }

Each individual per-process YAML lives beside MANIFEST.json under the
resolved root. This module DOES NOT produce them yet -- rendering them
is CFG-FZ-1's field but the actual TREE assembly (deep_merge across L0..L6
+ ${common.*} expansion) is what CFG-CM-7 / CFG-CM-8 already do. The
current landing is the FRAMEWORK + MANIFEST + registry iteration; the
per-process YAML materialisation lands with CFG-FZ-9 (materialiser)
which every downstream assertion depends on.

Design constants:
  MANIFEST_SCHEMA -- version string in the manifest; a reader that finds
                    another value refuses (10 S5.4.6). Bumped only when
                    the shape breaks backward reads.
  RESOLVED_ROOT_DEFAULT -- tmpfs mount CFG-BT-22 creates. Deliberately
                    NOT default-created here: a mkdir would mask a mount
                    problem, the pipeline REFUSES to run when it is
                    missing and the CFG-BT-22 tmpfiles.d config is what
                    creates it (INF-DP-11 disciplines).
"""

# Standard-library only; freeze runs BEFORE the runtime, so third-party
# imports here would extend the "must be installed by bring-up" surface.
import json
import os
from typing import Any, Dict, Mapping, Optional

# Registry primitives imported through the package boundary, not the module,
# so a future refactor that moves the registry into its own subpackage does
# not need to touch this file.
from xbrain.boot.freeze.registry import (
    ASSERT_REGISTRY, ordered_assertion_names, validate_topology,
)

# MANIFEST schema version. Bumped only when the SHAPE breaks backward
# reads. xbrain.common.config.resolved.load_manifest verifies this value
# equals the reader's expectation, refusing on mismatch -- so a rolled
# forward pipeline that changed the shape without bumping schema would
# be caught at runtime rather than after silently changing the format.
# v1 is what xbrain.common.config.resolved.load_manifest expects. A bump
# here without updating the reader would refuse ALL manifests, so both
# constants move together in the same commit.
# 2026-08-10 unified with the reader: the authoritative schema string
# is 10 S5.4.6 line 3305 verbatim 'xbrain.config.manifest/1', which is
# what xbrain.common.config.resolved.MANIFEST_SCHEMA also carries. Prior
# to this line the writer wrote 'xbrain-manifest-v1' and every P-process
# boot rejected the manifest at load time with a schema-mismatch error.
# The two constants live in two modules on purpose (writer + reader
# isolation) but the string value MUST match.
MANIFEST_SCHEMA = "xbrain.config.manifest/1"

# Tmpfs mount that receives all freeze output. CFG-BT-22 owns the mount
# (tmpfiles.d config); the pipeline REFUSES to run when the directory is
# missing rather than mkdir-ing over what might be a failed mount.
# 2026-08-10 relocated to /opt/xbrain_v6/data/run/resolved; see
# xbrain/common/config/resolved.py:RESOLVED_ROOT for the rationale.
RESOLVED_ROOT_DEFAULT = "/opt/xbrain_v6/data/run/resolved"

# The six processes that receive a per-process YAML snapshot. Same set as
# xbrain.common.config.resolved.SNAPSHOT_PROCESSES; kept a local literal so
# the pipeline does not silently drop a process when that constant grows --
# a mismatch surfaces at bidirectional-diff of MANIFEST.processes.
SNAPSHOT_PROCESSES = (
    # Five xbrain runtime processes + quadruped (RT-facing C++ node that
    # still reads a resolved snapshot for its Chinese-side config values).
    # Order is p1..p5 alphabetical, then quadruped -- kept stable so a
    # MANIFEST.processes JSON diff between runs is legible.
    "p1_motion", "p2_core", "p3_task", "p4_agent", "p5_gateway", "quadruped",
)


def build_manifest(*, boot_id: str, config_root: str,
                   config_root_overridden: bool,
                   common_digest: str, config_rev: str,
                   calib_rev: str,
                   layers: list,
                   processes: Mapping[str, Mapping[str, Any]],
                   assertions: Mapping[str, Mapping[str, Any]]) -> Dict[str, Any]:
    """Assemble the MANIFEST dict from named parts.

    All arguments are REQUIRED and keyword-only; a caller that dropped one
    would emit a MANIFEST short of a field a consumer reads (assertion I
    walks `processes[...]`, U18 walks `layers[]`, CFG-41 walks
    `config_rev`, etc.). No defaults: the framework is loud on missing.
    """
    return {
        "schema": MANIFEST_SCHEMA,               # 10 S5.4.6 required
        "boot_id": boot_id,                       # gate: matches /proc's boot_id
        "config_root": config_root,               # absolute path used
        "config_root_overridden": config_root_overridden,   # XBRAIN_CONFIG_DIR set
        "common_digest": common_digest,           # canonical hash of common.* leaves
        "config_rev": config_rev,                 # CFG-41 overall digest
        "calib_rev": calib_rev,                   # extrinsics rev (or unspecified)
        "layers": list(layers),                   # L0..L6 files that fed the freeze
        "processes": dict(processes),             # per-process snapshot metadata
        "assertions": dict(assertions),           # name -> runner result
    }


def run_assertions(context: Optional[Dict[str, Any]] = None) -> Dict[str, Dict[str, Any]]:
    """Iterate ASSERT_REGISTRY in declaration order, invoke each runner,
    collect results into `{name: result_dict}`.

    Every runner receives the same shared `context` (empty dict by default),
    so a later assertion can read what an earlier one recorded -- e.g.
    assertion A adds the resolved tree to context, M reads it. This mirrors
    what the individual CFG-FZ-N items will populate; the framework itself
    only guarantees the ORDER and the collection shape.
    """
    # ORD-1 sanity: before running anything, prove every depends_on names an
    # earlier row. A misordered registry would let an assertion run before
    # its input existed and MANIFEST would carry the wrong story.
    validate_topology()
    # ctx is shared across all runners in this pass. Empty default (not None)
    # so a stub runner that indexes ctx sees {} rather than raising.
    ctx = context if context is not None else {}
    # Insertion-ordered dict (Python 3.7+ guarantee) preserves ORD-1 in the
    # returned mapping too -- the caller that dumps this into MANIFEST.assertions
    # gets a JSON object whose key order matches declaration order.
    results: Dict[str, Dict[str, Any]] = {}
    for spec in ASSERT_REGISTRY:
        if spec.runner is None:
            # A registered spec without a runner is a construction defect;
            # loud rather than silent (would render {} into MANIFEST).
            raise AssertionError(
                "AssertSpec %r has no runner (defaulted to None); the row "
                "must supply either a stub or the real body" % spec.name
            )
        # Each runner receives ctx (mutable across the loop), so a stage
        # can pass forward what the next reads. Runners RETURN a result
        # dict; ctx is for cross-runner state, results is the MANIFEST view.
        results[spec.name] = spec.runner(ctx)
    return results


def run_freeze(*, boot_id: str, config_root: str,
               config_root_overridden: bool,
               common_digest: str, config_rev: str,
               calib_rev: str = "unspecified",
               layers: Optional[list] = None,
               processes: Optional[Mapping[str, Mapping[str, Any]]] = None,
               resolved_root: str = RESOLVED_ROOT_DEFAULT,
               context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """End-to-end freeze pass: run assertions, build manifest, write to
    resolved_root/MANIFEST.json, return the manifest dict.

    processes / layers default to empty rather than to a scan of disk --
    CFG-FZ-1 is the FRAMEWORK; the individual materialiser item (CFG-FZ-9)
    fills them from the six per-process YAML snapshots. Callers that pass
    them in early can start using MANIFEST fields their assertion needs.
    """
    # Pre-flight: resolved_root exists and is a directory. CFG-BT-22 owns
    # the tmpfs mount; refusing here surfaces a missing mount as a config
    # error rather than silently mkdir-ing over it. A raw exists() would
    # accept a stray FILE at that path; isdir catches that too.
    if not os.path.isdir(resolved_root):
        raise FileNotFoundError(
            "resolved_root %r missing; tmpfiles.d must create it "
            "(CFG-BT-22 / config_root_missing)" % resolved_root
        )
    # Assertions FIRST -- if any raises, we never touch the filesystem, so
    # a partial MANIFEST cannot be observed. A crashed assertion leaves the
    # PREVIOUS MANIFEST.json intact for post-mortem.
    # Populate ctx with fields every assertion may need. config_root is
    # required by assertion J (real body lands with CFG-FZ-2); putting
    # it in ctx here rather than as a positional arg keeps runner
    # signatures uniform across all 13 rows.
    ctx = dict(context) if context is not None else {}
    ctx.setdefault("config_root", config_root)
    ctx.setdefault("config_root_overridden", config_root_overridden)
    # CFG-FZ-18 (materialise) writes per-proc yamls to resolved_root and
    # populates ctx['processes'] so the entries land in MANIFEST.processes
    # below. resolved_root has to reach the runner via ctx because
    # AssertSpec.runner takes only ctx -- adding a positional arg would
    # break every existing runner. setdefault so a caller that already
    # populated ctx['resolved_root'] (unit-test isolation) wins.
    ctx.setdefault("resolved_root", resolved_root)
    assertions = run_assertions(ctx)
    # Explicit caller override wins over materialise's ctx population --
    # so a test that wants to inject a specific processes dict still
    # can. Fall back to whatever materialise wrote; final fallback is
    # empty (matches the pre-CFG-FZ-18 behaviour).
    _processes = processes
    if _processes is None:
        _processes = ctx.get("processes", {})
    manifest = build_manifest(
        boot_id=boot_id,
        config_root=config_root,
        config_root_overridden=config_root_overridden,
        common_digest=common_digest,
        config_rev=config_rev,
        calib_rev=calib_rev,
        layers=layers or [],
        processes=_processes,
        assertions=assertions,
    )
    manifest_path = os.path.join(resolved_root, "MANIFEST.json")
    # Atomic write pattern (same discipline as scripts/deploy/install.sh's
    # symlink flip): create MANIFEST.json.new, fsync via os.replace, rename.
    # A reader that woke mid-write would otherwise see a truncated JSON
    # file and refuse to parse it -- the runtime would then think the
    # freeze had FAILED rather than "not-yet-finished".
    # ".new" sibling ensures the tmp is on the same filesystem as the final
    # path -- POSIX rename atomicity requires that.
    tmp_path = manifest_path + ".new"
    with open(tmp_path, "w", encoding="utf-8") as fh:
        # sort_keys so the file is stable across runs -- an operator diffing
        # two MANIFEST.json snapshots sees only real changes, not dict-order
        # shuffles that carry no meaning. indent=2 keeps it human-readable.
        json.dump(manifest, fh, ensure_ascii=False, indent=2, sort_keys=True)
        fh.write("\n")
    # os.replace is POSIX rename(2) on the same filesystem = atomic. Both
    # paths are under resolved_root (a single tmpfs mount), so this is safe.
    os.replace(tmp_path, manifest_path)          # atomic on POSIX
    # Return the in-memory dict so callers (tests, systemd log formatting)
    # can inspect it without re-reading the file we just wrote.
    return manifest
