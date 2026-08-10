"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: materialise.py
Brief: Assertion materialise -- write per-process resolved YAML snapshots (CFG-FZ-18)

Description:
Runs LAST in the freeze pipeline (ORD-1). The one assertion that has a
SIDE EFFECT on the filesystem: writes /opt/xbrain_v6/data/run/resolved/{proc}.yaml
for every process in _L6_FILES with all ${common.*} references pre-expanded
against the already-resolved L0..L5 overlay tree.

* Why this file exists at all (measured 2026-08-10 blocker).
Before CFG-FZ-18 the freeze pipeline wrote MANIFEST.json only. The five
P-processes read /opt/xbrain_v6/data/run/resolved/{proc}.yaml at startup
through xbrain.common.config.resolved.load_resolved(). That file was
NEVER produced -- the freeze pipeline had no step that wrote it, and
neither did the CHK-0-56 fixture. So every P-process on every deploy
died with 'MANIFEST.json does not exist' (a misleading message: even if
MANIFEST.json were present, the per-proc YAML it names still wouldn't be).
10 S5.4.1 calls per-process reference expansion 'the most critical
structural decision' of the freeze line; a freeze that never wrote the
per-process product was silently violating that decision.

* Why LAST in ORD-1, and what happens if any earlier assertion fails.
This assertion depends_on every other assertion in the registry -- not
because it reads their output (only A's overlay and layer_trees) but
because 10 S5.4.4 defines run_freeze as ALL-OR-NOTHING: MANIFEST.json
is written only after all assertions pass. Writing per-proc YAMLs
BEFORE B/C/D/... would leave orphan snapshots on disk without a
matching MANIFEST when a later assertion raised -- and a P-process
opening those orphans would see a fresh yaml with a MANIFEST from the
PREVIOUS boot's config_rev, which is exactly the drift 10 S5.4.4 gate
R exists to prevent. Depending on every prior assertion pins the ORD-1
executor to run this one last; if any prior raises, this never runs
and no files land on disk.

* What this does NOT do (each with its own reason, so a later reader
does not add it here by mistake):
  * It does NOT expand references INSIDE common.*. That is assertion A
    (which already ran and cached the resolved overlay in ctx['overlay']).
    Doing it again here would either re-do work or, worse, drift from
    what A verified was resolvable -- and there is no guarantee resolve()
    is idempotent on trees where some values changed shape between calls.
  * It does NOT validate the per-proc yaml against a schema. Schema
    validation for p4_agent lives at xbrain.p4_agent.config.loader
    (load_p4_config), invoked by the process itself at startup so the
    error message reaches an operator via journalctl, not a boot-time
    log the operator has to chase. Doing it here would double-declare
    the schema and split its evolution across two files.
  * It does NOT enforce L6 R-6 (no top-level `common` key in per-proc
    files). That is assertion B, which runs earlier and refuses to
    proceed if any L6 file has that shape. If we get here, B passed.
  * It does NOT write MANIFEST.json. run_freeze does that AFTER
    run_assertions returns; this runner writes per-proc yamls plus
    populates ctx['processes'] so run_freeze picks up the entries that
    go into MANIFEST.processes.
  * It does NOT touch quadruped.yaml if the file is absent. quadruped is
    in _L6_FILES for the day it lands; today it silently produces no
    per-proc yaml when absent, matching the pattern that assertion J
    established (missing L6 files are 'process not yet in tree', not a
    freeze failure).

* File-write discipline (matches pipeline.py's MANIFEST.json write).
Each per-proc yaml is written with the ".new" + os.replace pattern so a
reader observing mid-write sees the previous complete file, not a
truncated one. The two writes ({proc}.yaml then MANIFEST.json) are NOT
transactional across each other -- a crash between them leaves the
per-proc yamls on disk without a MANIFEST that names them, which is
harmless (load_resolved opens MANIFEST.json first and refuses to
enumerate anything without it). The reverse order (MANIFEST first,
per-proc second) would allow the opposite: a MANIFEST that names files
that do not exist yet, and load_resolved would then open a stale file
from the previous boot -- so THIS order is the safe one.

* Contract with the pipeline:
  input:  ctx['config_root']       (required, from run_freeze)
          ctx['resolved_root']     (required, from run_freeze)
          ctx['overlay']           (optional, from assertion A; fresh-loaded
                                     via build_overlay if isolated call)
  writes: ctx['processes']         (dict per _L6_FILES, consumed by
                                     run_freeze to populate
                                     MANIFEST.processes)
  effect: /opt/xbrain_v6/data/run/resolved/{proc}.yaml x N
  returns: {status: 'pass', assertion: 'materialise', count: N,
            processes_written: [...]} (goes into MANIFEST.assertions)
  raises:  XbrainError(E_CONFIG_INVALID) on a per-proc resolve failure
           (a reference in {proc}.yaml points at a common.* path the
           overlay does not have) or on a filesystem write error.
"""

# Implementation notes -------------------------------------------------
# This module is one step in the freeze pipeline (ORD-1). Every step
# in that pipeline is written to the same shape:
#   * pure functions where possible (helpers with no side effects)
#   * a single run(ctx) entry point that raises XbrainError on any
#     failure and returns a small dict on success
#   * dependencies loaded from ctx (populated by earlier steps) with
#     a fresh-load fallback for isolated callers (unit tests)

import hashlib
import os
from typing import Any, Dict

import yaml

from xbrain.boot.freeze.assertions._layer_loader import (
    _L6_FILES, load_l6_files, load_layers,
)
from xbrain.common.config import build_overlay
from xbrain.common.config.merge import deep_merge
from xbrain.common.config.refs import ReferenceError_, resolve
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError


# The five xbrain runtime processes + quadruped. Same tuple as
# xbrain.common.config.resolved.SNAPSHOT_PROCESSES and as _L6_FILES; kept
# in _L6_FILES (which is the read-side authority) rather than duplicating
# a third copy here, so a future edit that adds a process to _L6_FILES
# automatically extends this runner's scope. Materialising for a proc
# NOT in _L6_FILES would silently drop it from MANIFEST.processes.
def _proc_name_from_filename(filename: str) -> str:
    """'p2_core.yaml' -> 'p2_core'. The five-character '.yaml' suffix
    is a hard convention across every _L6_FILES entry -- stripping by
    fixed slice is safer than str.rstrip('.yaml') which would also strip
    a trailing 'y', 'a', 'm', or 'l' from a filename that happened to
    end that way (rstrip works on the character SET, not the suffix)."""
    if not filename.endswith(".yaml"):
        # Defensive: _L6_FILES is currently all .yaml, but if a future
        # entry adds .yml or similar this raise names the mismatch loudly
        # rather than silently producing a broken proc name that
        # MANIFEST.processes would then key by.
        raise AssertionError(
            "_L6_FILES entry %r does not end in .yaml; materialise "
            "cannot derive a process name from it" % filename)
    return filename[: -len(".yaml")]


def _fail(kind: str, proc: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + proc name.

    kind: closed set for this assertion:
      per_proc_ref_unresolved -- a ${common.*} in a per-proc yaml
                                  cannot be expanded against the
                                  L0..L5 overlay
      per_proc_write_failed   -- OSError writing the per-proc yaml
                                  (permission, disk full, etc)
    """
    detail = {"kind": kind, "proc": proc}
    detail.update(extra)
    raise XbrainError(
        E_CONFIG_INVALID,
        "assertion materialise failed for proc %r: %s" % (proc, kind),
        detail,
    )


def _write_atomic(path: str, text: str) -> None:
    """Write `text` to `path` via `.new` + os.replace. POSIX rename is
    atomic on the same filesystem, so a reader that opens `path` sees
    either the OLD contents or the FULL new contents, never a partial
    write. Same discipline as pipeline.py uses for MANIFEST.json."""
    tmp = path + ".new"
    try:
        with open(tmp, "w", encoding="utf-8") as fh:
            fh.write(text)
        os.replace(tmp, path)
    except OSError as exc:
        # Try to clean up the .new tmp before re-raising so a partial
        # write does not leave a stray file behind that a next-boot
        # freeze would inherit.
        try:
            os.remove(tmp)
        except OSError:
            pass
        raise exc


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion materialise. Registered by
    registry.py as the LAST row (ORD-1); depends on every other assertion.

    Steps:
      1) Read resolved overlay from ctx (assertion A cached it) or fresh
         via build_overlay(load_layers(config_root)) for isolated callers.
      2) Load L6 per-proc source trees via load_l6_files.
      3) For each proc in _L6_FILES:
           a) Compose {common: overlay.tree.common, **proc_source}
           b) resolve() the composite tree (expands ${common.*} in proc)
           c) Drop the 'common' subtree; keep the proc-scoped values
           d) yaml.safe_dump + atomic write to resolved_root/{proc}.yaml
           e) sha256 the written bytes; record path + sha256 + size
      4) Populate ctx['processes'] so run_freeze picks it up for
         MANIFEST.processes.
    """
    # ---- ctx sanity -------------------------------------------------
    # Both are populated by run_freeze BEFORE run_assertions is called;
    # a missing key is a caller wiring bug, not a config problem, so
    # AssertionError (same shape as assertion A).
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion materialise requires ctx['config_root']; caller "
            "did not populate it (see xbrain.boot.freeze.pipeline.run_freeze)"
        )
    if "resolved_root" not in ctx:
        raise AssertionError(
            "assertion materialise requires ctx['resolved_root']; caller "
            "did not populate it (see xbrain.boot.freeze.pipeline.run_freeze)"
        )
    config_root = ctx["config_root"]
    resolved_root = ctx["resolved_root"]

    # ---- Overlay (prefer A's cache) --------------------------------
    # If assertion A ran we have both overlay and layer_trees in ctx.
    # A fresh-load fallback keeps this runner unit-testable in isolation
    # (a test that only wants to check materialise output without
    # constructing the full ctx-chain).
    overlay = ctx.get("overlay")
    if overlay is None:
        overlay = build_overlay(load_layers(config_root))

    # Resolve refs in the overlay (assertion A already verified there
    # are none unresolvable, so this raise-branch is a construction
    # guard, not a config error).
    try:
        resolved_overlay_tree = resolve(overlay.tree)
    except ReferenceError_ as exc:
        # If A cleared but we reach here, an untrapped drift happened.
        # This should be unreachable in production; keeping the raise
        # explicit rather than degrading to a warn per CLAUDE.md 3.6
        # 'never introduce assertion downgrade'.
        raise XbrainError(
            E_CONFIG_INVALID,
            "materialise: overlay re-resolve failed after assertion A "
            "reported pass -- registry drift or caller skipped A. %s"
            % exc,
            {"kind": "overlay_reresolve_failed", "reason": str(exc)},
        )

    # Isolate the common subtree; that is the ONLY namespace L6 refs
    # may point at (R-2). Wrapping in a dict keeps a fresh reference so
    # deep_merge below does not mutate the resolved_overlay we cached.
    common_only = {"common": resolved_overlay_tree.get("common", {})}

    # ---- Per-proc load ---------------------------------------------
    # load_l6_files silently skips missing files (assertion J vouches
    # for reachability; a proc without a yaml is 'not yet in tree').
    l6_trees = load_l6_files(config_root)

    processes: Dict[str, Dict[str, Any]] = {}
    written_names = []
    for filename in _L6_FILES:
        proc = _proc_name_from_filename(filename)
        proc_source = l6_trees.get(filename)
        if proc_source is None:
            # Missing L6 file -> skip. The proc will absent-key from
            # MANIFEST.processes; load_resolved refuses to open that
            # proc with a message naming the missing MANIFEST entry.
            continue

        # Compose common + proc into ONE tree so resolve()'s lookup can
        # find common.* paths. R-6 forbids proc_source from having a
        # top-level 'common' key (assertion B enforces this earlier);
        # deep_merge below trusts that guarantee -- if it were violated
        # the proc's common would override the overlay's, which would
        # be exactly the drift 10 S5.4.4 fails on.
        combined = deep_merge(common_only, proc_source)

        try:
            expanded = resolve(combined)
        except ReferenceError_ as exc:
            # A per-proc yaml references a common.* path that either
            # does not exist, has a cycle, or violates R-1..R-4. Detail
            # carries the proc name so an operator sees WHICH file has
            # the broken ref, plus the resolver's own message.
            _fail("per_proc_ref_unresolved", proc,
                  reason=str(exc))
            # _fail always raises; the return here is unreachable but
            # satisfies static-analysis tools that flag the try body.
            raise  # pragma: no cover -- _fail raises

        # Drop the common subtree; the per-proc yaml carries only its
        # own namespace values (with refs expanded). Keeping common in
        # the per-proc file would (a) bloat every snapshot by ~15 KB
        # and (b) duplicate values across six files, making a manual
        # inspection of one snapshot show values that belong to the
        # overlay, not to this proc.
        proc_only = {k: v for k, v in expanded.items() if k != "common"}

        # Serialise. allow_unicode so Chinese keyword strings survive
        # as themselves (not \uXXXX escapes); sort_keys so the sha256
        # is byte-stable across runs (an operator diffing two snapshots
        # sees only real content changes, not dict-order shuffles); no
        # default_flow_style so the output is block YAML, readable.
        text = yaml.safe_dump(
            proc_only,
            allow_unicode=True,
            sort_keys=True,
            default_flow_style=False,
        )

        out_path = os.path.join(resolved_root, filename)
        try:
            _write_atomic(out_path, text)
        except OSError as exc:
            _fail("per_proc_write_failed", proc,
                  path=out_path, errno=exc.errno,
                  strerror=exc.strerror)

        # sha256 of the WRITTEN bytes, not of the in-memory dict. This
        # is what a downstream reader (P-process at startup) can
        # recompute against the file on disk to verify integrity.
        sha = hashlib.sha256(text.encode("utf-8")).hexdigest()
        processes[proc] = {
            "path": out_path,
            "sha256": sha,
            "size_bytes": len(text.encode("utf-8")),
        }
        written_names.append(proc)

    # Publish the per-proc index into ctx so run_freeze can pull it
    # into MANIFEST.processes. Keeping the sha256 in the manifest is
    # what CFG-41 uses to detect a snapshot that was tampered with
    # between the freeze that wrote it and the process that reads it.
    ctx["processes"] = processes

    return {
        "status": "pass",
        "assertion": "materialise",
        "count": len(processes),
        # list, NOT tuple. This dict lands verbatim in
        # MANIFEST.assertions['materialise'], which is written via
        # json.dump; json emits tuples as lists, so a tuple here would
        # make the on-disk MANIFEST diverge from the in-memory dict --
        # tests comparing `run_freeze()` return vs json.loads(disk)
        # would then fail on a purely serialisation-shape difference.
        "processes_written": list(written_names),
    }
