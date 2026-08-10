#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: materialize_resolved.py
Brief: Dev-only helper -- materialise resolved config products via CHK-0-56 fixture

Description:
For the voice-loop smoke test, xbrain-config-freeze.service is NOT
run (systemd freeze writes to the production RESOLVED_ROOT which is
/opt/xbrain_v6/data/run/resolved and would still need root to
create on first boot). This helper reuses the CHK-0-56 fixture
(tests/fixtures/conftest.py + overrides.py) to build a filled
config tree and runs run_freeze against it.

After CFG-FZ-18 (2026-08-10) the freeze pipeline itself writes
per-process yamls plus MANIFEST.json, so this helper just:
  * runs the fixture's _build_and_freeze on a tmp staging dir
  * moves the entire resolved output (MANIFEST.json + N per-proc
    yamls) to the caller's chosen root

If freeze fails on a KNOWN_FAILING_ASSERTIONS entry (fixture's
list of pre-existing gaps waiting on BIZ review), this script
exits NON-ZERO with the specific assertion + reason. It does
NOT fabricate output -- a fake MANIFEST or a copied source yaml
would produce a manifest whose sha256 does not match the file
on disk (or worse, would let a P-process boot on unresolved
${...} strings), and neither failure is discoverable without
running the whole voice loop end-to-end.

Default output: /opt/xbrain_v6/data/run/resolved   (V6 rule -- all
                runtime dependencies MUST live under /opt/xbrain_v6/)
Env override:   XBRAIN_RESOLVED_ROOT

Usage:
  python3 scripts/dev/materialize_resolved.py
    -> writes /opt/xbrain_v6/data/run/resolved/{MANIFEST.json,*.yaml}
  python3 scripts/dev/materialize_resolved.py --root /custom/path
    -> writes to /custom/path
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path


DEFAULT_OUT = "/opt/xbrain_v6/data/run/resolved"
BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"


def _read_real_boot_id() -> str:
    """Return the running kernel's boot_id, stripped.

    The fixture writes 'fixture-boot-id' into MANIFEST.boot_id because
    it does not know the ambient boot id and pytest cases must be
    deterministic. On a real machine load_resolved's gate compares the
    manifest boot_id against /proc/sys/kernel/random/boot_id and
    refuses any snapshot from a previous boot. This helper reads the
    current boot_id so the dev-materialise flow can rewrite the
    fixture-shaped MANIFEST with a value that matches the running
    kernel -- exactly what xbrain-config-freeze.service would do on a
    real boot.
    """
    with open(BOOT_ID_PATH, encoding="utf-8") as fh:
        value = fh.read().strip()
    if not value:
        # Same guard as xbrain.common.config.resolved: an empty boot_id
        # cannot establish freshness. Refuse loudly.
        raise RuntimeError(
            "%s is empty; cannot rewrite MANIFEST.boot_id without a real "
            "kernel boot_id to substitute" % BOOT_ID_PATH)
    return value


def _rewrite_manifest_for_final_location(
    manifest_path: Path, boot_id: str, final_root: Path
) -> None:
    """Rewrite MANIFEST.json for post-copy state: boot_id + per-proc
    paths. Both are needed because materialise ran under the fixture's
    staging dir; after this helper moved the files to `final_root`,
    the MANIFEST still names the staging paths and the fixture boot_id.

    Path rewrite: for each MANIFEST.processes[proc]['path'], swap the
    basename onto `final_root`. Load_resolved's path-confinement gate
    (10 S5.4.1, CFG-ROOT-5) refuses any path outside RESOLVED_ROOT.

    Boot_id rewrite: fixture writes 'fixture-boot-id'; the ambient
    kernel boot_id is what load_resolved's freshness gate compares
    against /proc/sys/kernel/random/boot_id.

    Same .new + os.replace atomic-write pattern the freeze pipeline
    uses so a concurrent reader either sees the fixture-stamped
    MANIFEST or the fully-rewritten one, never a truncated file.
    """
    with open(manifest_path, encoding="utf-8") as fh:
        manifest = json.load(fh)
    manifest["boot_id"] = boot_id
    processes = manifest.get("processes") or {}
    for proc_name, entry in processes.items():
        if not isinstance(entry, dict):
            continue
        src_path = entry.get("path")
        if not src_path:
            continue
        entry["path"] = str(final_root / Path(src_path).name)
    tmp = manifest_path.with_suffix(manifest_path.suffix + ".new")
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, ensure_ascii=False, indent=2, sort_keys=True)
        fh.write("\n")
    os.replace(tmp, manifest_path)


def main() -> int:
    ap = argparse.ArgumentParser(prog="materialize_resolved")
    ap.add_argument("--root", default=os.environ.get(
        "XBRAIN_RESOLVED_ROOT", DEFAULT_OUT))
    args = ap.parse_args()

    # Cross-import into tests/ is INTENTIONAL for dev tooling. The
    # production stack never reaches tests/; scripts/dev is the seam
    # where the two connect and it is documented in the script header.
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

    from tests.fixtures.conftest import _build_and_freeze
    from tests.fixtures.overrides import assert_no_safety_overrides

    # CHK-0-56 (iv) guard: NULL_OVERRIDES must not touch common.safety.*.
    # If a future edit adds a safety key, this raises before any config
    # touches disk.
    assert_no_safety_overrides()

    out = Path(args.root)
    if out.exists():
        # Clean prior run's output so a stale MANIFEST from a previous
        # config_rev cannot be paired with fresh per-proc yamls. rmtree
        # + mkdir is safe here because 'out' is the sanctioned resolved
        # root; nothing else should be writing to it during this run.
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)

    # The fixture wants a parent tmpdir; it creates configs/ + resolved/
    # under it. Stage under a sibling of `out` so the subsequent move is
    # a same-filesystem shutil.move (no copy).
    stage = out.parent / (out.name + "_staging")
    if stage.exists():
        shutil.rmtree(stage)
    stage.mkdir(parents=True, exist_ok=True)

    handle = _build_and_freeze(stage)
    if handle.freeze_error is not None:
        # KNOWN_FAILING was hit. The fixture synthesised a manifest
        # for pytest downstream but no resolved products were written.
        # This script does NOT fabricate them -- a P-process reading a
        # fake resolved yaml would boot on unresolved ${...} placeholders
        # or on values from a MANIFEST that never matched what freeze
        # produced. Both are worse than 'refuses to start' because the
        # boot appears successful.
        print(
            "materialize_resolved: freeze aborted on a KNOWN_FAILING "
            "assertion; no resolved products were written.",
            file=sys.stderr)
        print(
            "  error kind: %s" % type(handle.freeze_error).__name__,
            file=sys.stderr)
        print(
            "  error msg:  %s" % handle.freeze_error, file=sys.stderr)
        print(
            "  next step:  fix the assertion (per its KNOWN_FAILING "
            "entry in tests/fixtures/conftest.py) or accept the "
            "voice-loop cannot boot until then.",
            file=sys.stderr)
        shutil.rmtree(stage, ignore_errors=True)
        return 2

    if handle.manifest is None:
        # Freeze succeeded (no known-failing) but returned no manifest.
        # This should be unreachable given _build_and_freeze contract;
        # print + exit rather than silently produce nothing.
        print("materialize_resolved: freeze produced no manifest -- "
              "aborting", file=sys.stderr)
        shutil.rmtree(stage, ignore_errors=True)
        return 1

    # Move the staged resolved contents (MANIFEST.json + N per-proc
    # yamls, written by CFG-FZ-18's materialiser assertion) to args.root.
    staged_resolved = Path(handle.resolved_root)
    for item in staged_resolved.iterdir():
        dst = out / item.name
        if item.is_dir():
            shutil.copytree(item, dst)
        else:
            shutil.copy2(item, dst)

    # Rewrite MANIFEST after the copy: substitute the ambient boot_id
    # (fixture writes 'fixture-boot-id') AND swap staging paths in
    # processes[proc].path to the final resolved-root location. Both
    # are load_resolved gates that would otherwise refuse the snapshot.
    manifest_path = out / "MANIFEST.json"
    if manifest_path.exists():
        real_boot_id = _read_real_boot_id()
        _rewrite_manifest_for_final_location(manifest_path, real_boot_id, out)

    written = sorted(p.name for p in out.iterdir())
    print("materialize_resolved: wrote %s" % out)
    print("  files: %s" % written)

    # Clean staging.
    shutil.rmtree(stage, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
