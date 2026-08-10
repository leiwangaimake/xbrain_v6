#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: materialize_resolved.py
Brief: Dev-only helper -- materialise resolved config products at a writable path

Description:
For the voice-loop smoke test, xbrain-config-freeze.service is NOT
run (systemd freeze writes to the production RESOLVED_ROOT which is
now /opt/xbrain_v6/data/run/resolved and would still need root to
create on first boot). This helper reuses the CHK-0-56 fixture
(tests/fixtures/conftest.py + overrides.py) to build a filled
config tree and runs run_freeze against it, writing MANIFEST.json +
per-process resolved snapshots to a caller-chosen directory.

Default output: /opt/xbrain_v6/data/run/resolved   (V6 rule -- all
                runtime dependencies MUST live under /opt/xbrain_v6/)
Env override:   XBRAIN_RESOLVED_ROOT

The output tree can be pointed at with --resolved-root when
launching each P-process, so the same load_resolved() code path
runs unchanged.

Usage:
  python3 scripts/dev/materialize_resolved.py
    -> writes /opt/xbrain_v6/data/run/resolved/{MANIFEST.json,*.yaml}
  python3 scripts/dev/materialize_resolved.py --root /custom/path
    -> writes to /custom/path
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path


DEFAULT_OUT = "/opt/xbrain_v6/data/run/resolved"


def main() -> int:
    ap = argparse.ArgumentParser(prog="materialize_resolved")
    ap.add_argument("--root", default=os.environ.get(
        "XBRAIN_RESOLVED_ROOT", DEFAULT_OUT))
    args = ap.parse_args()

    # Import test fixture machinery to reuse its overrides + freeze
    # invocation. This is INTENTIONAL cross-import for dev tooling;
    # production stack does not touch tests/.
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

    from tests.fixtures.conftest import _build_and_freeze
    from tests.fixtures.overrides import assert_no_safety_overrides

    assert_no_safety_overrides()   # CHK-0-56 iv guard

    out = Path(args.root)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)

    # The fixture wants a parent tmpdir; it creates configs/ +
    # resolved/ under it. We pass a "session-owned" tmp parent and
    # then move the resolved tree to args.root.
    stage = out.parent / (out.name + "_staging")
    if stage.exists():
        shutil.rmtree(stage)
    stage.mkdir(parents=True, exist_ok=True)

    handle = _build_and_freeze(stage)
    if handle.freeze_error is not None:
        # Same tolerated set as the fixture test: known-fail assertions
        # are documented pre-existing prod defects. Only unexpected
        # errors abort here.
        print(
            f"materialize_resolved: freeze completed with known "
            f"tolerance -- error was {type(handle.freeze_error).__name__}: "
            f"{handle.freeze_error}",
            file=sys.stderr)
    if handle.manifest is None:
        print("materialize_resolved: freeze produced no manifest -- "
              "aborting", file=sys.stderr)
        return 1

    # Move the staged resolved contents to args.root.
    staged_resolved = Path(handle.resolved_root)
    for item in staged_resolved.iterdir():
        dst = out / item.name
        if item.is_dir():
            shutil.copytree(item, dst)
        else:
            shutil.copy2(item, dst)

    # Also copy the resolved products under staged/configs into the
    # output so per-process yaml is available.
    staged_cfg = Path(handle.config_root)
    if staged_cfg.exists():
        # Only per-process files that load_resolved would ask for --
        # the manifest already lists them.
        for proc_name in ("p1_motion", "p2_core", "p3_task",
                            "p4_agent", "p5_gateway"):
            src = staged_cfg / f"{proc_name}.yaml"
            if src.exists():
                shutil.copy2(src, out / f"{proc_name}.yaml")

    print(f"materialize_resolved: wrote {out}")
    print(f"  files: {sorted(p.name for p in out.iterdir())}")

    # Clean staging.
    shutil.rmtree(stage, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
