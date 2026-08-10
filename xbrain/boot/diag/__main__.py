"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __main__.py
Brief: CHK-2-63 -- support-bundle collector entry-point

Description:
Wires the pure functions in collect.py to real paths on the target
machine, invoked as `python -m xbrain.boot.diag`. Writes a tarball
under {data_root}/diag/xbrain-diag-{robot_id}-{ts}.tar.gz.

Time source note. CLAUDE.md 3.4 (single-clock rule for age/timeout)
does NOT apply to the bundle FILENAME's timestamp -- the filename is
about "when did I take this snapshot", which is exactly what a wall
clock is for. We use time.time() DELIBERATELY here; a monotonic clock
would give unusable names ("what does .1739103.tar.gz mean at
2:00am?").

The robot_id source. This module reads it from environment variable
XBRAIN_ROBOT_ID, or falls back to gethostname. Tests inject via env.
"""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

from xbrain.boot.diag import collect


_DEFAULT_CLAUDE_MD = "/opt/xbrain_v6/CLAUDE.md"
_DEFAULT_DATA_ROOT = "/opt/xbrain_v6/data"
_DEFAULT_RESOLVED = "/opt/xbrain_v6/data/run/resolved"


def _systemctl_snapshot() -> str:
    """Best-effort snapshot of xbrain-*.service unit states.

    On dev machines systemctl may not know about xbrain units at all;
    that is not a failure -- the collector still emits the tarball
    with an empty status.txt. The catch is deliberately broad because
    we do not want a bundle collection to fail on any external issue."""
    try:
        r = subprocess.run(
            ["systemctl", "status", "--no-pager", "xbrain-*.service"],
            capture_output=True, text=True, timeout=10)
        return r.stdout + ("\n---STDERR---\n" + r.stderr if r.stderr else "")
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as exc:
        return "systemctl snapshot unavailable: " + str(exc)


def main() -> int:
    ap = argparse.ArgumentParser(description="XBRAIN support-bundle collector")
    ap.add_argument("--claude-md", default=os.environ.get(
        "XBRAIN_CLAUDE_MD", _DEFAULT_CLAUDE_MD))
    ap.add_argument("--data-root", default=os.environ.get(
        "XBRAIN_DATA_ROOT", _DEFAULT_DATA_ROOT))
    ap.add_argument("--resolved-dir", default=os.environ.get(
        "XBRAIN_RESOLVED_DIR", _DEFAULT_RESOLVED))
    ap.add_argument("--log-tail-bytes", type=int,
                    default=int(os.environ.get(
                        "XBRAIN_DIAG_TAIL_BYTES", 1024 * 1024)))
    ap.add_argument("--max-bundle-bytes", type=int,
                    default=int(os.environ.get(
                        "XBRAIN_DIAG_MAX_BYTES", 128 * 1024 * 1024)))
    ap.add_argument("--out-dir", default=None,
                    help="output dir (defaults to {data_root}/diag)")
    args = ap.parse_args()

    robot_id = os.environ.get("XBRAIN_ROBOT_ID") or socket.gethostname()
    ts = int(time.time())  # WALL-CLOCK-OK(record): bundle filename timestamp -- when the snapshot was taken; a monotonic value would be unreadable
    bundle_name = "xbrain-diag-{}-{}.tar.gz".format(robot_id, ts)

    data_root = Path(args.data_root)
    out_dir = Path(args.out_dir) if args.out_dir else data_root / "diag"
    out_tarball = out_dir / bundle_name

    processes = collect.read_process_list(args.claude_md)
    if not processes:
        print("ERROR: no processes discovered in %s" % args.claude_md,
              file=sys.stderr)
        return 1

    # Optional inputs -- missing files are fine, they end up in
    # manifest.skipped (not a hard error).
    boot_fail = data_root / "boot_fail.jsonl"
    bit_result = data_root / "bit" / "last.json"
    # build_version lives inside the package on Orin, but its physical
    # path is the file the version script writes. Prefer that so the
    # bundle has the on-disk artefact rather than a re-import.
    build_version = Path("/opt/xbrain_v6/xbrain/common/version/_build.py")

    manifest = collect.assemble(
        out_tarball=out_tarball,
        processes=processes,
        data_root=data_root,
        resolved_dir=Path(args.resolved_dir),
        boot_fail_path=boot_fail if boot_fail.is_file() else None,
        bit_result_path=bit_result if bit_result.is_file() else None,
        build_version_path=build_version,
        log_tail_bytes=args.log_tail_bytes,
        max_bundle_bytes=args.max_bundle_bytes,
        systemctl_snapshot=_systemctl_snapshot(),
    )

    print("wrote %s" % out_tarball)
    if manifest["truncated"]:
        print("truncated %d file(s) due to size cap"
              % len(manifest["truncated"]), file=sys.stderr)
    if manifest["warnings"]:
        for w in manifest["warnings"]:
            print("WARN: " + w, file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
