"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: aggregate.py
Brief: SEC checklist aggregator -- run checks, emit report, decide exit

Description:
Runs every executable file under scripts/sec/checks/ and rolls their
results into one JSON report. Each check is a self-contained script
that MUST print one JSON line to stdout of the shape:

  {"id": "SEC-11-04", "severity": "BLOCKING",
   "status": "PASS" | "FAIL" | "SKIPPED",
   "message": "human-readable",
   "detail": {...arbitrary...}}

status semantics:
  PASS    -- check ran and passed
  FAIL    -- check ran and failed; contributes to blocking iff BLOCKING
  SKIPPED -- check could not run (device absent, etc.); NEVER blocks;
             is a third state distinct from PASS to avoid the CLAUDE.md
             3.2 form 3 (self-catches) failure where "cannot run" gets
             counted as "passed"

exit code: 0 iff no BLOCKING check reported FAIL. WARNING FAILs and
SKIPPED are surfaced in the report but do not affect exit.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


def _run_one(check_path: Path) -> dict:
    """Run one check script; return its parsed JSON result. On any
    error (crash, malformed output), synthesize a FAIL/BLOCKING result
    so a broken check does not silently pass the checklist."""
    try:
        r = subprocess.run(
            ["bash", str(check_path)],
            capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return {
            "id": check_path.stem, "severity": "BLOCKING",
            "status": "FAIL",
            "message": "check timed out after 60s",
            "detail": {"kind": "check_timeout"},
        }
    lines = [l for l in r.stdout.splitlines() if l.strip()]
    if not lines:
        return {
            "id": check_path.stem, "severity": "BLOCKING",
            "status": "FAIL",
            "message": "check produced no output",
            "detail": {"kind": "no_output", "stderr": r.stderr[:1024]},
        }
    try:
        parsed = json.loads(lines[-1])
    except json.JSONDecodeError as exc:
        return {
            "id": check_path.stem, "severity": "BLOCKING",
            "status": "FAIL",
            "message": "check output not JSON: " + str(exc),
            "detail": {"kind": "bad_json", "stdout": lines[-1][:1024]},
        }
    # Every result must carry the four required fields.
    for k in ("id", "severity", "status", "message"):
        if k not in parsed:
            return {
                "id": check_path.stem, "severity": "BLOCKING",
                "status": "FAIL",
                "message": "check result missing key " + k,
                "detail": {"kind": "incomplete_result", "raw": parsed},
            }
    parsed.setdefault("detail", {})
    return parsed


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checks-dir", required=True)
    ap.add_argument("--out", required=True)
    args, extra = ap.parse_known_args()

    checks_dir = Path(args.checks_dir)
    if not checks_dir.is_dir():
        print("no checks dir: %s" % checks_dir, file=sys.stderr)
        return 2

    # Sort by filename so ordering is deterministic across machines.
    scripts = sorted(p for p in checks_dir.iterdir()
                     if p.is_file() and p.name.endswith(".sh"))

    results = [_run_one(s) for s in scripts]

    # Blocking iff any BLOCKING check failed.
    blocking_failed = any(
        r.get("severity") == "BLOCKING" and r.get("status") == "FAIL"
        for r in results)

    report = {
        "robot_id": os.environ.get("XBRAIN_ROBOT_ID", "unknown"),
        "timestamp": int(time.time()),
        "must_not_deliver": blocking_failed,
        "results": results,
        "summary": {
            "total": len(results),
            "pass": sum(1 for r in results if r["status"] == "PASS"),
            "fail": sum(1 for r in results if r["status"] == "FAIL"),
            "skipped": sum(1 for r in results if r["status"] == "SKIPPED"),
        },
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))

    if blocking_failed:
        print("MUST NOT DELIVER -- blocking SEC checks failed",
              file=sys.stderr)
        for r in results:
            if r["severity"] == "BLOCKING" and r["status"] == "FAIL":
                print("  %s: %s" % (r["id"], r["message"]), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
