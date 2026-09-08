"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_meta_clock_scan.py
Brief: A-CLK-1 / A-SCAN-1 meta-tests (P6 -- 20 S10.1/S13)

Description:
A-CLK-1: no rns/ module reads a wall clock (time.time / datetime.now / utcnow).
All ageing/timeout uses CLOCK_MONOTONIC (RNS-CLK-1). The repo-wide clock_scan.py
already covers xbrain/, but this focused meta-test names the rns/ surface so the
assertion has a home in the RNS suite and reddens on a planted wall-clock read.

A-SCAN-1: book 20 is a member of scan_manifest.json, so the SEC-12 doc checks
actually scan it. Deleting the member line would drop a doc from the scan surface
while the checks still PASS -- exactly the "scan surface not declared" failure
(CLAUDE.md 3.2 form 6). This test refuses that.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RNS = ROOT / "xbrain" / "p1_motion" / "rns"
MANIFEST = ROOT / "scripts" / "doccheck" / "scan_manifest.json"

# wall-clock reads forbidden in the module (RNS-CLK-1). Written by pattern, not
# by the literal call, so this file does not report itself (3.2 form 3).
_WALL_CLOCK = re.compile(r"time\.time\s*\(|datetime\.now\s*\(|datetime\.utcnow\s*\(")


def test_no_rns_module_reads_wall_clock_A_CLK_1():
    offenders = {}
    for f in RNS.glob("*.py"):
        hits = _WALL_CLOCK.findall(f.read_text(encoding="utf-8"))
        if hits:
            offenders[f.name] = hits
    assert not offenders, (
        "rns/ modules read a wall clock (RNS-CLK-1 forbids -- use "
        "time.monotonic): %s" % offenders)


def test_no_rns_test_or_module_uses_wall_clock_for_ageing():
    # a planted `time.time()` anywhere in rns/ reddens this (the A-CLK-1 mutant).
    # (the _legacy tree is excluded -- it is not the live module.)
    for f in RNS.glob("*.py"):
        text = f.read_text(encoding="utf-8")
        assert "time.time(" not in text, "%s uses time.time (A-CLK-1)" % f.name


def test_book20_is_in_scan_manifest_A_SCAN_1():
    # A-SCAN-1: deleting book 20's member line drops it from the scan surface
    # while checks still pass. This refuses that.
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    members = json.dumps(manifest)  # simplest robust membership check
    assert "20-RNS" in members, "book 20 not a scan_manifest member (A-SCAN-1)"
