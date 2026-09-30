"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_null_guard.py
Brief: common tests -- null guard

Description:
INF-DB-2 null_guard tests.
"""


import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.no_device


SCRIPT = Path(__file__).parent.parent.parent / "scripts" / "ci" / "null_guard.py"


def _run(*args):
    return subprocess.run([sys.executable, str(SCRIPT), *args],
                          capture_output=True, text=True, timeout=60)


def test_self_test_passes():
    r = _run("--self-test")
    assert r.returncode == 0, r.stdout


def test_the_repository_currently_passes():
    """Reality: 4 spec keys are null (V-01 open for those four), t_lat_s is
    0.4 (M-01 closed), max_vx_mps is 2.0 (user ruling 2026-09-10)."""
    r = _run()
    assert r.returncode == 0, r.stdout


#: The guarded set, spelled out rather than counted.
#
# Why the SET and not just the number: a count cannot tell "one key was
# dropped and another added" from "nothing changed", and the one thing this
# file has to notice is a safety key quietly leaving the guard. The count is
# still asserted below, from len() of this list, so the two cannot disagree.
#
# Why max_vx_mps is NOT here -- and why that is the correction rather than a
# hole. 21 V-01 was corrected on 2026-09-28 under CLAUDE.md iron rule 1,
# verbatim: "守卫已同步(受守键 9 -> 8)". The value 2.0 was landed by the user
# on 2026-09-10 on a three-fold basis (99 U54 ceiling + the vendor's "actual
# maximum 2 m/s" + the user's own confirmation), recorded at the value in
# configs/models/m20s.yaml. V-01 itself stays OPEN: the written spec and the
# bench cross-check are both unmet, so the other four spec keys stay guarded.
# Guarding max_vx too made the gate report a permanent red against a standing
# ruling, which is how a gate stops being read (CLAUDE.md 3.2 form 2).
_GUARDED_KEYS = (
    "common.spec.max_accel_mps2",   # V-01
    "common.spec.max_decel_mps2",   # V-01
    "common.spec.max_vy_mps",       # V-01
    "common.spec.max_wz_radps",     # V-01
    "ptz.k_ms_per_deg",             # M-PTZ-1
    "ptz.omega_pan",                # T-PTZ-3
    "ptz.omega_tilt",               # T-PTZ-3
    "ptz.preset_effective",         # T-PTZ-1
)


def test_scan_reports_the_guarded_key_set():
    r = _run("-v")
    listed = sorted(
        line.split("<-")[0].replace("guarded:", "").strip()
        for line in r.stdout.splitlines() if line.strip().startswith("guarded:")
    )
    assert listed == sorted(_GUARDED_KEYS), r.stdout
    assert "guards %d keys" % len(_GUARDED_KEYS) in r.stdout


def test_verbose_lists_debt_ids():
    r = _run("-v")
    assert "V-01" in r.stdout


def test_ptz_debts_are_guarded():
    """User 2026-08-09 correction: PTZ is NOT manual-only. AI cloud
    voice / text / local voice can control it via 18 intents.
    E01/E09 work; E02/E03/E04/E10 need real measurement first.
    Null-guard therefore covers the PTZ calibration keys so an
    operator cannot fill them prematurely (INF-DB-3 rejects intents
    but does not check config)."""
    r = _run("-v")
    lines = r.stdout.splitlines()
    ptz_lines = [ln for ln in lines if "guarded" in ln and "ptz" in ln.lower()]
    # Must have at least T-PTZ-1 preset_effective + T-PTZ-3 omega +
    # M-PTZ-1 k_ms_per_deg.
    debt_ids = {ln.split("<-")[-1].strip() for ln in ptz_lines}
    assert "T-PTZ-1" in debt_ids
    assert "T-PTZ-3" in debt_ids
    assert "M-PTZ-1" in debt_ids


def test_m01_closure_by_u54_recorded():
    """M-01 is in _CLOSED_DEBT_IDS (closed by U54 pinning t_lat_s)."""
    src = SCRIPT.read_text()
    assert "M-01" in src
    assert "U54" in src


def test_v01_still_open():
    """V-01 must NOT be in _CLOSED_DEBT_IDS (max_vx values pending vendor)."""
    src = SCRIPT.read_text()
    # V-01 appears in guarded list, not closed list.
    assert '"V-01": ' not in src, \
        "V-01 unexpectedly in _CLOSED_DEBT_IDS -- vendor did not commit yet"


def test_extra_keys_all_reference_debt(tmp_path):
    """Every _EXTRA_KEYS entry must cite a debt_id + reason (both non-empty)."""
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        import null_guard as ng
    finally:
        sys.path.pop(0)
    for key, (debt_id, reason) in ng._EXTRA_KEYS.items():
        assert debt_id, "_EXTRA_KEYS[%r] missing debt_id" % key
        assert reason and len(reason) >= 20, \
            "_EXTRA_KEYS[%r] reason too short: %r" % (key, reason)
