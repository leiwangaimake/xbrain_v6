"""INF-DB-2 null_guard tests."""

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
    """Reality: 5 spec keys are null (V-01 open), t_lat_s is 0.4 (M-01 closed)."""
    r = _run()
    assert r.returncode == 0, r.stdout


def test_scan_reports_guarded_key_count():
    r = _run("-v")
    # 5 V-01 spec keys + 4 PTZ keys (preset_effective / omega_pan /
    # omega_tilt / k_ms_per_deg). M-01 closed by U54.
    assert "guards 9 keys" in r.stdout


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
