"""CFG-BT-1 -- xbrain-probe (Stage 0 + GATE-6) tests + variants.

Every negative assertion is paired with a mutation (a variant that
must go red) per CLAUDE.md 3.3. Without the mutation, a "return None"
stub for any check would pass the test as-is.
"""

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest
import yaml


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent


# --- Small helper to run the probe as a subprocess -------------------

def _run_probe(env: dict) -> subprocess.CompletedProcess:
    """Invoke `python -m xbrain.boot.probe` with the given env layered
    on top. Returns the CompletedProcess so tests can assert on
    returncode and stderr JSON lines."""
    full_env = os.environ.copy()
    full_env.update(env)
    # Ensure repo root is on PYTHONPATH so `python -m xbrain.boot.probe`
    # resolves in test envs that haven't `pip install -e`'d the package.
    full_env["PYTHONPATH"] = str(REPO) + os.pathsep + full_env.get("PYTHONPATH", "")
    return subprocess.run(
        [sys.executable, "-m", "xbrain.boot.probe"],
        env=full_env, capture_output=True, text=True, timeout=30)


def _write_yaml(path: Path, data: dict) -> None:
    path.write_text(yaml.safe_dump(data))


def _emit_lines(stderr: str) -> list:
    """Parse stderr into list of dicts, one per JSON line."""
    out = []
    for line in stderr.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return out


# --- Fixture: minimal-happy-path config -----------------------------

@pytest.fixture()
def happy_env(tmp_path):
    """Set up a probe config + hw_profile that the probe accepts on
    the current machine. Uses a deliberately loose disk / mem / temp
    to avoid flakes on developer machines."""
    cfg_path = tmp_path / "thresholds.yaml"
    _write_yaml(cfg_path, {
        "disk": [{"path": str(tmp_path), "threshold_pct": 99.9}],
        "memory": {"min_free_kb": 1024},        # 1 MiB floor
        "temperature": {
            "sensors": ["/nonexistent/thermal_zone/temp"],
            "max_temp_c": 200.0,
        },
        "databases": [],   # empty list is valid; no DBs to check
    })
    hw_path = tmp_path / "hw_profile"
    _write_yaml(hw_path, {"interfaces": {}})
    return {
        "XBRAIN_PROBE_CONFIG": str(cfg_path),
        "XBRAIN_HW_PROFILE": str(hw_path),
    }


# --- Positive: probe passes on a benign env ------------------------

def test_probe_exits_zero_on_valid_env(happy_env):
    """POSITIVE assertion. Without this a "always exit 1" stub would
    only fail the negative tests below; a "always exit 0" stub would
    fail this one."""
    r = _run_probe(happy_env)
    assert r.returncode == 0, r.stderr


# --- Negative: missing config file ----------------------------------

def test_probe_fails_on_missing_config(happy_env, tmp_path):
    env = dict(happy_env)
    env["XBRAIN_PROBE_CONFIG"] = str(tmp_path / "nope.yaml")
    r = _run_probe(env)
    assert r.returncode != 0
    lines = _emit_lines(r.stderr)
    assert lines and lines[0]["code"] == "E_CONFIG_INVALID"
    assert lines[0]["detail"]["kind"] == "probe_config_invalid"


# --- Negative: config missing required key --------------------------

def test_probe_fails_on_incomplete_config(happy_env, tmp_path):
    """Variant of CLAUDE.md 3.1: absent safety key must crash, not
    default. Mutation: if a default value were introduced, this test
    would go green (probe would run through)."""
    cfg = tmp_path / "half.yaml"
    _write_yaml(cfg, {"disk": [], "memory": {"min_free_kb": 1}})
    env = dict(happy_env)
    env["XBRAIN_PROBE_CONFIG"] = str(cfg)
    r = _run_probe(env)
    assert r.returncode != 0
    lines = _emit_lines(r.stderr)
    assert any("temperature" in json.dumps(l) or "databases" in json.dumps(l)
               for l in lines), r.stderr


# --- Negative + variant: disk full ---------------------------------

def test_probe_reports_disk_full_when_threshold_exceeded(happy_env, tmp_path):
    """Set threshold_pct=0 so any non-empty filesystem trips it."""
    cfg = tmp_path / "diskfull.yaml"
    _write_yaml(cfg, {
        "disk": [{"path": str(tmp_path), "threshold_pct": 0.0}],
        "memory": {"min_free_kb": 1024},
        "temperature": {"sensors": [], "max_temp_c": 100.0},
        "databases": [],
    })
    hw = tmp_path / "hw"; _write_yaml(hw, {"interfaces": {}})
    r = _run_probe({
        "XBRAIN_PROBE_CONFIG": str(cfg),
        "XBRAIN_HW_PROFILE": str(hw),
    })
    assert r.returncode != 0
    lines = _emit_lines(r.stderr)
    assert any(l["code"] == "E_CONFIG_INVALID"
               and l["detail"]["kind"] == "disk_full" for l in lines), r.stderr


# --- DB corruption: E_STORAGE_CORRUPT + detail.db_name -------------

def test_db_corruption_reports_storage_corrupt_with_db_name(happy_env, tmp_path):
    """21-01 addition: E_STORAGE_CORRUPT must fire (NOT E_CONFIG_INVALID)
    so operators do not go hunt for a yaml bug when the DB is bad."""
    bad_db = tmp_path / "task.db"
    bad_db.write_bytes(b"this is not a sqlite file at all")
    cfg = tmp_path / "cfg.yaml"
    _write_yaml(cfg, {
        "disk": [], "memory": {"min_free_kb": 1024},
        "temperature": {"sensors": [], "max_temp_c": 100.0},
        "databases": [{"path": str(bad_db), "expected_version": 1}],
    })
    hw = tmp_path / "hw"; _write_yaml(hw, {"interfaces": {}})
    r = _run_probe({
        "XBRAIN_PROBE_CONFIG": str(cfg),
        "XBRAIN_HW_PROFILE": str(hw),
    })
    assert r.returncode != 0
    lines = _emit_lines(r.stderr)
    corrupt = [l for l in lines if l["code"] == "E_STORAGE_CORRUPT"]
    assert corrupt, "expected E_STORAGE_CORRUPT, got: %s" % r.stderr
    assert corrupt[0]["detail"]["db_name"] == "task.db"


def test_db_schema_mismatch_reports_config_invalid_not_storage_corrupt(
        happy_env, tmp_path):
    """VARIANT: mismatch is a config problem, not corruption. A stub
    that mapped both to E_STORAGE_CORRUPT would fail this test."""
    import sqlite3
    good = tmp_path / "task.db"
    conn = sqlite3.connect(str(good))
    conn.execute("PRAGMA user_version = 99")
    conn.close()
    cfg = tmp_path / "cfg.yaml"
    _write_yaml(cfg, {
        "disk": [], "memory": {"min_free_kb": 1024},
        "temperature": {"sensors": [], "max_temp_c": 100.0},
        "databases": [{"path": str(good), "expected_version": 1}],
    })
    hw = tmp_path / "hw"; _write_yaml(hw, {"interfaces": {}})
    r = _run_probe({
        "XBRAIN_PROBE_CONFIG": str(cfg),
        "XBRAIN_HW_PROFILE": str(hw),
    })
    assert r.returncode != 0
    lines = _emit_lines(r.stderr)
    assert any(l["code"] == "E_CONFIG_INVALID"
               and l["detail"]["kind"] == "db_schema_mismatch"
               for l in lines), r.stderr
    assert not any(l["code"] == "E_STORAGE_CORRUPT" for l in lines)


# --- GATE-6: iface missing ------------------------------------------

def test_gate6_missing_iface_reports_net_profile_mismatch(happy_env, tmp_path):
    """Use a made-up interface name that cannot exist ('zz99')."""
    hw = tmp_path / "hw"
    _write_yaml(hw, {
        "interfaces": {
            "zz99_missing": {
                "ipv4": "10.99.99.1", "netmask": "255.255.255.0",
                "network": "10.99.99.0/24",
            }
        }
    })
    env = dict(happy_env)
    env["XBRAIN_HW_PROFILE"] = str(hw)
    r = _run_probe(env)
    assert r.returncode != 0
    lines = _emit_lines(r.stderr)
    mism = [l for l in lines
            if l["detail"].get("kind") == "net_profile_mismatch"
            and l["detail"].get("interface") == "zz99_missing"]
    assert mism, r.stderr
    # Per-port expected vs actual must be present.
    assert mism[0]["detail"]["expected"]["ipv4"] == "10.99.99.1"
    assert mism[0]["detail"]["actual"]["ipv4"] is None


# --- NET-C1: two profile ifaces share a segment ---------------------

def test_gate6_profile_overlap_fires_before_actual_diff(happy_env, tmp_path):
    """VARIANT vs "always run actual diff": if the profile itself
    overlaps, the actual-diff must be skipped (both errors would
    otherwise blur the operator's diagnosis)."""
    hw = tmp_path / "hw"
    _write_yaml(hw, {
        "interfaces": {
            "z1": {"ipv4": "10.1.1.1", "netmask": "255.255.0.0",
                   "network": "10.1.0.0/16"},
            "z2": {"ipv4": "10.1.2.1", "netmask": "255.255.255.0",
                   "network": "10.1.2.0/24"},
        }
    })
    env = dict(happy_env)
    env["XBRAIN_HW_PROFILE"] = str(hw)
    r = _run_probe(env)
    assert r.returncode != 0
    lines = _emit_lines(r.stderr)
    over = [l for l in lines
            if l["detail"].get("kind") == "net_profile_overlap"]
    assert over, r.stderr
    # No per-iface diff should have been emitted (overlap short-circuits).
    diff = [l for l in lines
            if l["detail"].get("kind") == "net_profile_mismatch"]
    assert not diff, "overlap should suppress iface diff; got %s" % diff


# --- Unit file exists and points at us ------------------------------

def test_probe_service_unit_exists():
    p = REPO / "deploy" / "systemd" / "xbrain-probe.service"
    assert p.is_file()
    src = p.read_text()
    assert "Type=oneshot" in src
    assert "xbrain.boot.probe" in src
    # Downstream unit ordering: Before= config-freeze and both routers.
    assert "Before=" in src
    assert "config-freeze" in src


def test_probe_unit_has_documentation_link():
    """Reg-guard: the head comment must mention CFG-BT-1 so future
    hands know the lineage."""
    p = REPO / "deploy" / "systemd" / "xbrain-probe.service"
    assert "CFG-BT-1" in p.read_text().splitlines()[0]


# --- hw_profile templates present -----------------------------------

def test_hw_profile_templates_present():
    """DEC-15 lineage note: probe needs DBG + PROD templates so a
    deploy can copy one to /etc/xbrain/hw_profile. Templates must be
    all-null per CLAUDE.md 3.1 -- do NOT ship pre-filled IPs."""
    for name in ("DBG.yaml.template", "PROD.yaml.template"):
        p = REPO / "configs" / "hw_profile" / name
        assert p.is_file(), name
        doc = yaml.safe_load(p.read_text())
        for iface_name, spec in doc["interfaces"].items():
            assert spec["ipv4"] is None, \
                "%s.%s.ipv4 must be null (CLAUDE.md 3.1)" % (name, iface_name)
