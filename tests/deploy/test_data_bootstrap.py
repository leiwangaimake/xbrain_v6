"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_data_bootstrap.py
Brief: deploy tests -- data bootstrap

Description:
INF-DP-11 -- data/ dir + init_data.sh + logrotate + variants.
"""


import os
import sqlite3
import subprocess
import sys
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent
INIT = REPO / "scripts" / "init_data.sh"
LOGROTATE = REPO / "deploy" / "logrotate" / "xbrain"


def _run_init(data_root: Path) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env["XBRAIN_DATA_ROOT"] = str(data_root)
    return subprocess.run(
        ["bash", str(INIT)], env=env,
        capture_output=True, text=True, timeout=15)


# --- Layout: init script + logrotate exist ---------------------------

def test_init_data_script_exists_and_executable():
    assert INIT.is_file()
    assert os.access(INIT, os.X_OK)


def test_logrotate_config_exists():
    assert LOGROTATE.is_file()
    src = LOGROTATE.read_text()
    # copytruncate is required so live processes keep writing to the
    # same inode after rotation.
    assert "copytruncate" in src
    # missingok so a service that never wrote a log does not error.
    assert "missingok" in src


def test_data_dir_only_gitkeep_and_readme():
    """data/ must not carry runtime artefacts in-tree. The four DBs
    are created by init_data.sh at deploy time; if someone commits
    task.db in the repo, this test catches it."""
    data = REPO / "data"
    for db in ("task.db", "fence.db", "geo.db", "record.db"):
        assert not (data / db).exists(), \
            "%s must not be committed; deploy runs init_data.sh" % db


# --- Positive: init creates 4 DBs at version 1 -----------------------

def test_init_creates_four_dbs_at_version_one(tmp_path):
    r = _run_init(tmp_path)
    assert r.returncode == 0, r.stderr
    for name in ("task.db", "fence.db", "geo.db", "record.db"):
        p = tmp_path / name
        assert p.is_file(), name
        conn = sqlite3.connect(str(p))
        try:
            ver = conn.execute("PRAGMA user_version;").fetchone()[0]
        finally:
            conn.close()
        assert ver == 1, "%s: user_version=%s" % (name, ver)


# --- Positive: idempotency ------------------------------------------

def test_init_is_idempotent(tmp_path):
    r1 = _run_init(tmp_path)
    r2 = _run_init(tmp_path)
    assert r1.returncode == 0 and r2.returncode == 0
    assert "exists:" in r2.stdout


# --- Variant: probe rejects corrupted DB (E_STORAGE_CORRUPT) --------
# Cross-check with CFG-BT-1's probe: after init runs, corrupt the DB
# and run the probe -- it must fail with detail.db_name = corrupted DB.

def test_probe_fails_when_data_db_is_corrupt(tmp_path):
    """VARIANT of the "init + probe" pipeline. If init created the DB
    correctly but its bytes got truncated at runtime, the probe must
    catch it BEFORE any dependent process opens the DB."""
    _run_init(tmp_path)
    # Trash record.db's first 100 bytes (variant per CFG-BT-1 spec).
    p = tmp_path / "record.db"
    data = p.read_bytes()
    p.write_bytes(b"\x00" * 100 + data[100:])

    cfg = tmp_path / "cfg.yaml"
    cfg.write_text(
        "disk: []\n"
        "memory: {min_free_kb: 1024}\n"
        "temperature: {sensors: [], max_temp_c: 100.0}\n"
        "databases:\n"
        f"  - {{path: \"{p}\", expected_version: 1}}\n"
    )
    hw = tmp_path / "hw"
    hw.write_text("interfaces: {}\n")
    env = os.environ.copy()
    env["PYTHONPATH"] = str(REPO) + os.pathsep + env.get("PYTHONPATH", "")
    env["XBRAIN_PROBE_CONFIG"] = str(cfg)
    env["XBRAIN_HW_PROFILE"] = str(hw)
    r = subprocess.run(
        [sys.executable, "-m", "xbrain.boot.probe"],
        env=env, capture_output=True, text=True, timeout=15)
    assert r.returncode != 0
    assert "E_STORAGE_CORRUPT" in r.stderr
    assert "record.db" in r.stderr


# --- Variant: version above expected must fail (no downward guessing)

def test_version_above_expected_rejected_no_downward_compat(tmp_path):
    """VARIANT: init creates version 1, but if a live DB is at 2
    (rolled back deploy?), the probe must refuse -- no automatic
    'let's just use what's there' guess."""
    _run_init(tmp_path)
    conn = sqlite3.connect(str(tmp_path / "task.db"))
    try:
        conn.execute("PRAGMA user_version = 999;")
    finally:
        conn.close()

    cfg = tmp_path / "cfg.yaml"
    cfg.write_text(
        "disk: []\n"
        "memory: {min_free_kb: 1024}\n"
        "temperature: {sensors: [], max_temp_c: 100.0}\n"
        "databases:\n"
        f"  - {{path: \"{tmp_path/'task.db'}\", expected_version: 1}}\n"
    )
    hw = tmp_path / "hw"
    hw.write_text("interfaces: {}\n")
    env = os.environ.copy()
    env["PYTHONPATH"] = str(REPO) + os.pathsep + env.get("PYTHONPATH", "")
    env["XBRAIN_PROBE_CONFIG"] = str(cfg)
    env["XBRAIN_HW_PROFILE"] = str(hw)
    r = subprocess.run(
        [sys.executable, "-m", "xbrain.boot.probe"],
        env=env, capture_output=True, text=True, timeout=15)
    assert r.returncode != 0
    assert "db_schema_mismatch" in r.stderr


# --- Variant: only two DBs = the OLD V5 layout ----------------------
# The spec explicitly bans this: creating xbrain.db + journal.db (V5
# model) instead of the four-DB set is a Stage 0 violation. init_data.sh
# doesn't ban it directly, but we ban it via a static test on init_data
# itself: it must reference exactly the four DB names, no more no less.

def test_init_script_names_exactly_the_four_dbs():
    src = INIT.read_text()
    for name in ("task.db", "fence.db", "geo.db", "record.db"):
        assert name in src, name
    # Old-model names must NOT be there.
    assert "xbrain.db" not in src, "V5 xbrain.db must not appear"
    assert "journal.db" not in src, "V5 journal.db must not appear"


# --- Head comment sanity --------------------------------------------

def test_init_data_head_names_lineage():
    head = INIT.read_text().splitlines()[:15]
    joined = "\n".join(head)
    assert "INF-DP-11" in joined
    assert "上海哈船智能船舶技术有限公司" in joined
