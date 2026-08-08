"""CFG-DC-3 no_business_imports tests."""

import subprocess
import sys
from pathlib import Path


LINT = Path(__file__).parent.parent.parent / "scripts" / "lint" / "no_business_imports.py"


def _run(*args):
    return subprocess.run([sys.executable, str(LINT), *args],
                          capture_output=True, text=True)


def test_self_test_passes():
    r = _run("--self-test")
    assert r.returncode == 0


def test_the_repository_currently_passes():
    r = _run()
    assert r.returncode == 0, r.stdout


def test_rclpy_caught(tmp_path):
    (tmp_path / "x.py").write_text("import rclpy\n")
    r = _run(str(tmp_path))
    assert r.returncode == 1
    assert "rclpy" in r.stdout


def test_sqlite3_caught(tmp_path):
    (tmp_path / "x.py").write_text("import sqlite3\nimport aiosqlite\n")
    r = _run(str(tmp_path))
    assert r.returncode == 1
    assert "sqlite3" in r.stdout


def test_requests_caught(tmp_path):
    (tmp_path / "x.py").write_text("from requests import get\n")
    r = _run(str(tmp_path))
    assert r.returncode == 1
    assert "requests" in r.stdout


def test_exempt_marker_suppresses(tmp_path):
    (tmp_path / "x.py").write_text(
        "from requests import get  # BUSINESS-IMPORT-OK(ai-client)\n")
    r = _run(str(tmp_path))
    assert r.returncode == 0


def test_from_import_variant_caught(tmp_path):
    (tmp_path / "x.py").write_text("from rclpy.node import Node\n")
    r = _run(str(tmp_path))
    assert r.returncode == 1
