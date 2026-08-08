"""CFG-CM-13 no_safety_default lint tests."""

import subprocess
import sys
from pathlib import Path


LINT = Path(__file__).parent.parent.parent / "scripts" / "lint" / "no_safety_default.py"


def _run(*args):
    return subprocess.run([sys.executable, str(LINT), *args],
                          capture_output=True, text=True)


def test_self_test_passes():
    """The lint's own mutation test proves it can go red on all 3 patterns."""
    r = _run("--self-test")
    assert r.returncode == 0, r.stdout + r.stderr


def test_the_repository_currently_passes():
    """Repo-wide scan yields zero violations."""
    r = _run()
    assert r.returncode == 0, r.stdout + r.stderr


def test_dataclass_default_is_caught(tmp_path):
    (tmp_path / "x.py").write_text(
        "from dataclasses import dataclass\n"
        "@dataclass\nclass C:\n"
        "    max_decel_mps2: float = 2.5\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 1
    assert "dataclass_default" in r.stdout


def test_get_default_is_caught(tmp_path):
    (tmp_path / "x.py").write_text(
        "def f(cfg):\n"
        "    return cfg.get('common.safety.t_lat_s', 0.4)\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 1
    assert "get_with_default" in r.stdout


def test_or_fallback_is_caught(tmp_path):
    (tmp_path / "x.py").write_text(
        "def f(cfg):\n"
        "    a = cfg.brake.a_mps2 or 2.5\n"
        "    return a\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 1
    assert "or_fallback" in r.stdout


def test_no_lint_marker_suppresses(tmp_path):
    (tmp_path / "x.py").write_text(
        "@dataclass\nclass C:\n"
        "    max_decel_mps2: float = 2.5  # NO-SAFETY-DEFAULT-LINT: test fixture\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 0
