"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_start_stop_scripts.py
Brief: deploy tests -- start stop scripts

Description:
CFG-BT-7 -- start_all.sh / stop_all.sh / clean_pyc.sh sanity tests.
"""


import os
import subprocess
import sys
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO_ROOT = Path(__file__).parent.parent.parent
SCRIPTS = REPO_ROOT / "scripts"


def _script_text(name: str) -> str:
    return (SCRIPTS / name).read_text()


def test_start_all_exists_and_executable():
    p = SCRIPTS / "start_all.sh"
    assert p.is_file()
    assert os.access(p, os.X_OK), "start_all.sh not executable"


def test_stop_all_exists_and_executable():
    p = SCRIPTS / "stop_all.sh"
    assert p.is_file()
    assert os.access(p, os.X_OK)


def test_clean_pyc_exists_and_executable():
    p = SCRIPTS / "clean_pyc.sh"
    assert p.is_file()
    assert os.access(p, os.X_OK)


def test_start_all_has_set_e():
    """CFG-BT-7 variant ②: without set -euo pipefail, failing stages
    silently continue. Test asserts the flag is present."""
    src = _script_text("start_all.sh")
    assert "set -euo pipefail" in src


def test_clean_pyc_has_set_e():
    src = _script_text("clean_pyc.sh")
    assert "set -euo pipefail" in src


def test_start_all_derives_script_dir():
    """CLAUDE.md 6: no absolute path hard-coding; derive via SCRIPT_DIR."""
    src = _script_text("start_all.sh")
    assert "SCRIPT_DIR=" in src
    assert "BASH_SOURCE" in src


def test_stop_all_derives_script_dir():
    src = _script_text("stop_all.sh")
    assert "SCRIPT_DIR=" in src


def test_no_bare_var_rm_rf_in_any_script():
    """CFG-BT-7 variant ①: `rm -rf $VAR/` with empty $VAR is `rm -rf /`.
    Every rm -rf must name a concrete path, not a bare variable followed
    by a slash. Skips comment lines (which may legitimately DOCUMENT
    the anti-pattern by name)."""
    import re
    bad = re.compile(r'rm\s+-rf\s+\$[A-Za-z_][A-Za-z0-9_]*/')
    for name in ("start_all.sh", "stop_all.sh", "clean_pyc.sh"):
        src = _script_text(name)
        for lineno, line in enumerate(src.splitlines(), 1):
            stripped = line.lstrip()
            # A comment line is prose about the pattern, not the
            # pattern itself.
            if stripped.startswith("#"):
                continue
            m = bad.search(line)
            assert not m, \
                "%s:%d uses bare '$VAR/' rm -rf: %r" \
                % (name, lineno, m.group(0))


def test_clean_pyc_dry_run_lists_pyc():
    """clean_pyc.sh --DRY_RUN=1 must scan without deleting.

    Regression: an early version wrote find | rm without a DRY_RUN
    guard; this test locks the guard in.
    """
    env = dict(os.environ)
    env["DRY_RUN"] = "1"
    r = subprocess.run(
        ["bash", str(SCRIPTS / "clean_pyc.sh")],
        capture_output=True, text=True, env=env, timeout=30,
    )
    assert r.returncode == 0, r.stdout + r.stderr
    # Output should mention scanning; may or may not have files listed.
    assert "scanning" in r.stdout


def test_start_all_dry_run_prints_all_stages():
    """DRY_RUN=1 covers every stage header (0/0z-1/0z-2/0z-3/0c/1/2/3/4/5)."""
    env = dict(os.environ)
    env["DRY_RUN"] = "1"
    r = subprocess.run(
        ["bash", str(SCRIPTS / "start_all.sh")],
        capture_output=True, text=True, env=env, timeout=30,
    )
    assert r.returncode == 0, r.stdout + r.stderr
    for stage in ("[stage 0]", "[stage 0z-1]", "[stage 0z-2]",
                  "[stage 0z-3]", "[stage 0c]", "[stage 1]",
                  "[stage 2]", "[stage 3]", "[stage 4]", "[stage 5]"):
        assert stage in r.stdout, "missing %s in start_all output" % stage


def test_stop_all_dry_run_reverses_order():
    """stop_all.sh should tear down in reverse (5 -> ... -> 0)."""
    env = dict(os.environ)
    env["DRY_RUN"] = "1"
    r = subprocess.run(
        ["bash", str(SCRIPTS / "stop_all.sh")],
        capture_output=True, text=True, env=env, timeout=30,
    )
    assert r.returncode == 0
    # Find the positions of stage-5 and stage-1 headers; stage 5
    # must come first (higher stage torn down first).
    idx5 = r.stdout.find("[stage 5 down]")
    idx1 = r.stdout.find("[stage 1 down]")
    assert idx5 >= 0 and idx1 >= 0
    assert idx5 < idx1, "stop_all.sh did not iterate in reverse"


def test_clean_pyc_refuses_root_repo():
    """CFG-BT-7 sanity: script refuses to run if REPO_ROOT resolves
    to '/'. The bug it guards is a $VAR expansion accident that made
    the script rm -rf a system directory."""
    src = _script_text("clean_pyc.sh")
    assert 'refuse to run' in src, \
        "clean_pyc.sh missing the '/' refusal guard"
    assert '"$REPO_ROOT" == "/"' in src, \
        "clean_pyc.sh guard does not check for root '/'"
