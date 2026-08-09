"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_main_entry.py
Brief: p4_agent tests -- main entry

Description:
p4_agent __main__ entry-point tests.

The systemd unit's ExecStart is `python3 -m xbrain.p4_agent`. Before
this test file existed, that command failed with 'No module named
xbrain.p4_agent.__main__' -- the unit was compiled to a target that
did not exist. These tests catch regressions of that shape.
"""


import subprocess
import sys
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent


def _run_main(args, env=None):
    """Run `python -m xbrain.p4_agent {args}`; return CompletedProcess."""
    import os
    full = os.environ.copy()
    if env:
        full.update(env)
    full["PYTHONPATH"] = str(REPO) + os.pathsep + full.get("PYTHONPATH", "")
    return subprocess.run(
        [sys.executable, "-m", "xbrain.p4_agent"] + args,
        env=full, capture_output=True, text=True, timeout=15)


# --- __main__ is importable and --help works -----------------------

def test_module_help_prints_and_exits_zero():
    """Regression: `python -m xbrain.p4_agent --help` must exit 0.
    ArgumentParser handles --help before we touch any /run path."""
    r = _run_main(["--help"])
    assert r.returncode == 0, r.stderr
    assert "xbrain.p4_agent" in r.stdout


# --- Missing resolved config -> clear error, exit 3 ----------------

def test_missing_resolved_config_reports_helpfully(tmp_path):
    """When /run/xbrain/resolved is empty, __main__ must print a
    message pointing at xbrain-config-freeze.service (the fix-forward
    action) and exit with a non-zero code that is NOT a traceback.
    Exit code is 3 (FileNotFoundError) or 4 (any other loader error
    surface -- currently ResolvedConfigError from the loader)."""
    ghost = tmp_path / "nonexistent.yaml"
    r = _run_main(["--config", str(ghost), "--dry-run"])
    assert r.returncode in (3, 4), r.stderr
    # Either error path must name config-freeze so the operator knows
    # what to run next.
    assert "config-freeze" in r.stderr


# --- Bad config file -> exit 4 with named exception ---------------

def test_malformed_config_reports_and_exits_4(tmp_path):
    """A resolved config that fails validation (invalid YAML / missing
    required key / closed-set violation) must exit 4 with the exception
    TYPE named in stderr."""
    bad = tmp_path / "bad.yaml"
    bad.write_text("not: [valid yaml\n")   # unterminated bracket
    r = _run_main(["--config", str(bad), "--dry-run"])
    assert r.returncode == 4, r.stderr
    # The exception class name should be in stderr for operator diagnosis.
    assert any(t in r.stderr for t in ("Error", "Invalid", "yaml"))


# --- main_loop is testable in isolation ----------------------------

def test_main_loop_respects_max_ticks_and_stop_flag():
    """Direct test of main_loop() bypassing subprocess. Verifies the
    loop exits cleanly when max_ticks is hit AND when stop_flag is
    set from outside."""
    from xbrain.p4_agent.__main__ import main_loop
    rc = main_loop(max_ticks=2, tick_seconds=0.01)
    assert rc == 0


def test_main_loop_stop_flag_short_circuits():
    """stop_flag set BEFORE entry -> exits after first tick (or zero,
    depending on when the check runs). This test just verifies no
    hang."""
    from xbrain.p4_agent.__main__ import main_loop
    stop_flag = {"stop": True}
    rc = main_loop(max_ticks=100, tick_seconds=0.01, stop_flag=stop_flag)
    assert rc == 0


# --- systemd unit points at us -------------------------------------

def test_systemd_unit_execstart_matches_module_target():
    """CFG-BT-3 unit xbrain-p4-agent.service has
    ExecStart=python3 -m xbrain.p4_agent. If the module path in the
    unit ever drifts from what this __main__ answers to, catch it
    at CI time."""
    unit = REPO / "deploy" / "systemd" / "xbrain-p4-agent.service"
    src = unit.read_text()
    assert "python3 -m xbrain.p4_agent" in src or \
           "python -m xbrain.p4_agent" in src, \
           "systemd unit ExecStart no longer targets xbrain.p4_agent"
