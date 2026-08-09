"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p1_p2_main_entry.py
Brief: p_processes tests -- p1 p2 main entry

Description:
p1_motion / p2_core __main__ entry-point tests.

Regression guard for the "systemd unit ExecStart points at a module
that does not exist" failure. Both p1_motion and p2_core have systemd
units under deploy/systemd/ that ExecStart=`python3 -m xbrain.pN_*`;
these tests verify the module targets actually exist and behave
sensibly on the config-not-there path.
"""


import os
import subprocess
import sys
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent


def _run_module(module: str, args, env=None):
    full = os.environ.copy()
    if env:
        full.update(env)
    full["PYTHONPATH"] = str(REPO) + os.pathsep + full.get("PYTHONPATH", "")
    return subprocess.run(
        [sys.executable, "-m", module] + args,
        env=full, capture_output=True, text=True, timeout=15)


# --- p2_core -------------------------------------------------------

def test_p2_core_module_help_exits_zero():
    r = _run_module("xbrain.p2_core", ["--help"])
    assert r.returncode == 0, r.stderr
    assert "xbrain.p2_core" in r.stdout


def test_p2_core_missing_config_exits_nonzero_with_help(tmp_path):
    """Regression: without resolved snapshot, exit non-zero AND the
    stderr message must name config-freeze so the operator knows
    what to fix (not a bare traceback)."""
    r = _run_module("xbrain.p2_core", ["--dry-run"])
    assert r.returncode != 0, r.stderr
    assert "config-freeze" in r.stderr


def test_p2_core_main_loop_max_ticks_exits_clean():
    from xbrain.p2_core.__main__ import main_loop
    rc = main_loop(max_ticks=2, tick_seconds=0.01)
    assert rc == 0


# --- p1_motion -----------------------------------------------------

def test_p1_motion_module_help_exits_zero():
    r = _run_module("xbrain.p1_motion", ["--help"])
    assert r.returncode == 0, r.stderr
    assert "xbrain.p1_motion" in r.stdout


def test_p1_motion_missing_config_exits_nonzero_with_help():
    r = _run_module("xbrain.p1_motion", ["--dry-run"])
    assert r.returncode != 0, r.stderr
    assert "config-freeze" in r.stderr


def test_p1_motion_main_loop_max_ticks_exits_clean():
    from xbrain.p1_motion.__main__ import main_loop
    rc = main_loop(max_ticks=2, tick_seconds=0.01)
    assert rc == 0


# --- systemd unit targets consistent ------------------------------

def test_systemd_units_target_actual_modules():
    """CFG-BT-3 units for p1_motion / p2_core reference python -m
    xbrain.pN_*. Catch drift here rather than at runtime."""
    for proc in ("p1-motion", "p2-core"):
        unit = REPO / "deploy" / "systemd" / ("xbrain-" + proc + ".service")
        src = unit.read_text()
        module = "xbrain." + proc.replace("-", "_")
        assert module in src, \
            "systemd unit %s no longer targets %s" % (unit.name, module)


# --- Skeleton discipline: p1 heartbeat must NOT claim to publish cmd_vel
# The whole point of the skeleton is that it publishes NOTHING; a stub
# that pretended to publish cmd_vel would let downstream trust p1
# output when there is no arbitrated factor / speed gate / perception
# overlay behind it.

def test_p1_heartbeat_log_states_no_cmd_vel():
    """Read the p1_motion __main__.py source; assert the heartbeat
    log format explicitly names 'no cmd_vel published' so an operator
    reading the log knows the skeleton is not moving anything."""
    src = (REPO / "xbrain" / "p1_motion" / "__main__.py").read_text()
    assert "no cmd_vel published" in src
