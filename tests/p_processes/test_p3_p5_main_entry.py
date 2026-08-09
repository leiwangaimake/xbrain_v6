"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p3_p5_main_entry.py
Brief: p_processes tests -- p3 p5 main entry

Description:
p3_task / p5_gateway __main__ entry-point tests.

p5_gateway is the special case: it MUST NOT refuse to start on
missing config -- it enters minimal-mode observation window instead
(10 S3.3 W-1, INF-DP-8). All other P-processes exit non-zero.
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


# --- p3_task -------------------------------------------------------

def test_p3_task_module_help_exits_zero():
    r = _run_module("xbrain.p3_task", ["--help"])
    assert r.returncode == 0, r.stderr


def test_p3_task_missing_config_exits_nonzero():
    r = _run_module("xbrain.p3_task", ["--dry-run"])
    assert r.returncode != 0, r.stderr
    assert "config-freeze" in r.stderr


def test_p3_task_main_loop_max_ticks_exits_clean():
    from xbrain.p3_task.__main__ import main_loop
    rc = main_loop(max_ticks=2, tick_seconds=0.01)
    assert rc == 0


# --- p5_gateway (SPECIAL: minimal mode) ---------------------------

def test_p5_gateway_module_help_exits_zero():
    r = _run_module("xbrain.p5_gateway", ["--help"])
    assert r.returncode == 0, r.stderr


def test_p5_gateway_missing_config_enters_minimal_mode(tmp_path):
    """* CRITICAL: p5_gateway MUST NOT exit non-zero when config-freeze
    failed. It enters minimal mode. This is the observation window
    W-1 (10 S3.3). If future code regresses and adds `sys.exit(4)` on
    missing config, this test catches it."""
    r = _run_module("xbrain.p5_gateway", ["--dry-run"])
    # dry-run + no config = exit 0 in minimal mode.
    assert r.returncode == 0, r.stderr
    assert "MINIMAL" in r.stderr


def test_p5_gateway_force_minimal_mode_flag_works():
    """--force-minimal-mode + --dry-run: exit 0 and the "minimal_mode=True"
    literal appears in the log. Do NOT assert on "MINIMAL" alone --
    with force-minimal-mode we skip the config load path (where the
    "MINIMAL" warning is emitted), so the only reliable marker is
    the dry-run exit line which includes minimal_mode=True."""
    r = _run_module("xbrain.p5_gateway",
                    ["--force-minimal-mode", "--dry-run"])
    assert r.returncode == 0
    assert "minimal_mode=True" in r.stderr


def test_p5_gateway_minimal_mode_main_loop_logs_label():
    """Verify heartbeat log line distinguishes minimal from full mode."""
    import io
    import logging
    from xbrain.p5_gateway.__main__ import main_loop

    handler = logging.StreamHandler(io.StringIO())
    handler.setLevel(logging.DEBUG)
    logger = logging.getLogger("xbrain.p5_gateway")
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)

    try:
        rc = main_loop(minimal_mode=True, max_ticks=1, tick_seconds=0.01)
        assert rc == 0
        output = handler.stream.getvalue()
        assert "MINIMAL" in output
    finally:
        logger.removeHandler(handler)


# --- systemd unit targets ----------------------------------------

def test_systemd_units_target_p3_p5_modules():
    for proc, module in [("p3-task", "xbrain.p3_task"),
                         ("p5-gateway", "xbrain.p5_gateway")]:
        unit = REPO / "deploy" / "systemd" / ("xbrain-" + proc + ".service")
        src = unit.read_text()
        assert module in src, "unit %s no longer targets %s" % (unit.name, module)


# --- Design invariant: p5_gateway has NO Requires=config-freeze ---
# INF-DP-7 / CFG-BT-3 explicitly excluded p5_gateway from Requires=
# config-freeze so systemd itself will start it even after freeze
# failed. Regression guard here as well.

def test_p5_gateway_unit_has_no_requires_freeze():
    unit = REPO / "deploy" / "systemd" / "xbrain-p5-gateway.service"
    lines = [
        line for line in unit.read_text().splitlines()
        if not line.lstrip().startswith("#")
    ]
    body = "\n".join(lines)
    assert "Requires=xbrain-config-freeze.service" not in body, \
        "p5_gateway systemd unit MUST NOT Requires= freeze -- W-1"
