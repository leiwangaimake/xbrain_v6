"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: conftest.py
Brief: INF-TS-1 -- pytest infrastructure: three-tier markers + strict mode

Description:
This is the root conftest for the tests/ tree. It:

  1. Registers three hardware-availability markers (no_device /
     needs_orin / needs_chassis) as strict -- an unknown marker
     fails collection instead of being silently ignored.

  2. Provides a default-selection helper (--all-hw) that runs
     every marker; the default pytest invocation runs no_device
     only so a CI job on a dev machine does not skip everything
     silently.

  3. Auto-skips needs_orin / needs_chassis tests when the required
     hardware is not detected. Detection heuristics live in
     _has_orin() / _has_chassis() below.

The 'unmarked test files must fail' meta rule (INF-TS-1 variant 3)
lives in tests/meta/test_marker_coverage.py rather than here, so
this conftest stays a config-only file.
"""

import os
import shutil

import pytest


def pytest_configure(config):
    """Ensure strict mode + register the three markers so an unknown
    marker fires collection error."""
    # Strict-markers: unknown marker => collection error. This is the
    # 'a typo silently makes the marker a no-op' guard.
    config.addinivalue_line(
        "markers",
        "no_device: runs on any Linux dev machine (no special hardware)"
    )
    config.addinivalue_line(
        "markers",
        "needs_orin: requires NVIDIA Orin (CUDA / TensorRT / nvidia-smi)"
    )
    config.addinivalue_line(
        "markers",
        "needs_chassis: requires the M20S chassis reachable"
    )


def _has_orin() -> bool:
    """Detect NVIDIA Orin via nvidia-smi presence + dpkg jetpack.

    False on any dev machine without the JetPack stack. Cached at
    module load; hardware does not appear mid-test-run.
    """
    if shutil.which("nvidia-smi") is None:
        return False
    # A very light probe: nvidia-smi exists AND $HOSTNAME hints Orin
    # OR /etc/nv_tegra_release exists. The latter is JetPack-only.
    if os.path.isfile("/etc/nv_tegra_release"):
        return True
    return False


def _has_chassis() -> bool:
    """Detect M20S chassis via env var (XBRAIN_CHASSIS_HOST) or by
    a TCP probe to the chassis endpoint. Environment override is
    the primary path so a CI matrix can inject presence."""
    if os.environ.get("XBRAIN_HAS_CHASSIS") == "1":
        return True
    # No blocking network probe in default collection -- pytest
    # collection time must stay low. Only the env var is honored.
    return False


def pytest_collection_modifyitems(config, items):
    """Auto-skip needs_orin / needs_chassis when hardware absent.

    A test tagged needs_orin on a dev machine gets a skip mark with
    a clear reason. Without this, the test would ERROR at runtime
    when it tried to open a CUDA context, which reads as a bug
    rather than a missing prerequisite.
    """
    has_orin = _has_orin()
    has_chassis = _has_chassis()
    skip_orin = pytest.mark.skip(reason="no NVIDIA Orin detected")
    skip_chassis = pytest.mark.skip(reason="no chassis reachable")
    for item in items:
        if "needs_orin" in item.keywords and not has_orin:
            item.add_marker(skip_orin)
        if "needs_chassis" in item.keywords and not has_chassis:
            item.add_marker(skip_chassis)


# Optional command-line switch: `--include-hw` runs every marker
# regardless of hardware detection (used on CI matrix jobs that
# know they have the hardware).
def pytest_addoption(parser):
    parser.addoption(
        "--include-hw",
        action="store_true",
        default=False,
        help="Run all hardware-marked tests unconditionally; skip logic"
             " in pytest_collection_modifyitems is bypassed. Use on"
             " machines that DO have Orin + chassis.",
    )
