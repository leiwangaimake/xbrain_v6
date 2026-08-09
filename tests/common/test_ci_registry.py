"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ci_registry.py
Brief: common tests -- ci registry

Description:
INF-CI-5 -- checks.yaml <-> run_all.sh bidirectional diff.
"""


import re
import subprocess
import sys
from pathlib import Path

import pytest
import yaml


CI_DIR = Path(__file__).parent.parent.parent / "scripts" / "ci"
CHECKS_YAML = CI_DIR / "checks.yaml"
RUN_ALL_SH = CI_DIR / "run_all.sh"


def _registry_names():
    with open(CHECKS_YAML) as fh:
        data = yaml.safe_load(fh)
    return [c["name"] for c in data["checks"]]


def _shell_invocation_names():
    """Extract check names from run_check invocations in run_all.sh."""
    src = RUN_ALL_SH.read_text()
    # Match: run_check NAME PATH
    return [m.group(1) for m in re.finditer(
        r"^\s*run_check\s+(\S+)\s+", src, re.MULTILINE)]


def test_registry_and_shell_agree_forward():
    """Every entry in checks.yaml appears in run_all.sh."""
    reg = set(_registry_names())
    sh = set(_shell_invocation_names())
    missing = reg - sh
    assert not missing, (
        "registry entries not invoked by run_all.sh: %s. "
        "Add a `run_check` line for each." % sorted(missing))


def test_registry_and_shell_agree_reverse():
    """Every run_check invocation in run_all.sh appears in the registry."""
    reg = set(_registry_names())
    sh = set(_shell_invocation_names())
    extra = sh - reg
    assert not extra, (
        "run_all.sh invokes checks not registered: %s. "
        "Add each to scripts/ci/checks.yaml." % sorted(extra))


def test_registry_has_reason_for_every_entry():
    """Each registry entry MUST carry a non-trivial reason."""
    with open(CHECKS_YAML) as fh:
        data = yaml.safe_load(fh)
    for c in data["checks"]:
        assert c.get("reason", "").strip(), \
            "check %r missing reason" % c["name"]
        assert len(c["reason"]) >= 20, \
            "check %r reason too short (< 20 chars): %r" % (c["name"], c["reason"])


def test_registry_paths_exist():
    """Each registry entry's path must exist on disk (or be the
    literal 'pytest' shell command)."""
    repo_root = CI_DIR.parent.parent
    with open(CHECKS_YAML) as fh:
        data = yaml.safe_load(fh)
    for c in data["checks"]:
        if c["path"] == "pytest":
            continue
        p = repo_root / c["path"]
        assert p.is_file(), "check %r path not found: %s" % (c["name"], p)


def test_run_all_has_set_e():
    """INF-CI-5 hard rule: run_all.sh MUST have 'set -euo pipefail'
    and MUST NOT use '|| true' anywhere (variant ①)."""
    src = RUN_ALL_SH.read_text()
    assert "set -euo pipefail" in src, "run_all.sh missing set -euo pipefail"
    # || true bypasses set -e; INF-CI-5 variant ① names it verbatim.
    # Allow || true only inside comment lines (# ...).
    for lineno, line in enumerate(src.splitlines(), 1):
        stripped = line.strip()
        if stripped.startswith("#"):
            continue
        assert "|| true" not in line, \
            "run_all.sh line %d uses '|| true' (INF-CI-5 variant 1 forbidden)" \
            % lineno


def test_run_all_derives_script_dir():
    """CLAUDE.md 6 requires derived paths, no absolute hardcoding."""
    src = RUN_ALL_SH.read_text()
    assert "SCRIPT_DIR=" in src
    assert "BASH_SOURCE" in src


def test_registry_yaml_parses():
    """Registry must be valid YAML with the expected shape."""
    with open(CHECKS_YAML) as fh:
        data = yaml.safe_load(fh)
    assert isinstance(data, dict)
    assert "checks" in data
    assert isinstance(data["checks"], list)
    assert len(data["checks"]) >= 5, "expected at least 5 checks registered"


def test_no_duplicate_check_names():
    names = _registry_names()
    assert len(names) == len(set(names)), \
        "duplicate check names in registry: %s" % names


# Variant test: dropping a registered check from run_all.sh must fire
# the forward diff. Do this via a temp copy so we do not touch the
# real file.
def test_variant_1_drop_shell_invocation_fires_forward(tmp_path):
    src = RUN_ALL_SH.read_text()
    # Remove one specific run_check line.
    modified = re.sub(
        r"^\s*run_check\s+layout_gate\s+.*\n", "",
        src, count=1, flags=re.MULTILINE)
    # Write to tmp; the meta assertion needs to work on any path so
    # simulate the check by parsing directly.
    fake_run_all = tmp_path / "run_all.sh"
    fake_run_all.write_text(modified)
    # Extract shell names from the modified file.
    names_after = [m.group(1) for m in re.finditer(
        r"^\s*run_check\s+(\S+)\s+", modified, re.MULTILINE)]
    reg = set(_registry_names())
    missing = reg - set(names_after)
    assert "layout_gate" in missing, \
        "variant ① did not fire: layout_gate not in missing"


def test_variant_3_new_ci_script_not_registered(tmp_path):
    """A new scripts/ci/*.py that's not in the registry should be
    detectable. This is a partial variant -- the full check requires
    the runner to enumerate scripts/ci/ and diff. Here we simulate
    the discovery step."""
    fake_script = tmp_path / "new_check.py"
    fake_script.write_text("print('hello')")
    # Simulate: is fake_script in registry? No.
    reg = set(_registry_names())
    # The variant fires when a scripts/ci/*.py exists but no entry
    # names its path. Since we cannot mutate the real ci dir mid-test
    # (would race with other tests), assert the invariant abstractly.
    fake_path = "scripts/ci/new_check.py"
    with open(CHECKS_YAML) as fh:
        paths = {c["path"] for c in yaml.safe_load(fh)["checks"]}
    assert fake_path not in paths, \
        "test scaffold assumed fake_path was not registered"
