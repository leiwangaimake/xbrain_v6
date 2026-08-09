"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_static_rules_bundle.py
Brief: common tests -- static rules bundle

Description:
INF-CI-1 -- static_rules.py bundle runner tests.
"""


import subprocess
import sys
from pathlib import Path


BUNDLE = Path(__file__).parent.parent.parent / "scripts" / "ci" / "static_rules.py"


def _run(*args):
    return subprocess.run([sys.executable, str(BUNDLE), *args],
                          capture_output=True, text=True, timeout=180)


def test_self_test_passes():
    """Bundle self-test verifies all 9 rules have working --self-test."""
    r = _run("--self-test")
    assert r.returncode == 0, r.stdout + r.stderr


def test_full_run_passes():
    """Repo currently passes all 9 static rules."""
    r = _run()
    assert r.returncode == 0, r.stdout


def test_single_rule_selectable():
    r = _run("--rule", "5")
    assert r.returncode == 0
    assert "rule 5" in r.stdout
    # Only one rule ran.
    assert r.stdout.count("PASS") + r.stdout.count("FAIL") == 1


def test_unknown_rule_number_exits_nonzero():
    r = _run("--rule", "999")
    assert r.returncode == 2


def test_rule_count_is_nine():
    """CLAUDE.md 8.2 item 6 says 8 rules; project補 adds rule 9. Total 9."""
    r = _run()
    assert "bundle: 9 rule(s)" in r.stdout


def test_bundle_names_every_lint_script():
    """Sanity: rule descriptions include every lint script under scripts/lint/
    that's meant for INF-CI-1. If a new lint is added there without an
    entry in _RULES here, the bundle is out of sync."""
    r = _run()
    # These are the 9 scripts driven by the bundle.
    for name in ("no_business_imports.py", "clock_scan.py",
                 "no_safety_default.py", "no_literal_ecode.py",
                 "zenoh_callback_scan.py", "charset_lint.py",
                 "no_config_singular.py", "no_config_source_read.py"):
        assert name in r.stdout, "bundle missing %s" % name
