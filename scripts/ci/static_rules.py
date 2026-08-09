#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: static_rules.py
Brief: INF-CI-1 -- CLAUDE.md 8.2 item 6 static-rule bundle runner

Description:
CLAUDE.md 8.2 item 6 declares 8+1 self-defined static rules. Each is
individually landed as its own script under scripts/lint/. This is
the BUNDLED runner that iterates them, so CI can invoke one command
instead of nine and get a uniform pass/fail report.

The registry below binds each rule number to its (lint script,
one-line human meaning). The lint scripts themselves own the actual
checking; this file is a thin dispatch layer. Adding a new rule
means (1) writing the lint under scripts/lint/, (2) adding a row
here, (3) test_static_rules_bundle picks it up automatically.

Rule set (CLAUDE.md 8.2 item 6 verbatim):
  1. xbrain/**/*.py: no import rclpy/sqlite3/requests
  2. any *.py: no time.time()/datetime.now() for age/timeout
  3. any *.cc/*.h: no system_clock/CLOCK_REALTIME/naked rclcpp::Clock()
  4. safety param defaults: 3 patterns (dataclass/dict.get/or)
  5. E_* literals not from common/errors/
  6. declare_subscriber( callback discipline (async guards)
  7. source comments/prints: no emoji or Chinese punctuation
  8. no singular 'config/' path in any source
  9. no 'read configs/ source at runtime' fallback branch

Usage:
  python3 scripts/ci/static_rules.py
  python3 scripts/ci/static_rules.py --rule 5   # run only rule 5
  python3 scripts/ci/static_rules.py --self-test
"""

import argparse
import subprocess
import sys
from pathlib import Path


# The rule registry. Each row: (rule_num, script_name, one_liner).
# Rules 2 and 3 both fire from clock_scan.py (Python + C++ side); the
# script differentiates internally.
_RULES = [
    (1, "no_business_imports.py",
     "xbrain/**/*.py must not import rclpy/sqlite3/requests"),
    (2, "clock_scan.py",
     "no time.time()/datetime.now() for age/timeout in Python"),
    (3, "clock_scan.py",
     "no system_clock/CLOCK_REALTIME/naked rclcpp::Clock() in C++"),
    (4, "no_safety_default.py",
     "safety params must not carry code-side defaults"),
    (5, "no_literal_ecode.py",
     "E_* string literals must come from xbrain/common/errors"),
    (6, "zenoh_callback_scan.py",
     "zenoh subscriber callbacks must not do create_task/put/await/publish"),
    (7, "charset_lint.py",
     "no emoji or Chinese punctuation in source comments/messages"),
    (8, "no_config_singular.py",  # NO-CONFIG-SINGULAR-LINT
     "no singular 'config/*.yaml' path in any source"),  # NO-CONFIG-SINGULAR-LINT
    (9, "no_config_source_read.py",
     "no runtime read of configs/ source; use /run/xbrain/resolved"),
]


LINT_DIR = Path(__file__).parent.parent / "lint"


def _run_rule(rule_num: int, script: str) -> int:
    """Run one rule's lint script; return its exit code."""
    path = LINT_DIR / script
    if not path.is_file():
        print("[rule %d] MISSING %s" % (rule_num, path))
        return 1
    r = subprocess.run([sys.executable, str(path)],
                       capture_output=True, text=True, timeout=120)
    return r.returncode


def _self_test() -> int:
    """Verify every registered rule's script has --self-test that
    exits 0. Also verify the rule count is exactly 9 (the CLAUDE.md
    8.2 item 6 count + 补 rule 9)."""
    if len(_RULES) != 9:
        print("self-test FAIL: expected 9 rules, got %d" % len(_RULES))
        return 1
    failed = []
    for rule_num, script, _ in _RULES:
        path = LINT_DIR / script
        if not path.is_file():
            failed.append((rule_num, script, "missing script"))
            continue
        r = subprocess.run([sys.executable, str(path), "--self-test"],
                           capture_output=True, text=True, timeout=60)
        if r.returncode != 0:
            failed.append((rule_num, script,
                           "self-test exit %d" % r.returncode))
    if failed:
        print("self-test FAIL:")
        for rn, script, why in failed:
            print("  rule %d %s: %s" % (rn, script, why))
        return 1
    print("self-test PASS: %d rules, all --self-test exit 0"
          % len(_RULES))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--rule", type=int,
                    help="only run rule N (1..9)")
    args = ap.parse_args()
    if args.self_test:
        return _self_test()
    if args.rule is not None:
        matching = [(n, s, m) for n, s, m in _RULES if n == args.rule]
        if not matching:
            print("no rule numbered %d" % args.rule)
            return 2
        rules = matching
    else:
        rules = _RULES
    print("bundle: %d rule(s)" % len(rules))
    fails = 0
    for rule_num, script, meaning in rules:
        rc = _run_rule(rule_num, script)
        status = "PASS" if rc == 0 else "FAIL"
        print("  rule %d %s [%s]  %s" % (rule_num, status, script, meaning))
        if rc != 0:
            fails += 1
    print("  fails: %d / %d" % (fails, len(rules)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
