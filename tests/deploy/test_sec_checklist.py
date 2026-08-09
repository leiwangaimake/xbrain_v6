"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_sec_checklist.py
Brief: deploy tests -- sec checklist

Description:
CFG-BT-21 / INF-DB-5 -- SEC-1..SEC-12 checklist + severity + variants.
"""


import json
import os
import re
import subprocess
import tempfile
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent
CHECKS_DIR = REPO / "scripts" / "sec" / "checks"
RUN_SEC = REPO / "scripts" / "sec" / "run_sec.sh"


# --- Every check script exists and is executable --------------------

EXPECTED_SEC_IDS = [
    "SEC-01", "SEC-02", "SEC-03", "SEC-04", "SEC-05", "SEC-06",
    "SEC-07", "SEC-08", "SEC-09", "SEC-10", "SEC-11", "SEC-12",
]


@pytest.mark.parametrize("sec_id", EXPECTED_SEC_IDS)
def test_sec_check_script_exists(sec_id):
    """Each of SEC-01..SEC-12 has a script under scripts/sec/checks/.
    Filename convention: <SEC-ID>-<short-name>.sh."""
    hits = [p for p in CHECKS_DIR.iterdir()
            if p.is_file() and p.name.startswith(sec_id + "-") and p.suffix == ".sh"]
    assert len(hits) == 1, "expected exactly one script for %s, got %s" % (sec_id, hits)
    assert os.access(hits[0], os.X_OK)


# --- Meta: every check emits well-formed JSON on stdout -------------

@pytest.mark.parametrize("sec_id", EXPECTED_SEC_IDS)
def test_sec_check_emits_valid_json(sec_id, tmp_path):
    """Meta-check: aggregator counts on each script emitting exactly
    one JSON object on stdout with id/severity/status/message."""
    script = [p for p in CHECKS_DIR.iterdir()
              if p.name.startswith(sec_id + "-")][0]
    # Isolate env so device-optional vars are not set.
    env = {k: v for k, v in os.environ.items()
           if not k.startswith("XBRAIN_")}
    env["PATH"] = os.environ.get("PATH", "")
    r = subprocess.run(["bash", str(script)],
                       env=env, capture_output=True, text=True, timeout=15)
    lines = [l for l in r.stdout.splitlines() if l.strip()]
    assert lines, "no output from %s (stderr: %s)" % (script, r.stderr)
    parsed = json.loads(lines[-1])
    for k in ("id", "severity", "status", "message"):
        assert k in parsed, "%s: missing %s" % (script, k)
    assert parsed["id"] == sec_id
    assert parsed["severity"] in ("BLOCKING", "WARNING")
    assert parsed["status"] in ("PASS", "FAIL", "SKIPPED")


# --- Aggregator runs end-to-end and writes a report -----------------

def test_aggregate_runs_and_writes_report(tmp_path):
    env = os.environ.copy()
    env["XBRAIN_SEC_OUT_DIR"] = str(tmp_path)
    r = subprocess.run(["bash", str(RUN_SEC)],
                       env=env, capture_output=True, text=True, timeout=120)
    # Exit 0 even with SKIPPED, provided no BLOCKING+FAIL. On the
    # current bench most checks are SKIPPED so we expect 0.
    assert r.returncode == 0, r.stdout + r.stderr
    reports = list(tmp_path.glob("sec-*.json"))
    assert len(reports) == 1
    doc = json.loads(reports[0].read_text())
    assert doc["summary"]["total"] == 12
    assert doc["must_not_deliver"] is False


# --- Variant: SEC-08 catches restored legacy zenoh_bridge.json5 ----

def test_variant_sec08_flags_legacy_bridge(tmp_path):
    """Copy the SEC-08 script into a tmp repo layout that HAS the
    legacy V5 file, and verify the check reports FAIL. Isolates from
    the real repo so we do NOT create a violating file in-tree."""
    # Build a fake repo tree.
    fake = tmp_path / "fakerepo"
    (fake / "scripts" / "sec" / "checks").mkdir(parents=True)
    (fake / "ros2_ws" / "bridge" / "config").mkdir(parents=True)
    # Copy the real script over.
    src = CHECKS_DIR / "SEC-08-v5-bridge-deleted.sh"
    dst = fake / "scripts" / "sec" / "checks" / "SEC-08-v5-bridge-deleted.sh"
    dst.write_bytes(src.read_bytes())
    dst.chmod(0o755)
    # Plant the anti-pattern.
    (fake / "ros2_ws" / "bridge" / "config" / "zenoh_bridge.json5").write_text(
        "// V5 legacy junk\n")
    r = subprocess.run(["bash", str(dst)],
                       capture_output=True, text=True, timeout=15)
    line = [l for l in r.stdout.splitlines() if l.strip()][-1]
    doc = json.loads(line)
    assert doc["status"] == "FAIL"
    assert "V5" in doc["message"] or "GATE-5" in doc["message"]


# --- Variant: SEC-05 catches injected credential -------------------

def test_variant_sec05_flags_injected_credential(tmp_path):
    """Copy SEC-05 into a fake repo, plant a credential in
    configs/*.yaml (NOT inside configs/secrets/), verify FAIL."""
    fake = tmp_path / "fakerepo"
    (fake / "scripts" / "sec" / "checks").mkdir(parents=True)
    (fake / "configs").mkdir()
    src = CHECKS_DIR / "SEC-05-secrets-grep.sh"
    dst = fake / "scripts" / "sec" / "checks" / "SEC-05-secrets-grep.sh"
    dst.write_bytes(src.read_bytes())
    dst.chmod(0o755)
    (fake / "configs" / "bad.yaml").write_text(
        'server:\n  password: "supersecret123"\n')
    r = subprocess.run(["bash", str(dst)],
                       capture_output=True, text=True, timeout=15)
    line = [l for l in r.stdout.splitlines() if l.strip()][-1]
    doc = json.loads(line)
    assert doc["status"] == "FAIL", "expected FAIL, got: %s" % doc


# --- Variant: SEC-05 does NOT fire on a bare mention in a comment --

def test_variant_sec05_ignores_bare_mention(tmp_path):
    """CLAUDE.md 3.2 form 3 catch: judge that fires on its own mention
    of the anti-pattern is a defect. This test guards against SEC-05
    regressing to substring-match.

    Places a config that mentions 'password' in a comment (no key/value).
    """
    fake = tmp_path / "fakerepo"
    (fake / "scripts" / "sec" / "checks").mkdir(parents=True)
    (fake / "configs").mkdir()
    src = CHECKS_DIR / "SEC-05-secrets-grep.sh"
    dst = fake / "scripts" / "sec" / "checks" / "SEC-05-secrets-grep.sh"
    dst.write_bytes(src.read_bytes())
    dst.chmod(0o755)
    (fake / "configs" / "harmless.yaml").write_text(
        "# NOTE: the actual password lives in configs/secrets/\n"
        "server:\n  port: 7447\n")
    r = subprocess.run(["bash", str(dst)],
                       capture_output=True, text=True, timeout=15)
    line = [l for l in r.stdout.splitlines() if l.strip()][-1]
    doc = json.loads(line)
    assert doc["status"] == "PASS", \
        "SEC-05 must not false-fire on comment mentions; got: %s" % doc


# --- SEC-09 is WARNING severity, not BLOCKING ------------------------

def test_sec09_severity_is_warning_never_blocks(tmp_path):
    """SEC-09 is explicitly WARNING-level: the charge_manager state
    clears on next boot. A defect that upgrades it to BLOCKING would
    reject deploys unnecessarily; this test guards against that."""
    src = CHECKS_DIR / "SEC-09-charge-manager-stopped.sh"
    r = subprocess.run(["bash", str(src)],
                       capture_output=True, text=True, timeout=15)
    line = [l for l in r.stdout.splitlines() if l.strip()][-1]
    doc = json.loads(line)
    assert doc["severity"] == "WARNING", \
        "SEC-09 must be WARNING, got %s" % doc["severity"]


# --- Meta: aggregator FAILS if a BLOCKING check FAILs --------------

def test_aggregator_returns_nonzero_when_blocking_fails(tmp_path):
    """Inject a synthetic BLOCKING+FAIL check into a copy of the
    checks dir; verify aggregator exits non-zero and must_not_deliver=true."""
    fake_checks = tmp_path / "checks"
    fake_checks.mkdir()
    # Only one script: always-FAIL BLOCKING.
    (fake_checks / "SEC-99-always-fail.sh").write_text(
        "#!/usr/bin/env bash\n"
        'python3 -c \'import json; '
        'print(json.dumps({"id":"SEC-99","severity":"BLOCKING",'
        '"status":"FAIL","message":"synthetic"}))\'\n'
    )
    (fake_checks / "SEC-99-always-fail.sh").chmod(0o755)
    out = tmp_path / "report.json"
    r = subprocess.run(
        ["python3", str(REPO / "scripts" / "sec" / "aggregate.py"),
         "--checks-dir", str(fake_checks), "--out", str(out)],
        capture_output=True, text=True, timeout=30)
    assert r.returncode != 0
    doc = json.loads(out.read_text())
    assert doc["must_not_deliver"] is True


def test_aggregator_returns_zero_when_only_warning_fails(tmp_path):
    """WARNING+FAIL must NOT block delivery. This is the two-level
    severity story: SEC-09 falls in this bucket."""
    fake_checks = tmp_path / "checks"
    fake_checks.mkdir()
    (fake_checks / "SEC-99-warn-fail.sh").write_text(
        "#!/usr/bin/env bash\n"
        'python3 -c \'import json; '
        'print(json.dumps({"id":"SEC-99","severity":"WARNING",'
        '"status":"FAIL","message":"warn"}))\'\n'
    )
    (fake_checks / "SEC-99-warn-fail.sh").chmod(0o755)
    out = tmp_path / "report.json"
    r = subprocess.run(
        ["python3", str(REPO / "scripts" / "sec" / "aggregate.py"),
         "--checks-dir", str(fake_checks), "--out", str(out)],
        capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stdout + r.stderr
    doc = json.loads(out.read_text())
    assert doc["must_not_deliver"] is False


# --- Head comments name lineage -------------------------------------

def test_all_sec_scripts_have_head_comment():
    for p in CHECKS_DIR.glob("SEC-*.sh"):
        head = "\n".join(p.read_text().splitlines()[:12])
        assert "上海哈船智能船舶技术有限公司" in head, p.name
        assert "SEC-" in head, p.name


def test_run_sec_wrapper_exists_and_executable():
    assert RUN_SEC.is_file()
    assert os.access(RUN_SEC, os.X_OK)
    assert "CFG-BT-21" in RUN_SEC.read_text() or "INF-DB-5" in RUN_SEC.read_text()
