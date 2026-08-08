"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_dependency_lock.py
Brief: CHK-0-55 -- assertions on the runtime lock file + installed versions
       + zenoh wire alignment + import coverage, plus the four mutations

Description:
Runs each of CHK-0-55's four criterion checks and each of the four criterion
mutations. Mutations operate on temp files (never touch the committed lock),
so a red run does not require restoring the tree.
"""

import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "scripts", "deps"))

from check_deps import (                              # noqa: E402
    _IMPORT_TO_DIST, _LOCK_LINE_RE, _major,
    check_imports_covered, check_installed, check_zenoh_wire,
    load_lock, scan_third_party_imports,
)

LOCK_PATH = os.path.join(ROOT, "scripts", "deps", "requirements-runtime.txt")


# --------------------------------------------------------------------------
# ① lock file well-formed
# --------------------------------------------------------------------------

def test_lock_line_shape_pins_exact_versions():
    """Only `name==x.y.z` is accepted. Regex is asserted directly so a change
    to it that would let `>=` slip through is caught before any file is
    parsed."""
    assert _LOCK_LINE_RE.match("PyYAML==6.0.2")
    assert _LOCK_LINE_RE.match("eclipse-zenoh==1.9.0")
    assert not _LOCK_LINE_RE.match("PyYAML>=6.0"), "regex accepts >="
    assert not _LOCK_LINE_RE.match("PyYAML"), "regex accepts bare name"
    assert not _LOCK_LINE_RE.match("PyYAML==6"), "regex accepts single-segment"


def test_real_lock_loads_clean():
    """*** The committed lock parses without raising -- so every line is
    already `name==x.y.z`. A future edit that added `>=` fails load_lock."""
    entries = load_lock(LOCK_PATH)
    assert entries, "lock is empty"
    for name, ver in entries.items():
        assert _LOCK_LINE_RE.match("%s==%s" % (name, ver)), (name, ver)


# --------------------------------------------------------------------------
# ② installed == locked
# --------------------------------------------------------------------------

def test_installed_matches_lock_today():
    """*** As of the write of this file, every locked distribution is
    installed at the exact locked version. If a machine's environment drifts,
    this fails naming the delta."""
    lock = load_lock(LOCK_PATH)
    deltas = check_installed(lock)
    assert not deltas, deltas


# --------------------------------------------------------------------------
# ③ zenoh wire major agreement (when zenohd is present) OR loud skip
# --------------------------------------------------------------------------

def test_zenoh_wire_check_never_silently_passes():
    """Two acceptable outcomes: OK (major aligned) OR WARN-skip when zenohd
    is absent. The WARN message must be emitted -- silent success is the
    failure mode CLAUDE.md 3.2 form 6 names. Strict mode with no zenohd
    fails, so the two modes cover the two operator intents."""
    lock = load_lock(LOCK_PATH)
    ok, detail = check_zenoh_wire(lock, strict=False)
    assert ok
    # Whichever branch fired must surface enough text to tell the two apart.
    assert "eclipse-zenoh" in detail or "zenohd" in detail


def test_zenoh_wire_strict_mode_requires_zenohd():
    """When --strict is passed and zenohd is absent, the check REFUSES --
    i.e. --strict does not silently accept the WARN branch."""
    lock = load_lock(LOCK_PATH)
    if shutil.which("zenohd") is not None:
        pytest.skip("zenohd present; --strict cannot fail here on this host")
    ok, detail = check_zenoh_wire(lock, strict=True)
    assert not ok
    assert "zenohd" in detail


# --------------------------------------------------------------------------
# ④ every third-party import in xbrain/ + common/ is in the lock
# --------------------------------------------------------------------------

def test_all_xbrain_imports_are_locked():
    """*** Bidirectional-empty diff between (xbrain/ + common/ imports)
    and lock entries. A new dependency added under xbrain/ that forgets to
    land in the lock fails here."""
    imports = set()
    for sub in ("xbrain", "common"):
        p = os.path.join(ROOT, sub)
        if os.path.isdir(p):
            imports |= scan_third_party_imports(p)
    lock = load_lock(LOCK_PATH)
    uncovered = check_imports_covered(lock, imports)
    assert not uncovered, ("imports not in lock: %s "
                           "(add via _IMPORT_TO_DIST if the module name "
                           "differs from the dist name)" % uncovered)


def test_import_to_dist_only_names_real_translations():
    """A mapping entry only matters when the module name differs from the
    dist name; a redundant entry (`name -> name`) is noise. This forbids
    them so the table stays a minimum-necessary translation."""
    for import_name, dist_name in _IMPORT_TO_DIST.items():
        assert import_name != dist_name, (
            "%r maps to itself; remove the row" % import_name)


# --------------------------------------------------------------------------
# The four criterion mutations
# --------------------------------------------------------------------------

def test_mutation_a_ge_syntax_is_rejected(tmp_path):
    """*** Mutation (a): a lock line `aiosqlite>=0.19` fails load_lock at ①."""
    bad = tmp_path / "req.txt"
    bad.write_text("PyYAML==6.0.2\naiosqlite>=0.19\n")
    with pytest.raises(ValueError, match=r">="):
        load_lock(str(bad))


def test_mutation_b_installed_version_mismatch(tmp_path):
    """*** Mutation (b): if the lock says pydantic==0.0.1 but the installed
    version is real, ② reports the delta by name."""
    lock = {"pydantic": "0.0.1"}
    deltas = check_installed(lock)
    assert deltas
    name, want, got = deltas[0]
    assert name == "pydantic"
    assert want == "0.0.1"
    assert got != "0.0.1"                            # some real version installed


def test_mutation_c_zenohd_major_mismatch_via_stub(tmp_path, monkeypatch):
    """*** Mutation (c): a stub `zenohd` that reports major 0 while Python
    eclipse-zenoh is at major 1 must fail ③.

    Stub is a tiny shell script placed on PATH via monkeypatch; robust
    against a real zenohd being present on the host (the stubbed PATH wins).
    """
    stub_dir = tmp_path / "bin"
    stub_dir.mkdir()
    stub = stub_dir / "zenohd"
    stub.write_text("#!/bin/sh\necho 'zenohd 0.11.0 stub'\n")
    stub.chmod(0o755)
    monkeypatch.setenv("PATH", "%s:%s" % (str(stub_dir), os.environ["PATH"]))
    lock = {"eclipse-zenoh": "1.9.0"}
    ok, detail = check_zenoh_wire(lock, strict=False)
    assert not ok
    assert "major mismatch" in detail
    assert "1.x" in detail and "0.x" in detail


def test_mutation_d_unlocked_import_reported(tmp_path):
    """*** Mutation (d): an xbrain/-shaped file that imports an unlocked
    third-party module is reported by check_imports_covered."""
    src = tmp_path / "fake"
    src.mkdir()
    (src / "one.py").write_text("import numpy\n")     # numpy not in lock
    imports = scan_third_party_imports(str(src))
    lock = load_lock(LOCK_PATH)
    uncovered = check_imports_covered(lock, imports)
    assert "numpy" in uncovered


# --------------------------------------------------------------------------
# Reverse assertion: check_deps.py exits 0 on the real repo today
# --------------------------------------------------------------------------

def test_check_deps_script_exits_zero_on_current_repo():
    """*** Reverse: running the whole script on the committed lock and repo
    exits 0. A stray import in xbrain/ or a drift in the environment fails
    here. --strict deliberately NOT passed -- dev machines have no zenohd."""
    r = subprocess.run(
        [sys.executable, os.path.join(ROOT, "scripts", "deps", "check_deps.py")],
        capture_output=True, text=True, cwd=ROOT,
    )
    assert r.returncode == 0, r.stdout + r.stderr
