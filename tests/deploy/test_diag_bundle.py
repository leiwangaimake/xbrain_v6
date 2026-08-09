"""CHK-2-63 -- support-bundle collector + 4 variants."""

import json
import os
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest

from xbrain.boot.diag import collect


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent


# --- Process list from CLAUDE.md ------------------------------------

def test_read_process_list_returns_expected_set():
    xs = collect.read_process_list(str(REPO / "CLAUDE.md"))
    # Do NOT assert an exact count -- CLAUDE.md 3.7 forbids that.
    # Assert presence of specific must-have processes instead.
    for expected in ("p1_motion", "p2_core", "chassis_relay",
                     "quadruped", "perception", "zenohd-rt",
                     "zenohd-gen"):
        assert expected in xs, expected


# --- Variant (a): process table grew -> collector must pick it up ---

def test_variant_process_table_growth_is_reflected(tmp_path):
    """VARIANT: if a new row is added to CLAUDE.md 0.1 without any
    edits to the collector, the new process name must appear."""
    fake_claude = tmp_path / "CLAUDE.md"
    fake_claude.write_text(
        "## 0.1 Process list\n"
        "| 进程 | 语言 | 说明 |\n"
        "|---|---|---|\n"
        "| `hypothetical_new_proc` | Python | ★ 新进程 |\n"
        "| `another_new_proc` | C++ | ★ 也新 |\n"
    )
    xs = collect.read_process_list(str(fake_claude))
    assert "hypothetical_new_proc" in xs
    assert "another_new_proc" in xs


# --- is_under_secrets does not false-positive on similar names ------

def test_is_under_secrets_component_aware(tmp_path):
    """secrets-manifest.yaml is NOT under configs/secrets/, but a
    naive substring check would flag it."""
    secrets = tmp_path / "configs" / "secrets"
    secrets.mkdir(parents=True)
    similar = tmp_path / "configs" / "secrets-manifest.yaml"
    similar.parent.mkdir(exist_ok=True)
    similar.write_text("harmless")
    inside = secrets / "onvif.json"
    inside.write_text("secret")
    assert collect.is_under_secrets(str(inside), str(secrets)) is True
    assert collect.is_under_secrets(str(similar), str(secrets)) is False


# --- Full assemble on a fake tree -----------------------------------

@pytest.fixture()
def fake_tree(tmp_path):
    """Set up a fake data root and CLAUDE.md so assemble() runs
    end-to-end in isolation."""
    fake_claude = tmp_path / "CLAUDE.md"
    fake_claude.write_text(
        "## 0.1 Process list\n"
        "| 进程 |\n|---|\n"
        "| `p1_motion` |\n"
        "| `p2_core` |\n"
    )
    data_root = tmp_path / "data"
    (data_root / "logs").mkdir(parents=True)
    (data_root / "logs" / "p1_motion.log").write_text("p1 log content here\n" * 100)
    (data_root / "logs" / "p2_core.log").write_text("p2 log content here\n" * 50)
    resolved = tmp_path / "resolved"
    resolved.mkdir()
    (resolved / "MANIFEST.json").write_text('{"boot_id":"test123"}')
    (resolved / "p1_motion.yaml").write_text("frozen: config\n")
    bv = tmp_path / "_build.py"
    bv.write_text("build_version = 'test-123'\n")
    return {
        "claude": fake_claude,
        "data_root": data_root,
        "resolved": resolved,
        "build_version": bv,
    }


def test_assemble_produces_tarball_with_expected_contents(fake_tree, tmp_path):
    out = tmp_path / "out.tar.gz"
    xs = collect.read_process_list(str(fake_tree["claude"]))
    manifest = collect.assemble(
        out_tarball=out,
        processes=xs,
        data_root=fake_tree["data_root"],
        resolved_dir=fake_tree["resolved"],
        boot_fail_path=None,
        bit_result_path=None,
        build_version_path=fake_tree["build_version"],
        systemctl_snapshot="[test snapshot]",
    )
    assert out.is_file()
    with tarfile.open(str(out), "r:gz") as tar:
        names = tar.getnames()
    # Check inside the arcname stem
    joined = "\n".join(names)
    assert "MANIFEST.json" in joined
    assert "logs/p1_motion.log.tail" in joined
    assert "logs/p2_core.log.tail" in joined
    assert "resolved/MANIFEST.json" in joined
    assert "resolved/p1_motion.yaml" in joined
    assert "versions/build_version.py" in joined
    assert "systemd/status.txt" in joined


# --- Variant (b): configs/secrets/ must never appear in bundle -----

def test_variant_configs_secrets_are_never_included(fake_tree, tmp_path):
    """VARIANT: put a fake secret file into the fake configs/secrets/
    tree and assert the assembled tarball does not contain it. The
    collector's design is to NEVER copy from configs/secrets/, so this
    is a redundant safety net."""
    # Put a secret file where an operator might accidentally point at.
    secrets = tmp_path / "configs" / "secrets"
    secrets.mkdir(parents=True)
    secret_file = secrets / "onvif_credentials.json"
    secret_file.write_text(
        '{"user": "admin", "pass": "-----BEGIN RSA PRIVATE KEY-----"}')

    # Sanity: assembler does not include configs/secrets/ because it
    # was never asked to include it.
    out = tmp_path / "out.tar.gz"
    xs = collect.read_process_list(str(fake_tree["claude"]))
    collect.assemble(
        out_tarball=out,
        processes=xs,
        data_root=fake_tree["data_root"],
        resolved_dir=fake_tree["resolved"],
        boot_fail_path=None,
        bit_result_path=None,
        build_version_path=fake_tree["build_version"],
    )
    with tarfile.open(str(out), "r:gz") as tar:
        # Grep every extracted member's content for the secret sigil.
        for m in tar.getmembers():
            if not m.isfile():
                continue
            data = tar.extractfile(m).read()
            assert b"-----BEGIN" not in data, \
                "secret leaked into %s" % m.name
            assert b"onvif_credentials" not in data, \
                "secret filename leaked into %s" % m.name


# --- Variant (c): size cap truncation MUST be in MANIFEST -----------

def test_variant_size_cap_truncation_recorded_in_manifest(fake_tree, tmp_path):
    """VARIANT: set a tiny max_bundle_bytes so the collector must
    drop at least one log. The dropped file must be listed in
    manifest.truncated -- silent drop is banned."""
    # Bloat one log so we exceed cap.
    (fake_tree["data_root"] / "logs" / "p1_motion.log").write_bytes(
        b"x" * (2 * 1024 * 1024))
    out = tmp_path / "out.tar.gz"
    xs = collect.read_process_list(str(fake_tree["claude"]))
    manifest = collect.assemble(
        out_tarball=out,
        processes=xs,
        data_root=fake_tree["data_root"],
        resolved_dir=fake_tree["resolved"],
        boot_fail_path=None,
        bit_result_path=None,
        build_version_path=fake_tree["build_version"],
        log_tail_bytes=2 * 1024 * 1024,     # tail = the whole 2 MiB log
        max_bundle_bytes=512 * 1024,        # 512 KiB cap => must truncate
    )
    assert manifest["truncated"], \
        "collector must record truncation, not silently drop"
    # And the dropped file must actually be gone from the tarball.
    with tarfile.open(str(out), "r:gz") as tar:
        names = tar.getnames()
    # The truncated file names should NOT be under any log tail file
    # in the tarball (or the manifest is lying).
    for t in manifest["truncated"]:
        assert not any(t["path"] in n for n in names
                       if not n.endswith("MANIFEST.json")), \
            "manifest claims dropped, but file present: %s" % t["path"]


# --- Variant (d): missing resolved/ MUST be annotated --------------

def test_variant_missing_resolved_dir_is_annotated(fake_tree, tmp_path):
    """VARIANT: /run/xbrain/resolved cleared (fresh reboot). Bundle
    must still produce, but MANIFEST.warnings must name it."""
    out = tmp_path / "out.tar.gz"
    xs = collect.read_process_list(str(fake_tree["claude"]))
    ghost = tmp_path / "does_not_exist"
    manifest = collect.assemble(
        out_tarball=out,
        processes=xs,
        data_root=fake_tree["data_root"],
        resolved_dir=ghost,
        boot_fail_path=None,
        bit_result_path=None,
        build_version_path=fake_tree["build_version"],
    )
    assert out.is_file()
    warns = " ".join(manifest["warnings"])
    assert "resolved/" in warns
    assert "MISSING" in warns.upper() or "not populated" in warns
    # And the tarball must contain the MISSING.txt placeholder so an
    # off-site engineer opening the tarball sees the state at collection.
    with tarfile.open(str(out), "r:gz") as tar:
        assert any(n.endswith("resolved/MISSING.txt")
                   for n in tar.getnames())


# --- Bash wrapper exists + head comment -----------------------------

def test_bash_wrapper_exists_and_executable():
    p = REPO / "scripts" / "diag" / "collect_bundle.sh"
    assert p.is_file() and os.access(p, os.X_OK)
    assert "CHK-2-63" in p.read_text()
