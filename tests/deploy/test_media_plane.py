"""INF-MD-1 -- media plane MED static checks + variants."""

import os
import subprocess
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent
CHECK = REPO / "scripts" / "ci" / "check_med.sh"


def test_check_med_script_exists_and_executable():
    assert CHECK.is_file()
    assert os.access(CHECK, os.X_OK)


def test_check_med_passes_on_current_tree():
    r = subprocess.run(["bash", str(CHECK)],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + r.stderr


# --- Variant: DNAT to a non-media port ------------------------------
# MED-C1 says only 554 + 8554 may be DNATed. Anything else (e.g., 80
# for PTZ Web) exposes the device management plane.

def test_variant_dnat_to_ptz_web_port_flagged(tmp_path):
    """Copy check_med.sh into a fake repo, plant a PTZ Web DNAT rule
    (dnat to :80), verify check exits non-zero."""
    fake = tmp_path / "fakerepo"
    (fake / "scripts" / "ci").mkdir(parents=True)
    (fake / "scripts" / "sec" / "checks").mkdir(parents=True)
    (fake / "deploy" / "net").mkdir(parents=True)

    # Copy the three scripts check_med.sh depends on.
    for src, dst in [
        (REPO / "scripts" / "ci" / "check_med.sh",
         fake / "scripts" / "ci" / "check_med.sh"),
        (REPO / "scripts" / "check_net.sh",
         fake / "scripts" / "check_net.sh"),
        (REPO / "scripts" / "sec" / "checks" / "SEC-05-secrets-grep.sh",
         fake / "scripts" / "sec" / "checks" / "SEC-05-secrets-grep.sh"),
    ]:
        dst.write_bytes(src.read_bytes())
        dst.chmod(0o755)
    (fake / "configs").mkdir()

    # Plant a nft with a DNAT to PTZ Web port 80 (violates MED-C1).
    (fake / "deploy" / "net" / "bad.nft").write_text(
        "table inet t {\n"
        "  chain forward {\n"
        "    type filter hook forward priority 0; policy drop;\n"
        "  }\n"
        "  chain prerouting {\n"
        "    ip saddr 10.1.1.0/24 tcp dport 80 dnat to 10.2.2.2:80\n"
        "  }\n"
        "}\n")

    r = subprocess.run(["bash", str(fake / "scripts" / "ci" / "check_med.sh")],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode != 0
    assert "MED-C1" in (r.stdout + r.stderr)


# --- Variant: DNAT source 0.0.0.0/0 (delegate to check_net.sh) -----

def test_variant_dnat_wildcard_source_delegated_flag(tmp_path):
    """MED-C2 is enforced via check_net.sh. If check_net.sh reports
    an issue (e.g., 0.0.0.0/0 source), check_med.sh also fails."""
    fake = tmp_path / "fakerepo"
    (fake / "scripts" / "ci").mkdir(parents=True)
    (fake / "scripts" / "sec" / "checks").mkdir(parents=True)
    (fake / "deploy" / "net").mkdir(parents=True)
    for src, dst in [
        (REPO / "scripts" / "ci" / "check_med.sh",
         fake / "scripts" / "ci" / "check_med.sh"),
        (REPO / "scripts" / "check_net.sh",
         fake / "scripts" / "check_net.sh"),
        (REPO / "scripts" / "sec" / "checks" / "SEC-05-secrets-grep.sh",
         fake / "scripts" / "sec" / "checks" / "SEC-05-secrets-grep.sh"),
    ]:
        dst.write_bytes(src.read_bytes())
        dst.chmod(0o755)
    (fake / "configs").mkdir()

    (fake / "deploy" / "net" / "bad.nft").write_text(
        "table inet t {\n"
        "  chain forward {\n"
        "    type filter hook forward priority 0; policy drop;\n"
        "  }\n"
        "  chain prerouting {\n"
        "    ip saddr 0.0.0.0/0 tcp dport 554 dnat to 10.2.2.2:554\n"
        "  }\n"
        "}\n")

    r = subprocess.run(["bash", str(fake / "scripts" / "ci" / "check_med.sh")],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode != 0


# --- Head comment lineage -------------------------------------------

def test_check_med_head_names_lineage():
    head = "\n".join(CHECK.read_text().splitlines()[:12])
    assert "INF-MD-1" in head
    assert "上海哈船智能船舶技术有限公司" in head
