"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_net_config.py
Brief: deploy tests -- net config

Description:
INF-DP-9 / CFG-BT-20 -- deploy/net + check_net.sh + variants.
"""


import os
import re
import subprocess
import tempfile
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent
NET_DIR = REPO / "deploy" / "net"
CHECK = REPO / "scripts" / "check_net.sh"


# --- Existence -------------------------------------------------------

@pytest.mark.parametrize("name", [
    "DBG.network", "DBG.nft", "PROD.network", "PROD.nft",
])
def test_net_template_exists(name):
    assert (NET_DIR / name).is_file()


# --- check_net.sh sanity --------------------------------------------

def test_check_net_passes_on_current_tree():
    r = subprocess.run(["bash", str(CHECK)],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stdout + r.stderr


def test_check_net_self_test_passes():
    r = subprocess.run(["bash", str(CHECK), "--self-test"],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0
    assert "self-test PASS" in r.stdout


# --- Variant 1: FORWARD accept must be caught (DEP-6) --------------

def test_variant_forward_accept_is_flagged(tmp_path):
    """VARIANT: change FORWARD policy to accept -> check must flag."""
    tmp_repo = tmp_path
    (tmp_repo / "scripts").mkdir()
    (tmp_repo / "deploy" / "net").mkdir(parents=True)
    # Copy the check script (script derives repo root via SCRIPT_DIR).
    (tmp_repo / "scripts" / "check_net.sh").write_bytes(CHECK.read_bytes())
    os.chmod(tmp_repo / "scripts" / "check_net.sh", 0o755)
    # Write a nft file with FORWARD accept.
    (tmp_repo / "deploy" / "net" / "bad.nft").write_text(
        "table inet t {\n"
        "  chain forward {\n"
        "    type filter hook forward priority 0; policy accept;\n"
        "  }\n"
        "}\n")
    r = subprocess.run(
        ["bash", str(tmp_repo / "scripts" / "check_net.sh")],
        capture_output=True, text=True, timeout=30)
    assert r.returncode != 0
    assert "FORWARD default is not drop" in (r.stdout + r.stderr)


# --- Variant 2: DNAT with source 0.0.0.0/0 must be caught (SEC-2) --

def test_variant_dnat_wildcard_source_is_flagged(tmp_path):
    tmp_repo = tmp_path
    (tmp_repo / "scripts").mkdir()
    (tmp_repo / "deploy" / "net").mkdir(parents=True)
    (tmp_repo / "scripts" / "check_net.sh").write_bytes(CHECK.read_bytes())
    os.chmod(tmp_repo / "scripts" / "check_net.sh", 0o755)
    (tmp_repo / "deploy" / "net" / "bad.nft").write_text(
        "table inet t {\n"
        "  chain forward {\n"
        "    type filter hook forward priority 0; policy drop;\n"
        "  }\n"
        "  chain prerouting {\n"
        "    ip saddr 0.0.0.0/0 tcp dport 554 dnat to 10.1.1.1:554\n"
        "  }\n"
        "}\n")
    r = subprocess.run(
        ["bash", str(tmp_repo / "scripts" / "check_net.sh")],
        capture_output=True, text=True, timeout=30)
    assert r.returncode != 0
    assert "SEC-2 violation" in (r.stdout + r.stderr)


# --- Variant 3: unknown placeholder ${ptz_ip} -----------------------

def test_variant_unknown_placeholder_is_flagged(tmp_path):
    """VARIANT NETD-2 (typo). A lower-cased snake variable like
    ${ptz_ip} would resolve to empty via envsubst (not a known env
    var) and yield a broken rule. Whitelist catches it."""
    tmp_repo = tmp_path
    (tmp_repo / "scripts").mkdir()
    (tmp_repo / "deploy" / "net").mkdir(parents=True)
    (tmp_repo / "scripts" / "check_net.sh").write_bytes(CHECK.read_bytes())
    os.chmod(tmp_repo / "scripts" / "check_net.sh", 0o755)
    (tmp_repo / "deploy" / "net" / "typo.nft").write_text(
        "table inet t {\n"
        "  chain forward {\n"
        "    type filter hook forward priority 0; policy drop;\n"
        "  }\n"
        "  chain prerouting {\n"
        "    ip saddr 10.1.1.0/24 dnat to ${ptz_ip}:554\n"   # lowercase typo
        "  }\n"
        "}\n")
    r = subprocess.run(
        ["bash", str(tmp_repo / "scripts" / "check_net.sh")],
        capture_output=True, text=True, timeout=30)
    assert r.returncode != 0
    assert "unknown placeholder" in (r.stdout + r.stderr)


# --- Static: templates use ${LAN?_IP} pattern for iface addresses --

def test_prod_network_binds_all_five_interfaces():
    src = (NET_DIR / "PROD.network").read_text()
    for iface in ("lan1", "lan2", "lan3", "lan4", "wlan0"):
        assert "Name=" + iface in src, iface


def test_no_hardcoded_ips_in_templates():
    """Templates must not carry hard-coded IPv4 addresses -- those
    should live in the per-site hw_profile.yaml, not the template.

    Bench IP ranges (RFC1918) that appear in COMMENTS are stripped
    before the check, matching how check_net.sh strips comments."""
    ipv4_re = re.compile(r"\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b")
    for f in NET_DIR.glob("*.n*"):
        # Strip # comments AND -- multiline comment blocks; anything
        # inside a `# ...` block on any line is documentation.
        body_lines = []
        for line in f.read_text().splitlines():
            stripped = line.split("#", 1)[0]
            body_lines.append(stripped)
        body = "\n".join(body_lines)
        ips = ipv4_re.findall(body)
        # 0.0.0.0 sentinels are policy sigils, not addresses.
        real_ips = [i for i in ips if i != "0.0.0.0"]
        assert not real_ips, \
            "%s carries hard-coded IPs %s -- put them in hw_profile" % (f.name, real_ips)


# --- Head comments name lineage -------------------------------------

@pytest.mark.parametrize("name", ["PROD.nft", "DBG.nft", "PROD.network", "DBG.network"])
def test_head_comment_names_lineage(name):
    head = (NET_DIR / name).read_text().splitlines()[0]
    assert "CFG-BT-20" in head or "INF-DP-9" in head, name
