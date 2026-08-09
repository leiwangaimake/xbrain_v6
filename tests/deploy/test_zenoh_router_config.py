"""INF-ZN-8 / CFG-BT-2 -- Zenoh router config + unit + check sanity."""

import os
import subprocess
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


REPO_ROOT = Path(__file__).parent.parent.parent
RT_CFG = REPO_ROOT / "deploy" / "zenoh" / "zenohd-rt.json5"
GEN_CFG = REPO_ROOT / "deploy" / "zenoh" / "zenohd-gen.json5"
RT_UNIT = REPO_ROOT / "deploy" / "systemd" / "xbrain-zenohd-rt.service"
GEN_UNIT = REPO_ROOT / "deploy" / "systemd" / "xbrain-zenohd-gen.service"
CHECK = REPO_ROOT / "scripts" / "check_router_config.sh"


# --- Config files exist -----------------------------------------------

def test_rt_config_exists():
    assert RT_CFG.is_file()


def test_gen_config_exists():
    assert GEN_CFG.is_file()


def test_rt_unit_exists():
    assert RT_UNIT.is_file()


def test_gen_unit_exists():
    assert GEN_UNIT.is_file()


# --- NET-C9: no 0.0.0.0 anywhere (in non-comment lines) --------------

def _non_comment_text(path: Path) -> str:
    lines = []
    for line in path.read_text().splitlines():
        # Strip inline // comments; also skip pure comment lines.
        stripped = line.split("//", 1)[0]
        lines.append(stripped)
    return "\n".join(lines)


def test_rt_config_no_wildcard_bind():
    assert "0.0.0.0" not in _non_comment_text(RT_CFG)


def test_gen_config_no_wildcard_bind():
    assert "0.0.0.0" not in _non_comment_text(GEN_CFG)


def test_rt_config_disables_multicast():
    src = RT_CFG.read_text()
    assert "multicast" in src
    # 'enabled: false' must appear inside the multicast block.
    import re
    m = re.search(r"multicast:\s*\{[^}]*enabled:\s*false", src, re.DOTALL)
    assert m, "RT config must explicitly set multicast.enabled=false"


def test_rt_config_disables_gossip():
    src = RT_CFG.read_text()
    import re
    m = re.search(r"gossip:\s*\{[^}]*enabled:\s*false", src, re.DOTALL)
    assert m, "RT config must explicitly set gossip.enabled=false"


def test_gen_config_disables_multicast():
    src = GEN_CFG.read_text()
    import re
    m = re.search(r"multicast:\s*\{[^}]*enabled:\s*false", src, re.DOTALL)
    assert m


def test_gen_config_disables_gossip():
    src = GEN_CFG.read_text()
    import re
    m = re.search(r"gossip:\s*\{[^}]*enabled:\s*false", src, re.DOTALL)
    assert m


def test_rt_binds_loopback_only():
    src = _non_comment_text(RT_CFG)
    assert "127.0.0.1:7449" in src
    # No other TCP endpoints.
    import re
    tcp_lines = re.findall(r'"tcp/[^"]+"', src)
    # exactly one endpoint, loopback.
    assert tcp_lines == ['"tcp/127.0.0.1:7449"'], \
        "RT must bind only loopback, got: %s" % tcp_lines


def test_gen_binds_loopback_and_templated():
    src = _non_comment_text(GEN_CFG)
    assert "127.0.0.1:7447" in src
    assert "${LAN2_IP}" in src
    assert "${WIFI_IP}" in src


# --- systemd unit sanity ---------------------------------------------

def test_rt_unit_hard_gated():
    src = RT_UNIT.read_text()
    assert "Restart=always" in src
    assert "Type=simple" in src


def test_gen_unit_reads_env_file():
    """GEN unit needs LAN2_IP / WIFI_IP from a per-site env file."""
    src = GEN_UNIT.read_text()
    assert "EnvironmentFile=" in src
    assert "network.env" in src


def test_gen_unit_refuses_unresolved_placeholders():
    """GEN unit's ExecStartPre must abort if a ${...} placeholder
    survived envsubst. Otherwise a missing LAN2_IP would silently
    make the endpoint spec invalid."""
    src = GEN_UNIT.read_text()
    # Look for a grep on the resolved file that ! grep -q "\${"
    assert '"\\${"' in src or "\\${" in src


def test_gen_unit_refuses_wildcard_in_resolved():
    """Belt-and-braces: even after envsubst, unit refuses 0.0.0.0
    in the resolved config."""
    src = GEN_UNIT.read_text()
    assert "0\\\\.0\\\\.0\\\\.0" in src or "0.0.0.0" in src


def test_rt_unit_starts_before_config_freeze():
    src = RT_UNIT.read_text()
    assert "Before=" in src
    assert "config-freeze" in src or "xbrain-config-freeze" in src


def test_gen_unit_after_rt_unit():
    """GEN comes AFTER RT (Stage 0z-1 before 0z-2)."""
    src = GEN_UNIT.read_text()
    assert "After=" in src
    assert "zenohd-rt" in src


# --- check_router_config.sh sanity -----------------------------------

def test_check_script_exists_and_executable():
    assert CHECK.is_file()
    assert os.access(CHECK, os.X_OK)


def test_check_script_passes_on_current_tree():
    r = subprocess.run(["bash", str(CHECK)],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stdout + r.stderr


def test_check_script_self_test_passes():
    r = subprocess.run(["bash", str(CHECK), "--self-test"],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0
    assert "self-test PASS" in r.stdout


def test_gate5_no_v5_legacy_bridge():
    """GATE-5 verbatim: ros2_ws/bridge/config/zenoh_bridge.json5 must
    NOT exist. It would hijack port 7447 if left."""
    legacy = REPO_ROOT / "ros2_ws" / "bridge" / "config" / "zenoh_bridge.json5"
    assert not legacy.exists(), \
        "V5 legacy zenoh_bridge.json5 found -- must be removed (GATE-5)"
