"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_render_zenohd_gen.py
Brief: The GEN router config renderer keeps LAN2 mandatory and drops an unusable wifi endpoint (U92)

Description:
deploy/zenoh/render_zenohd_gen.sh replaced the unit's bare envsubst on
2026-10-26. The bare form rendered network.env's wifi placeholder
127.0.0.1 next to the template's own 127.0.0.1, and zenohd exits 255 on a
listen list that repeats an address (measured on .1.26) -- so the GEN router
could never start. 99 U92 ruling 5 says wifi is optional and must never take
LAN2 down; LAN2 itself is the delivery network and is required.

Each case runs the real script on the real template with a fake `ip` on
PATH, which decides which addresses "exist" on the host:

  * wifi placeholder 127.0.0.1 -> no duplicate, two endpoints
  * wifi present on an interface -> three endpoints
  * wifi absent / equal to LAN2 / 0.0.0.0 / empty -> skipped
  * LAN2 empty / 0.0.0.0 / absent -> non-zero exit, no output file
  * the unit's own two guards (no "${", no "0.0.0.0" anywhere in the file)
    pass on every rendered output -- the template's comments contain both
    strings, so a renderer that keeps comments makes the router unstartable
"""
from __future__ import annotations

import os  # env for the subprocess
import pathlib  # paths
import re  # endpoint extraction
import subprocess  # run the script

import pytest  # parametrize, tmp_path

pytestmark = pytest.mark.no_device  # runs anywhere with bash + envsubst

ROOT = pathlib.Path(__file__).resolve().parents[2]  # repo root
SCRIPT = ROOT / "deploy" / "zenoh" / "render_zenohd_gen.sh"  # under test
TEMPLATE = ROOT / "deploy" / "zenoh" / "zenohd-gen.json5"  # the real template


def _render(tmp_path, lan2, wifi, host_addrs):
    """Run the script; (returncode, endpoints or None, stdout+stderr)."""
    fake = tmp_path / "bin"  # holds the fake `ip`
    fake.mkdir(exist_ok=True)  # once per test
    lines = "".join("2: eth inet %s/24 brd x scope global\\n" % a for a in host_addrs)  # ip -o format
    (fake / "ip").write_text("#!/usr/bin/env bash\nprintf '%s'\n" % lines)  # prints the addresses
    (fake / "ip").chmod(0o755)  # executable
    out = tmp_path / "out" / "zenohd-gen.json5"  # rendered file
    env = dict(os.environ, PATH="%s:%s" % (fake, os.environ["PATH"]))  # fake ip first
    for k, v in (("LAN2_IP", lan2), ("WIFI_IP", wifi)):  # as EnvironmentFile sets them
        if v is None:  # unset entirely
            env.pop(k, None)
        else:  # set, possibly empty
            env[k] = v
    r = subprocess.run(["bash", str(SCRIPT), str(TEMPLATE), str(out)], env=env,
                       capture_output=True, text=True)  # the script itself
    if not out.exists():  # refused before writing
        return r.returncode, None, r.stdout + r.stderr
    text = out.read_text()  # rendered config
    # The unit's two ExecStartPre guards, applied exactly as it applies them:
    # grep over the WHOLE file, comments included (that is how the template's
    # own comments, which mention both strings, used to make them fail).
    assert "${" not in text  # ! grep -qF '${'
    assert "0.0.0.0" not in text  # ! grep -q "0\.0\.0\.0"
    eps = re.findall(r'"(tcp/[^"]+)"', text)  # every tcp endpoint
    return r.returncode, eps, r.stdout + r.stderr


def test_the_wifi_placeholder_no_longer_duplicates_loopback(tmp_path):
    """network.env today: WIFI_IP=127.0.0.1 -> loopback once, plus LAN2."""
    rc, eps, _ = _render(tmp_path, "192.168.188.200", "127.0.0.1",
                         ["127.0.0.1", "192.168.188.200"])  # .1.26 as deployed
    assert rc == 0  # rendered
    assert sorted(eps) == ["tcp/127.0.0.1:7447", "tcp/192.168.188.200:7447"]  # no duplicate
    assert len(eps) == len(set(eps))  # zenohd exits on a repeat


def test_a_present_wifi_is_bound_too(tmp_path):
    """U92 transitional period: wifi present -> bound next to LAN2."""
    rc, eps, _ = _render(tmp_path, "192.168.188.200", "192.168.1.26",
                         ["127.0.0.1", "192.168.188.200", "192.168.1.26"])  # wifi up
    assert rc == 0  # rendered
    assert sorted(eps) == ["tcp/127.0.0.1:7447", "tcp/192.168.1.26:7447",
                           "tcp/192.168.188.200:7447"]  # all three


@pytest.mark.parametrize("wifi", [
    "192.168.1.26",  # configured but not on any interface (wifi off)
    "192.168.188.200",  # same as LAN2
    "0.0.0.0",  # wildcard
    "",  # empty
    None,  # unset
])
def test_an_unusable_wifi_is_skipped_and_lan2_still_binds(tmp_path, wifi):
    """Wifi failing must never take LAN2 down (U92 ruling 5)."""
    rc, eps, log = _render(tmp_path, "192.168.188.200", wifi,
                           ["127.0.0.1", "192.168.188.200"])  # wifi not up
    assert rc == 0  # the router still gets a config
    assert sorted(eps) == ["tcp/127.0.0.1:7447", "tcp/192.168.188.200:7447"]  # LAN2 kept
    assert "wifi endpoint skipped" in log  # said, not silent


@pytest.mark.parametrize("lan2,addrs", [
    ("", ["127.0.0.1"]),  # empty
    (None, ["127.0.0.1"]),  # unset
    ("0.0.0.0", ["127.0.0.1"]),  # wildcard
    ("192.168.188.200", ["127.0.0.1"]),  # not on any interface (cable / NIC down)
])
def test_lan2_is_required(tmp_path, lan2, addrs):
    """No LAN2 -> refuse loudly, write nothing (never loopback-only and 'healthy')."""
    rc, eps, log = _render(tmp_path, lan2, "127.0.0.1", addrs)  # LAN2 missing
    assert rc != 0 and eps is None  # refused, no file
    assert "LAN2_IP" in log  # names the key
