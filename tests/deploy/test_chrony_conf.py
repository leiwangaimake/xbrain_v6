"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chrony_conf.py
Brief: deploy tests -- chrony (NTP) time-sync config sanity

Description:
Guards the design-mandated shape of deploy/chrony/chrony.conf, the wall-clock
discipline source rtk_driver reads to judge ClockStatus.sync (11 S3.11 / CLK-A1).
Each assertion carries the mutant that turns it red (CLAUDE.md 3.3):

  * makestep is '1.0 3' (11 S1.5.6: step only the first 3 updates, slew after).
    MUTATION: widen it to a permanent step and the running-timeline guard is gone.
  * China domestic NTP servers are present (user 2026-08-16: Ubuntu pools may be
    unreachable for a domestic delivery). MUTATION: drop them and a CN site loses
    a reachable source.
  * no `allow` line opens 0.0.0.0 (NET-C8/NET-C9 spirit: never serve time wide).
    MUTATION: `allow 0.0.0.0/0` and a captured payload device could steer time.
"""

from pathlib import Path

import pytest

pytestmark = pytest.mark.no_device

REPO_ROOT = Path(__file__).parent.parent.parent
CHRONY_CONF = REPO_ROOT / "deploy" / "chrony" / "chrony.conf"


def _active_lines():
    """The non-comment, non-blank directive lines (a commented placeholder is
    NOT an active directive -- PPS/local-server/offline are intentionally #-ed)."""
    text = CHRONY_CONF.read_text(encoding="utf-8")
    return [ln.strip() for ln in text.splitlines()
            if ln.strip() and not ln.lstrip().startswith("#")]


def test_conf_exists():
    assert CHRONY_CONF.is_file()


def test_makestep_is_first_three_only():
    """11 S1.5.6: step only the first 3 updates. Accept '1.0 3' or '1 3' (chrony
    treats them identically); reject any other count."""
    steps = [ln for ln in _active_lines() if ln.startswith("makestep")]
    assert len(steps) == 1, "exactly one active makestep expected, got %r" % steps
    parts = steps[0].split()
    # makestep <threshold> <limit>; limit 3 = first three updates.
    assert parts[1] in ("1.0", "1") and parts[2] == "3", (
        "makestep must be '1.0 3' (11 S1.5.6), got %r" % steps[0])


def test_china_domestic_ntp_present():
    """User 2026-08-16: ship reliable CN servers for domestic delivery."""
    active = "\n".join(_active_lines())
    # At least the authoritative NTSC plus one cloud provider, so a domestic site
    # has a reachable source even when the Ubuntu/international pools are blocked.
    assert "ntp.ntsc.ac.cn" in active, "National Time Service Center server missing"
    assert "ntp.aliyun.com" in active or "ntp.tencent.com" in active, (
        "at least one CN cloud NTP server expected")
    assert "cn.pool.ntp.org" in active, "cn.pool.ntp.org missing"


def test_no_wide_open_serving():
    """NET-C8/NET-C9 spirit: never serve time to 0.0.0.0. The served-subnet
    `allow` is a commented placeholder; no ACTIVE allow may be a wildcard."""
    for ln in _active_lines():
        if ln.startswith("allow"):
            assert "0.0.0.0" not in ln and "::" not in ln, (
                "active `allow` must name a subnet, never a wildcard: %r" % ln)
