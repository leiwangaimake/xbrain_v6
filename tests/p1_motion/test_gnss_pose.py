"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_gnss_pose.py
Brief: Unit test for p1 gnss->pose bridge (3.1/3.3 fail-safe on missing data)

Description:
The load-bearing cases are the mutations of CLAUDE.md 3.3: assemble_pose must NOT
default heading_valid to true, and mirror_clock must NOT default sync to true.
Either fail-open would let a downstream consumer act on a heading/clock it never
had (NAV-02 over-trust / CLK-A3 violation). The fix half of the pose is asserted
to stay None -- rtk_driver does not publish rt/gnss/fix yet, and a fabricated
fix_type is exactly the silent default 3.1 forbids.
"""

from __future__ import annotations

from xbrain.p1_motion.path import gnss_pose


def test_assemble_pose_l1_passthrough():
    gh = {
        "heading_rad": 1.23, "heading_valid": True, "source": "dual_antenna",
        "level": 1, "speed_mps": 0.4, "cov_rad": 0.01, "i_heading": 1.0,
        "yaw_capable": True,
    }
    p = gnss_pose.assemble_pose(gh)
    assert p["heading_rad"] == 1.23
    assert p["heading_valid"] is True
    assert p["heading_source"] == "dual_antenna"
    assert p["heading_level"] == 1
    assert p["i_heading"] == 1.0
    assert p["yaw_capable"] is True
    # Fix half stays None (rt/gnss/fix not published yet) -- never fabricated.
    assert p["fix_type"] is None
    assert p["lat"] is None and p["lon"] is None


def test_assemble_pose_none_is_safe_l3():
    p = gnss_pose.assemble_pose(None)
    assert p["heading_valid"] is False        # 3.3: never defaults to True
    assert p["heading_rad"] is None
    assert p["heading_source"] is None
    assert p["fix_type"] is None
    assert p["i_fix"] is None
    assert p["yaw_capable"] is False


def test_assemble_pose_merges_fix_half():
    gf = {"fix_type": "rtk_fixed", "lat": 34.7, "lon": 135.5, "alt": 40.0,
          "cov_h_m": 0.02, "sats": 24}
    p = gnss_pose.assemble_pose({"heading_valid": True, "level": 1}, gf)
    assert p["fix_type"] == "rtk_fixed"
    assert p["lat"] == 34.7 and p["lon"] == 135.5
    assert p["cov_h_m"] == 0.02
    assert p["i_fix"] == 1.0                  # rtk_fixed -> full trust (11 S3.2.1)
    assert p["num_satellites"] == 24          # feeds G44
    # heading half still present
    assert p["heading_valid"] is True


def test_assemble_pose_i_fix_by_type():
    # 3.3: i_fix follows fix_type exactly, single/no_fix -> 0 (no autonomous motion).
    assert gnss_pose.assemble_pose(None, {"fix_type": "rtk_float"})["i_fix"] == 0.4
    assert gnss_pose.assemble_pose(None, {"fix_type": "single"})["i_fix"] == 0.0
    assert gnss_pose.assemble_pose(None, {"fix_type": "no_fix"})["i_fix"] == 0.0


def test_assemble_pose_no_fix_position_none():
    # A no_fix GnssFix (module had no position): lat/lon stay None, not 0 (NAV-02).
    p = gnss_pose.assemble_pose(None, {"fix_type": "no_fix", "lat": None, "lon": None})
    assert p["fix_type"] == "no_fix"
    assert p["lat"] is None and p["lon"] is None
    assert p["i_fix"] == 0.0


def test_assemble_pose_invalid_stays_invalid():
    # 3.3 red mutant: an L3 GnssHeading (valid=False) must map to valid=False, and
    # heading_source must pass through, not be hardcoded to a plausible value.
    gh = {"heading_valid": False, "source": "none", "level": 3, "heading_rad": 0.0}
    p = gnss_pose.assemble_pose(gh)
    assert p["heading_valid"] is False
    assert p["heading_source"] == "none"
    assert p["heading_level"] == 3


def test_mirror_clock_passthrough():
    cs = {"sync": True, "source": "rtk"}
    m = gnss_pose.mirror_clock(cs)
    assert m["sync"] is True
    assert m["source"] == "rtk"


def test_mirror_clock_none_is_failsafe():
    # 3.3 red mutant: no ClockStatus -> sync MUST be False (CLK-A3), never True.
    m = gnss_pose.mirror_clock(None)
    assert m["sync"] is False
    assert m["source"] == "none"


def test_stamp_envelope_shape():
    env = gnss_pose.stamp_envelope(
        {"heading_valid": False}, rid="m20s", boot="abc12345", seq=7,
        src="p1_motion", ts_sync=False)
    for f in ("v", "rid", "ts", "mono", "boot", "seq", "src", "ts_sync", "data"):
        assert f in env
    assert env["rid"] == "m20s"
    assert env["seq"] == 7
    assert env["src"] == "p1_motion"
    assert env["ts_sync"] is False
    assert env["data"]["heading_valid"] is False
