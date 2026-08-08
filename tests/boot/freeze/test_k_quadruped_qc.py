"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_k_quadruped_qc.py
Brief: CFG-FZ-12 -- assertion K five variants + baseline + per-QC coverage

Description:
Every QC-N rule gets at least one red-turning test. The five variants
named verbatim in CFG-FZ-12 are:

  (1) codebook: legacy_decimal + empty codebook_table.legacy_decimal
                -> QC-13 red
  (2) prone_forbidden_gaits without stair_standard
                -> QC-9 red
  (3) tier1.cmd_timeout_ms = 50
                -> QC-2 red
  (4) chassis_dds.domain_id == uplink.ros_domain_id
                -> QC-4 red
  (5) TLS cred_dir missing / credential files absent
                -> QC-15 red

Plus per-QC targeted tests (QC-1, QC-3, QC-5, QC-6, QC-7, QC-8, QC-10,
QC-11, QC-12, QC-13-hex32-half, QC-14, QC-16, QC-17) so a future
refactor cannot silently drop a rule.
"""

import os
from typing import Any, Dict

import pytest

from xbrain.boot.freeze.assertions.k_quadruped_qc import run
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Green scaffold: matches 13 S8.2 example values
# ---------------------------------------------------------------------------

def _green_quadruped() -> Dict[str, Any]:
    """Return a healthy quadruped tree; every QC-N passes."""
    return {"quadruped": {
        "robot_id": "xb-001",
        "chassis_link": {
            "endpoint_candidates": [
                {"proto": "udp", "host": "10.21.31.103", "port": 30000,
                 "tls": False, "enabled": True},
            ],
            "tls": {"lib": "mbedtls", "cred_dir": "", "verify_peer": True},
            "probe_timeout_ms": 2000,
            "heartbeat_hz": 2.0,
            "codebook": "hex32",
            "codebook_table": {"legacy_decimal": {}},
            "asdu_format": "json",
            "axis_cmd_socket_fixed": True,
            "single_tx_owner": True,
        },
        "chassis_dds": {
            "backend": "cyclone_raw",
            "domain_id": 0,
            "imu_topic": "/IMU",
            "forward_imu_to_rt": False,
            "imu_rt_key": "",
        },
        "uplink": {
            "zenoh_rt_endpoint": "tcp/127.0.0.1:7449",
            "ros_domain_id": 42,
            "rmw": "rmw_cyclonedds_cpp",
            "odom_topic": "/odom_quadruped",
        },
        "odom": {
            "publish_hz": 100.0,
            "stale_warn_ms": 150,
            "stale_invalid_ms": 300,
            "stale_stop_publish_ms": 1000,
        },
        "motion": {
            "prone_forbidden_gaits": ["stair_agile", "stair_standard"],
            "axes": {"always_active": ["vx", "vy", "wz"],
                     "special_gaits": []},
            "not_implemented": {"gaits": ["stair_standard"]},
        },
        "tier1": {
            "cmd_timeout_ms": 200,
            "control_loop_hz": 100.0,
        },
    }}


def _ctx(quadruped_raw=None, tmp_path=None, **extra):
    """Build ctx with a config_root + quadruped raw override."""
    c = {"config_root": str(tmp_path) if tmp_path else "/tmp",
         "quadruped_raw": (quadruped_raw if quadruped_raw is not None
                           else _green_quadruped())}
    c.update(extra)
    return c


# ---------------------------------------------------------------------------
# Baseline
# ---------------------------------------------------------------------------

def test_green_scaffold_passes(tmp_path):
    """The healthy scaffold triggers no QC-N."""
    result = run(_ctx(tmp_path=tmp_path))
    assert result["status"] == "pass"
    assert result["assertion"] == "K"
    assert result["checks_run"] == 17


# ---------------------------------------------------------------------------
# QC-1: no spec.* definitions
# ---------------------------------------------------------------------------

def test_qc1_spec_key_defined_is_red(tmp_path):
    """Defining spec.max_vx_mps under quadruped violates QC-1."""
    tree = _green_quadruped()
    tree["quadruped"]["spec"] = {"max_vx_mps": 2.0}
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-1"
    assert ei.value.detail["reason"] == "spec_key_defined"


def test_qc1_reference_form_is_legal(tmp_path):
    """A string value that LOOKS like ${common.spec.*} is not a definition."""
    tree = _green_quadruped()
    # Reference form is a scalar value under a legitimate key, not
    # a top-level spec.* definition.
    tree["quadruped"]["odom"]["a_max"] = "${common.spec.max_decel_mps2}"
    result = run(_ctx(tree, tmp_path=tmp_path))
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# QC-2 variant (3): cmd_timeout_ms = 50
# ---------------------------------------------------------------------------

def test_variant_3_cmd_timeout_below_floor_is_qc2_red(tmp_path):
    """CFG-FZ-12 variant (3) verbatim: tier1.cmd_timeout_ms = 50."""
    tree = _green_quadruped()
    tree["quadruped"]["tier1"]["cmd_timeout_ms"] = 50
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-2"
    assert ei.value.detail["value"] == 50
    assert ei.value.detail["limit"] == 200


def test_qc2_exact_floor_passes(tmp_path):
    """cmd_timeout_ms = 200 (exact floor) must pass -- >= comparison."""
    tree = _green_quadruped()
    tree["quadruped"]["tier1"]["cmd_timeout_ms"] = 200
    assert run(_ctx(tree, tmp_path=tmp_path))["status"] == "pass"


# ---------------------------------------------------------------------------
# QC-3
# ---------------------------------------------------------------------------

def test_qc3_low_control_loop_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["tier1"]["control_loop_hz"] = 50.0
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-3"


# ---------------------------------------------------------------------------
# QC-4 variant (4): domain collision
# ---------------------------------------------------------------------------

def test_variant_4_domain_collision_is_qc4_red(tmp_path):
    """CFG-FZ-12 variant (4) verbatim: chassis_dds.domain_id ==
    uplink.ros_domain_id."""
    tree = _green_quadruped()
    # Force the collision: both to 42.
    tree["quadruped"]["chassis_dds"]["domain_id"] = 42
    tree["quadruped"]["uplink"]["ros_domain_id"] = 42
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-4"
    assert ei.value.detail["reason"] == "domain_collision"


# ---------------------------------------------------------------------------
# QC-5
# ---------------------------------------------------------------------------

def test_qc5_domain_0_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["uplink"]["ros_domain_id"] = 0
    tree["quadruped"]["chassis_dds"]["domain_id"] = 1
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-5"


def test_qc5_unknown_rmw_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["uplink"]["rmw"] = "rmw_zenoh_cpp"
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-5"
    assert "rmw_zenoh_cpp" == ei.value.detail["value"]


# ---------------------------------------------------------------------------
# QC-6
# ---------------------------------------------------------------------------

def test_qc6_empty_endpoint_candidates_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["endpoint_candidates"] = []
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-6"


def test_qc6_bad_port_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["endpoint_candidates"][0]["port"] = 0
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-6"


# ---------------------------------------------------------------------------
# QC-7 / QC-8
# ---------------------------------------------------------------------------

def test_qc7_non_monotone_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["odom"]["stale_invalid_ms"] = 100  # < warn 150
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-7"


def test_qc8_non_integer_period_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["odom"]["publish_hz"] = 30.0  # 1000/30 = 33.33 ms
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-8"


# ---------------------------------------------------------------------------
# QC-9 variant (2): stair_standard missing
# ---------------------------------------------------------------------------

def test_variant_2_prone_missing_stair_standard_is_qc9_red(tmp_path):
    """CFG-FZ-12 variant (2) verbatim: motion.prone_forbidden_gaits
    without stair_standard."""
    tree = _green_quadruped()
    tree["quadruped"]["motion"]["prone_forbidden_gaits"] = ["stair_agile"]
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-9"
    assert "stair_standard" in ei.value.detail["missing"]


def test_qc9_widened_is_legal(tmp_path):
    """Widening prone_forbidden_gaits (adding more entries) is legal."""
    tree = _green_quadruped()
    tree["quadruped"]["motion"]["prone_forbidden_gaits"] = [
        "stair_agile", "stair_standard", "damped_prone"]
    assert run(_ctx(tree, tmp_path=tmp_path))["status"] == "pass"


# ---------------------------------------------------------------------------
# QC-10
# ---------------------------------------------------------------------------

def test_qc10_unregistered_gait_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["motion"]["axes"]["special_gaits"] = ["moonwalk"]
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-10"


# ---------------------------------------------------------------------------
# QC-11 / QC-12 / QC-16
# ---------------------------------------------------------------------------

def test_qc11_low_heartbeat_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["heartbeat_hz"] = 0.5
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-11"


def test_qc12_axis_cmd_socket_not_fixed_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["axis_cmd_socket_fixed"] = False
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-12"


def test_qc16_single_tx_owner_off_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["single_tx_owner"] = False
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-16"


# ---------------------------------------------------------------------------
# QC-13 variant (1): legacy_decimal + empty table
# ---------------------------------------------------------------------------

def test_variant_1_legacy_decimal_empty_table_is_qc13_red(tmp_path):
    """CFG-FZ-12 variant (1) verbatim: codebook=legacy_decimal +
    empty codebook_table.legacy_decimal."""
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["codebook"] = "legacy_decimal"
    tree["quadruped"]["chassis_link"]["codebook_table"]["legacy_decimal"] = {}
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-13"
    # Every one of the five required rows is missing.
    assert set(ei.value.detail["missing"]) == {
        "heartbeat", "usage_mode_switch", "motion_state_switch",
        "gait_switch", "real_axis_cmd"}


def test_qc13_hex32_with_empty_table_is_legal(tmp_path):
    """hex32 does not require legacy_decimal table completeness."""
    # Baseline scaffold IS hex32 with empty table -- already tested
    # by test_green_scaffold_passes. Explicit here for clarity.
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["codebook"] = "hex32"
    tree["quadruped"]["chassis_link"]["codebook_table"]["legacy_decimal"] = {}
    assert run(_ctx(tree, tmp_path=tmp_path))["status"] == "pass"


def test_qc13_partial_table_with_legacy_decimal_is_red(tmp_path):
    """codebook=legacy_decimal but only 3 of 5 rows filled -> red."""
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["codebook"] = "legacy_decimal"
    tree["quadruped"]["chassis_link"]["codebook_table"]["legacy_decimal"] = {
        "heartbeat": "0x10", "usage_mode_switch": "0x20",
        "motion_state_switch": "0x30"}
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-13"


def test_qc13_unknown_codebook_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["codebook"] = "custom"
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-13"


# ---------------------------------------------------------------------------
# QC-14
# ---------------------------------------------------------------------------

def test_qc14_forward_true_empty_key_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["chassis_dds"]["forward_imu_to_rt"] = True
    tree["quadruped"]["chassis_dds"]["imu_rt_key"] = ""
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-14"
    assert ei.value.detail["reason"] == "empty_key"


def test_qc14_forward_true_unregistered_key_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["chassis_dds"]["forward_imu_to_rt"] = True
    tree["quadruped"]["chassis_dds"]["imu_rt_key"] = "rt/frobnicate/imu"
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-14"


def test_qc14_forward_true_registered_key_passes(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["chassis_dds"]["forward_imu_to_rt"] = True
    tree["quadruped"]["chassis_dds"]["imu_rt_key"] = "rt/chassis/imu"
    assert run(_ctx(tree, tmp_path=tmp_path))["status"] == "pass"


def test_qc14_default_forward_false_is_noop(tmp_path):
    """forward_imu_to_rt=false + imu_rt_key='' passes (default state)."""
    tree = _green_quadruped()   # already has forward=false, key=''
    assert run(_ctx(tree, tmp_path=tmp_path))["status"] == "pass"


# ---------------------------------------------------------------------------
# QC-15 variant (5): TLS creds absent
# ---------------------------------------------------------------------------

def test_variant_5_tls_enabled_missing_creds_is_qc15_red(tmp_path):
    """CFG-FZ-12 variant (5) verbatim: enabled TLS candidate but
    cred files absent."""
    tree = _green_quadruped()
    # Enable a TLS candidate. Point cred_dir at an empty dir on disk.
    cred_dir = tmp_path / "secrets" / "chassis_tls"
    cred_dir.mkdir(parents=True)
    tree["quadruped"]["chassis_link"]["endpoint_candidates"].insert(0, {
        "proto": "tcp", "host": "10.21.31.103", "port": 30003,
        "tls": True, "enabled": True,
    })
    tree["quadruped"]["chassis_link"]["tls"]["cred_dir"] = str(cred_dir)
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-15"
    assert ei.value.detail["reason"] == "no_credentials"


def test_qc15_tls_enabled_cred_dir_missing_is_red(tmp_path):
    """Enabled TLS + cred_dir string that does not exist on disk."""
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["endpoint_candidates"].insert(0, {
        "proto": "tcp", "host": "10.21.31.103", "port": 30003,
        "tls": True, "enabled": True,
    })
    tree["quadruped"]["chassis_link"]["tls"]["cred_dir"] = str(
        tmp_path / "nowhere")
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-15"
    assert ei.value.detail["reason"] == "cred_dir_absent"


def test_qc15_tls_enabled_cred_files_present_passes(tmp_path):
    """PSK pair present + 0o600 perms -> QC-15 passes."""
    tree = _green_quadruped()
    cred_dir = tmp_path / "secrets" / "chassis_tls"
    cred_dir.mkdir(parents=True)
    psk = cred_dir / "psk.hex"
    psk_id = cred_dir / "psk_identity"
    psk.write_text("deadbeef")
    psk_id.write_text("client")
    os.chmod(str(psk), 0o600)
    os.chmod(str(psk_id), 0o600)
    tree["quadruped"]["chassis_link"]["endpoint_candidates"].insert(0, {
        "proto": "tcp", "host": "10.21.31.103", "port": 30003,
        "tls": True, "enabled": True,
    })
    tree["quadruped"]["chassis_link"]["tls"]["cred_dir"] = str(cred_dir)
    assert run(_ctx(tree, tmp_path=tmp_path))["status"] == "pass"


def test_qc15_tls_enabled_cred_file_too_permissive_is_red(tmp_path):
    """PSK pair present but perms 0o644 -> QC-15 red on perm."""
    tree = _green_quadruped()
    cred_dir = tmp_path / "secrets" / "chassis_tls"
    cred_dir.mkdir(parents=True)
    for name in ("psk.hex", "psk_identity"):
        p = cred_dir / name
        p.write_text("x")
        os.chmod(str(p), 0o644)   # too permissive
    tree["quadruped"]["chassis_link"]["endpoint_candidates"].insert(0, {
        "proto": "tcp", "host": "10.21.31.103", "port": 30003,
        "tls": True, "enabled": True,
    })
    tree["quadruped"]["chassis_link"]["tls"]["cred_dir"] = str(cred_dir)
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-15"
    assert ei.value.detail["reason"] == "perm_too_loose"


def test_qc15_tls_disabled_candidate_skips_check(tmp_path):
    """A tls:true / enabled:false candidate is not enforced."""
    tree = _green_quadruped()
    tree["quadruped"]["chassis_link"]["endpoint_candidates"].insert(0, {
        "proto": "tcp", "host": "10.21.31.103", "port": 30003,
        "tls": True, "enabled": False,   # disabled -> no check
    })
    assert run(_ctx(tree, tmp_path=tmp_path))["status"] == "pass"


# ---------------------------------------------------------------------------
# QC-17
# ---------------------------------------------------------------------------

def test_qc17_missing_stair_standard_is_red(tmp_path):
    tree = _green_quadruped()
    tree["quadruped"]["motion"]["not_implemented"]["gaits"] = []
    with pytest.raises(XbrainError) as ei:
        run(_ctx(tree, tmp_path=tmp_path))
    assert ei.value.detail["rule"] == "QC-17"


# ---------------------------------------------------------------------------
# Wiring guard
# ---------------------------------------------------------------------------

def test_requires_config_root():
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
