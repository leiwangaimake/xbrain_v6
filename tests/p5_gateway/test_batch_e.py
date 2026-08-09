"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_e.py
Brief: GWY-P5-17/18/19/20/21/22/23/09/24 yaml + lifecycle + HMI data + PTZ + media + FTP + QoS

Description:
Batch E: bind assertions (NET-C9 no 0.0.0.0, port range, no port
reuse); gateway lifecycle SM allowed transitions; minimal-mode
allowlist for W-1 window; HMI data groups closed set; UI rotation
disable on either denial category; PTZ HOLD_FIRST -> REPEAT_ARMED
-> HOLD_REPEAT; 5 Hz lease renewal fixed; media file existence
required before submit; FTP write verbs refused; DSCP DSCP_EF /
AF41 / AF31 per plane; disk watermark policy soft<hard.
"""

import os
import tempfile

import pytest

from xbrain.p5_gateway.config.assertions import (
    FreezeAssertionFailure,
    check_bind_entries, check_bind_no_port_reuse, is_pending_key,
)
from xbrain.p5_gateway.ftp.vsftpd import (
    DiskWatermark, DiskWatermarkPolicy, FtpWriteForbidden,
    UnknownPlane, check_verb_read_only, dscp_for,
)
from xbrain.p5_gateway.hmi.data_sets import (
    HMI_DATA_GROUPS, UiRotationState, UnknownDataGroup,
    redact_ptz_angle_for_ui, validate_group,
)
from xbrain.p5_gateway.lifecycle.state_machine import (
    GatewayState, InvalidGatewayTransition,
    minimal_mode_allows, transition,
)
from xbrain.p5_gateway.media.reference import (
    MediaFileMissing, MediaRef, VALID_MEDIA_KINDS,
    validate_kind, verify_file_exists_before_submit,
)
from xbrain.p5_gateway.ptz.state_machine import (
    InvalidPtzTransition, LEASE_RENEWAL_HZ, PtzButtonState,
    lease_renewal_period_ms, transition as ptz_transition,
)


pytestmark = pytest.mark.no_device


# --- GWY-P5-17 config assertions ---

def test_bind_rejects_0_0_0_0():
    with pytest.raises(FreezeAssertionFailure, match="0.0.0.0"):
        check_bind_entries(["0.0.0.0:8080"])


def test_bind_rejects_reserved_port():
    with pytest.raises(FreezeAssertionFailure, match="out of range"):
        check_bind_entries(["127.0.0.1:80"])


def test_bind_accepts_explicit_host():
    check_bind_entries(["127.0.0.1:8080", "hmi.local:9000"])


def test_bind_no_port_reuse_rejects():
    with pytest.raises(FreezeAssertionFailure, match="reused"):
        check_bind_no_port_reuse({
            "hmi": ["127.0.0.1:8080"],
            "rest": ["localhost:8080"],
        })


def test_bind_no_port_reuse_disjoint_ok():
    check_bind_no_port_reuse({
        "hmi": ["127.0.0.1:8080"],
        "rest": ["localhost:8081"],
    })


def test_pending_key_membership():
    assert is_pending_key("configs.hmi.some_key",
                            ["configs.hmi.some_key"]) is True
    assert is_pending_key("configs.hmi.other", ["configs.hmi.some_key"]) is False


# --- GWY-P5-18 lifecycle SM ---

def test_lifecycle_starting_to_minimal_ok():
    transition(GatewayState.STARTING, GatewayState.MINIMAL)


def test_lifecycle_starting_cant_jump_to_full():
    with pytest.raises(InvalidGatewayTransition):
        transition(GatewayState.STARTING, GatewayState.FULL)


def test_lifecycle_full_to_stopping_ok():
    transition(GatewayState.FULL, GatewayState.STOPPING)


def test_lifecycle_stopped_is_terminal():
    with pytest.raises(InvalidGatewayTransition):
        transition(GatewayState.STOPPED, GatewayState.FULL)


def test_minimal_mode_only_allows_whitelist():
    assert minimal_mode_allows("api_health") is True
    assert minimal_mode_allows("link_probe") is True
    assert minimal_mode_allows("cmd_teleop") is False


# --- GWY-P5-19/21 HMI data ---

def test_hmi_data_groups_are_a_through_f():
    assert HMI_DATA_GROUPS == {"A", "B", "C", "D", "E", "F"}


def test_validate_group_ok():
    validate_group("A")


def test_validate_group_unknown_rejected():
    with pytest.raises(UnknownDataGroup):
        validate_group("G")


def test_rotate_button_disabled_on_either_denial():
    """Both denial kinds shown separately, but the button collapses
    to disabled when either is set."""
    assert UiRotationState(True, False).rotate_button_enabled() is False
    assert UiRotationState(False, True).rotate_button_enabled() is False
    assert UiRotationState(False, False).rotate_button_enabled() is True


def test_ptz_angle_redacted_in_ui():
    payload = {"pan_deg": 45.0, "tilt_deg": -10.0, "zoom": 3.0}
    ui = redact_ptz_angle_for_ui(payload)
    assert ui["pan_deg"] is None and ui["tilt_deg"] is None
    assert ui["zoom"] == 3.0
    # State payload unchanged.
    assert payload["pan_deg"] == 45.0


# --- GWY-P5-22 PTZ SM ---

def test_ptz_idle_to_hold_first_ok():
    ptz_transition(PtzButtonState.IDLE, PtzButtonState.HOLD_FIRST)


def test_ptz_idle_cant_jump_to_hold_repeat():
    with pytest.raises(InvalidPtzTransition):
        ptz_transition(PtzButtonState.IDLE, PtzButtonState.HOLD_REPEAT)


def test_ptz_lease_renewal_is_5hz():
    """Fixed 200 ms period (5 Hz)."""
    assert LEASE_RENEWAL_HZ == 5
    assert lease_renewal_period_ms() == 200


# --- GWY-P5-23 media reference ---

def test_valid_media_kinds_frozen_set():
    assert VALID_MEDIA_KINDS == frozenset({"photo", "video", "audio"})


def test_media_kind_unknown_rejected():
    with pytest.raises(ValueError, match="not in"):
        validate_kind("halfway")


def test_media_file_missing_before_submit_raises():
    ref = MediaRef(path="/nonexistent/photo.jpg", kind="photo", bytes_size=0)
    with pytest.raises(MediaFileMissing):
        verify_file_exists_before_submit(ref)


def test_media_file_exists_before_submit_ok():
    with tempfile.NamedTemporaryFile(delete=False) as f:
        path = f.name
    try:
        ref = MediaRef(path=path, kind="photo", bytes_size=0)
        verify_file_exists_before_submit(ref)
    finally:
        os.unlink(path)


# --- GWY-P5-09/24 FTP + QoS ---

def test_dscp_lookup_per_plane():
    assert dscp_for("control") == 46
    assert dscp_for("data") == 34
    assert dscp_for("media") == 26


def test_dscp_unknown_plane_raises():
    with pytest.raises(UnknownPlane):
        dscp_for("halfway")


def test_ftp_read_verbs_ok():
    for v in ("LIST", "RETR", "PWD", "TYPE"):
        check_verb_read_only(v)


def test_ftp_write_verbs_refused():
    for v in ("STOR", "DELE", "RNFR", "RNTO", "MKD", "RMD"):
        with pytest.raises(FtpWriteForbidden):
            check_verb_read_only(v)


def test_watermark_policy_soft_lt_hard():
    DiskWatermarkPolicy(soft_pct=80.0, hard_pct=90.0)


def test_watermark_policy_inverted_rejected():
    with pytest.raises(ValueError):
        DiskWatermarkPolicy(soft_pct=90.0, hard_pct=80.0)


def test_watermark_allow_write_below_soft():
    w = DiskWatermark(DiskWatermarkPolicy(80.0, 90.0))
    assert w.allow_write(50.0) is True
    assert w.allow_write(85.0) is False


def test_watermark_health_critical_at_hard():
    w = DiskWatermark(DiskWatermarkPolicy(80.0, 90.0))
    assert w.health_critical(89.9) is False
    assert w.health_critical(90.0) is True
