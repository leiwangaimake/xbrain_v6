"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chassis_lock_w9.py
Brief: HMI chassis-lock button backend (99 U159) -- W9 unlock frame + lock read-back

Description:
The HMI chassis-lock button has two halves on the P5 side, and this file pins
both:

  * W9 chassis_enable (11 S12.1.1, 99 U159) -> 11 S9.3.4
    cmd/chassis/ctrl{action: enable}. enable is a LOOSENING command, so E-1's
    L2 confirm is required; origin is stamped "hmi" (CH-2) and the
    confirm_token is the prefixed req_id. The built body must pass P2's
    BOOT-L1 gate (validate_unlock_request), so the pair is asserted, not just
    the builder.
  * The colour of the button: status.chassis_lock in the snapshot, read back
    from state/robot as hes_lock OR timeout_lock (11 S4.1 / S9.12.2). The
    derived `locked` field is compat-only and is ignored; a missing or stale
    state/robot is "unknown", never "unlocked".

What it does not cover: the S3.0 envelope around the published body lives in
the P5 wiring closure (runtime/main_wiring _send_uplink) and is exercised on
the robot, not here; the frontend states are in test_chassis_lock_view_js.py.
"""

from __future__ import annotations

import pytest

from xbrain.common.errors import E_CONFIRM_REQUIRED
from xbrain.p2_core.boot.timeout_lock import validate_unlock_request
from xbrain.p5_gateway.hmi import data_readers, uplink
from xbrain.p5_gateway.runtime import main_wiring


def _frame(**kw):
    frame = {"type": "chassis_enable", "req_id": "u1"}
    frame.update(kw)
    return frame


# -- W9 frame ---------------------------------------------------------------

def test_w9_is_in_the_whitelist():
    # RED MUTANT: drop "chassis_enable" from UPSTREAM_TYPES -> parse_envelope
    # refuses every click with E_SCHEMA and the button can never unlock.
    assert uplink.parse_envelope(_frame())["type"] == "chassis_enable"


def test_unlock_without_l2_confirm_is_refused():
    # 11 S9.3.4 E-1. RED MUTANT: skip the _has_confirm check -> a single
    # stray click unlocks a robot that locked itself after losing upstream.
    ref = uplink.build_chassis_enable_command(_frame())
    assert isinstance(ref, uplink.UplinkRefusal)
    assert ref.code == E_CONFIRM_REQUIRED
    ref = uplink.build_chassis_enable_command(_frame(confirm={"level": "L1"}))
    assert isinstance(ref, uplink.UplinkRefusal)


def test_unlock_builds_the_s934_enable_body():
    built = uplink.build_chassis_enable_command(_frame(confirm={"level": "L2"}))
    assert isinstance(built, uplink.UplinkCommand)
    assert built.key == "cmd/chassis/ctrl"
    assert built.req_type == "chassis_enable"
    assert built.payload["action"] == "enable"
    assert built.payload["cmd_id"] == "h-u1"          # routes the ack back
    assert built.payload["confirm_token"] == "h-u1"   # S9.3.4: required for enable


def test_origin_is_stamped_hmi_even_when_the_frame_claims_cloud():
    # CH-2: the unauthenticated browser cannot relabel itself.
    # RED MUTANT: copy origin from the frame -> "cloud" goes out.
    built = uplink.build_chassis_enable_command(
        _frame(confirm={"level": "L2"}, origin="cloud"))
    assert built.payload["origin"] == "hmi"


def test_built_body_passes_p2s_boot_l1_gate():
    # The pair, not just the builder: P2's BOOT-L1 gate refuses an enable
    # without a confirm_token. RED MUTANT: drop confirm_token -> refused.
    built = uplink.build_chassis_enable_command(_frame(confirm={"level": "L2"}))
    verdict = validate_unlock_request(built.payload, "CR-11")
    assert verdict.accepted, verdict.reason


# -- lock read-back ---------------------------------------------------------

@pytest.mark.parametrize("hes_lock, timeout_lock, locked", [
    (False, False, False),
    (False, True, True),      # upstream lost (the common case after a restart)
    (True, False, True),      # hardware stop
    (True, True, True),
])
def test_locked_is_either_tier1_lock(hes_lock, timeout_lock, locked):
    # RED MUTANT: locked = timeout_lock only -> a pressed hardware stop shows
    # green "unlocked".
    g = data_readers.chassis_lock_group(
        {"hes_lock": hes_lock, "timeout_lock": timeout_lock, "hes": hes_lock})
    assert g["available"] is True and g["locked"] is locked


def test_derived_locked_field_is_not_trusted():
    # 11 S4.1 marks `locked` compat-only. RED MUTANT: read robot["locked"]
    # -> this frame (locked:false but timeout_lock:true) shows unlocked.
    g = data_readers.chassis_lock_group(
        {"locked": False, "hes_lock": False, "timeout_lock": True})
    assert g["locked"] is True


@pytest.mark.parametrize("robot", [
    None,                                         # never arrived / stale
    {},                                           # empty body
    {"hes_lock": False},                          # one lock missing
    {"hes_lock": "no", "timeout_lock": False},    # not a boolean
])
def test_missing_read_back_is_unknown_not_unlocked(robot):
    # RED MUTANT: default a missing lock to False -> available:true,
    # locked:false -> the button paints green over an unknown chassis.
    g = data_readers.chassis_lock_group(robot)
    assert g["available"] is False and g["locked"] is None


def test_snapshot_carries_the_lock_in_the_status_group():
    snap = data_readers.build_snapshot(
        robot={"hes_lock": False, "timeout_lock": True, "hes": False})
    assert snap["status"]["chassis_lock"]["locked"] is True
    assert data_readers.build_snapshot()["status"]["chassis_lock"]["available"] is False


def test_stale_state_robot_is_dropped():
    # RED MUTANT: return the cached robot whatever its age -> a dead
    # chassis_relay leaves the last "unlocked" on screen.
    robot = {"hes_lock": False, "timeout_lock": False}
    st = {"robot": robot, "robot_updated_ms": 1000}
    assert main_wiring._robot_if_fresh(st, 1000 + main_wiring.ROBOT_STALE_AFTER_MS) is robot
    assert main_wiring._robot_if_fresh(st, 1001 + main_wiring.ROBOT_STALE_AFTER_MS) is None
    assert main_wiring._robot_if_fresh({"robot": None}, 0) is None
