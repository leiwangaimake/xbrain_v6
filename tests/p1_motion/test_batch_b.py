"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_b.py
Brief: MOT-PM-16..25 batch B tests (odom + path + nav2 + teleop + config)

Description:
Ten modules landed as P1 batch B. Each covers a single MOT-PM item;
tests focus on the spec's named variants: pose_assembly PS-4 byte-
for-byte identity, path_follow LP-3a loops=0 no auto-arrive,
nav2 double-gate (the MOT-PM-18 relative_move executor tests left with the module)
PG-2, teleop TL-1/TL-2 estop-before-normalize, cloud teleop vy
reject + link-down zero, target_oriented no-default schema,
hello_ack version mismatch refusal, config forbidden alias.
"""

import pytest

from xbrain.p1_motion.config.loader import (
    P1SelfcheckError,
    check_no_alias_keys,
    check_rcg_constants,
)
from xbrain.p1_motion.handshake.hello import (
    HandshakeError,
    build_hello,
    build_hello_ack,
    validate_hello_ack,
)
from xbrain.p1_motion.path.nav2_proxy import (
    DoubleGate,
    VerifyState,
    can_correct,
    consume_correction,
    needs_correction,
)
from xbrain.p1_motion.path.pose_assembly import (
    MotionSnapshot,
    to_cmd_vel_gate,
    to_pose_motion,
)
from xbrain.p1_motion.teleop.four_source import (
    TeleopFrame,
    TeleopSource,
    is_fresh,
    parse_estop_first,
)
from xbrain.p1_motion.teleop.teleop_cloud import (
    CloudTeleopFrame,
    CloudTeleopReject,
    clamp_and_check,
)

pytestmark = pytest.mark.no_device


# --- MOT-PM-16 pose_assembly PS-4 ---

def test_ps4_cmd_vel_and_pose_are_byte_identical():
    """PS-4: same MotionSnapshot serialised twice yields identical
    dicts. A drift here would leak health_factor updates to only
    one side."""
    snap = MotionSnapshot(
        vx_mps=1.0, wz_radps=0.2,
        speed_factor=0.5, limiter="f_speed",
        heading_deg=90.0, h_factor=0.8, rtk_factor=1.0, gen=42,
    )
    assert to_cmd_vel_gate(snap) == to_pose_motion(snap)


# --- MOT-PM-17 path_follow: source deleted 2026-09-28 (PM1.3 ruling,
# docs/rns-legacy-audit.md S4 verbatim "删 (折线跟随归 RNS route.py,
# P1.3/P1.8 接管其路径指针消费)", 12 v0.8 #20-1). The three LP tests that
# lived here went with it: sources/arbiter_p1.py has had no PATH_FOLLOW
# member since PM1.3, so they pinned a behaviour source that cannot win
# the P1 output slot.

# --- MOT-PM-18 relative_move executor: removed 2026-09-13 (dead since #20-9; the
# relmove path is nav/relmove_intake.py + RNS goto, abort_reason seven values in
# nav/report_map.py, 11 S9.3.2A.6) ---

# --- MOT-PM-19 nav2 double-gate PG-2 ---

def test_pg2_missing_cmd_id_rejects():
    """PG-2: a cmd_vel without cmd_id is treated as unmatched."""
    g = DoubleGate(expected_cmd_id="c1", expected_gen=42)
    assert g.accept(frame_cmd_id=None, frame_gen=42) is False


def test_pg2_matching_pair_accepts():
    g = DoubleGate(expected_cmd_id="c1", expected_gen=42)
    assert g.accept(frame_cmd_id="c1", frame_gen=42) is True


def test_pg2_wrong_gen_rejects():
    g = DoubleGate(expected_cmd_id="c1", expected_gen=42)
    assert g.accept(frame_cmd_id="c1", frame_gen=41) is False


# --- MOT-PM-20 verify state ---

def test_verify_correction_countdown():
    vs = VerifyState(corrections_left=2, tolerance_deg=3.0)
    assert needs_correction(4.0, 3.0)
    assert can_correct(vs)
    consume_correction(vs)
    consume_correction(vs)
    assert not can_correct(vs)


# --- MOT-PM-21 teleop estop-first ---

def test_estop_parses_from_corrupt_frame():
    """TL-1/TL-2: even a corrupt-body frame with estop bit set MUST
    still fire the stop path."""
    corrupt = TeleopFrame(source=TeleopSource.KEYBOARD_LOCAL,
                           raw_bytes=b"\x01",    # only estop byte
                           arrived_mono_ms=0)
    r = parse_estop_first(corrupt)
    assert r.estop_asserted is True
    assert r.raw_ok is False   # rest of body did NOT parse cleanly


def test_estop_bit_absent_on_normal_frame():
    frame = TeleopFrame(source=TeleopSource.KEYBOARD_LOCAL,
                         raw_bytes=b"\x00\x01\x02\x03",
                         arrived_mono_ms=0)
    r = parse_estop_first(frame)
    assert r.estop_asserted is False
    assert r.raw_ok is True


def test_teleop_source_is_exactly_the_contract_closed_set():
    """11 S12A.9.7 closes the device set at four values.

    Pinned as a SET equality, not member by member: a fifth member
    added here would otherwise pass, and a source with no priority and
    no deadline in S12A.9.6/9.7 is one the arbitration cannot rank.
    `none` is deliberately absent -- S12A.9.7 makes it an active_source
    sentinel, not a member of sources[].

    Until 2026-09-30 this module carried keyboard / joystick / hmi /
    cloud, none of which is in the set, and nothing was red.

    MUTATION: rename any member value back (e.g. KEYBOARD_LOCAL ->
    "keyboard") -> red.
    """
    assert {s.value for s in TeleopSource} == {
        "gamepad", "keyboard_local", "keyboard_hmi", "virtual_stick"}


def test_tl3_freshness_is_200_local_and_400_network():
    """TL-3 / 11 S12A.9.6: local rt/teleop/input 200 ms, HMI cmd/teleop
    400 ms. 11 S13 E_TELEOP_STALE repeats the same pair verbatim.

    Both edges are asserted for each link, so a deadline that is too
    LONG fails too -- that is the direction that matters: an expired
    source left in the arbitration keeps the robot driving on the last
    frame it got before the link died. The module used to give the HMI
    500 ms and a `cloud` source 1000 ms; 12 S4.7.1 TL-3 carries the
    500 ms reading struck through as 已作废 for exactly this reason.

    MUTATION: put keyboard_hmi back to 500 -> the 401 ms assertion
    below goes red.
    """
    for local in (TeleopSource.GAMEPAD, TeleopSource.KEYBOARD_LOCAL):
        f = TeleopFrame(local, b"\x00", 0)
        assert is_fresh(f, now_mono_ms=200) is True
        assert is_fresh(f, now_mono_ms=201) is False
    for net in (TeleopSource.KEYBOARD_HMI, TeleopSource.VIRTUAL_STICK):
        f = TeleopFrame(net, b"\x00", 0)
        assert is_fresh(f, now_mono_ms=400) is True
        assert is_fresh(f, now_mono_ms=401) is False


# --- MOT-PM-22 teleop_cloud vy reject + link-down ---

def test_cloud_vy_nonzero_rejected():
    f = CloudTeleopFrame(vx_mps=0.3, vy_mps=0.1, wz_radps=0.0,
                          arrived_mono_ms=0)
    with pytest.raises(CloudTeleopReject):
        clamp_and_check(f, now_mono_ms=0)


def test_cloud_link_down_forces_zero():
    """Link stale > 1 s -> all zero (fail-safe)."""
    f = CloudTeleopFrame(vx_mps=1.0, vy_mps=0, wz_radps=1.0,
                          arrived_mono_ms=0)
    result = clamp_and_check(f, now_mono_ms=2000)
    assert result == (0.0, 0.0, 0.0)


def test_cloud_vx_clamped_to_obstacle_avoid_max():
    f = CloudTeleopFrame(vx_mps=5.0, vy_mps=0, wz_radps=0,
                          arrived_mono_ms=0)
    vx, vy, wz = clamp_and_check(f, now_mono_ms=100,
                                   obstacle_avoid_max_mps=0.5)
    assert vx == 0.5


def test_cloud_wz_clipped_to_wz_blind():
    f = CloudTeleopFrame(vx_mps=0, vy_mps=0, wz_radps=2.0,
                          arrived_mono_ms=0)
    vx, vy, wz = clamp_and_check(f, now_mono_ms=100,
                                   wz_blind_radps=0.5)
    assert wz == 0.5


# --- MOT-PM-23 target_oriented: source deleted 2026-09-28 (PM1.3 ruling,
# same table row, verbatim "删 (并入 RNS follow_target, 本期预留 #20-13 --
# 删源不删预留位)", #20-9). The four mode tests went with it. The RESERVED
# SLOT is untouched: #20-13 keeps follow_target as an RNS behaviour, and
# nothing in this commit touches that reservation.

# --- MOT-PM-24 hello handshake ---

def test_hello_wire_shape():
    h = build_hello()
    assert h == {"type": "hello", "proto_version": "1.0",
                  "client": "p1_motion"}


def test_hello_ack_valid_passes():
    validate_hello_ack(build_hello_ack())


def test_hello_ack_version_mismatch_raises():
    """Refuse startup on version mismatch; no silent upgrade."""
    bad = {"type": "hello_ack", "proto_version": "2.0",
           "server": "quadruped"}
    with pytest.raises(HandshakeError) as ei:
        validate_hello_ack(bad)
    assert "proto_version" in str(ei.value)


def test_hello_ack_wrong_server_raises():
    with pytest.raises(HandshakeError):
        validate_hello_ack({
            "type": "hello_ack", "proto_version": "1.0",
            "server": "impostor",
        })


# --- MOT-PM-25 config forbidden alias ---

def test_alias_keep_dist_m_in_p1_config_raises():
    """keep_dist_m belongs to p2_core.yaml.mode_motion; presence in
    p1_motion.yaml is a duplicate-truth defect refused at startup."""
    cfg = {"target_oriented": {"keep_dist_m": 1.0}}
    with pytest.raises(P1SelfcheckError) as ei:
        check_no_alias_keys(cfg)
    assert "keep_dist_m" in str(ei.value)


def test_alias_check_ok_when_clean():
    check_no_alias_keys({"rns": {"corridor": {"lambda_len": 0.5}}})


def test_rcg_constants_missing_raises():
    with pytest.raises(P1SelfcheckError):
        check_rcg_constants({"rns": {}})    # rcg block missing
    with pytest.raises(P1SelfcheckError):
        check_rcg_constants({"rns": {"rcg": {}}})   # r_eff_fallback_m missing


def test_rcg_constants_present_ok():
    check_rcg_constants({"rns": {"rcg": {"r_eff_fallback_m": 0.6}}})
