"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_b.py
Brief: MOT-PM-16..25 batch B tests (odom + path + relative_move + nav2 + teleop + config)

Description:
Ten modules landed as P1 batch B. Each covers a single MOT-PM item;
tests focus on the spec's named variants: pose_assembly PS-4 byte-
for-byte identity, path_follow LP-3a loops=0 no auto-arrive,
relative_move wire enum vs internal enum split, nav2 double-gate
PG-2, teleop TL-1/TL-2 estop-before-normalize, cloud teleop vy
reject + link-down zero, target_oriented no-default schema,
hello_ack version mismatch refusal, config forbidden alias.
"""

import pytest

from xbrain.p1_motion.config.loader import (
    FORBIDDEN_ALIAS_KEYS, P1SelfcheckError,
    check_no_alias_keys, check_rcg_constants,
)
from xbrain.p1_motion.handshake.hello import (
    HandshakeError, PROTO_VERSION, build_hello, build_hello_ack,
    validate_hello_ack,
)
from xbrain.p1_motion.path.nav2_proxy import (
    DoubleGate, VerifyState, can_correct, consume_correction,
    needs_correction,
)
from xbrain.p1_motion.path.path_follow import (
    LoopState, PathFollowConfig, PathFollowState,
    advance_waypoint, is_arrived, pure_pursuit_target,
)
from xbrain.p1_motion.path.pose_assembly import (
    MotionSnapshot, to_cmd_vel_gate, to_pose_motion,
)
from xbrain.p1_motion.path.relative_move import (
    ABORT_REASONS, RelativeMoveError, WIRE_STATE_MAP,
    TrapezoidProfile, _InternalState, to_wire_state, validate_abort_reason,
)
from xbrain.p1_motion.path.target_oriented import (
    SchemaError, TargetOrientedParams, compute_face_target,
)
from xbrain.p1_motion.teleop.four_source import (
    ParsedEstop, TeleopFrame, TeleopSource,
    is_fresh, parse_estop_first,
)
from xbrain.p1_motion.teleop.teleop_cloud import (
    CloudTeleopFrame, CloudTeleopReject, clamp_and_check,
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


# --- MOT-PM-17 path_follow LP-3a ---

def test_lp3a_loops_zero_never_arrives():
    """LP-3a: infinite loop (loops_total=0) MUST NEVER return arrived."""
    cfg = PathFollowConfig(waypoints=[(0, 0)], loops_total=0, lookahead_m=0.5)
    st = PathFollowState()
    # Advance many "traversals"; still not arrived.
    for _ in range(100):
        advance_waypoint(st, cfg)
    assert is_arrived(st, cfg) is False


def test_lp4_finite_loops_arrives_after_n():
    cfg = PathFollowConfig(waypoints=[(0, 0), (1, 0)],
                            loops_total=2, lookahead_m=0.5)
    st = PathFollowState()
    for _ in range(4):
        advance_waypoint(st, cfg)
    assert is_arrived(st, cfg) is True


def test_target_none_when_arrived():
    cfg = PathFollowConfig(waypoints=[(0, 0)], loops_total=1, lookahead_m=0.5)
    st = PathFollowState(loop_index=1)
    assert pure_pursuit_target(st, cfg, 0, 0) is None


# --- MOT-PM-18 relative_move enum split ---

def test_wire_enum_is_lowercase():
    """Serialised state names MUST be 5-value lowercase; UPPERCASE
    internal names must never leak out."""
    for s in _InternalState:
        wire = to_wire_state(s)
        assert wire.islower()
        assert wire in {"accepted", "running", "arrived", "aborted", "timeout"}


def test_abort_reason_closed_set_enforced():
    for r in ABORT_REASONS:
        validate_abort_reason(r)   # must not raise
    with pytest.raises(RelativeMoveError):
        validate_abort_reason("custom_reason_that_isnt_registered")


def test_trapezoid_triangular_when_short():
    p = TrapezoidProfile(v_max_mps=2.0, a_max_mps2=1.0, d_target_m=1.0)
    t_accel, t_cruise, t_decel = p.phase_lengths()
    # d_accel = 2 -> triangular; t_cruise == 0.
    assert t_cruise == 0.0


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
    corrupt = TeleopFrame(source=TeleopSource.KEYBOARD,
                           raw_bytes=b"\x01",    # only estop byte
                           arrived_mono_ms=0)
    r = parse_estop_first(corrupt)
    assert r.estop_asserted is True
    assert r.raw_ok is False   # rest of body did NOT parse cleanly


def test_estop_bit_absent_on_normal_frame():
    frame = TeleopFrame(source=TeleopSource.KEYBOARD,
                         raw_bytes=b"\x00\x01\x02\x03",
                         arrived_mono_ms=0)
    r = parse_estop_first(frame)
    assert r.estop_asserted is False
    assert r.raw_ok is True


def test_tl3_freshness_per_source():
    kb = TeleopFrame(TeleopSource.KEYBOARD, b"\x00", 0)
    assert is_fresh(kb, now_mono_ms=150) is True
    assert is_fresh(kb, now_mono_ms=250) is False
    hmi = TeleopFrame(TeleopSource.HMI, b"\x00", 0)
    assert is_fresh(hmi, now_mono_ms=400) is True     # HMI gets 500 ms
    cloud = TeleopFrame(TeleopSource.CLOUD, b"\x00", 0)
    assert is_fresh(cloud, now_mono_ms=900) is True   # cloud gets 1000 ms


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


# --- MOT-PM-23 target_oriented no-default ---

def test_target_oriented_hold_returns_zeros():
    p = TargetOrientedParams(keep_dist_m=1.0, max_speed_mps=0.5,
                              stop_at_fence=False)
    assert compute_face_target(2.0, 0.0, p, "hold") == (0.0, 0.0, 0.0)


def test_target_oriented_stop_only_wz():
    p = TargetOrientedParams(keep_dist_m=1.0, max_speed_mps=0.5,
                              stop_at_fence=False)
    vx, vy, wz = compute_face_target(2.0, 0.0, p, "face_target_stop")
    assert vx == 0.0
    assert vy == 0.0


def test_target_oriented_follow_moves_toward():
    p = TargetOrientedParams(keep_dist_m=1.0, max_speed_mps=0.5,
                              stop_at_fence=False)
    vx, vy, wz = compute_face_target(3.0, 0.0, p, "face_target_follow")
    assert vx > 0    # target further than keep_dist -> move toward


def test_target_oriented_unknown_mode_raises():
    p = TargetOrientedParams(keep_dist_m=1.0, max_speed_mps=0.5,
                              stop_at_fence=False)
    with pytest.raises(SchemaError):
        compute_face_target(1, 0, p, "magic_mode")


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
