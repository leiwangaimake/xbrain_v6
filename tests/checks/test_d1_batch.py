"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_d1_batch.py
Brief: D-1 batch INF-BT-1/3 + CHK-2-11/19/24 tests

Description:
INF-BT-1 five stages + BOOT-I2 (initial factor is hard-zero,
never grace); INF-BT-3 timeout_lock readback discipline;
CHK-2-11 motion throttle + AI-74 ctx cap; CHK-2-19 per-link
health item mapping + LNK-D non-item; CHK-2-24 gpu/dla
fail-not-unknown.
"""

from __future__ import annotations

import pytest

from xbrain.common.errors import E_CHANNEL_DENIED, E_CONFIRM_REQUIRED
from xbrain.p2_core.bit.gpu_dla import (
    BitAssertViolation, BitItemState, Pc3Violation,
    assert_no_pc3_key, assert_no_unknown_state_in_source,
    evaluate_dla, evaluate_gpu,
)
from xbrain.p2_core.bit.network_links import (
    LNK_D_DETAIL_ONLY, LinkClassificationError, LinkDetail,
    OnvifInBitPath,
    assert_no_onvif_in_bit_source, classify_link_failure,
    is_link_healthy, lnk_d_writes_detail_only,
)
from xbrain.p2_core.boot.stage_machine import (
    BootFailure, BootI2Violation, BootStage, BootStageMachine,
    InvalidBootTransition, MotionFactor,
    check_boot_i2_initial, initial_motion_factor,
)
from xbrain.p2_core.boot.timeout_lock import (
    ALLOWED_UNLOCK_CHANNELS, HesLockConflation, TimeoutLockGate,
    assert_locks_are_separate, validate_unlock_request,
)
from xbrain.p4_agent.throttle.motion_gate import (
    CtxCapViolation, LlmRequestRefused, MotionThrottleConfigError,
    PromptSections, ThrottleConfig,
    admit_llm_request, should_throttle, trim_to_cap,
)


pytestmark = pytest.mark.no_device


# ---------------- INF-BT-1 stage machine ----------------

def test_initial_motion_factor_is_hard_zero():
    """BOOT-I2 initial value: not True, not 1.0, not 'reasonable'."""
    mf = initial_motion_factor()
    assert mf.allow_motion is False
    assert mf.speed_factor == 0.0
    assert mf.v_max_mps == 0.0


def test_check_boot_i2_initial_rejects_permissive():
    """A caller synthesising an 'open' initial factor is rejected."""
    for bad in (
            MotionFactor(allow_motion=True, speed_factor=0.0, v_max_mps=0.0),
            MotionFactor(allow_motion=False, speed_factor=1.0, v_max_mps=0.0),
            MotionFactor(allow_motion=False, speed_factor=0.0, v_max_mps=2.0)):
        with pytest.raises(BootI2Violation):
            check_boot_i2_initial(bad)


def test_check_boot_i2_initial_accepts_hard_zero():
    check_boot_i2_initial(initial_motion_factor())


def test_stage_a_to_b_ok():
    sm = BootStageMachine()
    sm.transition(BootStage.STAGE_B)
    assert sm.stage == BootStage.STAGE_B


def test_stage_a_cant_jump_to_d():
    sm = BootStageMachine()
    with pytest.raises(InvalidBootTransition):
        sm.transition(BootStage.STAGE_D)


def test_stage_c_fatal_fail_blocks():
    sm = BootStageMachine(stage=BootStage.STAGE_C)
    sm.enter_stage_c_result(any_fatal_fail=True,
                              timeout_lock=False,
                              common_digest_mismatch=False)
    assert sm.stage == BootStage.BLOCKED
    assert any(f.item == "bit" for f in sm.blocked_reasons)


def test_stage_c_timeout_lock_blocks():
    sm = BootStageMachine(stage=BootStage.STAGE_C)
    sm.enter_stage_c_result(any_fatal_fail=False,
                              timeout_lock=True,
                              common_digest_mismatch=False)
    assert sm.stage == BootStage.BLOCKED
    assert any(f.item == "chassis" for f in sm.blocked_reasons)


def test_stage_c_digest_mismatch_blocks():
    sm = BootStageMachine(stage=BootStage.STAGE_C)
    sm.enter_stage_c_result(any_fatal_fail=False,
                              timeout_lock=False,
                              common_digest_mismatch=True)
    assert sm.stage == BootStage.BLOCKED


def test_stage_c_all_clear_promotes_to_d():
    sm = BootStageMachine(stage=BootStage.STAGE_C)
    sm.enter_stage_c_result(any_fatal_fail=False,
                              timeout_lock=False,
                              common_digest_mismatch=False)
    assert sm.stage == BootStage.STAGE_D


def test_blocked_reasons_shown_for_hmi():
    sm = BootStageMachine(stage=BootStage.STAGE_C)
    sm.enter_stage_c_result(any_fatal_fail=True, timeout_lock=True,
                              common_digest_mismatch=True)
    kinds = {r.item for r in sm.blocked_reasons}
    assert kinds == {"bit", "chassis", "common_digest"}


def test_blocked_is_sink():
    sm = BootStageMachine()
    sm.block([BootFailure(item="test", reason="test")])
    with pytest.raises(InvalidBootTransition):
        sm.transition(BootStage.STAGE_A)


def test_can_publish_only_in_stage_d():
    sm = BootStageMachine()
    assert sm.can_publish_motion_factor() is False
    sm.transition(BootStage.STAGE_B)
    assert sm.can_publish_motion_factor() is False
    sm.transition(BootStage.STAGE_C)
    assert sm.can_publish_motion_factor() is False
    sm.enter_stage_c_result(any_fatal_fail=False, timeout_lock=False,
                              common_digest_mismatch=False)
    assert sm.stage == BootStage.STAGE_D
    assert sm.can_publish_motion_factor() is True


def test_boot_i2_never_grace_at_29s():
    """*** THE variant: even 2.9 s in, never-received factor
    is still zero. No T-07 grace."""
    sm = BootStageMachine()
    factor = sm.factor_for_downstream(now_mono_ms=2900)
    assert factor.allow_motion is False
    assert factor.speed_factor == 0.0
    assert factor.v_max_mps == 0.0


def test_boot_i2_real_factor_used_after_first_receive():
    sm = BootStageMachine()
    real = MotionFactor(allow_motion=True, speed_factor=1.0, v_max_mps=1.5)
    sm.note_factor_received(real)
    got = sm.factor_for_downstream(now_mono_ms=5000)
    assert got.allow_motion is True and got.speed_factor == 1.0


# ---------------- INF-BT-3 timeout_lock ----------------

def test_unlock_channel_closed_set():
    assert ALLOWED_UNLOCK_CHANNELS == frozenset({"CR-11", "CR-12"})


def test_unlock_wrong_channel_denied():
    """BOOT-L1: only CR-11/CR-12."""
    v = validate_unlock_request(
        {"action": "enable", "confirm_token": "tk"},
        channel="HMI")
    assert v.code == E_CHANNEL_DENIED


def test_unlock_wrong_action_refused():
    v = validate_unlock_request(
        {"action": "reboot", "confirm_token": "tk"},
        channel="CR-11")
    assert v.code == E_CONFIRM_REQUIRED


def test_unlock_missing_token_refused():
    v = validate_unlock_request(
        {"action": "enable"}, channel="CR-11")
    assert v.code == E_CONFIRM_REQUIRED


def test_unlock_all_gates_ok():
    v = validate_unlock_request(
        {"action": "enable", "confirm_token": "tk"},
        channel="CR-11")
    assert v.accepted


def test_gate_starts_locked():
    g = TimeoutLockGate()
    assert g.may_publish_factor() is False


def test_ack_alone_does_not_unlock():
    """BOOT-L3: ack accepted but readback still True -> stay blocked."""
    g = TimeoutLockGate(timeout_lock=True)
    g.note_ack_only(ack_accepted=True)
    assert g.may_publish_factor() is False


def test_heartbeat_resume_does_not_unlock():
    """BOOT-L2."""
    g = TimeoutLockGate(timeout_lock=True)
    g.note_heartbeat_resumed()
    assert g.may_publish_factor() is False


def test_cmd_age_ok_does_not_unlock():
    g = TimeoutLockGate(timeout_lock=True)
    g.note_cmd_age_ok()
    assert g.may_publish_factor() is False


def test_readback_false_unlocks():
    """Only readback: True -> False lifts."""
    g = TimeoutLockGate(timeout_lock=True)
    g.note_readback(readback_lock=False)
    assert g.may_publish_factor() is True


def test_readback_still_true_stays_locked():
    g = TimeoutLockGate(timeout_lock=True)
    g.note_readback(readback_lock=True)
    assert g.may_publish_factor() is False


def test_locks_must_be_separate():
    """Combining hes_lock + timeout_lock into one bit rejected."""
    with pytest.raises(HesLockConflation):
        assert_locks_are_separate(("combined_lock",))


def test_locks_missing_hes_lock_rejected():
    with pytest.raises(HesLockConflation, match="hes_lock"):
        assert_locks_are_separate(("timeout_lock", "estop_lock"))


def test_locks_both_present_ok():
    assert_locks_are_separate(("hes_lock", "timeout_lock"))


# ---------------- CHK-2-11 motion throttle ----------------

def test_throttle_config_zero_speed_refused():
    with pytest.raises(MotionThrottleConfigError):
        ThrottleConfig(speed_threshold_mps=0, max_ctx_tokens=2048)


def test_throttle_config_zero_ctx_refused():
    with pytest.raises(MotionThrottleConfigError):
        ThrottleConfig(speed_threshold_mps=1.0, max_ctx_tokens=0)


def test_should_throttle_high_speed():
    """Variant B guard: reversed direction (throttle at LOW speed)
    would fail this."""
    assert should_throttle(current_speed_mps=1.5,
                              threshold_mps=1.0) is True


def test_should_throttle_low_speed():
    assert should_throttle(current_speed_mps=0.5,
                              threshold_mps=1.0) is False


def test_admit_llm_refuses_when_high_speed():
    cfg = ThrottleConfig(speed_threshold_mps=1.0, max_ctx_tokens=2048)
    with pytest.raises(LlmRequestRefused, match="AI-73"):
        admit_llm_request(current_speed_mps=1.5, cfg=cfg)


def test_admit_llm_ok_when_low_speed():
    cfg = ThrottleConfig(speed_threshold_mps=1.0, max_ctx_tokens=2048)
    admit_llm_request(current_speed_mps=0.5, cfg=cfg)


def test_ctx_cap_no_trim_when_under():
    r = trim_to_cap(PromptSections("a", "b", "c", "d"), max_tokens=100)
    assert r.tokens_dropped_history == 0
    assert r.tokens_dropped_context == 0


def test_ctx_cap_trims_history_first():
    """AI-40g order: history first, context second."""
    s = PromptSections(
        system="sys sys",             # 2
        mission="mis mis mis",         # 3
        context="ctx ctx",             # 2
        history="his his his his")     # 4
    r = trim_to_cap(s, max_tokens=7)
    # 2+3+2+4=11, cap 7. Drop history=4 -> 2+3+2=7 <= 7. OK.
    assert r.tokens_dropped_history == 4
    assert r.tokens_dropped_context == 0


def test_ctx_cap_trims_context_when_needed():
    s = PromptSections(
        system="sys",
        mission="mis mis",
        context="ctx ctx ctx",
        history="his his his his his")
    r = trim_to_cap(s, max_tokens=4)
    # sys+mis+ctx=6 > 4; drop ctx too -> sys+mis=3 <= 4
    assert r.tokens_dropped_history > 0
    assert r.tokens_dropped_context > 0


def test_ctx_cap_refuses_if_system_mission_exceeds_cap():
    """Truncating mission would be worse than refusing."""
    s = PromptSections(
        system="s s s s s s",
        mission="m m m m m m",
        context="",
        history="")
    with pytest.raises(CtxCapViolation, match="alone exceed"):
        trim_to_cap(s, max_tokens=5)


# ---------------- CHK-2-19 network links ----------------

def test_lnk1_classifies_to_chassis_fatal():
    """CHK-2-19 (i) variant: putting LNK-1 into 'network' would be
    wrong (LAN1 drop is chassis-fatal, not network-warn)."""
    item, level = classify_link_failure("LNK-1")
    assert item == "chassis" and level == "fatal"


def test_lnk2_classifies_to_network_warn():
    item, level = classify_link_failure("LNK-2")
    assert item == "network" and level == "warn"


def test_lnk3_classifies_to_payload():
    item, _ = classify_link_failure("LNK-3")
    assert item == "payload_svc"


def test_lnk_d_returns_none_item():
    """CHK-2-19 (iv): LNK-D never becomes a health item."""
    item, level = classify_link_failure("LNK-D")
    assert item is None and level is None


def test_lnk_d_detail_only_flag():
    assert lnk_d_writes_detail_only() is True


def test_unknown_link_raises():
    with pytest.raises(LinkClassificationError, match="unknown link"):
        classify_link_failure("LNK-99")


def test_bit_source_no_onvif_clean_ok():
    assert_no_onvif_in_bit_source("import zenoh\nimport asyncio\n")


def test_bit_source_onvif_detected():
    """P-3 guard."""
    with pytest.raises(OnvifInBitPath):
        assert_no_onvif_in_bit_source("cam = ONVIFCamera(...)")


def test_link_detail_carrier_only_not_healthy():
    """P-2 guard: single-bool compression would fail this case."""
    assert is_link_healthy(LinkDetail(carrier_up=True,
                                        l3_reachable=False)) is False


def test_link_detail_both_true_healthy():
    assert is_link_healthy(LinkDetail(carrier_up=True,
                                        l3_reachable=True)) is True


# ---------------- CHK-2-24 gpu/dla ----------------

def test_gpu_fresh_event_ok():
    r = evaluate_gpu(golden_event_age_s=100.0)
    assert r.state == BitItemState.OK.value


def test_gpu_stale_event_fail_not_unknown():
    """CHK-2-24 (ii) key rule: stale MUST fail, NEVER unknown."""
    r = evaluate_gpu(golden_event_age_s=3601.0)
    assert r.state == BitItemState.FAIL.value
    assert r.reason == "stale"


def test_gpu_no_event_fail_not_unknown():
    """CHK-2-24 (iii) key rule: never received -> fail (not unknown)."""
    r = evaluate_gpu(golden_event_age_s=None)
    assert r.state == BitItemState.FAIL.value
    assert r.reason == "no_event"


def test_dla_disabled_and_no_engine_always_ok():
    """CHK-2-24 (iv): dla.enabled False + no engine -> ok/not_used."""
    r = evaluate_dla(dla_enabled=False, has_dla_engine=False,
                       golden_event_age_s=None)
    assert r.state == BitItemState.OK.value
    assert r.detail == "not_used"


def test_dla_enabled_stale_still_fail():
    r = evaluate_dla(dla_enabled=True, has_dla_engine=True,
                       golden_event_age_s=3601.0)
    assert r.state == BitItemState.FAIL.value
    assert r.detail == "stale"


def test_dla_engine_present_no_event_fail():
    r = evaluate_dla(dla_enabled=False, has_dla_engine=True,
                       golden_event_age_s=None)
    assert r.state == BitItemState.FAIL.value
    assert r.detail == "no_event"


def test_no_pc3_key_clean_ok():
    assert_no_pc3_key("normal source")


def test_no_pc3_key_hit_reddens():
    """19 §9.4 (1)."""
    with pytest.raises(Pc3Violation):
        assert_no_pc3_key('items = ["PC-1", "PC-2", "PC-3"]')


def test_no_unknown_state_in_source_clean():
    assert_no_unknown_state_in_source("state = BitItemState.OK")


def test_no_unknown_state_in_source_hit():
    with pytest.raises(BitAssertViolation, match="UNKNOWN"):
        assert_no_unknown_state_in_source(
            'state = BitItemState.UNKNOWN     # halfway')


def test_no_unknown_state_string_hit():
    with pytest.raises(BitAssertViolation):
        assert_no_unknown_state_in_source('return "unknown"')
