"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_c.py
Brief: MOT-PM-1/26..32 batch C tests (ctrl loop + hot update + failure + observability)

Description:
Seven P1 modules: ctrl_loop (20 Hz + lifecycle SM), hot_update (key
whitelist), fallback (fence_guard / hold), failure handlers with
event closed set, gate observability (PX-1..PX-4), Zenoh plane
selfcheck (RT-C3), recording lock (TR-1..3). Each module gets 2-4
tests focused on the named-variant behavior in the spec.
"""

import pytest

from xbrain.p1_motion.config.hot_update import (
    HOT_UPDATABLE_KEYS, HotUpdateError,
    check_hot_updatable, is_mirror_read_only,
)
from xbrain.p1_motion.config.zenoh_planes import (
    PlaneViolation, RT_ONLY_PUB,
    check_gen_pub_not_rt_only, check_rt_key_on_rt_only,
)
from xbrain.p1_motion.ctrl_loop import (
    CtrlLoop, CtrlState,
)
from xbrain.p1_motion.failure.handlers import (
    MotionFailKind, build_event,
)
from xbrain.p1_motion.gate.observability import build_clip_report
from xbrain.p1_motion.path.recording_lock import (
    RecordingLock, RecordingRejection,
)
from xbrain.p1_motion.sources.fallback import (
    FenceGuardOutput, build_fence_guard, hold_output,
)


pytestmark = pytest.mark.no_device


# --- MOT-PM-1 ctrl_loop ---

def test_ctrl_loop_publishes_every_tick_even_in_wait():
    """Every state MUST publish cmd_vel each tick (chassis Tier 1
    requires no gap)."""
    pubs = []
    loop = CtrlLoop(publish_cmd_vel=lambda vx, wz: pubs.append((vx, wz)))
    for _ in range(5):
        loop.run_one_tick()
    assert len(pubs) == 5
    # All zero because state = INIT.
    assert all(p == (0.0, 0.0) for p in pubs)


def test_ctrl_loop_active_uses_computed_vel():
    pubs = []
    loop = CtrlLoop(publish_cmd_vel=lambda vx, wz: pubs.append((vx, wz)))
    loop.transition(CtrlState.ACTIVE)
    loop.run_one_tick(computed_vx=1.0, computed_wz=0.5)
    assert pubs == [(1.0, 0.5)]


def test_ctrl_loop_safe_stop_forces_zero():
    pubs = []
    loop = CtrlLoop(publish_cmd_vel=lambda vx, wz: pubs.append((vx, wz)))
    loop.transition(CtrlState.SAFE_STOP)
    loop.run_one_tick(computed_vx=2.0, computed_wz=1.0)
    # SAFE_STOP ignores computed; always zero.
    assert pubs == [(0.0, 0.0)]


def test_ctrl_loop_publish_exception_does_not_kill_loop():
    """A publish failure must not stop the tick counter -- the next
    tick keeps running."""
    def bad_publish(vx, wz):
        raise RuntimeError("wire down")
    loop = CtrlLoop(publish_cmd_vel=bad_publish)
    loop.run_one_tick()   # must not raise
    loop.run_one_tick()
    assert len(loop.history) == 2
    assert all(not h.published_cmd_vel for h in loop.history)


# --- MOT-PM-26 hot update ---

def test_hot_updatable_whitelisted_ok():
    for key in HOT_UPDATABLE_KEYS:
        check_hot_updatable(key)


def test_hot_updatable_non_whitelisted_raises():
    """RCG constants are NOT hot-updatable (require restart)."""
    with pytest.raises(HotUpdateError):
        check_hot_updatable("rns.rcg.r_eff_fallback_m")


def test_mirror_read_only_recognises_forwarded_keys():
    assert is_mirror_read_only("cmd/chassis/mode")
    assert is_mirror_read_only("state/clock")
    assert not is_mirror_read_only("cmd/motion/factor")


# --- MOT-PM-27 failure handlers ---

def test_fail_event_severity_from_table_not_caller():
    ev = build_event(MotionFailKind.FENCE_LOST, {"note": "grid missing"})
    assert ev.severity == "fault"
    ev2 = build_event(MotionFailKind.CMD_AGE_TOO_HIGH, {"age_ms": 250})
    assert ev2.severity == "warn"


def test_fail_event_kind_is_string_value():
    """kind serialises as string, not enum member."""
    ev = build_event(MotionFailKind.RNS_MODULE_DEAD, {})
    assert ev.kind == "rns_module_dead"


# --- MOT-PM-29 fence_guard veto-only + hold ---

def test_fence_guard_output_has_only_veto_fields():
    """Structural: FenceGuardOutput HAS NO vx/wz fields."""
    fg = build_fence_guard(veto_forward=True)
    assert not hasattr(fg, "vx")
    assert not hasattr(fg, "wz")


def test_hold_output_is_fixed_zero():
    assert hold_output() == (0.0, 0.0)


# --- MOT-PM-31 clip observability PX-1..PX-4 ---

def test_clip_report_no_op_when_both_clean():
    r = build_clip_report(outer_reasons=[], inner_reasons=[])
    assert r.outer_clipped is False
    assert r.inner_clipped is False


def test_clip_report_both_clipped_reported_separately():
    """PX-4: outer + inner both clipped -> BOTH flags true, NOT merged."""
    r = build_clip_report(outer_reasons=["f_speed"],
                           inner_reasons=["L_min"])
    assert r.outer_clipped is True
    assert r.inner_clipped is True
    assert "f_speed" in r.outer_reasons
    assert "L_min" in r.inner_reasons


# --- MOT-PM-32 Zenoh plane RT-C3 ---

def test_rt_only_key_on_gen_plane_rejected():
    for key in RT_ONLY_PUB:
        with pytest.raises(PlaneViolation):
            check_rt_key_on_rt_only(key, plane="gen")


def test_rt_only_key_on_rt_plane_ok():
    for key in RT_ONLY_PUB:
        check_rt_key_on_rt_only(key, plane="rt")


def test_gen_pub_selfcheck_catches_rt_only_leak():
    """Startup selfcheck: GEN plane publisher set containing an RT
    key is a startup refusal."""
    with pytest.raises(PlaneViolation):
        check_gen_pub_not_rt_only(frozenset({"rt/motion/cmd_vel"}))


def test_gen_pub_selfcheck_ok_when_clean():
    check_gen_pub_not_rt_only(frozenset({"state/mode", "health/summary"}))


# --- MOT-PM-30 recording lock TR-1/2/3 ---

def test_recording_lock_permits_teleop_only():
    lock = RecordingLock(active=True)
    assert lock.is_source_permitted("teleop_keyboard") is True
    assert lock.is_source_permitted("nav2_proxy") is False
    assert lock.is_source_permitted("path_follow") is False


def test_recording_lock_new_delegate_rejected():
    lock = RecordingLock(active=True)
    with pytest.raises(RecordingRejection):
        lock.check_new_delegate("path_follow")


def test_recording_lock_teleop_delegate_ok():
    lock = RecordingLock(active=True)
    lock.check_new_delegate("teleop_keyboard")


def test_recording_lock_inactive_permits_all():
    lock = RecordingLock(active=False)
    for s in ("path_follow", "nav2_proxy", "rns_avoid"):
        assert lock.is_source_permitted(s) is True
