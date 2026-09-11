"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_watchdog_estop_backup.py
Brief: watchdog / estop / backup (P5 -- 20 S7.3A/S1.1/S7.5)

Description:
Guards the watchdog (A-CVG-2, two branches), estop suspension (A-ES-1/2), and
bounded backup (A-BK-1/2). Each test names its mutant.
"""

from __future__ import annotations

from xbrain.p1_motion.rns.backup import backup_permitted
from xbrain.p1_motion.rns.source import EstopSuspension
from xbrain.p1_motion.rns.types import NavFailReason, VelocityCandidate
from xbrain.p1_motion.rns.watchdog import (
    ProgressWatchdog, WatchdogResult, no_progress_failure,
)
from xbrain.common.types.units import Mps


# ── watchdog (A-CVG-2) ────────────────────────────────────────────────────────
def _wd(window_ms=1000, delta_w=0.5, report_after_windows=6):
    return ProgressWatchdog(window_ms, delta_w, report_after_windows)


def test_watchdog_reports_when_escalation_is_exhausted():
    # A-CVG-7 (20 S7.3A v1.41 branch three): a boundary exists, escalation
    # keeps firing, progress never rises -> after report_after_windows * T_w
    # the watchdog REPORTS anyway (bounded termination, RNS-T-1). Hot leg04
    # record: 400 s of every-tick escalation with the pose frozen. mutant:
    # drop the third threshold -> ESCALATE forever -> reddens.
    wd = _wd(window_ms=500, report_after_windows=3)
    wd.tick(0.0, 100, "detour", False, None, True)
    seen = []
    for _ in range(20):                       # 2.0 s of stall
        seen.append(wd.tick(0.0, 100, "detour", False, None, boundary_exists=True))
    # windows 1..3 escalate (reverse: it does NOT report early) ...
    assert seen[6] == WatchdogResult.ESCALATE_WALL
    assert seen[13] == WatchdogResult.ESCALATE_WALL
    # ... and past 3 windows it reports, flagged as exhausted.
    assert seen[-1] == WatchdogResult.REPORT_NO_PROGRESS and wd.exhausted
    f = no_progress_failure("unknown", wd.exhausted)
    assert f.detail["escalations_exhausted"] is True
    # progress resets the budget
    assert wd.tick(5.0, 100, "detour", False, None, True) == WatchdogResult.OK
    assert wd.exhausted is False


def test_watchdog_ok_while_progressing():
    wd = _wd()
    wd.tick(0.0, 100, "follow", False, None, True)
    r = wd.tick(1.0, 100, "follow", False, None, True)  # s* rose 1.0 > delta 0.5
    assert r == WatchdogResult.OK


def test_watchdog_escalates_wall_on_stall_with_boundary():
    # A-CVG-2: no progress for window, boundary exists -> escalate wall-follow.
    # mutant: remove the watchdog (never escalate) -> stuck oscillating -> red.
    wd = _wd(window_ms=500)
    wd.tick(0.0, 100, "follow", False, None, True)
    r = None
    for _ in range(10):
        r = wd.tick(0.0, 100, "follow", False, None, boundary_exists=True)
    assert r == WatchdogResult.ESCALATE_WALL


def test_watchdog_reports_no_progress_without_boundary():
    # branch two: stall with NO boundary -> report (not escalate on a wall that
    # isn't there). mutant: escalate anyway -> wall-follow with no wall.
    wd = _wd(window_ms=500)
    wd.tick(0.0, 100, "follow", False, None, False)
    r = None
    for _ in range(10):
        r = wd.tick(0.0, 100, "follow", False, None, boundary_exists=False)
    assert r == WatchdogResult.REPORT_NO_PROGRESS


def test_watchdog_timer_excludes_wait_and_heading_limiter():
    # RNS-N-15: waiting and heading/RTK limiters do NOT count as lost.
    wd = _wd(window_ms=300)
    wd.tick(0.0, 100, "follow", False, None, True)
    # 10 ticks of waiting -> timer never accumulates -> still OK
    for _ in range(10):
        r = wd.tick(0.0, 100, "follow", is_waiting=True, argmax_limiter=None,
                    boundary_exists=True)
    assert r == WatchdogResult.OK
    # 10 ticks with heading limiter -> also excluded
    for _ in range(10):
        r = wd.tick(0.0, 100, "follow", False, argmax_limiter="heading",
                    boundary_exists=True)
    assert r == WatchdogResult.OK


def test_no_progress_failure_carries_limiter():
    f = no_progress_failure("fence")
    assert f.reason is NavFailReason.WATCHDOG_NO_PROGRESS
    assert f.detail["limiter"] == "fence"


# ── estop suspension (A-ES-1/2) ───────────────────────────────────────────────
def test_estop_forces_zero_output():
    # A-ES-1: while suspended, output is forced to None (zero). mutant: pass the
    # candidate through -> non-zero under estop -> reddens.
    es = EstopSuspension()
    c = VelocityCandidate(vx=Mps(1.5), vy=Mps(0.0), wz=0.3)
    assert es.gate_output(c) is c          # not suspended -> passes
    es.on_estop_or_preempt()
    assert es.gate_output(c) is None        # suspended -> zero
    assert es.suspended() is True


def test_release_does_not_replay_intent():
    # A-ES-2: on release, no cached velocity is replayed -- the caller must
    # re-evaluate. The gate returns to pass-through, but there is no stored intent
    # to lurch on. mutant: cache and replay last candidate on release -> lurch.
    es = EstopSuspension()
    c = VelocityCandidate(vx=Mps(2.0), vy=Mps(0.0), wz=0.0)
    es.gate_output(c)
    es.on_estop_or_preempt()
    es.gate_output(c)          # discarded while suspended
    es.on_release()
    assert es.suspended() is False
    # the gate has no memory of c -- a fresh compute must supply the next output.
    assert es.gate_output(None) is None   # nothing to replay


# ── bounded backup (A-BK-1/2) ─────────────────────────────────────────────────
def test_backup_permitted_into_recent_free():
    assert backup_permitted(True, rear_memory_age_s=1.0, backed_so_far_m=0.0,
                            t_backup_memory_s=3.0, max_backup_m=1.0) is True


def test_backup_refused_into_unobserved():
    # A-BK-1: reverse only into observed-FREE. mutant: drop the check -> reverse
    # into unobserved space -> reddens.
    assert backup_permitted(False, 1.0, 0.0, 3.0, 1.0) is False


def test_backup_stops_when_memory_stale():
    # A-BK-2: memory expired -> stop immediately.
    assert backup_permitted(True, rear_memory_age_s=5.0, backed_so_far_m=0.0,
                            t_backup_memory_s=3.0, max_backup_m=1.0) is False


def test_backup_stops_at_distance_cap():
    assert backup_permitted(True, 1.0, backed_so_far_m=1.0,
                            t_backup_memory_s=3.0, max_backup_m=1.0) is False
