"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_dynamic.py
Brief: two-threshold distance rule + wait budget (P3 -- 20 S5.3/S5.3A)

Description:
Guards dynamic.py: stop only in corridor (A-DYN-1), two thresholds with
hysteresis (A-DYN-2), and the state-bound wait budget (RNS-N-16, A-CVG-3) that a
rotating blocker cannot reset. Each test names its mutant.
"""

from __future__ import annotations

from xbrain.p1_motion.rns.dynamic import (
    DynamicAction, WaitBudget, distance_action,
)
from xbrain.p1_motion.rns.types import NavFailReason


def _act(in_corridor, r_near, prev=DynamicAction.RUN, stop=3.0, resume=4.0):
    return distance_action(in_corridor, r_near, prev, stop, resume)


def test_side_traffic_does_not_stop():
    # A-DYN-1: dynamic obstacle NOT in the selected corridor -> RUN. mutant:
    # "any dynamic in view stops" -> side traffic stops us -> reddens.
    assert _act(in_corridor=False, r_near=1.0) == DynamicAction.RUN


def test_stop_when_close_in_corridor():
    assert _act(in_corridor=True, r_near=2.0) == DynamicAction.STOP   # < stop 3.0
    assert _act(in_corridor=True, r_near=5.0) == DynamicAction.RUN    # > resume 4.0


def test_two_thresholds_prevent_chatter():
    # A-DYN-2: in the band [stop, resume], the action HOLDS (hysteresis). With a
    # single threshold, r=3.5 would flip. mutant: resume==stop -> no band -> a
    # noisy sequence around 3.0 flips RUN/STOP -> reddens.
    # were stopped, now r=3.5 (in band) -> stay stopped
    assert _act(in_corridor=True, r_near=3.5, prev=DynamicAction.STOP) == DynamicAction.STOP
    # were running, now r=3.5 (in band) -> slow (not stop, not run)
    assert _act(in_corridor=True, r_near=3.5, prev=DynamicAction.RUN) == DynamicAction.SLOW


def test_wait_budget_fires_on_timeout():
    # RNS-N-16: accumulated stopped time over budget -> blocked_by_dynamic.
    # mutant: never fire -> robot stands forever -> reddens.
    wb = WaitBudget(wait_budget_s=2.0)
    assert wb.tick(True, now_ms=1000, blocking_track_id=5, blocking_class="person") is None
    assert wb.tick(True, now_ms=2500, blocking_track_id=5, blocking_class="person") is None  # 1.5 s
    f = wb.tick(True, now_ms=3500, blocking_track_id=5, blocking_class="person")   # 2.5 s > 2 s
    assert f is not None
    assert f.reason is NavFailReason.BLOCKED_BY_DYNAMIC
    assert f.detail["track_id"] == 5
    assert f.detail["class_name"] == "person"


def test_wait_budget_resets_when_stop_clears():
    wb = WaitBudget(wait_budget_s=2.0)
    wb.tick(True, now_ms=1000, blocking_track_id=5, blocking_class="car")
    wb.tick(False, now_ms=1500, blocking_track_id=None, blocking_class=None)  # cleared
    assert wb.waiting() is False
    # a fresh stop starts the clock over -- not carried from before
    assert wb.tick(True, now_ms=2000, blocking_track_id=6, blocking_class="car") is None
    assert wb.tick(True, now_ms=2500, blocking_track_id=6, blocking_class="car") is None  # 0.5 s


def test_rotating_blocker_does_not_reset_budget():
    # A-CVG-3 / RNS-N-16: A leaves, B arrives -- the STATE stays stopped, so the
    # timer accumulates across the swap. mutant: bind timer to track_id -> each
    # swap resets -> budget never fires -> reddens.
    wb = WaitBudget(wait_budget_s=2.0)
    wb.tick(True, now_ms=1000, blocking_track_id=1, blocking_class="person")  # A
    wb.tick(True, now_ms=2000, blocking_track_id=2, blocking_class="person")  # B (swap)
    f = wb.tick(True, now_ms=3500, blocking_track_id=3, blocking_class="person")  # C, 2.5 s total
    assert f is not None   # fired despite three different blockers
    assert f.detail["track_id"] == 3   # reports the CURRENT blocker
