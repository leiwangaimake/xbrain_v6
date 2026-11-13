"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chassis_lock_events.py
Brief: event/warn/estop estop.locked -- every period_s while locked, silent once unlocked (99 U196)

Description:
Pins xbrain/p2_core/health/chassis_lock_events.py (user ruling 2026-11-13: the
chassis lock state is reported as an event once every 3 s while it lasts, and
stops once unlocked). Each test names the mutant that turns it red. The traps:
  * a locked chassis reported once only (the 11-13 stress test: nobody noticed);
  * an unlocked chassis still reported, or a fresh lock waiting a full period;
  * a stale or field-less state/robot read as "locked" (or as "unlocked");
  * the derived `locked` field trusted instead of the two Tier 1 locks.
"""

from __future__ import annotations  # postponed annotations

import pytest  # parametrize / raises

from xbrain.p2_core.health.chassis_lock_events import (  # the unit under test
    EVENT_TYPE,  # detail.type the HMI words on
    ChassisLockReporter,  # the per-spell judgement
    chassis_lock_event,  # the 11 S6.1 Event builder
)

PERIOD_S = 3.0  # the user's 3 s
MAX_AGE_S = 2.0  # state/robot freshness bound


def _robot(hes_lock=False, timeout_lock=False, hes=False, **extra):
    """One state/robot body with the two Tier 1 locks (11 S4.1)."""
    body = {"hes_lock": hes_lock, "timeout_lock": timeout_lock, "hes": hes,
            "motion_state": "rl_control"}  # as the 11-13 stress test read it
    body.update(extra)  # e.g. the derived `locked`
    return body  # a dict, like the P2 state_cache entry


def _run(rep, robot, t0, t1, step=1.0):
    """Feed one frame per 1 Hz health pass from t0 to t1; return report times."""
    out, t = [], t0  # the times a report came back
    while t <= t1 + 1e-9:  # inclusive end
        if rep.observe(robot, t, t) is not None:  # a fresh frame every pass
            out.append(round(t, 3))  # this pass reported
        t += step  # the next 1 Hz pass
    return out  # report times


def test_locked_reports_at_once_then_every_period():
    """MUTATION: report only on the edge -> [0.0] -> red; no period check ->
    a report every second -> red; `<` -> `<=` -> every 4 s -> red."""
    rep = ChassisLockReporter(PERIOD_S, MAX_AGE_S)  # configured as on the robot
    assert _run(rep, _robot(timeout_lock=True), 0.0, 9.0) == [0.0, 3.0, 6.0, 9.0]  # 3 s cadence


def test_unlock_stops_and_a_new_lock_reports_at_once():
    """MUTATION: unlock does not reset -> the relock at 4 s waits until 6 s ->
    red; unlocked still reported -> reports in 2..3 -> red."""
    rep = ChassisLockReporter(PERIOD_S, MAX_AGE_S)  # configured as on the robot
    assert _run(rep, _robot(timeout_lock=True), 0.0, 1.0) == [0.0]  # locked
    assert _run(rep, _robot(), 2.0, 3.0) == []  # unlocked: silent
    assert _run(rep, _robot(hes_lock=True), 4.0, 7.0) == [4.0, 7.0]  # relocked: at once


def test_locked_s_counts_from_the_start_of_the_spell():
    """MUTATION: locked_s from the last report -> 3.0 at t=6 instead of 6.0 -> red."""
    rep = ChassisLockReporter(PERIOD_S, MAX_AGE_S)  # configured as on the robot
    locked = _robot(timeout_lock=True)  # the boot lock
    assert rep.observe(locked, 0.0, 0.0)["locked_s"] == 0.0  # first report
    assert rep.observe(locked, 3.0, 3.0)["locked_s"] == 3.0  # second
    assert rep.observe(locked, 6.0, 6.0)["locked_s"] == 6.0  # third


@pytest.mark.parametrize("hes_lock,timeout_lock", [(True, False), (False, True), (True, True)])
def test_either_lock_is_locked(hes_lock, timeout_lock):
    """MUTATION: `or` -> `and` -> a single lock silent -> red."""
    rep = ChassisLockReporter(PERIOD_S, MAX_AGE_S)  # configured as on the robot
    got = rep.observe(_robot(hes_lock=hes_lock, timeout_lock=timeout_lock), 0.0, 0.0)  # one pass
    assert got is not None and got["hes_lock"] is hes_lock and got["timeout_lock"] is timeout_lock


def test_derived_locked_field_is_not_read():
    """MUTATION: read body["locked"] -> a compat field saying True reports while
    both real locks are clear -> red (11 S4.1: compat only)."""
    rep = ChassisLockReporter(PERIOD_S, MAX_AGE_S)  # configured as on the robot
    assert rep.observe(_robot(locked=True), 0.0, 0.0) is None  # both real locks clear


@pytest.mark.parametrize("robot", [None, {}, {"hes_lock": True}, {"timeout_lock": True},
                                   {"hes_lock": "true", "timeout_lock": False}])
def test_unknown_lock_state_reports_nothing(robot):
    """MUTATION: a missing field defaulted to True -> a report -> red."""
    rep = ChassisLockReporter(PERIOD_S, MAX_AGE_S)  # configured as on the robot
    assert rep.observe(robot, 0.0, 0.0) is None  # unknown is not locked


def test_stale_frame_reports_nothing_and_keeps_the_spell():
    """MUTATION: age ignored -> a report at t=10 from a frame heard at t=0 -> red;
    unknown treated as unlocked -> locked_s restarts at 0 on the fresh frame -> red."""
    rep = ChassisLockReporter(PERIOD_S, MAX_AGE_S)  # configured as on the robot
    locked = _robot(timeout_lock=True)  # the boot lock
    assert rep.observe(locked, 0.0, 0.0) is not None  # fresh: reported
    assert rep.observe(locked, 0.0, 10.0) is None  # same frame, 10 s old: unknown
    again = rep.observe(locked, 12.0, 12.0)  # the chassis is heard again, still locked
    assert again is not None and again["locked_s"] == 12.0  # same spell


def test_frame_exactly_max_age_old_still_counts():
    """MUTATION: `>` -> `>=` -> a 2.0 s old frame dropped -> red."""
    rep = ChassisLockReporter(PERIOD_S, MAX_AGE_S)  # configured as on the robot
    assert rep.observe(_robot(timeout_lock=True), 0.0, MAX_AGE_S) is not None  # on the bound


@pytest.mark.parametrize("bad", [None, 0, -1.0, True, "3"])
def test_config_must_be_positive_numbers(bad):
    """MUTATION: a default for a missing key -> no raise -> red (CLAUDE.md 3.1)."""
    with pytest.raises(ValueError):  # p2 refuses to start
        ChassisLockReporter(bad, MAX_AGE_S)  # period_s
    with pytest.raises(ValueError):  # p2 refuses to start
        ChassisLockReporter(PERIOD_S, bad)  # max_age_s


def test_event_shape():
    """MUTATION: title carries locked_s -> repeats stop folding in the HMI ring ->
    red; reason picks the timeout lock over a pressed stop -> red."""
    ev = chassis_lock_event({"hes_lock": True, "timeout_lock": True, "hes": True,
                             "locked_s": 6.0}, eid="lock-p2-ab-3", ts=1.0)  # both locks
    assert (ev["sev"], ev["cat"], ev["channel"], ev["code"]) == (
        "warn", "estop", "normal", "CHASSIS_LOCKED")  # 99 U196
    assert ev["title"] == "chassis locked"  # constant: the ring folds repeats
    assert ev["detail"]["type"] == EVENT_TYPE == "estop.locked"  # the HMI words on it
    assert ev["detail"]["reason"] == "hes_engaged"  # the stop is the lock to name
    assert ev["detail"]["locked_s"] == 6.0 and ev["eid"] == "lock-p2-ab-3"  # carried
    only_timeout = chassis_lock_event({"hes_lock": False, "timeout_lock": True,
                                       "hes": False, "locked_s": 0.0}, eid="e", ts=1.0)
    assert only_timeout["detail"]["reason"] == "cmd_timeout"  # the boot lock
