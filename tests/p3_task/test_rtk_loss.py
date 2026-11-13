"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_rtk_loss.py
Brief: 11 S3.3.4 / 99 U142 -- single / no_fix suspends the task, raises event/alarm/rtk, holds dispatch; recovery reports, never resumes

Description:
Measured on the robot 2026-11-06 with a GNSS injector: on a single fix the
robot stood still with the task "running", silent, and drove on by itself
when the fix came back. The user ruled the same day that only single /
no_fix is a positioning failure (rtk_fixed / rtk_float / dgps keep working).
Each case names the mutant it kills:
  * RtkLossMonitor: float / dgps are not lost; single / no_fix / an absent
    fix are, after lost_sustain_s; a short dip is not; recovery needs
    recover_sustain_s; no pose at all is no judgement; the last good
    position is kept for the alarm;
  * suspend_running_for_rtk commits passive / rtk_degraded;
  * __main__ refuses to start without the sustain pair (EX_CONFIG 78);
  * through the real P3 loop: the loss withdraws P1's mission, suspends the
    task and publishes the alarm with the task facts; a submit during the
    loss is refused with E_DEGRADED; recovery publishes event/info/rtk naming
    the waiting task, which never resumes by itself (RL-7), while a task that
    became ready during the loss was held and starts only now.
"""
from __future__ import annotations  # postponed annotations

import asyncio  # the live loop
import json  # ack frames

import aiosqlite  # task db
import pytest  # marks, parametrize
import pytest_asyncio  # async fixture

from xbrain.common.errors import E_DEGRADED  # the refusal code
from xbrain.p3_task.dao.tasks_dao import TasksDAO  # seed rows
from xbrain.p3_task.lifecycle.rtk_loss import (  # under test
    RtkLossConfigError, RtkLossMonitor, suspend_running_for_rtk)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS  # task.db DDL
from tests.p3_task.live_loop import (  # the real loop over fake planes
    goto_row, insert_rows, route_ops, start_p3, stop_p3, task_state, wait_for)

pytestmark = pytest.mark.no_device  # pure logic plus an in-process loop


def _pose(fix, lat=35.0, lon=135.0):
    """A state/pose body with the given fix (positions as P1 sends them)."""
    return {"fix_type": fix, "lat": lat, "lon": lon, "heading_valid": True,
            "heading_rad": 0.5, "heading_level": 1, "heading_source": "dual_antenna"}


# -- the judgement -------------------------------------------------------------------
@pytest.mark.parametrize("fix,lost", [("rtk_fixed", False), ("rtk_float", False), ("dgps", False),
                                      ("single", True), ("no_fix", True), (None, True)])
def test_what_counts_as_lost(fix, lost):
    """99 U142. MUTATION: count dgps as lost (the old 11 S3.2.1 row) -> red."""
    m = RtkLossMonitor(1.0, 2.0)  # the shipped pair
    events = [m.update(_pose(fix), t / 10.0) for t in range(20)]  # 2 s of frames
    assert ("lost" in events) is lost  # declared exactly for the lost fixes


def test_lost_once_after_the_sustain_and_a_dip_is_not_a_loss():
    """MUTATION: reset the run on every bad frame -> never fires -> red;
    no sustain -> the 0.5 s dip fires -> red."""
    m = RtkLossMonitor(1.0, 2.0)  # 1 s to declare
    dip = [m.update(_pose("single"), t / 10.0) for t in range(5)]  # 0.5 s bad
    assert m.update(_pose("rtk_fixed"), 0.6) is None and "lost" not in dip  # a dip only
    run = [m.update(_pose("single"), 1.0 + t / 10.0) for t in range(15)]  # 1.4 s bad
    assert run.count("lost") == 1 and m.lost  # declared once


def test_recovery_needs_its_own_sustain():
    """MUTATION: recover on the first good frame -> red."""
    m = RtkLossMonitor(1.0, 2.0)  # 2 s to recover
    for t in range(15):
        m.update(_pose("single"), t / 10.0)  # lost at 1.0 s
    early = [m.update(_pose("rtk_float"), 2.0 + t / 10.0) for t in range(15)]  # 1.4 s good
    assert "recovered" not in early and m.lost  # not yet
    late = [m.update(_pose("rtk_float"), 3.5 + t / 10.0) for t in range(10)]  # past 2.0 s
    assert late.count("recovered") == 1 and not m.lost  # once


def test_no_pose_is_no_judgement_and_the_last_good_position_is_kept():
    """MUTATION: treat None as lost -> a P1 restart suspends the task -> red."""
    m = RtkLossMonitor(1.0, 2.0)  # shipped pair
    m.update(_pose("rtk_fixed", 35.001, 135.002), 0.0)  # the last good fix
    assert all(m.update(None, t / 10.0) is None for t in range(30)) and not m.lost  # no pose
    for t in range(15):
        m.update(_pose("single", 36.0, 136.0), 3.0 + t / 10.0)  # lost, wrong position
    assert m.lost and m.last_valid_pos == {"lat": 35.001, "lon": 135.002}  # the clue kept


@pytest.mark.parametrize("bad", [None, 0, -1.0, True])
def test_the_sustain_pair_is_required(bad):
    """CLAUDE.md 3.1: no default. MUTATION: accept 0 -> red."""
    with pytest.raises(RtkLossConfigError):
        RtkLossMonitor(bad, 2.0)  # refused


# -- the suspension --------------------------------------------------------------------
@pytest_asyncio.fixture
async def db(tmp_path):
    """(connection, path) of a fresh file task.db."""
    path = str(tmp_path / "task.db")  # a file: a second reader sees commits only
    conn = await aiosqlite.connect(path)  # under test
    for stmt in ALL_DDL_STATEMENTS:
        await conn.execute(stmt)  # real schema
    await conn.commit()  # durable
    yield conn, path  # both
    await conn.close()  # cleanup


@pytest.mark.asyncio
async def test_the_running_task_is_suspended_rtk_degraded(db):
    """RL-2. MUTATION: drop the commit -> the fresh reader sees running -> red."""
    conn, path = db  # connection and file
    await TasksDAO(conn).insert(goto_row("t-run", "running", 1))  # the running goto
    await conn.commit()  # as on disk
    got = await suspend_running_for_rtk(TasksDAO(conn), conn, 1000)  # RL-2
    assert got == ("t-run", "suspended", "rtk_degraded")  # reported
    assert await task_state(path, "t-run") == ("suspended", "passive", "rtk_degraded")  # committed
    assert not conn.in_transaction  # nothing left open


# -- __main__ ----------------------------------------------------------------------------
@pytest.mark.parametrize("rtk,rc", [(None, 78), ({"lost_sustain_s": 1.0}, 78),
                                    ({"lost_sustain_s": 0.0, "recover_sustain_s": 2.0}, 78),
                                    ({"lost_sustain_s": 1.0, "recover_sustain_s": 2.0}, 0)])
def test_main_refuses_without_the_pair(monkeypatch, rtk, rc):
    """EX_CONFIG 78 by key path. MUTATION: drop the check -> the null cases
    start with the judgement off -> red."""
    import xbrain.common.config.resolved as resolved  # where main loads from
    import xbrain.p3_task.runtime.main_wiring as mw  # the wiring main calls
    from xbrain.p3_task import __main__ as p3main  # unit under test
    cfg = {"geo": {"enu_origin": {"lat": 31.0, "lon": 121.0, "alt": 0.0}},  # minimal resolved config
           "teach": {"snap_max_age_ms": 8000, "min_dist_m": 0.5, "sample_hz": 1.0,
                     "record_duration_s": 10800, "record_warn_before_s": [600, 60]},
           "route": {"recorded_arrive_radius_m": 1.0, "membership_m": 5.0},
           "fence": {"speed_limit_default_mps": 0.6, "speed_limit_min_mps": 0.1, "speed_limit_max_mps": 2.0,
                     "min_area_m2": 1.0},  # 99 U185 / U191
           "low_battery": {"trigger_soc_pct": 20.0, "stop_soc_pct": 5.0,
                           "cloud_alarm_period_s": 2.0,  # 99 U146 rule
                           "charge_full_soc_pct": 100.0,  # 99 U157
                           "swap_resume_soc_pct": 80.0,
                           "swap_absent_min_s": 5.0, "cold_swap_min_rise_pct": 10.0,
                           "plug_confirm_s": 5.0, "unplug_confirm_s": 5.0}}
    if rtk is not None:
        cfg["rtk_loss"] = rtk  # the case under test
    monkeypatch.setattr(resolved, "load_resolved", lambda *_a, **_k: cfg)  # no file
    seen = {}  # what the wiring received
    monkeypatch.setattr(mw, "run_voice_loop_wiring", lambda **kw: seen.update(kw) or 0)
    assert p3main.main(["--voice-loop"]) == rc  # the exit code
    if rc == 0:
        assert (seen["rtk_lost_sustain_s"], seen["rtk_recover_sustain_s"]) == (1.0, 2.0)  # passed


# -- through the real P3 loop --------------------------------------------------------------
async def _feed(h, fix, seconds):
    """Publish state/pose with this fix every 0.1 s for `seconds`."""
    from xbrain.p3_task.runtime.main_wiring import STATE_POSE_TOPIC  # P1's key
    for _ in range(int(seconds * 10)):
        h.send(STATE_POSE_TOPIC, {"data": _pose(fix)})  # as P1 publishes it
        await asyncio.sleep(0.1)  # 10 Hz


@pytest.mark.asyncio
async def test_loss_suspends_alarms_refuses_and_recovery_does_not_resume(tmp_path, monkeypatch):
    """The 2026-11-06 robot case end to end. MUTATION: no stop_motion on the
    suspension -> no clear -> red; drop the submit refusal -> the new goto
    is accepted -> red; drop the dispatch hold -> t-due starts during the
    loss -> red; resume t-goto on recovery -> red; drop the alarm's task
    facts -> red."""
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, task_db = await start_p3(
        tmp_path, monkeypatch, rows=[goto_row("t-goto", "ready", 1)],
        rtk_lost_sustain_s=1.0, rtk_recover_sustain_s=2.0)  # judgement on
    try:
        await _feed(h, "rtk_fixed", 0.5)  # a good fix first
        assert await wait_for(lambda: route_ops(h))  # the goto runs
        await _feed(h, "single", 2.0)  # lost (> 1.0 s)
        assert await wait_for(lambda: h.frames("event/alarm/rtk")), "no alarm"  # RL-4
        alarm = h.frames("event/alarm/rtk")[0]  # the 11 S3.3.4 event
        d = alarm["detail"]  # its facts
        assert (alarm["sev"], alarm["channel"], d["type"], d["reason"]) == (
            "alarm", "alarm", "rtk_lost", "fix_lost")  # shape
        assert d["task_id"] == "t-goto" and d["action_taken"] == "stop_and_suspend"  # task facts
        assert d["last_valid_pos"] == {"lat": 35.0, "lon": 135.0}  # the clue
        assert route_ops(h) == ["set", "clear"]  # mission withdrawn
        assert await task_state(task_db, "t-goto") == ("suspended", "passive", "rtk_degraded")
        h.send(CMD_TASK_TOPIC, {"cmd_id": "c-new", "action": "submit", "source": "voice",
                                "task": {"type": "goto", "priority": 5,
                                         "params": {"waypoint_id": "w-gate"}}})  # during the loss
        assert await wait_for(lambda: h.frames("cmd/task/ack"))  # answered
        ack = h.frames("cmd/task/ack")[-1]  # the refusal
        assert (ack["result"], ack["code"]) == ("rejected", E_DEGRADED)  # 11 S3.2.1 single
        await insert_rows(task_db, goto_row("t-due", "ready", 2))  # e.g. a timed task coming due
        await _feed(h, "single", 1.0)  # still lost: two passes
        assert route_ops(h) == ["set", "clear"]  # dispatch held during the loss
        assert (await task_state(task_db, "t-due"))[0] == "ready"  # waiting for a fix
        await _feed(h, "rtk_float", 3.0)  # float is usable (> 2.0 s)
        assert await wait_for(lambda: h.frames("event/info/rtk")), "no recovery event"
        back = h.frames("event/info/rtk")[0]["detail"]  # the recovery
        assert back["type"] == "heading_recovered" and back["task_id"] == "t-goto"  # who waits
        assert await wait_for(lambda: route_ops(h) == ["set", "clear", "set"])  # t-due starts now
        assert (await task_state(task_db, "t-due"))[0] == "running"  # the queue resumes
        assert (await task_state(task_db, "t-goto"))[0] == "suspended"  # RL-7: never by itself
    finally:
        await stop_p3(stop, loop)  # never leave the loop running
