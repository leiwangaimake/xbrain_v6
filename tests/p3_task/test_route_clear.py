"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_route_clear.py
Brief: a cancelled / paused / preempted running task withdraws P1's mission (cmd/motion/route op=clear)

Description:
The 2026-11-06 audit measured, on the robot, that a voice "取消任务" or
"暂缓任务" only rewrote tasks.state: 20 s later P1 still reported nav_state
follow / route_state running, so outdoors the robot would have driven on to
the old target. 15 S7.2 step 1 asked for cmd/motion/behavior=hold, which
P3 can no longer send (11 S2.2, 99 U90 struck it from the publishers); the
stop P3 owns is cmd/motion/route op=clear (11 S3.5A v2.0, 12 S4.2c.4).
99 U141 records the fix.

What is pinned here, each with the mutant it kills:
  * the clear frame is one P1's own parser accepts as a RouteClear;
  * route_clear_needed: only a task leaving `running` for suspended /
    cancelled / failed -- never a queued task, never a P1-reported arrival;
  * _make_publish calls stop_motion FIRST and survives it raising;
  * through the real P3 loop (tests/p3_task/live_loop.py): cancel, pause,
    preempt and an estop suspension each put op=clear on cmd/motion/route;
    a terminal P1 reported does not; a cancel inside the ack window is not
    followed by the RA-3 re-push; an unconfirmed push is withdrawn when it
    fails; a resumed task is pushed again (RP-2, not a second RP-1).
"""
from __future__ import annotations  # postponed annotations

import asyncio  # settle waits

import pytest  # marks, parametrize

from xbrain.p3_task.route.push import build_route_clear, route_clear_needed  # under test
from tests.p3_task.live_loop import (  # the real loop over fake planes
    goto_row, insert_rows, route_ops, start_p3, stop_p3, task_state, wait_for)

pytestmark = pytest.mark.no_device  # pure logic plus an in-process loop

#: What P4 sends for a voice cancel / pause / resume (11 S7.2 contract shape).
def _cmd(action: str, cmd_id: str = "c-1") -> dict:
    return {"cmd_id": cmd_id, "action": action, "task_id": "t-goto", "source": "voice"}  # S7.2


# -- the frame ---------------------------------------------------------------------
def test_the_clear_frame_is_a_route_clear_to_p1s_own_parser():
    """P1's RouteAssembler is the consumer; a frame it rejects is silent on
    the wire. MUTATION: send op "cancel" / drop cmd_id -> RouteIntakeError,
    red."""
    from xbrain.p1_motion.nav.route_intake import RouteAssembler, RouteClear  # the real parser
    from xbrain.p1_motion.path.local_frame import LocalFrame  # any site frame will do
    out = RouteAssembler(LocalFrame(35.0, 135.0)).accept(build_route_clear())  # one frame
    assert isinstance(out, RouteClear)  # P1 cancels its mission on exactly this


# -- when -------------------------------------------------------------------------
@pytest.mark.parametrize("frm,to,want", [
    ("running", "cancelled", True),    # operator cancel
    ("running", "suspended", True),    # pause / preempt / estop
    ("running", "failed", True),       # P3's own failure (RA fatal)
    ("running", "done", False),        # P1 reported arrival: already idle
    ("ready", "cancelled", False),     # queued: may not touch the robot
    ("scheduled", "cancelled", False),  # a timed task never had a mission
    ("suspended", "cancelled", False),  # already withdrawn when it suspended
    ("ready", "running", False),       # a dispatch is a push, not a clear
])
def test_route_clear_needed(frm, to, want):
    """MUTATION: clear on every to-state in the set regardless of from ->
    a queued task's cancel stops the robot running ANOTHER task -> red."""
    assert route_clear_needed(frm, to) is want  # the truth table


# -- the hook ---------------------------------------------------------------------
@pytest.mark.asyncio
async def test_stop_motion_runs_first_and_survives_raising():
    """15 S7.2: stop motion before anything else. MUTATION: call it after
    publish_state -> order red; let its exception escape -> the broadcast is
    lost and the transition raises -> red."""
    from xbrain.p3_task.runtime.main_wiring import _make_publish  # the factory as wired
    calls = []  # what ran, in order

    def stop(task_id):  # records, then raises like a dead publisher
        calls.append(("stop", task_id))  # it was attempted
        raise RuntimeError("publisher gone")  # and failed

    async def publish_state():  # the state/task broadcast
        calls.append(("state", None))  # it still happened

    hook = _make_publish(None, None, publish_state, stop_motion=stop)  # as the task path builds it
    await hook("t-1", "running", "cancelled", "")  # an operator cancel
    assert calls == [("stop", "t-1"), ("state", None)]  # stop first, broadcast kept
    calls.clear()  # second scenario
    await hook("t-1", "running", "done", "")  # P1 arrival
    assert calls == [("state", None)]  # no clear


# -- through the real P3 loop -----------------------------------------------------
async def _goto_running(tmp_path, monkeypatch):
    """Start the loop with one ready goto and wait for its RP-1 push."""
    run = await start_p3(tmp_path, monkeypatch, rows=[goto_row("t-goto", "ready", 1)])  # loop up
    ok = await wait_for(lambda: route_ops(run[0]))  # dispatched and pushed
    assert ok, "the ready goto was never pushed"  # precondition of every case below
    return run  # (harness, stop, loop task, task_db)


@pytest.mark.asyncio
@pytest.mark.parametrize("action,want_state", [("cancel", "cancelled"), ("pause", "suspended")])
async def test_cancel_and_pause_of_the_running_goto_clear_p1(tmp_path, monkeypatch,
                                                             action, want_state):
    """MUTATION: do not pass stop_motion on the cmd/task path -> only the
    set frame, red (the 2026-11-06 robot finding)."""
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, task_db = await _goto_running(tmp_path, monkeypatch)  # running goto
    try:
        h.send(CMD_TASK_TOPIC, _cmd(action))  # the voice command
        assert await wait_for(lambda: "clear" in route_ops(h)), route_ops(h)  # P1 told to stop
        assert route_ops(h) == ["set", "clear"]  # the push, then exactly one clear
        assert (await task_state(task_db, "t-goto"))[0] == want_state  # and the task moved
    finally:
        await stop_p3(stop, loop)  # never leave the loop running


@pytest.mark.asyncio
async def test_a_preempted_goto_is_cleared_before_the_winner_is_pushed(tmp_path, monkeypatch):
    """15 S7.2 trigger 2. MUTATION: do not pass stop_motion to the scheduler
    tick -> set, set (no clear between) -> red."""
    h, stop, loop, task_db = await _goto_running(tmp_path, monkeypatch)  # running goto, prio 5
    try:
        await insert_rows(task_db, goto_row("t-urgent", "ready", 2, priority=9))  # outranks it
        assert await wait_for(lambda: route_ops(h).count("set") >= 2), route_ops(h)  # winner pushed
        assert route_ops(h)[:3] == ["set", "clear", "set"], route_ops(h)  # loser withdrawn first
        assert (await task_state(task_db, "t-goto"))[0] == "suspended"  # yielding, not cancelled
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_an_estop_suspension_clears_p1(tmp_path, monkeypatch):
    """15 S7.2 trigger 3 / ES-2. The estop zeroes P1 now, but its release
    would let RNS resume the mission of a task that is suspended. MUTATION:
    do not pass stop_motion at the ES-2 site -> no clear -> red."""
    from xbrain.p3_task.runtime.main_wiring import CMD_ESTOP_TOPIC  # P3 subscribes it directly
    h, stop, loop, task_db = await _goto_running(tmp_path, monkeypatch)  # running goto
    try:
        h.send(CMD_ESTOP_TOPIC, {"v": 1, "data": {"cmd_id": "e-1"}})  # any frame freezes (fail-safe)
        assert await wait_for(lambda: "clear" in route_ops(h)), route_ops(h)  # withdrawn
        assert (await task_state(task_db, "t-goto"))[0] == "suspended"  # ES-2 suspended it
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
@pytest.mark.parametrize("p1_state,fail_reason,want_state", [
    ("arrived", None, "done"),               # P1 reached the target
    ("failed", "goal_unreachable", "failed"),  # P1 gave up on its own
])
async def test_a_terminal_reported_by_p1_sends_no_clear(tmp_path, monkeypatch,
                                                        p1_state, fail_reason, want_state):
    """P1 is idle after its own terminal report; a clear there would preempt
    whatever follows (a relative_move that superseded the route, the next
    task's push). MUTATION: pass stop_motion to the path_progress relay ->
    the `failed` case sends a clear -> red. (The `arrived` case alone could
    not catch it: done is not in the clear set.)"""
    from xbrain.p3_task.runtime.main_wiring import STATE_PATH_PROGRESS_TOPIC  # P1-12
    h, stop, loop, task_db = await _goto_running(tmp_path, monkeypatch)  # running goto
    try:
        rev = h.frames("cmd/motion/route")[0]["route_rev"]  # the rev P1 would echo
        h.send(STATE_PATH_PROGRESS_TOPIC, {"data": {  # every 11 S3.5B key, as P1 sends
            "route_id": "goto-w-gate", "route_rev": rev, "loading": False,  # the pushed route
            "state": p1_state, "waypoint_index": 0, "waypoint_total": 1,  # terminal frame
            "seg_done_m": 0.0, "dist_done_m": 0.0, "loop_index": 1, "loop_total": 1,  # one pass
            "dir_sign": 1, "fail_reason": fail_reason}})  # reason only with failed
        await asyncio.sleep(1.0)  # let the loop drain it (two passes)
        assert (await task_state(task_db, "t-goto"))[0] == want_state  # the relay closed the task
        assert "clear" not in route_ops(h)  # and sent nothing to P1
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_cancel_inside_the_ack_window_is_not_re_pushed(tmp_path, monkeypatch):
    """No path_progress confirms the push, so after 3 s the RA-3 branch would
    re-push the WHOLE route -- of a task already cancelled. MUTATION: drop the
    ack_window.reset() in _stop_motion -> a second set after the clear, red."""
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, _db = await _goto_running(tmp_path, monkeypatch)  # push unconfirmed
    try:
        h.send(CMD_TASK_TOPIC, _cmd("cancel"))  # inside the 3 s window
        assert await wait_for(lambda: "clear" in route_ops(h))  # withdrawn
        await asyncio.sleep(4.5)  # past ROUTE_ACK_TIMEOUT_S plus a tick
        assert route_ops(h) == ["set", "clear"], route_ops(h)  # nothing re-pushed
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_an_unconfirmed_push_fails_and_is_withdrawn(tmp_path, monkeypatch):
    """15 S2.4.4: no path_progress ever confirms the push; after the retries
    the task fails (route_unconfirmed). RA-3 is a timeout, not a refusal, so
    P1 may still finish loading the geometry -- it must be withdrawn.
    MUTATION: withdraw=False on the RA-fatal call -> no clear -> red."""
    h, stop, loop, task_db = await _goto_running(tmp_path, monkeypatch)  # never confirmed
    try:
        ok = await wait_for(lambda: "clear" in route_ops(h), n=900)  # retries take ~9 s
        assert ok, route_ops(h)  # it was withdrawn at all
        assert route_ops(h)[-1] == "clear" and route_ops(h).count("set") >= 2  # retries, then clear
        assert (await task_state(task_db, "t-goto"))[0] == "failed"  # route_unconfirmed
    finally:
        await stop_p3(stop, loop)  # cleanup


@pytest.mark.asyncio
async def test_a_resumed_goto_is_pushed_again(tmp_path, monkeypatch):
    """15 S7.3 step 5. Once a pause really withdraws the mission, the resume
    must re-push it. Measured 2026-11-06: the re-dispatch went out as RP-1,
    re-opened the progress run (UNIQUE patrol_progress.task_id), the tick
    raised and the resumed task sat `running` with P1 idle. MUTATION: always
    RP-1 -> no second set -> red."""
    from xbrain.p3_task.runtime.main_wiring import CMD_TASK_TOPIC  # where P4 sends
    h, stop, loop, task_db = await _goto_running(tmp_path, monkeypatch)  # running goto
    try:
        h.send(CMD_TASK_TOPIC, _cmd("pause"))  # suspend it
        assert await wait_for(lambda: "clear" in route_ops(h))  # withdrawn
        h.send(CMD_TASK_TOPIC, _cmd("resume", "c-2"))  # "继续"
        assert await wait_for(lambda: route_ops(h).count("set") >= 2), route_ops(h)  # re-pushed
        assert route_ops(h)[:3] == ["set", "clear", "set"]  # push, withdraw, push again
        assert (await task_state(task_db, "t-goto"))[0] == "running"  # and running
    finally:
        await stop_p3(stop, loop)  # cleanup
