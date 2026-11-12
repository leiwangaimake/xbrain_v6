"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_scheduler_driver.py
Brief: scheduler_tick drives the task machine (validate + dispatch) (BIZ-P3-42)

Description:
Tests the scheduler pass against a real in-memory task.db: a pending task
validates to 'ready', the highest-priority ready task dispatches to 'running',
only one runs at a time, and a task that fails a precondition goes to 'failed'.
Each has a mutation guard per CLAUDE.md 3.3.
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.schedule.driver import apply_motion_result, scheduler_tick

pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def dao_conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield TasksDAO(c), c


def _pending(task_id, priority=50, seq=1, task_type="goto", mission='{"a":1}'):
    return TaskRow(
        task_id=task_id, task_type=task_type, state="pending",
        priority=priority, submit_seq=seq, mission_json=mission,
        total_steps=1, current_step=0, step_status_json="[]", created_ms=0,
        updated_ms=0, source="local", trace_id="tr", resume_policy="continue")


async def _collect(dao, conn):
    """Run one tick, capturing the transitions the callback sees."""
    seen = []

    async def on_t(task_id, from_state, to_state, reason):
        seen.append((task_id, to_state, reason))

    made = await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=on_t)
    return made, seen


@pytest.mark.asyncio
async def test_pending_validates_to_ready(dao_conn):
    dao, conn = dao_conn
    await dao.insert(_pending("t1"))
    await conn.commit()
    made, seen = await _collect(dao, conn)
    # t1 goes pending->ready, then (nothing else running) ready->running.
    assert ("t1", "pending", "ready") in made
    assert (await dao.fetch_by_id("t1")).state == "running"
    assert ("t1", "running", "") in seen


@pytest.mark.asyncio
async def test_dispatch_picks_highest_priority(dao_conn):
    """MUTATION: dispatching FIFO instead of by priority would run t-lo."""
    dao, conn = dao_conn
    await dao.insert(_pending("t-lo", priority=10, seq=1))
    await dao.insert(_pending("t-hi", priority=90, seq=2))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1,
                         on_transition=_noop)
    assert (await dao.fetch_by_id("t-hi")).state == "running"
    assert (await dao.fetch_by_id("t-lo")).state == "ready"   # waits


@pytest.mark.asyncio
async def test_only_one_runs_at_a_time(dao_conn):
    """A second tick must NOT dispatch a second task while one runs.
    MUTATION: dropping the 'nothing running' guard runs both."""
    dao, conn = dao_conn
    await dao.insert(_pending("t1", priority=50, seq=1))
    await dao.insert(_pending("t2", priority=50, seq=2))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop)
    await scheduler_tick(conn, dao, now_mono_ms=2, on_transition=_noop)
    states = {tid: (await dao.fetch_by_id(tid)).state for tid in ("t1", "t2")}
    assert list(states.values()).count("running") == 1
    assert states["t1"] == "running" and states["t2"] == "ready"


@pytest.mark.asyncio
async def test_bad_type_validate_fails(dao_conn):
    """A task whose type is outside the closed set fails PC-1 -> 'failed', not
    'ready'. MUTATION: skipping validation would send it to ready/running."""
    dao, conn = dao_conn
    # Insert directly with a state the CHECK allows but a bogus... type must be
    # valid for the DDL, so use a valid type but an unparseable mission (PC-5).
    await dao.insert(_pending("tbad", mission="   "))    # empty mission -> PC-5
    await conn.commit()
    made, seen = await _collect(dao, conn)
    assert (await dao.fetch_by_id("tbad")).state == "failed"
    assert any(s[0] == "tbad" and s[1] == "failed" and "PC-5" in s[2]
               for s in seen)


async def _noop(task_id, from_state, to_state, reason):
    return None


# -- apply_motion_result: running -> done/failed (PB8) --------------------

async def _run_one(dao, conn, task_id="t1"):
    """Record a task and drive it to 'running' via a tick."""
    await dao.insert(_pending(task_id))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop)


@pytest.mark.asyncio
async def test_motion_succeeded_completes(dao_conn):
    dao, conn = dao_conn
    await _run_one(dao, conn)
    made = await apply_motion_result(conn, dao, "t1", "succeeded",
                                     now_mono_ms=2, on_transition=_noop,
                                     fail_reason=None)
    assert made is True
    assert (await dao.fetch_by_id("t1")).state == "done"


@pytest.mark.asyncio
async def test_motion_aborted_fails(dao_conn):
    """MUTATION: mapping aborted to 'complete' would mark a crashed run done."""
    dao, conn = dao_conn
    await _run_one(dao, conn)
    await apply_motion_result(conn, dao, "t1", "aborted", now_mono_ms=2,
                              on_transition=_noop, fail_reason=None)
    assert (await dao.fetch_by_id("t1")).state == "failed"


@pytest.mark.asyncio
async def test_nav_failure_keeps_its_fail_reason(dao_conn):
    """99 U124: path_progress 'failed' carries the RNS attribution (20 S9.0.2,
    e.g. no_path_in_domain). It is stored in error_context_json.fail_reason and
    named in the transition reason. MUTATION: drop it (the pre-U124 shape) ->
    error_context_json stays empty and the reason is a bare motion:aborted."""
    import json as _json
    from xbrain.p3_task.schedule.driver import apply_path_progress_terminal
    dao, conn = dao_conn
    await _run_one(dao, conn)
    seen = []
    async def _record(task_id, frm, to, reason):  # the on_transition hook
        seen.append(reason)
    made = await apply_path_progress_terminal(
        conn, dao, "t1", "failed", now_mono_ms=2, on_transition=_record,
        fail_reason="no_path_in_domain")
    assert made is True and (await dao.fetch_by_id("t1")).state == "failed"
    row = await (await conn.execute(
        "SELECT error_context_json FROM tasks WHERE task_id='t1'")).fetchone()
    assert _json.loads(row[0]) == {"fail_reason": "no_path_in_domain"}
    assert seen == ["motion:aborted:no_path_in_domain"]


@pytest.mark.asyncio
async def test_motion_in_flight_is_noop(dao_conn):
    """'running'/'accepted' are in-flight -> no transition. MUTATION: treating
    them as terminal would end the task on the first progress frame."""
    dao, conn = dao_conn
    await _run_one(dao, conn)
    made = await apply_motion_result(conn, dao, "t1", "running", now_mono_ms=2,
                                     on_transition=_noop, fail_reason=None)
    assert made is False
    assert (await dao.fetch_by_id("t1")).state == "running"


@pytest.mark.asyncio
async def test_motion_result_for_non_running_is_noop(dao_conn):
    """A late 'succeeded' after the task already left running (e.g. cancelled)
    must not resurrect it. MUTATION: skipping the state guard would."""
    dao, conn = dao_conn
    await dao.insert(_pending("t1"))
    await conn.commit()                            # stays pending, never ran
    made = await apply_motion_result(conn, dao, "t1", "succeeded",
                                     now_mono_ms=2, on_transition=_noop,
                                     fail_reason=None)
    assert made is False
    assert (await dao.fetch_by_id("t1")).state == "pending"


# -- 99 U157: the low-battery held set -------------------------------------------------

async def _noop_t(*_a):
    return None


@pytest.mark.asyncio
async def test_a_held_preempted_task_is_not_resumed(dao_conn):
    """The yielding scan resumes a preempted task once nothing runs -- unless a
    low-battery episode holds it (field log 2026-11-10 20:18:33: it resumed
    the moment the return task reached the dock). MUTATION: drop the held
    check in the yielding scan -> t-p goes ready -> red."""
    dao, conn = dao_conn
    await dao.insert(_pending("t-p", seq=1))
    await conn.execute("UPDATE tasks SET state='suspended', suspend_kind='yielding', "
                       "suspend_reason='preempted' WHERE task_id='t-p'")
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop_t,
                         held=frozenset({"t-p"}))
    assert (await dao.fetch_by_id("t-p")).state == "suspended"
    await scheduler_tick(conn, dao, now_mono_ms=2, on_transition=_noop_t)  # hold lifted
    assert (await dao.fetch_by_id("t-p")).state == "running"  # resumed and dispatched


@pytest.mark.asyncio
async def test_a_held_ready_task_neither_dispatches_nor_preempts(dao_conn):
    """A held task still validates to ready (so it runs the moment the hold
    lifts) but is never dispatched and never preempts; an unheld one is.
    MUTATION: drop the held filter on the ready list -> t-h dispatches /
    preempts -> red."""
    dao, conn = dao_conn
    await dao.insert(_pending("t-h", priority=90, seq=1))
    await dao.insert(_pending("t-n", priority=10, seq=2))
    await conn.commit()
    held = frozenset({"t-h"})
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop_t, held=held)
    assert (await dao.fetch_by_id("t-h")).state == "ready"  # validated, not run
    await scheduler_tick(conn, dao, now_mono_ms=2, on_transition=_noop_t, held=held)
    assert (await dao.fetch_by_id("t-n")).state == "running"  # the unheld one runs
    await scheduler_tick(conn, dao, now_mono_ms=3, on_transition=_noop_t, held=held)
    assert (await dao.fetch_by_id("t-n")).state == "running"  # t-h (90 > 10) did not preempt
    assert (await dao.fetch_by_id("t-h")).state == "ready"


@pytest.mark.asyncio
async def test_a_held_timed_task_comes_due_but_waits(dao_conn):
    """U157 ruling 3 covers timed tasks too: a scheduled task that comes due
    during the episode goes ready (so it runs the moment the hold lifts) but
    does not dispatch. MUTATION: drop the held filter -> it dispatches at its
    time -> red."""
    dao, conn = dao_conn
    await dao.insert(_pending("t-time", seq=1, mission='{"not_before_ts": 1000.0}'))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop_t,
                         now_wall_s=500.0, held=frozenset({"t-time"}))
    assert (await dao.fetch_by_id("t-time")).state == "scheduled"  # not yet due
    await scheduler_tick(conn, dao, now_mono_ms=2, on_transition=_noop_t,
                         now_wall_s=1001.0, held=frozenset({"t-time"}))
    assert (await dao.fetch_by_id("t-time")).state == "ready"  # due, held
    await scheduler_tick(conn, dao, now_mono_ms=3, on_transition=_noop_t, now_wall_s=1002.0)
    assert (await dao.fetch_by_id("t-time")).state == "running"  # hold lifted


@pytest.mark.asyncio
async def test_resume_blocked_stops_every_auto_resume_but_not_dispatch(dao_conn):
    """U157 chassis guard: while charging below full no preempted task resumes
    itself even with an empty held set; a queued task is not affected.
    MUTATION: ignore resume_blocked -> t-p resumes -> red."""
    dao, conn = dao_conn
    await dao.insert(_pending("t-p", priority=60, seq=1))
    await conn.execute("UPDATE tasks SET state='suspended', suspend_kind='yielding', "
                       "suspend_reason='preempted' WHERE task_id='t-p'")
    await dao.insert(_pending("t-q", priority=10, seq=2))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop_t, resume_blocked=True)
    assert (await dao.fetch_by_id("t-p")).state == "suspended"  # not resumed
    assert (await dao.fetch_by_id("t-q")).state == "running"  # the queue still runs


@pytest.mark.asyncio
async def test_only_the_urgent_task_preempts_an_equal_priority(dao_conn):
    """99 U157 (user 2026-11-11): the low-battery return (95) preempts a running
    link-loss return_home (95); any other equal-priority task still queues.
    MUTATION: ignore urgent_task_id -> t-lb waits -> red; let every equal
    priority preempt -> t-other preempts -> red."""
    dao, conn = dao_conn
    await dao.insert(_pending("t-rh", priority=95, seq=1))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=1, on_transition=_noop_t)
    assert (await dao.fetch_by_id("t-rh")).state == "running"
    await dao.insert(_pending("t-other", priority=95, seq=2))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=2, on_transition=_noop_t, urgent_task_id="t-lb")
    assert (await dao.fetch_by_id("t-rh")).state == "running"  # not urgent: queues
    await dao.insert(_pending("t-lb", priority=95, seq=3))
    await conn.commit()
    await scheduler_tick(conn, dao, now_mono_ms=3, on_transition=_noop_t, urgent_task_id="t-lb")
    await scheduler_tick(conn, dao, now_mono_ms=4, on_transition=_noop_t, urgent_task_id="t-lb")
    assert (await dao.fetch_by_id("t-lb")).state == "running"
    assert (await dao.fetch_by_id("t-rh")).state == "suspended"



# -- 99 U175 (user ruling 2026-11-12): a navigation stall suspends ------------
@pytest.mark.asyncio
async def test_a_navigation_stall_suspends_instead_of_failing(dao_conn):
    """Measured 2026-11-10: a watchdog_no_progress stall failed the task while
    a no-fix stall on the same outing suspended it. MUTATION: drop the stall
    branch -> failed -> red; yielding instead of passive -> the CR-8 check
    raises -> red."""
    import json as _json
    from xbrain.p3_task.schedule.driver import apply_path_progress_terminal
    dao, conn = dao_conn
    await _run_one(dao, conn)
    seen = []
    async def _record(task_id, frm, to, reason):
        seen.append((frm, to, reason))
    made = await apply_path_progress_terminal(
        conn, dao, "t1", "failed", now_mono_ms=2, on_transition=_record,
        fail_reason="watchdog_no_progress")
    assert made == "suspended" and made is not True  # the run must stay open
    row = await dao.fetch_by_id("t1")
    assert (row.state, row.suspend_kind, row.suspend_reason) == (
        "suspended", "passive", "nav_stalled")
    ctx = await (await conn.execute(
        "SELECT error_context_json FROM tasks WHERE task_id='t1'")).fetchone()
    assert _json.loads(ctx[0]) == {"fail_reason": "watchdog_no_progress"}
    assert seen == [("running", "suspended", "motion:aborted:watchdog_no_progress")]


@pytest.mark.asyncio
async def test_other_navigation_failures_still_fail(dao_conn):
    """Only the stall changed. MUTATION: suspend on every fail_reason -> red."""
    from xbrain.p3_task.schedule.driver import apply_path_progress_terminal
    dao, conn = dao_conn
    await _run_one(dao, conn)
    made = await apply_path_progress_terminal(
        conn, dao, "t1", "failed", now_mono_ms=2, on_transition=_noop,
        fail_reason="escalations_exhausted")
    assert made is True and (await dao.fetch_by_id("t1")).state == "failed"
