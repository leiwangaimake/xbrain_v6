"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_command.py
Brief: cmd/task TaskCommand -- envelope + five actions against task.db (11 S7.2)

Description:
The contract-shaped cmd/task path, which until 2026-08-20 did not exist: P3's
receiver understood only p4_agent's private `task_request` shape and SKIPPED
everything else, so an HMI or cloud frame in the S7.2 shape was dropped with no
ack at all.

The cases that carry this batch:

  * `task_id` is required and there is no "omit = the current task". S7.2 gives
    four reasons; the first is that the queue is live, so the shorthand pauses
    whatever happens to be running when the command lands.
  * `clear_queue` must not touch running or suspended. An operator clearing a
    backlog is not asking to stop the robot that is currently driving.
  * `pause` writes suspend_kind/suspend_reason, and `resume` clears them. The
    tasks DDL pairs them with the state, so getting either half wrong is an
    IntegrityError on an operation the operator expects to just work.
"""
from __future__ import annotations

import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.common.enums import TASK_ACTION
from xbrain.common.errors import E_NOT_FOUND, E_SCHEMA, E_TASK_STATE
from xbrain.p3_task.dao.tasks_dao import TasksDAO
from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload
from xbrain.p3_task.ingest.task_command import (
    TaskCommandError, parse_task_command,
)
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS

pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def ctx():
    conn = await aiosqlite.connect(":memory:")
    for stmt in ALL_DDL_STATEMENTS:
        await conn.execute(stmt)
    await conn.commit()
    yield TaskContext(task_conn=conn, dao=TasksDAO(conn))
    await conn.close()


def _submit(task_id="t-20260820-001", **over):
    task = {"task_id": task_id, "type": "patrol", "priority": 50,
            "route_id": "r-east", "params": {"loops": 2, "text": "去东门巡逻"}}
    task.update(over.pop("task", {}))
    frame = {"cmd_id": "c-1", "action": "submit", "task": task,
             "source": "hmi"}
    frame.update(over)
    return frame


async def _seed(ctx, task_id, state, task_type="patrol"):
    kind = "passive" if state == "suspended" else None
    reason = "operator_pause" if state == "suspended" else None
    await ctx.task_conn.execute(
        "INSERT INTO tasks (task_id, task_type, state, priority, submit_seq, "
        " mission_json, total_steps, current_step, step_status_json, source, "
        " resume_policy, trace_id, created_ms, updated_ms, suspend_kind, "
        " suspend_reason) "
        "VALUES (?, ?, ?, 50, 1, '{}', 0, 0, '[]', 'local', 'restart', "
        " 'tr', 1, 1, ?, ?)",
        (task_id, task_type, state, kind, reason))
    await ctx.task_conn.commit()


async def _state_of(ctx, task_id):
    cur = await ctx.task_conn.execute(
        "SELECT state, suspend_kind, suspend_reason FROM tasks WHERE task_id=?",
        (task_id,))
    return await cur.fetchone()


# ------------------------------------------------------------ envelope -----

def test_action_closed_set():
    assert set(TASK_ACTION) == {"submit", "cancel", "pause", "resume",
                                "clear_queue"}
    with pytest.raises(TaskCommandError, match="action") as ei:
        parse_task_command({"cmd_id": "c-1", "action": "abort",
                            "task_id": "t-1"})
    assert ei.value.code == E_SCHEMA


@pytest.mark.parametrize("action", ["cancel", "pause", "resume"])
def test_task_id_is_required_with_no_current_task_shorthand(action):
    """*** S7.2 forbids "omit = the current task", and the first of its four
    reasons is the one that bites: the queue is LIVE. Between the operator
    seeing "A is running" and the command arriving, A may have finished and B
    started -- the shorthand pauses B, and nothing in the log shows it.

    MUTATION: fall back to "the running task" when task_id is absent -- this
    reddens, and on the robot it silently acts on the wrong task.
    """
    with pytest.raises(TaskCommandError, match="task_id") as ei:
        parse_task_command({"cmd_id": "c-1", "action": action})
    assert ei.value.code == E_SCHEMA


def test_submit_takes_its_id_from_the_task_body():
    cmd = parse_task_command(_submit())
    assert cmd.task_id == "t-20260820-001" and cmd.action == "submit"


def test_two_disagreeing_ids_are_refused_not_resolved():
    """S7.2: on submit the two must be equal if both appear. Refused rather
    than resolved by precedence -- two ids in one frame means the sender is
    confused about which task it is submitting, and picking one buries that."""
    frame = _submit()                    # task.task_id = t-20260820-001
    frame["task_id"] = "t-other"         # envelope disagrees with the body
    with pytest.raises(TaskCommandError, match="disagree"):
        parse_task_command(frame)
    # Equal is fine (a sender may legitimately carry both).
    frame["task_id"] = frame["task"]["task_id"]
    assert parse_task_command(frame).task_id == "t-20260820-001"


def test_missing_cmd_id_is_refused():
    """cmd_id is the idempotency key (S2.3): without it a redelivery cannot be
    told from a second intent, and submit's duplicate rule cannot hold."""
    frame = _submit()
    del frame["cmd_id"]
    with pytest.raises(TaskCommandError, match="cmd_id"):
        parse_task_command(frame)


def test_the_legacy_p4_shape_is_no_longer_a_task_command():
    """*** p4's private task_request shape is gone (2026-08-20).

    It is asserted rather than deleted with the shim: if a stale p4 build were
    redeployed, this shape must be REFUSED with a schema error and acked as
    such -- not quietly re-accepted by a receiver that still knows how to read
    it. A second accepted shape is what put the two senders out of step to
    begin with (CLAUDE.md 9.3).
    """
    with pytest.raises(TaskCommandError, match="cmd_id"):
        parse_task_command({"task_request": {"task_type": "patrol"}})


# -------------------------------------------------------------- submit ----

@pytest.mark.asyncio
async def test_submit_records_the_task_with_its_route_geo_id(ctx):
    """*** route_id lands in tasks.route_geo_id -- the column that was NULL on
    every task recorded until now.

    MUTATION: drop it and geo_refs is back to name-matching only, which is what
    made an id-only impact query answer "referenced by nothing" for a route
    three tasks were about to run (11 S7.9.4).
    """
    ack = await handle_task_payload(_submit(), ctx, now_mono_ms=1000,
                                    created_at="2026-08-20T10:00:00Z")
    assert ack["result"] == "accepted", ack
    assert ack["detail"]["applied"]["route_id"] == "r-east"
    cur = await ctx.task_conn.execute(
        "SELECT task_type, state, route_geo_id, source, command_text, "
        " mission_json FROM tasks WHERE task_id='t-20260820-001'")
    row = await cur.fetchone()
    assert row[0] == "patrol" and row[1] == "pending"
    # 15 S4.2 maps the HMI channel onto the `local` origin class -- the
    # scheduler ranks on that five-value axis, and `hmi` is not one of them.
    assert row[2] == "r-east" and row[3] == "local"
    assert row[4] == "去东门巡逻"
    assert json.loads(row[5])["params"]["loops"] == 2


@pytest.mark.asyncio
async def test_submit_without_a_task_id_gets_one_allocated(ctx):
    """*** S7.2 as corrected 2026-08-20: task.task_id is optional.

    It has to be. The form is t-YYYYMMDD-NNN with a per-day sequence only P3
    holds, so p4_agent and p5_gateway -- both listed publishers of this key
    (S2.2) -- cannot produce a legal one. The original "required" reading made
    them structurally unable to submit at all.

    MUTATION: go back to requiring it -- every voice and HMI task is refused
    with E_SCHEMA, and the two senders have no way to comply.
    """
    frame = _submit()
    del frame["task"]["task_id"]
    ack = await handle_task_payload(frame, ctx, now_mono_ms=1,
                                    date_str="20260820")
    assert ack["result"] == "accepted", ack
    allocated = ack["detail"]["applied"]["task_id"]
    # 15 S9.5 form is preserved -- P3 is the one that can produce it.
    assert allocated.startswith("t-20260820-") and allocated.endswith("001")
    cur = await ctx.task_conn.execute("SELECT task_id FROM tasks")
    assert (await cur.fetchone())[0] == allocated


@pytest.mark.asyncio
async def test_a_redelivered_cmd_id_does_not_mint_a_second_task(ctx):
    """*** 11 S2.3: the receiver de-duplicates on cmd_id. With the id now
    allocated by P3, a redelivered submit has NOTHING else to be recognised by
    -- the task it created has an id the sender never saw.

    MUTATION: drop the cmd-log check and a retry after a lost ack mints a
    SECOND patrol the operator never asked for, with a different id, and both
    run.
    """
    frame = _submit()
    del frame["task"]["task_id"]
    first = await handle_task_payload(frame, ctx, now_mono_ms=1,
                                      date_str="20260820")
    again = await handle_task_payload(frame, ctx, now_mono_ms=2,
                                      date_str="20260820")
    assert first["result"] == "accepted" and again["result"] == "duplicate"
    assert again["detail"]["replayed"] is True
    # And the replay tells the sender which id it got the first time.
    assert again["detail"]["applied"]["task_id"] == \
        first["detail"]["applied"]["task_id"]
    cur = await ctx.task_conn.execute("SELECT COUNT(*) FROM tasks")
    assert (await cur.fetchone())[0] == 1


@pytest.mark.asyncio
async def test_redelivered_cancel_replays_rather_than_reapplying(ctx):
    """The same rule on a state change. MUTATION: skip the log for
    cancel/pause/resume and a retry re-runs the transition -- harmless for
    cancel, but a re-applied pause on a task that was resumed in between
    suspends it again behind the operator's back."""
    await _seed(ctx, "t-1", "ready")
    frame = {"cmd_id": "c-same", "action": "cancel", "task_id": "t-1"}
    first = await handle_task_payload(frame, ctx, now_mono_ms=5)
    again = await handle_task_payload(frame, ctx, now_mono_ms=6)
    assert first["result"] == "accepted" and again["result"] == "duplicate"


@pytest.mark.asyncio
async def test_submit_is_idempotent_on_task_id(ctx):
    """S7.2: a repeat returns duplicate and does NOT re-execute."""
    first = await handle_task_payload(_submit(), ctx, now_mono_ms=1)
    again = await handle_task_payload(_submit(cmd_id="c-2"), ctx,
                                      now_mono_ms=2)
    assert first["result"] == "accepted" and again["result"] == "duplicate"
    assert again["detail"]["applied"]["changed"] is False
    cur = await ctx.task_conn.execute("SELECT COUNT(*) FROM tasks")
    assert (await cur.fetchone())[0] == 1


@pytest.mark.asyncio
async def test_unknown_channel_is_refused_not_mapped_to_the_nearest(ctx):
    """*** 11 S13.6 forbids interpreting an off-set value as the nearest thing,
    and here that rule guards the SCHEDULER'S ORDERING.

    15 S4.2 ranks on tasks.source: cloud 80 > wecom 60 > local 40. A silent
    fallback to `local` means the first time the cloud spells its channel
    differently -- "Cloud", a new "cloud_v2" -- its tasks quietly drop from 80
    to 40 and start losing to a WeChat message. Nothing logs it; the task runs,
    just later, and the priority column reads like somebody chose it.

    MUTATION: `_CHANNEL_TO_SOURCE.get(channel, "local")` -- this case reddens
    and no other does, because every other case sends a channel that IS known.
    """
    for bad in ("Cloud", "satellite", "cloud_v2"):
        ack = await handle_task_payload(
            _submit(source=bad), ctx, now_mono_ms=1)
        assert ack["result"] in ("rejected", "error"), bad
        assert bad in json.dumps(ack, ensure_ascii=False), bad
    cur = await ctx.task_conn.execute("SELECT COUNT(*) FROM tasks")
    assert (await cur.fetchone())[0] == 0


@pytest.mark.asyncio
async def test_priority_defaults_to_the_origin_table_not_a_middle_value(ctx):
    """15 S4.2 gives a priority per origin. MUTATION: default to 50 (the old
    made-up middle) -- an omitted priority then sits between cloud (80) and
    wecom (60), so a LOCAL task outranks a WeChat one and loses to a cloud one
    regardless of where it came from."""
    frame = _submit(source="cloud")
    del frame["task"]["priority"]
    await handle_task_payload(frame, ctx, now_mono_ms=1)
    cur = await ctx.task_conn.execute(
        "SELECT priority, source FROM tasks WHERE task_id='t-20260820-001'")
    assert await cur.fetchone() == (80, "cloud")


@pytest.mark.asyncio
async def test_submit_refuses_an_off_set_type(ctx):
    ack = await handle_task_payload(
        _submit(task={"type": "hover"}), ctx, now_mono_ms=1)
    assert ack["result"] == "error" or ack["result"] == "rejected"
    cur = await ctx.task_conn.execute("SELECT COUNT(*) FROM tasks")
    assert (await cur.fetchone())[0] == 0


# ---------------------------------------------------- cancel / pause -------

@pytest.mark.asyncio
async def test_cancel_moves_a_queued_task_to_cancelled(ctx):
    await _seed(ctx, "t-1", "ready")
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "cancel", "task_id": "t-1",
         "reason": "operator_hmi"}, ctx, now_mono_ms=5)
    assert ack["result"] == "accepted"
    assert ack["detail"]["applied"]["state"] == "cancelled"
    assert (await _state_of(ctx, "t-1"))[0] == "cancelled"


@pytest.mark.asyncio
async def test_cancel_on_a_terminal_task_is_duplicate_not_error(ctx):
    """S7.2: already terminal -> duplicate. The operator asked for a state the
    task is already in; an error would send them looking for a problem that
    does not exist. MUTATION: return E_TASK_STATE and every re-click of cancel
    on a finished task reads as a failure."""
    await _seed(ctx, "t-done", "done")
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "cancel", "task_id": "t-done"}, ctx,
        now_mono_ms=5)
    assert ack["result"] == "duplicate"
    assert (await _state_of(ctx, "t-done"))[0] == "done"


@pytest.mark.asyncio
async def test_cancel_on_a_missing_task(ctx):
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "cancel", "task_id": "t-nope"}, ctx,
        now_mono_ms=5)
    assert ack["result"] == "rejected" and ack["code"] == E_NOT_FOUND


@pytest.mark.asyncio
async def test_pause_writes_the_paired_suspend_fields(ctx):
    """*** The tasks DDL pairs suspend_kind/suspend_reason with the state
    (non-null IFF suspended). MUTATION: write only the state -- sqlite rejects
    it, and a pause the operator has every reason to expect works comes back as
    an internal error.

    CR-8: an operator pause is passive; `yielding` pairs ONLY with preempted /
    mode_takeover.
    """
    await _seed(ctx, "t-run", "running")
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "pause", "task_id": "t-run"}, ctx,
        now_mono_ms=5)
    assert ack["result"] == "accepted"
    assert await _state_of(ctx, "t-run") == ("suspended", "passive",
                                             "operator_pause")


@pytest.mark.asyncio
async def test_resume_clears_the_suspend_fields(ctx):
    """The other half of the same pairing. MUTATION: leave the fields behind on
    resume -- the CHECK rejects the row and the task is stuck suspended."""
    await _seed(ctx, "t-susp", "suspended")
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "resume", "task_id": "t-susp"}, ctx,
        now_mono_ms=5)
    assert ack["result"] == "accepted"
    assert await _state_of(ctx, "t-susp") == ("ready", None, None)


@pytest.mark.parametrize("action,state", [
    ("pause", "ready"), ("pause", "pending"), ("pause", "suspended"),
    ("resume", "running"), ("resume", "ready"),
])
@pytest.mark.asyncio
async def test_pause_and_resume_reject_the_wrong_state(action, state, ctx):
    """S7.2: pause only from running, resume only from suspended. MUTATION:
    accept any state -- a pause on a queued task would suspend something that
    was never driving, and the scheduler would then skip it forever."""
    await _seed(ctx, "t-x", state)
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": action, "task_id": "t-x"}, ctx,
        now_mono_ms=5)
    assert ack["result"] == "rejected" and ack["code"] == E_TASK_STATE
    assert (await _state_of(ctx, "t-x"))[0] == state


# --------------------------------------------------------- clear_queue ----

@pytest.mark.asyncio
async def test_clear_queue_spares_running_and_suspended(ctx):
    """*** S7.2 in one case. An operator clearing a backlog is NOT asking to
    stop the robot that is currently driving.

    MUTATION: include running/suspended in the sweep -- the patrol under way is
    cancelled by a button whose label says "clear the queue".
    """
    await _seed(ctx, "t-run", "running")
    await _seed(ctx, "t-susp", "suspended")
    for i, state in enumerate(("pending", "scheduled", "blocked", "ready")):
        await _seed(ctx, "t-q%d" % i, state)
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "clear_queue"}, ctx, now_mono_ms=5)
    assert ack["result"] == "accepted"
    assert sorted(ack["detail"]["cleared_ids"]) == ["t-q0", "t-q1", "t-q2",
                                                    "t-q3"]
    assert (await _state_of(ctx, "t-run"))[0] == "running"
    assert (await _state_of(ctx, "t-susp"))[0] == "suspended"


@pytest.mark.asyncio
async def test_clear_queue_reports_an_empty_sweep_as_accepted(ctx):
    """S7.2: "一个都没清也要回 accepted + 空列表". MUTATION: return duplicate
    or an error on an empty queue and the HMI shows a failure for a button that
    did exactly what it should."""
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "clear_queue"}, ctx, now_mono_ms=5)
    assert ack["result"] == "accepted" and ack["detail"]["cleared_ids"] == []


@pytest.mark.asyncio
async def test_clear_queue_does_no_duplicate_check(ctx):
    """S7.2: it is a SET operation, not an object operation -- the same cmd_id
    twice legitimately clears two different sets. MUTATION: dedupe on cmd_id
    and a retry after a lost ack silently leaves the second batch queued."""
    await _seed(ctx, "t-a", "ready")
    first = await handle_task_payload(
        {"cmd_id": "c-same", "action": "clear_queue"}, ctx, now_mono_ms=5)
    await _seed(ctx, "t-b", "ready")
    second = await handle_task_payload(
        {"cmd_id": "c-same", "action": "clear_queue"}, ctx, now_mono_ms=6)
    assert first["detail"]["cleared_ids"] == ["t-a"]
    assert second["result"] == "accepted"
    assert second["detail"]["cleared_ids"] == ["t-b"]


@pytest.mark.asyncio
async def test_clear_queue_honours_a_source_filter(ctx):
    await _seed(ctx, "t-local", "ready")
    await ctx.task_conn.execute(
        "UPDATE tasks SET source='cloud' WHERE task_id='t-local'")
    await _seed(ctx, "t-other", "ready")
    await ctx.task_conn.commit()
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "clear_queue",
         "filter": {"source": "cloud"}}, ctx, now_mono_ms=5)
    assert ack["detail"]["cleared_ids"] == ["t-local"]
    assert (await _state_of(ctx, "t-other"))[0] == "ready"


# ------------------------------------------------------------- the ack ----

@pytest.mark.asyncio
async def test_every_outcome_produces_an_ack(ctx):
    """W2/W7 require an ack within 2 s, and a frame that produced none leaves
    the browser's dialog spinning. MUTATION: return None on a refusal path."""
    for frame in ({"cmd_id": "c-1", "action": "nope"},
                  {"cmd_id": "c-2", "action": "cancel", "task_id": "t-gone"},
                  {"action": "cancel", "task_id": "t-1"}):
        ack = await handle_task_payload(frame, ctx, now_mono_ms=1)
        assert ack is not None and ack["result"] in ("rejected", "error")
        assert "code" in ack


@pytest.mark.asyncio
async def test_applied_is_independently_readable(ctx):
    """AP-2: detail.applied must read as a whole statement without the caller
    remembering what it asked for. MUTATION: return {"ok": true} -- AP-2 says
    in as many words that does not count."""
    await _seed(ctx, "t-1", "ready")
    ack = await handle_task_payload(
        {"cmd_id": "c-1", "action": "cancel", "task_id": "t-1"}, ctx,
        now_mono_ms=5)
    applied = ack["detail"]["applied"]
    assert applied["state"] == "cancelled" and applied["from"] == "ready"


@pytest.mark.asyncio
async def test_transitions_reach_the_publish_seam(ctx):
    """An HMI-driven change must reach state/task and the S6.2 task events the
    same way a scheduled one does. MUTATION: skip on_transition and the HMI
    cancels a task that every other consumer still believes is queued."""
    seen = []

    async def _on(task_id, from_state, to_state, reason):
        # from_state 一并记下: 事件判别现在按 (from, to) 做, 只记 to 就看不出
        # "从哪来"这一半, 而那一半正是拒绝与取消的唯一区别.
        seen.append((task_id, from_state, to_state, reason))

    await _seed(ctx, "t-1", "ready")
    await handle_task_payload(
        {"cmd_id": "c-1", "action": "cancel", "task_id": "t-1",
         "reason": "operator_hmi"}, ctx, now_mono_ms=5, on_transition=_on)
    assert seen == [("t-1", "ready", "cancelled", "operator_hmi")]
