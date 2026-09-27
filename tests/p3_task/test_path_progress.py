"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_path_progress.py
Brief: EX-3 / EX-4 -- path_progress parse, PP-1 flush, persistence, terminal close

Description:
Guards the three rules of the state/motion/path_progress intake:

  * 11 S3.5B parse (closed-set state, mandatory dir_sign, fail_reason paired
    with `failed` and only with it);
  * 15 S9.5B PP-1 "waypoint change OR progress_flush_s, whichever comes
    first" -- both halves, because each one alone passes half these tests;
  * 12 S4.3.1 LP-3 arrived -> task done, plus the deliberate omission of
    path_progress 'aborted'.

The last test drives the REAL wiring (_amain against a fake session) and
asserts a frame fed into the subscriber reaches task.db. Kept separate from
the unit tests above on purpose: this repo has been bitten by suites that
build the objects themselves, assert on them, and stay green whether or not
anything is actually connected -- the module-level tests here would all pass
with the subscriber commented out of main_wiring.
"""
from __future__ import annotations

import asyncio
import json

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.simple_daos import PatrolProgressDAO
from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.runtime.progress_sink import apply_path_progress
from xbrain.p3_task.state.path_progress import (MOTION_RESULT_FOR_PATH_STATE,
                                                PathProgressError,
                                                ProgressTracker,
                                                parse_path_progress)

pytestmark = pytest.mark.no_device


def _frame(**over):
    """A valid 11 S3.5B body; override any field."""
    body = {"v": 1, "route_id": "r-east_gate", "route_rev": 7,
            "loading": False, "waypoint_index": 3, "waypoint_total": 64,
            "seg_done_m": 12.4, "dist_done_m": 431.8, "fail_reason": None,
            "loop_index": 0, "loop_total": 2, "dir_sign": 1,
            "state": "running", "ts": 1753660790.4, "mono": 88231.442}
    body.update(over)
    return body


# --- 11 S3.5B parse -------------------------------------------------------

def test_a_valid_frame_parses_to_the_wire_field_names():
    p = parse_path_progress(_frame())
    assert (p.route_id, p.route_rev, p.waypoint_index, p.dir_sign) == (
        "r-east_gate", 7, 3, 1)
    assert p.is_terminal is False


@pytest.mark.parametrize("state", ["arrived", "aborted", "failed"])
def test_the_three_terminal_states_are_terminal(state):
    over = {"state": state}
    if state == "failed":
        over["fail_reason"] = "corridor_blocked"
    assert parse_path_progress(_frame(**over)).is_terminal is True


def test_a_state_outside_the_closed_set_raises():
    """CLAUDE.md 3.5: no silent pass-through, no nearest-known-value.

    MUTATION: accept the unknown value and treat it as 'running'. Nothing
    fails here, and a P1 that starts publishing a new terminal state would be
    read as "still going" -- the task never closes and nobody sees why.
    """
    with pytest.raises(PathProgressError) as exc:
        parse_path_progress(_frame(state="finished"))
    assert "closed set" in str(exc.value)


def test_failed_without_fail_reason_raises():
    """11 S3.5B v2.0 verbatim: fail_reason is mandatory when state ==
    'failed'. Splitting `failed` out of `aborted` exists precisely so P3 can
    say WHY; a failed frame with no reason puts that hole straight back.

    MUTATION: make fail_reason optional -> this goes red, and in the field a
    navigation failure becomes indistinguishable from a preemption again.
    """
    with pytest.raises(PathProgressError):
        parse_path_progress(_frame(state="failed"))


def test_fail_reason_on_a_non_failed_state_raises():
    """The pairing is both ways. A reason on a `running` frame means the
    sender and this parser disagree about what the field is for, and the
    cheap reading ("just ignore it") is how a real failure reason ends up
    attached to a frame nobody inspects."""
    with pytest.raises(PathProgressError):
        parse_path_progress(_frame(state="running", fail_reason="oops"))


def test_missing_dir_sign_raises():
    """11 S3.5B calls dir_sign mandatory and records that the contract once
    omitted it, leaving 15 S9.5's NOT NULL column with no source.

    MUTATION: default it to +1 and a pingpong patrol running backwards is
    recorded as running forwards -- the resume then drives the wrong way
    (the exact bug S9.5B's `direction` rule exists to prevent).
    """
    body = _frame()
    del body["dir_sign"]
    with pytest.raises(PathProgressError):
        parse_path_progress(body)


def test_a_boolean_is_not_accepted_as_an_index():
    """bool is an int subclass in Python and json `true` decodes to it, so
    `waypoint_index: true` would silently read as 1."""
    with pytest.raises(PathProgressError):
        parse_path_progress(_frame(waypoint_index=True))


def test_waypoint_index_past_total_raises_before_the_db_does():
    """Same bound as the 15 S9.5 CHECK. Caught here it names the frame;
    caught at the UPDATE it is an IntegrityError with no way back."""
    with pytest.raises(PathProgressError):
        parse_path_progress(_frame(waypoint_index=64, waypoint_total=64))


# --- 15 S9.5B PP-1 --------------------------------------------------------

def test_pp1_flushes_on_a_waypoint_change():
    t = ProgressTracker(flush_period_s=5.0)
    assert t.decide("t1", parse_path_progress(_frame(waypoint_index=3)),
                    now_mono_ms=1000).reason == "first"
    d = t.decide("t1", parse_path_progress(_frame(waypoint_index=4)),
                 now_mono_ms=1200)
    assert (d.flush, d.reason) == (True, "waypoint")


def test_pp1_does_not_flush_a_2hz_frame_with_an_unchanged_index():
    """The other half of PG-2. Without it, 2 Hz all day is ~170k fsyncs.

    MUTATION: always return flush=True and this goes red -- and on the robot
    the card wear is real but invisible until it is not.
    """
    t = ProgressTracker(flush_period_s=5.0)
    t.decide("t1", parse_path_progress(_frame()), now_mono_ms=1000)
    d = t.decide("t1", parse_path_progress(_frame(seg_done_m=13.9)),
                 now_mono_ms=1500)
    assert (d.flush, d.reason) == (False, "memory_only")


def test_pp1_flushes_on_the_period_even_with_an_unchanged_index():
    """15 S9.5B rewrote PP-1 for exactly this case: a 300 m segment takes
    minutes at patrol speed, and index-only flushing loses the whole segment
    on a power cut. 15 TC-41 is the same scenario.

    MUTATION: drop the period branch (index-only, the pre-v0.4 rule) -> red.
    That mutation is the one that matters: the suite passes without this test
    because every other case changes the index.
    """
    t = ProgressTracker(flush_period_s=5.0)
    t.decide("t1", parse_path_progress(_frame()), now_mono_ms=1000)
    assert t.decide("t1", parse_path_progress(_frame()),
                    now_mono_ms=3000).flush is False
    d = t.decide("t1", parse_path_progress(_frame()), now_mono_ms=6000)
    assert (d.flush, d.reason) == (True, "period")
    # And the period restarts from the flush, not from the first frame.
    assert t.decide("t1", parse_path_progress(_frame()),
                    now_mono_ms=9000).flush is False


def test_a_terminal_frame_always_flushes():
    """It is the last frame there will be, so "wait for the period" means
    never -- the final position would never reach the disk."""
    t = ProgressTracker(flush_period_s=5.0)
    t.decide("t1", parse_path_progress(_frame()), now_mono_ms=1000)
    assert t.decide("t1", parse_path_progress(_frame(state="arrived")),
                    now_mono_ms=1100).flush is True


def test_a_task_never_seen_is_stale_not_fresh():
    """11 S3.5B: no frame for 3 s -> progress.persisted false. "We have heard
    nothing" and "all is well" must not give the same answer.

    MUTATION: return False for an unknown task and a P1 that never publishes
    at all reads as healthy progress forever.
    """
    t = ProgressTracker()
    assert t.is_stale("nobody", now_mono_ms=1000) is True
    t.decide("t1", parse_path_progress(_frame()), now_mono_ms=1000)
    assert t.is_stale("t1", now_mono_ms=3900) is False
    assert t.is_stale("t1", now_mono_ms=4000) is True


def test_a_zero_flush_period_is_refused():
    """Zero looks like extra safety and is an fsync per 2 Hz frame."""
    with pytest.raises(ValueError):
        ProgressTracker(flush_period_s=0.0)


# --- 12 S4.3.1 LP-3 terminal mapping --------------------------------------

def test_arrived_maps_to_the_completing_result_and_aborted_maps_to_nothing():
    """LP-3 verbatim: P1 only says 'arrived'; mapping it to task `done` is
    P3's job. path_progress 'aborted' is preemption or e-stop, which P3 has
    already turned into a resumable `suspended` -- closing the task here
    would overwrite that and destroy the breakpoint.

    MUTATION: add "aborted": "aborted" to the table. Every other test here
    still passes, and a preempted patrol becomes permanently `failed` -- U07
    resume silently stops working.
    """
    assert MOTION_RESULT_FOR_PATH_STATE["arrived"] == "succeeded"
    assert MOTION_RESULT_FOR_PATH_STATE["failed"] == "aborted"
    assert "aborted" not in MOTION_RESULT_FOR_PATH_STATE


# --- the db half (EX-3 persistence + EX-4 close) --------------------------

@pytest_asyncio.fixture
async def db():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield c


async def _running_patrol(conn, *, task_id="t1", route_id="r-east_gate",
                          route_rev=7):
    dao = TasksDAO(conn)
    await dao.insert(TaskRow(
        task_id=task_id, task_type="patrol", state="pending", priority=5,
        submit_seq=1, mission_json="{}", total_steps=1, current_step=0,
        step_status_json="[]", created_ms=0, updated_ms=0, source="local",
        trace_id="tr", resume_policy="continue"))
    await dao.dispatch_task(task_id, 0, started_at="2026-09-28T00:00:00Z",
                            started_mono=0.0, started_boot="b1")
    await dao.update_state(task_id, "running", 0)
    pp = PatrolProgressDAO(conn)
    await pp.start_run(task_id=task_id, route_name="east gate",
                       route_geo_id=route_id, route_rev=route_rev,
                       direction="forward", loop_mode="closed",
                       waypoint_total=64, route_total_m=1620.0, loop_total=2,
                       now_iso="2026-09-28T00:00:00.000Z")
    await conn.commit()
    return dao, pp


async def _noop_transition(*_a, **_k):
    return None


@pytest.mark.asyncio
async def test_a_frame_lands_the_breakpoint_columns(db):
    """EX-3: the U07a breakpoint (waypoint_index, seg_done_m) plus the S7.3A
    remap inputs reach the row. Before this batch every one of them stayed at
    its DEFAULT while a patrol ran.

    MUTATION: bind dist_done_m where seg_done_m goes (a plausible slip, both
    are metres) -> red here. Nothing else in the tree would notice; the
    remap would just compute the wrong L0.
    """
    dao, pp = await _running_patrol(db)
    t = ProgressTracker()
    out = await apply_path_progress(
        _frame(waypoint_index=17), conn=db, dao=dao, progress_dao=pp,
        tracker=t, now_mono_ms=1000, now_wall_ms=1_785_000_000_000,
        on_transition=_noop_transition, boot_id="b1")
    assert (out["accepted"], out["flushed"]) == (True, True)
    row = await pp.fetch_active("t1")
    # (route_geo_id, route_rev, direction, loop_mode, waypoint_index,
    #  waypoint_total, seg_done_m, dist_done_m, ...)
    assert row[4] == 17 and row[6] == 12.4 and row[7] == 431.8


@pytest.mark.asyncio
async def test_a_frame_for_another_route_is_refused(db):
    """P1 keeps its geometry across a P3 restart (11 S3.5A RG-3), so the first
    frames after a restart can describe the PREVIOUS route. Writing those onto
    the new run moves the breakpoint onto a different path, and U07a then
    resumes there -- a robot driving somewhere nobody asked for.

    MUTATION: drop route_matches (or compare route_id only, ignoring
    route_rev) -> red. The route_rev half matters on its own: a re-recorded
    route keeps its route_id.
    """
    dao, pp = await _running_patrol(db)
    t = ProgressTracker()
    out = await apply_path_progress(
        _frame(route_id="r-west_gate"), conn=db, dao=dao, progress_dao=pp,
        tracker=t, now_mono_ms=1000, now_wall_ms=1_785_000_000_000,
        on_transition=_noop_transition, boot_id="b1")
    assert (out["accepted"], out["reason"]) == (False, "route_mismatch")
    out = await apply_path_progress(
        _frame(route_rev=8), conn=db, dao=dao, progress_dao=pp, tracker=t,
        now_mono_ms=1000, now_wall_ms=1_785_000_000_000,
        on_transition=_noop_transition, boot_id="b1")
    assert out["reason"] == "route_mismatch"
    assert (await pp.fetch_active("t1"))[4] == -1      # untouched


@pytest.mark.asyncio
async def test_arrived_closes_the_task_and_the_run(db):
    """EX-4. 12 S4.3.1 LP-3: P3 maps `arrived` to task `done`. Without this
    the patrol stays `running` forever and the operator cannot tell a finished
    patrol from a hung P1.

    MUTATION: stop calling apply_path_progress_terminal -> red on the state.
    """
    dao, pp = await _running_patrol(db)
    seen = []

    async def _rec(task_id, frm, to, reason):
        seen.append((task_id, frm, to, reason))

    out = await apply_path_progress(
        _frame(state="arrived", waypoint_index=63), conn=db, dao=dao,
        progress_dao=pp, tracker=ProgressTracker(), now_mono_ms=1000,
        now_wall_ms=1_785_000_000_000, on_transition=_rec, boot_id="b1")
    assert out["closed"] is True
    assert (await dao.fetch_by_id("t1")).state == "done"
    # The same on_transition the scheduler uses, so the outward events are
    # identical to any other close.
    assert seen and seen[0][1:3] == ("running", "done")
    # The progress run is closed too, and the final position is on it.
    assert await pp.fetch_active("t1") is None
    cur = await db.execute("SELECT status, waypoint_index FROM patrol_progress"
                           " WHERE task_id='t1'")
    assert await cur.fetchone() == ("completed", 63)


@pytest.mark.asyncio
async def test_failed_fails_the_task_but_aborted_leaves_it_running(db):
    """The whole point of 11 S3.5B's v2.0 split. `failed` is this navigation
    dying; `aborted` is preemption or e-stop, which P3 has already handled.

    MUTATION: map 'aborted' as well -> the second half goes red, and in the
    field an e-stopped patrol becomes permanently failed instead of
    resumable.
    """
    dao, pp = await _running_patrol(db, task_id="t1")
    out = await apply_path_progress(
        _frame(state="aborted"), conn=db, dao=dao, progress_dao=pp,
        tracker=ProgressTracker(), now_mono_ms=1000,
        now_wall_ms=1_785_000_000_000, on_transition=_noop_transition,
        boot_id="b1")
    assert out["closed"] is False
    assert (await dao.fetch_by_id("t1")).state == "running"
    assert await pp.fetch_active("t1") is not None

    out = await apply_path_progress(
        _frame(state="failed", fail_reason="corridor_blocked"), conn=db,
        dao=dao, progress_dao=pp, tracker=ProgressTracker(),
        now_mono_ms=2000, now_wall_ms=1_785_000_001_000,
        on_transition=_noop_transition, boot_id="b1")
    assert out["closed"] is True
    assert (await dao.fetch_by_id("t1")).state == "failed"


@pytest.mark.asyncio
async def test_a_malformed_frame_is_counted_not_raised(db):
    """The p3 loop that handles this also runs task scheduling and the geo
    single writer; one bad frame must not stop either."""
    dao, pp = await _running_patrol(db)
    out = await apply_path_progress(
        {"state": "running"}, conn=db, dao=dao, progress_dao=pp,
        tracker=ProgressTracker(), now_mono_ms=1000,
        now_wall_ms=1_785_000_000_000, on_transition=_noop_transition)
    assert (out["accepted"], out["reason"]) == (False, "malformed")


# --- the wiring itself ----------------------------------------------------

@pytest.mark.asyncio
async def test_the_live_wiring_subscribes_and_the_frame_reaches_task_db(
        tmp_path, monkeypatch):
    """*** The one test that can tell "wired" from "not wired".

    Every test above builds apply_path_progress's arguments itself and would
    pass with the subscriber deleted from main_wiring. This one runs the real
    _amain (the p3 loop) against a fake Zenoh session, pushes a frame through
    the subscriber it actually declared, and reads task.db afterwards.

    MUTATION: remove the STATE_PATH_PROGRESS_TOPIC line from the _subs list
    and this goes red on the missing key -- the only place in the suite that
    would notice.
    """
    from xbrain.p3_task.runtime import main_wiring as mw

    subs = {}

    class _Pub:
        def __init__(self, key):
            self.key = key

        def put(self, _payload):
            return None

    class _Session:
        def declare_subscriber(self, key, cb):
            subs[key] = cb
            return object()

        def declare_publisher(self, key):
            return _Pub(key)

        def declare_queryable(self, key, cb):
            subs[key] = cb
            return object()

        def put(self, _key, _payload):
            return None

    class _Sample:
        def __init__(self, body):
            self.payload = json.dumps(body).encode("utf-8")

    class _Planes:
        def __enter__(self):
            return _Session()

        def __exit__(self, *_a):
            return False

    monkeypatch.setattr(mw, "open_planes", lambda _p: _Planes(),
                        raising=False)
    # open_planes is imported inside run_wiring, so patch the source module.
    import xbrain.common.runtime.session_ctx as sctx
    monkeypatch.setattr(sctx, "open_planes", lambda _p: _Planes())

    db_path = str(tmp_path / "task.db")
    stop = {"stop": False}
    task = asyncio.create_task(mw._amain(
        stop, 99.0, db_path,
        geo_db_path=str(tmp_path / "geo.db"),
        fence_db_path=str(tmp_path / "fence.db")))
    try:
        for _ in range(200):                       # let the wiring come up
            if mw.STATE_PATH_PROGRESS_TOPIC in subs:
                break
            await asyncio.sleep(0.01)
        assert mw.STATE_PATH_PROGRESS_TOPIC in subs, (
            "p3 did not subscribe %s; the P1-12 breakpoint has no reader"
            % mw.STATE_PATH_PROGRESS_TOPIC)

        # Seed a running patrol with an active progress row through the same
        # connection the wiring opened. Opening a second handle to the same
        # file would be a different transaction and the wiring would not see
        # it, so go through the wiring's own db by doing it before the frame.
        async with aiosqlite.connect(db_path) as seed:
            await _running_patrol(seed)

        # Enveloped exactly the way p1 publishes (11 S3.0).
        subs[mw.STATE_PATH_PROGRESS_TOPIC](
            _Sample({"v": 1, "src": "p1_motion",
                     "data": _frame(waypoint_index=21)}))
        for _ in range(300):
            async with aiosqlite.connect(db_path) as chk:
                cur = await chk.execute(
                    "SELECT waypoint_index FROM patrol_progress"
                    " WHERE task_id='t1' AND status='active'")
                row = await cur.fetchone()
            if row and row[0] == 21:
                break
            await asyncio.sleep(0.02)
        assert row == (21,), (
            "the frame did not reach patrol_progress; got %r" % (row,))
    finally:
        stop["stop"] = True
        await asyncio.wait_for(task, timeout=10.0)
