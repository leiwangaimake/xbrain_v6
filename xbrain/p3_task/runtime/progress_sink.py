"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: progress_sink.py
Brief: state/motion/path_progress -> patrol_progress + task terminal (EX-3 / EX-4)

Description:
The database half of the path_progress intake. state/path_progress.py decides
(parse, PP-1 flush, terminal mapping) and holds no handle; this applies the
decision to task.db. Split that way because every rule worth arguing about is
in the pure half and can be tested without a database, while this file is the
one place that touches rows.

What it closes. 15 S2.1 calls state/motion/path_progress "断点的唯一来源" and
15 S9.5 makes patrol_progress its landing place; 12 S4.3.1 LP-3 says P1 only
publishes state == "arrived" and that mapping it to task `done` is P3's job.
Before 2026-09-28 p3_task did not subscribe to the key at all, so a finished
patrol stayed `running` forever and every breakpoint column stayed at its
DEFAULT. docs/NEXT.md EX-3 / EX-4.

Ordering inside one frame, and why it is this way:

  1. flush progress (if PP-1 says so)   the breakpoint must be on disk BEFORE
     the task is closed. Reversed, a power cut between the two leaves a task
     marked done whose last recorded position is minutes old -- and `done`
     means nobody will ever look at the progress row again to notice.
  2. close the task (if terminal)
  3. forget the tracker entry (only after 2 succeeds)

Boundary: it publishes nothing. state/task and the task event are emitted by
the on_transition callback the caller passes in, which is the same callback
the scheduler tick uses -- so a task closed by a progress frame produces
exactly the same outward events as one closed any other way.

Multi-pass patrols (99 U131). P1 is single-pass: every push ends in its own
`arrived`, and its frames say loop_index 0 / loop_total 1 whatever the task
asked for. So P3 owns the pass count -- the row's loop_index / loop_total are
written by dispatch (loop_total) and by advance_pass (loop_index) and are
NEVER overwritten from a frame, and dist_done_m is stored cumulative
(loop_index x route_total_m + the frame's in-pass distance, the 15 S7.3A
convention) so pct = dist_done_m / (loop_total x route_total_m) keeps rising
across passes instead of restarting at 0 on each one. An `arrived` with
passes left does not close the task: loop_index += 1, the caller's next_pass
pushes the route again (RP-5), and the task stays running. loop_total = 0 is
the endless patrol (LM-4): arrival never closes it (PP-3b).

The pass latch. Between that re-push and P1 loading it, P1 keeps sending the
OLD pass's `arrived` (2 Hz periodic, same route_id / rev -- the route is the
same route). Counting those would skip passes or end the patrol early. The
task is latched from the re-push until the first `running` frame; latched
frames are ignored outright (no flush either: their in-pass distance belongs
to the pass already counted). The latch is in memory only: a P3 restart
suspends every running task (15 S10 INIT), so it never needs to survive one.

A frame for which there is no active patrol_progress row is NORMAL, not an
error: the row is opened by the dispatch path (EX-2) for route-bearing tasks
only, and P1 keeps publishing for a few frames after a task ends. It is
counted and logged, never raised -- but it is also never treated as a
successful write, which is why update_progress returns a row count.
"""

from __future__ import annotations

import logging
from typing import Any, Awaitable, Callable, Dict, Optional, Set

from xbrain.p3_task.persistence.schema_task import iso_from_wall_ms
from xbrain.p3_task.schedule.driver import apply_path_progress_terminal
from xbrain.p3_task.state.path_progress import (
    PathProgress,
    PathProgressError,
    parse_path_progress,
)

_logger = logging.getLogger("xbrain.p3.progress")


def predates_push(body: Any, pushed_ms: int) -> bool:
    """True when this frame was built before the latest route push.

    Such a frame describes the previous run even when its route_id and
    route_rev equal the new push's (see RouteAckWindow.pushed_ms for the
    defect it closes). A frame without a usable mono is NOT judged here: the
    parser downstream rejects it with its reason.
    """
    if pushed_ms <= 0 or not isinstance(body, dict):
        return False  # no push yet / not a frame: nothing to compare
    mono = body.get("mono")  # P1's tick time, seconds (11 S3.5B)
    if isinstance(mono, bool) or not isinstance(mono, (int, float)):
        return False  # malformed: left to parse_path_progress
    return mono * 1000.0 < pushed_ms


async def find_running_task(dao) -> Optional[str]:
    """The task a progress frame belongs to.

    PathProgress carries route_id / route_rev but NO task_id (11 S3.5B), so
    the association is "the one running task" -- 15 S6.1 allows exactly one at
    a time and the scheduler enforces it. The route cross-check below is what
    keeps that from being a blind assumption.
    """
    for task_id, _prio, _seq, state in await dao.list_by_priority():
        if state == "running":
            return task_id
    return None


def route_matches(active_row, progress: PathProgress) -> bool:
    """Guard a frame against the run it claims to describe.

    fetch_active returns (route_geo_id, route_rev, ...) -- see
    PatrolProgressDAO.fetch_active for the full order. Both must agree with
    the frame.

    Why check at all when there is only one running task: P1 keeps its
    geometry resident across a P3 restart (11 S3.5A RG-3), so the first frames
    P3 sees after coming back can describe the PREVIOUS route. Writing those
    onto the new run's row would move the breakpoint to a point on a different
    path -- and U07a would then resume there, which is a robot driving
    somewhere nobody asked for. This is the same pairing RA-1 (15 S2.4.4)
    insists on for the opposite direction, and for the same reason: loading
    and index alone do not identify a route.
    """
    if active_row is None:
        return False
    route_geo_id, route_rev = active_row[0], active_row[1]
    return route_geo_id == progress.route_id and route_rev == progress.route_rev


async def apply_path_progress(body: Any, *, conn, dao, progress_dao, tracker,
                              now_mono_ms: int, now_wall_ms: int,
                              on_transition, boot_id: str = "",
                              next_pass: Optional[Callable[[str], Awaitable[Any]]],
                              pass_latch: Set[str]) -> Dict[str, Any]:
    """Handle one state/motion/path_progress frame end to end.

    next_pass(task_id) pushes the next pass of a multi-pass patrol (RP-5);
    None means this caller cannot re-push, and an arrival then closes the
    task as before. pass_latch is the caller-owned set of tasks with a
    re-push in flight (see the module docstring). Both are required: a
    caller that forgets them would silently end every patrol after pass 1.

    Returns a small dict for the caller's counters and for tests:
      {'accepted': bool, 'flushed': bool, 'reason': str, 'closed': bool,
       'next_pass': bool}
    `reason` names why nothing happened when accepted is False, so a p3 log
    line can say WHICH guard dropped a frame rather than that one was dropped.
    """
    out: Dict[str, Any] = {"accepted": False, "flushed": False,
                           "reason": "", "closed": False, "next_pass": False}
    try:
        progress = parse_path_progress(body)
    except PathProgressError as exc:
        # One malformed frame must not take down the loop that also runs task
        # scheduling. Logged at warning with the reason: a dropped progress
        # frame is invisible otherwise, and the symptom (progress frozen) looks
        # exactly like a stopped robot.
        _logger.warning("p3 path_progress rejected: %s", exc)
        out["reason"] = "malformed"
        return out
    task_id = await find_running_task(dao)
    if task_id is None:
        out["reason"] = "no_running_task"
        return out
    active = await progress_dao.fetch_active(task_id)
    if not route_matches(active, progress):
        out["reason"] = "route_mismatch" if active is not None else "no_active_row"
        return out
    if task_id in pass_latch:
        if progress.state != "running":
            # the previous pass's arrived, re-sent while the re-push loads
            out["reason"] = "pass_pending"
            return out
        pass_latch.discard(task_id)  # the next pass is loaded and running
    out["accepted"] = True
    decision = tracker.decide(task_id, progress, now_mono_ms=now_mono_ms)
    out["reason"] = decision.reason
    if decision.flush:
        now_iso = iso_from_wall_ms(now_wall_ms)
        n = await progress_dao.update_progress(
            task_id,
            waypoint_index=progress.waypoint_index,
            waypoint_total=progress.waypoint_total,
            seg_done_m=progress.seg_done_m,
            # cumulative across passes (U131; 15 S7.3A alpha x T1 + loop_index
            # x T1): P1 restarts its distance at 0 on every pass
            dist_done_m=active[10] * active[9] + progress.dist_done_m,
            # odom_dist_m is wear/energy statistics only and never feeds
            # progress (15 S9.5). PathProgress does not carry it, so it stays
            # at whatever the row holds -- NOT set to dist_done_m, which would
            # make a derived-looking column quietly wrong.
            odom_dist_m=active[8],
            # P3's own pass count (U131), never the frame's: P1 is
            # single-pass and says 0 / 1 on every pass
            loop_index=active[10],
            loop_total=active[11],
            # skipped_m accumulates at remap time (S7.3A), not per frame.
            skipped_m=active[12],
            dir_sign=progress.dir_sign,
            now_iso=now_iso)
        if n:
            # FS-a / PWR-3: the breakpoint write is synchronous=FULL, so the
            # commit is the fsync. Committing here rather than batching is the
            # point of PP-1 -- a batched write is one that is not on disk when
            # the power goes.
            await conn.commit()
            out["flushed"] = True
        else:
            # The row vanished between fetch_active and the UPDATE. Nothing to
            # do, but it must not read as a successful flush.
            _logger.warning("p3 path_progress: no active row for %s at flush",
                            task_id)
            out["reason"] = "flush_lost_row"
    if (progress.state == "arrived" and next_pass is not None
            and (active[11] == 0 or active[10] + 1 < active[11])):
        # U131: a pass done, passes left (or endless, LM-4 / PP-3b). Count it
        # and commit BEFORE pushing: a crash between the two then resumes
        # with the pass already counted, never with it counted twice.
        await progress_dao.advance_pass(task_id, iso_from_wall_ms(now_wall_ms))
        await conn.commit()
        pass_latch.add(task_id)
        _logger.info("p3 patrol %s pass %d/%s done, pushing the next",
                     task_id, active[10] + 1,
                     active[11] if active[11] else "endless")
        await next_pass(task_id)
        out["next_pass"] = True
        return out
    if progress.is_terminal:
        closed = await apply_path_progress_terminal(
            conn, dao, task_id, progress.state,
            now_mono_ms=now_mono_ms,
            on_transition=on_transition,
            fail_reason=progress.fail_reason,  # 99 U124: keep the RNS attribution
            finished_at=iso_from_wall_ms(now_wall_ms),
            boot_id=boot_id)
        out["closed"] = closed
        # 99 U175: a stall SUSPENDED the task (closed == "suspended"); its
        # progress run stays active so "继续" resumes from it (RP-2).
        if closed is True:
            # Close the progress run to match the task. 'arrived' completed
            # the run; 'failed' did not. path_progress 'aborted' never reaches
            # here (it maps to no task transition, see
            # MOTION_RESULT_FOR_PATH_STATE) so the run stays active and
            # resumable, which is what U07 needs.
            await progress_dao.close_run(
                task_id,
                "completed" if progress.state == "arrived" else "aborted",
                iso_from_wall_ms(now_wall_ms))
            await conn.commit()
            tracker.forget(task_id)
    return out
