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

A frame for which there is no active patrol_progress row is NORMAL, not an
error: the row is opened by the dispatch path (EX-2) for route-bearing tasks
only, and P1 keeps publishing for a few frames after a task ends. It is
counted and logged, never raised -- but it is also never treated as a
successful write, which is why update_progress returns a row count.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from xbrain.p3_task.persistence.schema_task import iso_from_wall_ms
from xbrain.p3_task.schedule.driver import apply_path_progress_terminal
from xbrain.p3_task.state.path_progress import (
    PathProgress,
    PathProgressError,
    parse_path_progress,
)

_logger = logging.getLogger("xbrain.p3.progress")


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
                              on_transition, boot_id: str = "") -> Dict[str, Any]:
    """Handle one state/motion/path_progress frame end to end.

    Returns a small dict for the caller's counters and for tests:
      {'accepted': bool, 'flushed': bool, 'reason': str, 'closed': bool}
    `reason` names why nothing happened when accepted is False, so a p3 log
    line can say WHICH guard dropped a frame rather than that one was dropped.
    """
    out: Dict[str, Any] = {"accepted": False, "flushed": False,
                           "reason": "", "closed": False}
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
            dist_done_m=progress.dist_done_m,
            # odom_dist_m is wear/energy statistics only and never feeds
            # progress (15 S9.5). PathProgress does not carry it, so it stays
            # at whatever the row holds -- NOT set to dist_done_m, which would
            # make a derived-looking column quietly wrong.
            odom_dist_m=active[8],
            loop_index=progress.loop_index,
            loop_total=progress.loop_total,
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
    if progress.is_terminal:
        closed = await apply_path_progress_terminal(
            conn, dao, task_id, progress.state,
            now_mono_ms=now_mono_ms,
            on_transition=on_transition,
            finished_at=iso_from_wall_ms(now_wall_ms),
            boot_id=boot_id)
        out["closed"] = closed
        if closed:
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
