"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: retention.py
Brief: BIZ-P3-22 retention S9.11 + WAL checkpoint + tombstone cleanup

Description:
WAL checkpoint: called periodically (e.g. every N minutes) to fold
WAL into the main file. Truncate mode reclaims space; the caller
picks the mode. Refusing to checkpoint when in DegradedWriteMode
prevents amplifying I/O errors.

Tombstone cleanup: soft-deleted geo_object rows (tombstone=1) are
kept as long as pending_push may reference them. Once every push
targeting a tombstoned row has been ack'd, the row is safe to
hard-delete. 15 S9.11 verbatim for the geo tombstone row: "只清理已
synced_rev == rev 的墓碑 (11 S7.11.4)".

*** THREE THINGS WERE REMOVED FROM THIS FILE ON 2026-10-09 (RT-2), and the
reason is worth more than the code was.

Removed: RetentionWindows, RetentionOrderViolation, validate_windows -- a
three-window model (keep_task_days / keep_progress_days /
keep_snapshot_days) with an ascending requirement between them.

(1) THE THREE KEY NAMES EXIST NOWHERE IN THE FOURTEEN VOLUMES. grep across
    docs/ returns zero hits for all three. They were invented here.

(2) THEY WERE A MISREADING OF common.retention. That key has three day
    counts and they are tasks / events / commands (15 S9.11), not task /
    progress / snapshot. The test that covered this called
    RetentionWindows(30, 90, 180) -- which IS 30/90/180, the
    common.retention triple -- so the misreading is visible in the
    criterion itself. It was green, and it was green about nothing.

(3) THE ASCENDING CHECK WAS A SECOND IMPLEMENTATION OF A FREEZE-LINE
    ASSERTION, and 17 S10 v0.7.3 forbids exactly that in as many words:
    the ordering "由 10 S5.4.4 断言 C (执行者 xbrain-config-freeze.service,
    Stage 0c) 在启动期保证, 不由任何一册的运行期代码保证". That assertion
    is implemented in xbrain/boot/freeze/assertions/c_cross_file.py
    (_check_retention_monotone).

    Worse: with the REAL per-table values the invented ascending rule is
    unsatisfiable. 15 S9.11 gives patrol_progress 90 days and
    task_route_snapshot snapshot_keep_days 30 -- so progress <= snapshot
    reads 90 <= 30, false. Landing the contract's own numbers would have
    made this check refuse.

WHAT IS NOT HERE AND IS A REAL GAP: nothing in this repo deletes a row
because it aged out. 15 S9.11 specifies retention for tasks (30) /
events (90) / commands (180) / patrol_progress (90) /
task_route_snapshot (snapshot_keep_days 30), and a grep for "DELETE FROM"
across xbrain/p3_task/ finds three statements, all of which delete by id
(an explicit discard, a push that succeeded, a recording session that
ended) -- none by age. p5_gateway has none at all. Registered in
docs/NEXT.md; the cleanup TIMING is not specified by any volume, so it
needs a ruling before implementation.
"""

from __future__ import annotations


def should_checkpoint(now_ms: int, last_ms: int, interval_ms: int,
                       degraded: bool) -> bool:
    """Skip when in degraded mode; otherwise, checkpoint when we've
    crossed the interval since the last one."""
    if degraded:
        return False
    return (now_ms - last_ms) >= interval_ms


def tombstone_safe_to_hard_delete(pending_count: int) -> bool:
    """Only when zero pending pushes reference the tombstoned row."""
    return pending_count == 0
