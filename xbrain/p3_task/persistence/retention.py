"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: retention.py
Brief: BIZ-P3-22 retention S9.11 + WAL checkpoint + tombstone cleanup

Description:
15 S9.11 three retention windows (ascending):
  keep_task_days       full task rows visible for N days
  keep_progress_days   patrol_progress + step_status kept for M days
  keep_snapshot_days   task_route_snapshot kept for K days

Requirement: keep_task_days <= keep_progress_days <= keep_snapshot_days
(so we never have snapshot data older than the task row itself,
which would leave orphans).

WAL checkpoint: called periodically (e.g. every N minutes) to fold
WAL into the main file. Truncate mode reclaims space; the caller
picks the mode. Refusing to checkpoint when in DegradedWriteMode
prevents amplifying I/O errors.

Tombstone cleanup: soft-deleted geo_object rows (tombstone=1) are
kept as long as pending_push may reference them. Once every push
targeting a tombstoned row has been ack'd, the row is safe to
hard-delete.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RetentionWindows:
    keep_task_days: int
    keep_progress_days: int
    keep_snapshot_days: int


class RetentionOrderViolation(Exception):
    pass


def validate_windows(w: RetentionWindows) -> None:
    if not (w.keep_task_days <= w.keep_progress_days <= w.keep_snapshot_days):
        raise RetentionOrderViolation(
            f"retention windows must be ascending: "
            f"task={w.keep_task_days} progress={w.keep_progress_days} "
            f"snapshot={w.keep_snapshot_days}")


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
