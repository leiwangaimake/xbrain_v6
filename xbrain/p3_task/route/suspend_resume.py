"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: suspend_resume.py
Brief: BIZ-P3-12 suspend/resume §7.2/7.3 + §4.1A mode linkage + §7.5 resume_policy

Description:
15 §7.2 suspend: on suspend, capture the current progress atomically
into patrol_progress (waypoint_ix, progress within segment,
direction). The direction field records whether the task was
traversing waypoints forward or in reverse -- resume must resume in
the SAME direction unless resume_policy overrides it.

15 §7.3 resume: read patrol_progress, decide the resume mode (see
resume_policy below), rebuild the route from the persisted position,
and re-push it (RP-2 trigger).

15 §7.5 resume_policy: one of
  * 'exact'        resume from the exact segment (default)
  * 'nearest_wp'   round to the nearest waypoint
  * 'restart'      restart the task from waypoint 0
resume_policy is LOCKED IN THE TASK ROW AT ADMISSION -- 15 §7.5 was
explicit that letting the user change it mid-flight leads to
"different tasks each time you look".

§4.1A mode linkage: suspend/resume in E-mode (emergency) is
different from B-mode -- E-mode never auto-resumes, only user
resumes.
"""

from __future__ import annotations

from dataclasses import dataclass


RESUME_POLICIES = frozenset({"exact", "nearest_wp", "restart"})


class InvalidResumePolicy(Exception):
    pass


class DirectionMismatch(Exception):
    """Resume attempted with a direction inconsistent with the
    persisted suspend snapshot."""


@dataclass(frozen=True)
class ResumeSnapshot:
    task_id: str
    waypoint_ix: int
    within_segment_progress: float   # 0.0..1.0
    direction: str                    # 'forward' or 'reverse'
    resume_policy: str


def validate_policy(policy: str) -> None:
    if policy not in RESUME_POLICIES:
        raise InvalidResumePolicy(
            f"resume_policy={policy!r} not in {sorted(RESUME_POLICIES)}")


def build_resume_snapshot(task_id: str,
                            waypoint_ix: int,
                            within_segment_progress: float,
                            direction: str,
                            resume_policy: str) -> ResumeSnapshot:
    validate_policy(resume_policy)
    if direction not in ("forward", "reverse"):
        raise DirectionMismatch(f"direction={direction!r}")
    if not (0.0 <= within_segment_progress <= 1.0):
        raise ValueError(
            f"within_segment_progress={within_segment_progress}")
    return ResumeSnapshot(
        task_id=task_id, waypoint_ix=waypoint_ix,
        within_segment_progress=within_segment_progress,
        direction=direction, resume_policy=resume_policy)


def resume_start_index(snap: ResumeSnapshot) -> int:
    """Where to start pushing from on resume.
    * exact       -> snap.waypoint_ix (rebuild segment)
    * nearest_wp  -> snap.waypoint_ix or +1 depending on within_segment
    * restart     -> 0"""
    if snap.resume_policy == "restart":
        return 0
    if snap.resume_policy == "nearest_wp":
        return (snap.waypoint_ix + 1
                 if snap.within_segment_progress >= 0.5
                 else snap.waypoint_ix)
    return snap.waypoint_ix


def check_direction_consistency(snap_dir: str,
                                  new_dir: str) -> None:
    """§7.2 direction preservation: resume must not silently flip
    the traversal direction."""
    if snap_dir != new_dir:
        raise DirectionMismatch(
            f"snapshot={snap_dir!r}, resume attempt={new_dir!r}")
