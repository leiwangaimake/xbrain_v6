"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: progress.py
Brief: BIZ-P3-19 progress publish (PP-1..PP-3b + state/task 1 Hz)

Description:
15 §5 progress reporting has two orthogonal cadences:

  event-driven (PP-1..PP-3):
    PP-1  step boundary crossed (increment current_step)
    PP-2  suspend / resume / abort
    PP-3  route push accepted / rejected
    PP-3b geographic waypoint reached (patrol only)

  heartbeat (1 Hz):
    state/task emits a snapshot of every non-terminal task, whether
    or not something changed. Downstream (P5 HMI) uses this for
    presence detection -- silence -> assume p3 crash.

The PP-* events are best-effort but MUST land on disk before we
tell downstream (write-then-publish). If disk is degraded, the
event is queued in DegradedWriteMode.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass(frozen=True)
class ProgressEvent:
    """One event ready to publish."""
    task_id: str
    kind: str        # PP-1 / PP-2 / PP-3 / PP-3b
    detail: str


VALID_PROGRESS_KINDS = frozenset({"PP-1", "PP-2", "PP-3", "PP-3b"})


class UnknownProgressKind(Exception):
    pass


def build_step_event(task_id: str, current_step: int,
                       total_steps: int) -> ProgressEvent:
    return ProgressEvent(task_id=task_id, kind="PP-1",
                           detail=f"step {current_step}/{total_steps}")


def build_state_event(task_id: str, kind: str, reason: str) -> ProgressEvent:
    if kind not in ("suspend", "resume", "abort"):
        raise UnknownProgressKind(f"state kind {kind!r}")
    return ProgressEvent(task_id=task_id, kind="PP-2",
                           detail=f"{kind}:{reason}")


def build_route_event(task_id: str, accepted: bool,
                        route_seq: int) -> ProgressEvent:
    return ProgressEvent(task_id=task_id, kind="PP-3",
                           detail=f"seq={route_seq} "
                                   f"{'accepted' if accepted else 'rejected'}")


def build_waypoint_event(task_id: str, waypoint_ix: int) -> ProgressEvent:
    return ProgressEvent(task_id=task_id, kind="PP-3b",
                           detail=f"waypoint_ix={waypoint_ix}")


@dataclass
class HeartbeatState:
    """Snapshot of one non-terminal task."""
    task_id: str
    state: str
    current_step: int
    total_steps: int


def heartbeat_snapshot(active_tasks) -> List[HeartbeatState]:
    """Filter out terminal tasks from the heartbeat set."""
    from xbrain.p3_task.state.machine import TERMINAL_STATES
    return [t for t in active_tasks if t.state not in TERMINAL_STATES]
