"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: projection.py
Brief: CHK-1-43 outbound projection (event/** + state/** -> task/ack + task/progress + task/result)

Description:
The cloud contract expects THREE key families for task lifecycle:
  cmd/task/ack       -- admission verdict
  cmd/task/progress  -- ongoing execution updates
  cmd/task/result    -- final verdict

The gateway MUST project internal event/** and state/task into
those three; the projection has FIVE named rules:

  * per-task fan-out: two concurrent task_id's produce two
    disjoint progress streams (never a whole-machine snapshot
    that a Qt subscriber has to filter)
  * unknown route_total_m -> progress_percent MUST be None
    (never 0 -- 0 is indistinguishable from 'stuck'; classic
    fail-silent)
  * terminal-state ownership: p3_task emits the terminal ROW on
    state transition; the gateway PROJECTS that row; a gateway
    that synthesises the terminal from a snapshot delta loses
    the terminal on gateway restart
  * every ack/progress/result carries BOTH timestamp (wall
    clock for human display) AND mono (monotonic ms for latency
    stats)
  * bidirectional diff against 11 §2.2's outbound row for each
    key family
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


class ProjectionShapeError(Exception):
    pass


OUTBOUND_KEYS = ("cmd/task/ack", "cmd/task/progress", "cmd/task/result")


@dataclass(frozen=True)
class ProgressFrame:
    """One projected progress entry. progress_percent is Optional;
    None when route_total_m is unknown."""
    task_id: str
    step: str
    progress_percent: Optional[float]
    timestamp: float          # wall clock UTC seconds (Qt-facing)
    mono_ms: int              # monotonic (latency stat)


def project_progress(task_id: str,
                       route_total_m: Optional[float],
                       distance_travelled_m: float,
                       step: str,
                       timestamp: float,
                       mono_ms: int) -> ProgressFrame:
    """Compose one progress frame. Unknown route_total_m -> None."""
    if not task_id:
        raise ProjectionShapeError("task_id required")
    if route_total_m is None or route_total_m <= 0:
        pct = None
    else:
        pct = min(100.0, max(0.0, 100.0 * distance_travelled_m / route_total_m))
    return ProgressFrame(
        task_id=task_id, step=step, progress_percent=pct,
        timestamp=timestamp, mono_ms=mono_ms)


def split_progress_by_task(frames: List[ProgressFrame]) -> dict:
    """Two concurrent task_ids produce two separate streams. The
    key is the task_id; the value is a list of that task's
    ProgressFrames in insertion order."""
    out: dict = {}
    for f in frames:
        out.setdefault(f.task_id, []).append(f)
    return out


@dataclass(frozen=True)
class TerminalFrame:
    """cmd/task/result terminal frame. Comes from p3_task's state
    transition; the gateway MUST NOT synthesise this from a
    snapshot delta (that would lose the terminal on gateway
    restart)."""
    task_id: str
    verdict: str          # 'success' / 'failed' / 'aborted'
    reason: str
    timestamp: float
    mono_ms: int
    v6_code: str = ""     # optional internal code echo (not shown to Qt)


class TerminalSourceViolation(Exception):
    """The terminal frame was synthesised locally rather than
    delivered from p3_task."""


def check_terminal_source(source_process: str) -> None:
    """CHK-1-43 (iv): terminal must be from p3_task."""
    if source_process != "p3_task":
        raise TerminalSourceViolation(
            f"cmd/task/result must originate at p3_task; got "
            f"source_process={source_process!r} (a gateway-side "
            f"synthesis would lose the terminal on restart)")


def has_both_time_fields(frame_dict: dict) -> bool:
    """CHK-1-43 (v): both timestamp AND mono must be present."""
    return "timestamp" in frame_dict and "mono" in frame_dict


def outbound_keys_bidirectional_diff(expected: tuple,
                                       actual: tuple) -> tuple:
    """CHK-1-43 (i) helper: return (expected_only, actual_only)."""
    e, a = set(expected), set(actual)
    return tuple(sorted(e - a)), tuple(sorted(a - e))
