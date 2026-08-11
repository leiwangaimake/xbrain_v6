"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: mission_json.py
Brief: BIZ-P3-28 mission_json / total_steps / current_step / step_status_json (ST-1..3 + SS-1..3)

Description:
15 S5.10 defines the semantics of the mission_json / step columns:

  ST-1  total_steps is authoritative -- current_step MUST be
        in [0, total_steps]. Progress > total_steps is a bug.
  ST-2  current_step is monotone non-decreasing WHILE running
        (no rewind); 'resume' with resume_policy='restart' resets
        the value to 0 by explicit write while NOT running (the task
        is suspended/ready at that point), not by a running rewind.
  ST-3  mission_json is IMMUTABLE after admission. Editing it
        while the task is running is a spec violation; only new
        tasks may replace it.

  SS-1  step_status_json is a JSON list with total_steps entries,
        each being one of 'pending' / 'ok' / 'skipped' / 'failed'.
  SS-2  Length invariant: len(step_status_json) == total_steps.
  SS-3  Terminal-state consistency: if state=='done' (11 S4.4 normal
        completion), all entries must be 'ok' or 'skipped'; if
        'failed', at least one must be 'failed'.
"""

from __future__ import annotations

import json


VALID_STEP_STATUS = frozenset({"pending", "ok", "skipped", "failed"})


class MissionJsonInvariantViolation(Exception):
    pass


def assert_current_in_range(current: int, total: int) -> None:
    """ST-1: current in [0, total]."""
    if not (0 <= current <= total):
        raise MissionJsonInvariantViolation(
            f"current_step={current} not in [0, {total}]")


def assert_monotone(prev: int, new: int, state: str) -> None:
    """ST-2: current_step must not rewind WHILE running. A resume with
    resume_policy='restart' resets it to 0, but that write happens while the
    task is not running (it is suspended/ready during resume prep, 11 S4.4),
    so keying the guard on state=='running' is both correct and independent of
    the transient state names (the old 'resuming' state no longer exists)."""
    if new < prev and state == "running":
        raise MissionJsonInvariantViolation(
            f"current_step rewind {prev}->{new} while running")


def parse_step_status(step_status_json: str, total: int):
    """SS-1 SS-2: parse and validate the list."""
    try:
        entries = json.loads(step_status_json)
    except json.JSONDecodeError as e:
        raise MissionJsonInvariantViolation(f"step_status_json: {e}")
    if not isinstance(entries, list):
        raise MissionJsonInvariantViolation(
            "step_status_json must be a list")
    if len(entries) != total:
        raise MissionJsonInvariantViolation(
            f"len={len(entries)}, total_steps={total}")
    for i, v in enumerate(entries):
        if v not in VALID_STEP_STATUS:
            raise MissionJsonInvariantViolation(
                f"step[{i}] = {v!r}")
    return entries


def assert_ss3_terminal(entries, state: str) -> None:
    """SS-3: terminal-state consistency. 'done' is 11 S4.4 normal completion
    (the old vocabulary spelled it 'completed')."""
    if state == "done":
        for i, v in enumerate(entries):
            if v not in ("ok", "skipped"):
                raise MissionJsonInvariantViolation(
                    f"done task step[{i}]={v!r}")
    elif state == "failed":
        if not any(v == "failed" for v in entries):
            raise MissionJsonInvariantViolation(
                "failed task has no failed step")
