"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: executor.py
Brief: BIZ-P3-17 charge task execution t0..t5 (charge_stage 4-value + CD-1..6)

Description:
15 §8.5 charge execution timeline:

  t0  approach          drive to handover pose  charge_stage='approach'
  t1  arrived_handover  arbiter grants dock     charge_stage='arrived'
  t2  contact           make electrical contact charge_stage='charging'
  t3  charging          normal charge loop      charge_stage='charging'
  t4  detach            leave contact           charge_stage='detach'
  t5  clear             leave handover cone     charge_stage='detach'

charge_stage is a 4-value closed set: approach / arrived /
charging / detach. §8.6/§8.7 add six dedup rules CD-1..6 so a
retry after network loss does not duplicate a completed stage.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ChargeStage(str, Enum):
    APPROACH = "approach"
    ARRIVED  = "arrived"
    CHARGING = "charging"
    DETACH   = "detach"


# Stage progression (single direction, no jumps allowed).
STAGE_ORDER = (
    ChargeStage.APPROACH,
    ChargeStage.ARRIVED,
    ChargeStage.CHARGING,
    ChargeStage.DETACH,
)


class InvalidChargeStageTransition(Exception):
    pass


@dataclass(frozen=True)
class StageEvent:
    """One recorded stage-entry event; dedup key is
    (task_id, stage) so a retry with the same pair is idempotent."""
    task_id: str
    stage: str
    entered_ms: int


def next_stage(current: str) -> str:
    """Return the next stage after `current`. Terminal stage
    (DETACH) has no next: caller ends the task."""
    try:
        ix = STAGE_ORDER.index(ChargeStage(current))
    except ValueError:
        raise InvalidChargeStageTransition(f"unknown stage {current!r}")
    if ix + 1 >= len(STAGE_ORDER):
        raise InvalidChargeStageTransition(
            f"no next stage after {current!r}")
    return STAGE_ORDER[ix + 1].value


def is_forward_transition(from_stage: str, to_stage: str) -> bool:
    try:
        f = STAGE_ORDER.index(ChargeStage(from_stage))
        t = STAGE_ORDER.index(ChargeStage(to_stage))
    except ValueError:
        return False
    return t == f + 1


def dedup_key(task_id: str, stage: str) -> tuple:
    """CD-1..6: idempotency key for stage entries. Repeated same
    (task_id, stage) event = no-op."""
    return (task_id, stage)
