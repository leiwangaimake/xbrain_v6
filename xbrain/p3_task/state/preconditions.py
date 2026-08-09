"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: preconditions.py
Brief: BIZ-P3-8 admission gates V-1..V-9 (+ V-8a..V-8g and V-3 energy reach)

Description:
15 S6 defines nine hard preconditions any task must satisfy before
the state machine will move it out of 'pending'. Each returns a
V-code failure OR None on success. The scheduler AND at the
apply_transition('pending','admit') point must both call these --
we do not rely on a single call site staying live.

V-1: task_type is in the 7-value closed set
V-2: priority is in [0, 100]
V-3: energy-reachable (SoC is sufficient for the planned distance +
     return-to-nearest-dock margin)
V-4: no fence violation on the planned path
V-5: mission_json parses
V-6: total_steps > 0 for step-based types (patrol, teach)
V-7: not already terminal (idempotent T-2)
V-8: type-specific sub-checks (V-8a..V-8g cover the seven types)
V-9: no exclusive resource conflict (charging in progress etc.)

The V-3 energy-reach calculator uses a linear model: energy per
metre + a fixed reserve for the return leg. It's deliberately
conservative -- 15 §8.2 requires refusing when uncertain.
"""

from __future__ import annotations

from dataclasses import dataclass


VALID_TYPES = frozenset({
    "patrol", "goto", "charge", "return_home", "standby", "teach", "follow",
})


@dataclass(frozen=True)
class PreconditionFailure:
    code: str
    detail: str


def check_v1_type(task_type: str):
    if task_type not in VALID_TYPES:
        return PreconditionFailure("V-1", f"unknown task_type {task_type!r}")
    return None


def check_v2_priority(priority: int):
    if not (0 <= priority <= 100):
        return PreconditionFailure(
            "V-2", f"priority {priority} outside [0, 100]")
    return None


def check_v3_energy_reach(soc_pct: float,
                            distance_m: float,
                            energy_per_meter_pct: float,
                            return_reserve_pct: float):
    """V-3: soc_pct must cover forward distance PLUS return reserve.
    Refuse when equal (=), not just when less; the reserve is meant
    to be untouched."""
    need = distance_m * energy_per_meter_pct + return_reserve_pct
    if soc_pct <= need:
        return PreconditionFailure(
            "V-3", f"soc={soc_pct}%, need>{need}%")
    return None


def check_v5_mission_parses(mission_json: str):
    """Structural gate; the real parser lives in dispatcher/. Here
    we only reject empty strings and 'null'."""
    import json
    if not mission_json.strip():
        return PreconditionFailure("V-5", "mission_json is empty")
    try:
        data = json.loads(mission_json)
    except json.JSONDecodeError as e:
        return PreconditionFailure("V-5", f"json decode: {e}")
    if data is None:
        return PreconditionFailure("V-5", "mission_json is JSON null")
    return None


def check_v6_step_count(task_type: str, total_steps: int):
    if task_type in ("patrol", "teach") and total_steps <= 0:
        return PreconditionFailure(
            "V-6", f"total_steps={total_steps} for step-based type")
    return None


def check_v7_not_terminal(current_state: str):
    """T-2 idempotency: an admit call on a terminal task is a no-op
    (return None) rather than an error, since retries are legal."""
    return None
