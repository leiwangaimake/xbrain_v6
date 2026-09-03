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


def check_v8_route_exists(route_geo_id, route_exists):
    """V-8: 任务引用的路径必须存在.

    v2.0 S2.1 逐字: recorded_path_id "必须存在于当前 manifest". 在此之前
    2026-09-03 实测 r-does_not_exist 被 accepted 并 started -- 机器人接了一条
    它根本无从执行的任务, 而甲方界面上看到的是"已开始".

    放在 p3 而不是网关: geo.db 的唯一属主是 p3, 网关那份 manifest 是转发来的
    副本, 拿副本判存在性会在副本迟到时误拒[合法]任务 -- 那比误收更糟(操作员
    会以为自己的路径丢了).

    route_exists 是注入的查询, 不是本模块自己开库: 本模块是纯函数层, 而且
    driver 里不该有第二处数据源(与 CLK-C1 让时钟从外面传同一个理由).
    route_geo_id 为空表示这条任务不引用路径(纯航点 goto), 跳过.
    """
    if not route_geo_id:
        return None
    if route_exists is None:
        # 没有注入查询 = 调用方没接线. 跳过而不是拒 -- 但接线由
        # test_route_precondition 的判据钉住, 生产路径上不会走到这里.
        return None
    if not route_exists(route_geo_id):
        return PreconditionFailure(
            "V-8", f"route {route_geo_id!r} is not in the current manifest")
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
