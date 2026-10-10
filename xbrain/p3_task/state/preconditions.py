"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: preconditions.py
Brief: BIZ-P3-8 admission gates, P3-local PC-n codes mapped to 15 S5.2 V-n

Description:
Admission checks a task must pass before the state machine moves it out of
'pending'. Each returns a PC-code failure OR None on success. The scheduler
AND the apply_transition('pending','admit') point must both call these -- we
do not rely on a single call site staying live.

*** RENUMBERED 2026-10-10 (NEXT EX-1 gap 4): these codes used to be V-1..V-9
and they COLLIDED with 15 S5.2's V-1..V-9 while meaning different things.
Nine codes, and only two lined up:

  code    this module checks              15 S5.2's V-n of the same number
  ------  -----------------------------   --------------------------------
  PC-1    task_type in the 7-value set    route/waypoint/dock exists, not
                                          tombstoned
  PC-2    priority in [0, 100]            target point inside the hard fence
  PC-3    energy-reachable                energy-reachable          <- same
  PC-4    -- does not exist --             body capability supports the cmd
  PC-5    mission_json parses              health allows motion
                                           (allow_motion)
  PC-6    total_steps > 0                  fix quality (fix_type)
  PC-7    not already terminal (a no-op,   clock synced (ts_sync)
          see its docstring)
  PC-8    route exists in the manifest     parameters legal (V-8a..V-8g)
  PC-9    -- does not exist --             refuse new tasks per SOURCE while
                                           the link is down

  Only PC-3 / V-3 are the same check, and PC-8 is 15's V-1 under the wrong
  number. 15's V-2 (hard-fence containment) is NOT implemented here -- there
  is no PC-4.

  *** THIS TABLE WAS WRONG FOR ONE COMMIT (corrected 2026-10-10, same day).
  Its first version listed PC-4 as "no fence violation on the path" and so
  claimed 15's V-2 was covered. There is no check_pc4 in this module; the
  row was written from the renumbering notes rather than from the file.
  A mapping table that overstates coverage is worse than none -- it is the
  thing a reader consults INSTEAD of reading the code.
  tests/p3_task/test_precondition_codes.py now holds the table to the set of
  functions that actually exist.

*** WHAT 15 S5.2 ACTUALLY GETS FROM THIS MODULE, counted 2026-10-10:
ONE of its nine runs in production. V-1 does, via check_pc8_route_exists,
which schedule/driver.py's validate_pending() calls. V-3 is implemented
(check_pc3_energy_reach) and has NO production caller. The remaining seven
-- V-2 hard-fence containment, V-4 body capability, V-5 allow_motion,
V-6 fix_type, V-7 ts_sync, V-8 parameters legal (V-8a..V-8g, zero hits in
the tree), V-9 link-down-by-source -- have no implementation at all.

Meanwhile five checks here answer to nothing in 15 S5.2: PC-1, PC-2, PC-5,
PC-6, PC-7. Of those, PC-6 has no production caller and PC-7 is a no-op.
XBRAIN_V6_TODO.md's BIZ-P3-8 claims "前置校验 V-1~V-9"; see docs/NEXT.md.

*** WHY A NEW PREFIX RATHER THAN RENUMBERING ONTO 15's SCHEME. Seven of
15 S5.2's nine are NOT implemented anywhere. Moving
this module's checks onto those numbers would put a check named "V-5" in the
codebase while 15's V-5 remains unimplemented -- a reader grepping for V-5
would find something and conclude it was done. A separate prefix keeps
"15 S5.2 V-n" meaning exactly one thing, and makes the five missing ones
visibly missing. They are registered in docs/NEXT.md.

The precedent is 15's own handling of its CR-* collision (S8.2A's closing
note): a prefix with two meanings inside one scan surface has to be renamed,
and the one that moves is the one without prior claim to the number.

*** The node number in the old header was also wrong: it said "15 S6
defines nine hard preconditions". S6 is 调度 and contains no V-n at all; the
table is 15 S5 前置校验 (TSK-14), S5.2 校验项.

The PC-3 energy-reach calculator uses a linear model: energy per metre + a
fixed reserve for the return leg. 15 S5.3's PRIMARY model is different
(可用里程 = RemainMile x (1 - reserve_ratio)) and this is its fallback,
whose authorising condition (T-CHG-1 measured) has not been met -- that is
NEXT EX-1 gap 3, not fixed here.
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


def check_pc1_type(task_type: str):
    if task_type not in VALID_TYPES:
        return PreconditionFailure(
            "PC-1", f"unknown task_type {task_type!r}")
    return None


def check_pc2_priority(priority: int):
    if not (0 <= priority <= 100):
        return PreconditionFailure(
            "PC-2", f"priority {priority} outside [0, 100]")
    return None


def check_pc3_energy_reach(soc_pct: float,
                            distance_m: float,
                            energy_per_meter_pct: float,
                            return_reserve_pct: float):
    """PC-3: soc_pct must cover forward distance PLUS return reserve.
    Refuse when equal (=), not just when less; the reserve is meant
    to be untouched."""
    need = distance_m * energy_per_meter_pct + return_reserve_pct
    if soc_pct <= need:
        return PreconditionFailure(
            "PC-3", f"soc={soc_pct}%, need>{need}%")
    return None


def check_pc5_mission_parses(mission_json: str):
    """Structural gate; the real parser lives in dispatcher/. Here
    we only reject empty strings and 'null'."""
    import json
    if not mission_json.strip():
        return PreconditionFailure(
            "PC-5", "mission_json is empty")
    try:
        data = json.loads(mission_json)
    except json.JSONDecodeError as e:
        return PreconditionFailure(
            "PC-5", f"json decode: {e}")
    if data is None:
        return PreconditionFailure(
            "PC-5", "mission_json is JSON null")
    return None


def check_pc8_route_exists(route_geo_id, route_exists):
    """PC-8: 任务引用的路径必须存在.

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
            "PC-8", f"route {route_geo_id!r} is not in the current manifest")
    return None


def check_pc6_step_count(task_type: str, total_steps: int):
    if task_type in ("patrol", "teach") and total_steps <= 0:
        return PreconditionFailure(
            "PC-6", f"total_steps={total_steps} for step-based type")
    return None


def check_pc7_not_terminal(current_state: str):
    """T-2 idempotency: an admit on a terminal task is a no-op, not an error.

    *** THIS FUNCTION CHECKS NOTHING. It returns None for every input,
    including the terminal states its name is about. That is the T-2
    semantic -- a retried admit must not fail -- but it means the function
    is a declaration, not a gate: nothing downstream behaves differently
    because it was called.

    Noted explicitly (2026-10-10) because the shape is CLAUDE.md S3.2
    form 1: a check that any implementation passes. If T-2 is the whole
    requirement then the right form is for the CALLER to skip terminal
    rows, and this function should go; if admitting a terminal task should
    in fact be refused, then this is a missing check wearing a pass.
    Either way `current_state` is read by nothing, which is the tell.
    Registered in docs/NEXT.md.
    """
    return None
