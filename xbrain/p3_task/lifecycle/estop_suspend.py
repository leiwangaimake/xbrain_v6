"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: estop_suspend.py
Brief: ES-2 -- suspend the running task on a soft-estop (15 S11.1, CLD-1c)

Description:
ES-1(freeze, 批64)冻结了[新任务的派发]; ES-2 是另一件事: 把[当前正在跑]的
那条任务主动挂起. 两者互补 -- ES-1 挡住后面的, ES-2 处理眼前的.

*** 挂起用 kind=passive, reason=estop_soft, 不是 pause 那套.
task_apply 的 pause 用 reason=operator_pause(人主动暂停); estop 是
estop_soft. 两者都 passive(不自动恢复, 与 ES-3 "等人" 一致), 但 reason 必须
分开 -- 审计要能区分"人按了暂停"和"急停停的". CR-8 配对: estop_soft 不在
{preempted, mode_takeover}, 所以配 passive(validate_suspend_fields 会核).

*** 只挂[当前 running]那一条, 不碰 ready/queued.
ready/queued 的任务由 ES-1 freeze 拦着不派发, 它们停在原地即可; 主动把它们
也转 suspended 是多余的, 而且会在 resume 时造出一批要逐条恢复的任务.
scheduler 不变量: 同一时刻至多一条 running(15 S6.1), 所以最多挂一条.

*** 幂等: 挂起后没有 running 任务了, 再调是空操作.
freeze 期间调度被 ES-1 冻住, 不会再派新任务成 running, 所以 ES-2 挂掉那条
之后, 后续每拍 list 里都没有 running, 直接返回 None. 不需要额外的"已处理"
标志.

*** 事务由本函数显式管, NO 不能只 execute 完就返回.
TasksDAO 的每个方法都只 execute 不 commit -- 事务边界归调用方(task_apply
的 submit 就是 BEGIN IMMEDIATE ... commit/rollback). 本函数原先调完
dao.suspend_task 直接 return, 于是 aiosqlite 那条隐式事务一直开着, 之后任何
BEGIN IMMEDIATE 都报 "cannot start a transaction within a transaction".
后果不是这次挂起没落盘那么简单 -- 是[急停之后每一条 cmd/task submit 都失败],
一直到 p3 重启. 2026-09-01 联调预演实测: 11:50 GOTO 成功, 11:50 急停, 11:51
下一条 GOTO 就回 E_INTERNAL.
*** 原有 4 条 ES-2 判据都没抓到, 因为它们只在同一连接上读回那一行 --
而未提交的写在同一连接上本来就读得到. 判据验的是"写可见", 不是"已提交",
更没有验"连接之后还能用".

Boundaries: 只做[running -> suspended]这一步的 DB 写 + 归因 + 提交. 不发布
(调用方用返回值发 state/task), 不解冻(ES-3 的事), 不判断该不该急停(急停无条件).
"""

from __future__ import annotations

from typing import Optional, Tuple

from xbrain.p3_task.state.machine import (apply_transition,
                                          validate_suspend_fields)

#: ES-2 的挂起归因(15 S11.1 / common.enums). passive 因为 estop 不自动恢复;
#: estop_soft 因为软急停(硬急停 estop_hes 走别的路径, 不是本 cmd/estop).
ESTOP_SUSPEND_KIND = "passive"
ESTOP_SUSPEND_REASON = "estop_soft"


async def suspend_running_for_estop(dao, conn, now_mono_ms: int
                                    ) -> Optional[Tuple[str, str, str]]:
    """ES-2: 把当前 running 任务挂起. 返回 (task_id, "suspended", reason)
    供调用方发 state/task; 没有 running 任务时返回 None.

    * 走 apply_transition 而不是直接写 "suspended": 让状态机确认 running ->
    suspend 这条边合法(它就该合法, 但绕过状态机就等于多了一处它管不到的
    状态写). validate_suspend_fields 再核一遍 kind/reason 配对, 把 CR-8
    违规挡在 DB CHECK 之前(报错信息更能指出是配对错, 而不是一句 sqlite
    CHECK failed).
    """
    rows = await dao.list_by_priority()          # (task_id, prio, seq, state)
    running = [tid for tid, _p, _s, state in rows if state == "running"]
    if not running:
        return None
    task_id = running[0]                          # 至多一条(scheduler 不变量)
    # 状态机确认这条边合法(running -> suspend -> suspended).
    result = apply_transition("running", "suspend")
    # kind/reason 配对再核一遍(CR-8), 越界在这里抛而不是到 sqlite.
    validate_suspend_fields(result.to_state, ESTOP_SUSPEND_KIND,
                            ESTOP_SUSPEND_REASON)
    # BEGIN IMMEDIATE ... commit/rollback: 与 task_apply 的 submit 同一范式.
    # IMMEDIATE 而不是裸 BEGIN -- 立刻取写锁, 与既有写路径一致, 不给两条写
    # 路径留下不同的加锁时机.
    await conn.execute("BEGIN IMMEDIATE")
    try:
        await dao.suspend_task(task_id, ESTOP_SUSPEND_KIND,
                               ESTOP_SUSPEND_REASON, now_mono_ms)
        await conn.commit()
    except Exception:
        await conn.rollback()
        raise
    return task_id, result.to_state, ESTOP_SUSPEND_REASON


__all__ = ["suspend_running_for_estop", "ESTOP_SUSPEND_KIND",
           "ESTOP_SUSPEND_REASON"]
