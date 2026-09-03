"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: driver.py
Brief: BIZ-P3-42 scheduler tick -- drive the task machine (validate + dispatch)

Description:
The step that MOVES a recorded task through the machine (15 S5 / S6). Before
this, a task recorded by the ingest sat at 'pending' forever -- the state
machine, preconditions, and pick_next were pure functions with no driver.
scheduler_tick is that driver, one atomic pass over the task table:

  1. VALIDATE each 'pending' task (15 S5 TSK-14): run the preconditions that
     apply BEFORE route expansion -- V-1 type, V-2 priority, V-5 mission
     parses. A failure -> validate_fail -> 'failed' (with the code as the
     reason); all pass -> validate_ok -> 'ready'. (V-3 energy and V-6 step
     count need an expanded route/soc, which is the execution-wiring batch, so
     they are NOT run here -- running V-6 on an unexpanded patrol would fail a
     valid task for having 0 steps.)
  2. DISPATCH: if NOTHING is 'running', pick the highest-priority 'ready' task
     (priority DESC, submit_seq ASC -- the 15 S6.1 order) and move it
     'ready' -> 'running'. One running task at a time (single robot).

Each transition goes through apply_transition (the graph is the authority) +
TasksDAO.update_state, and calls on_transition(task_id, from_state, to_state,
reason) so
the caller can publish state/task + an event. The whole tick is one
transaction: a reader on another connection sees either the pre-tick or the
post-tick state, never a half-applied pass. It reads no clock (now_mono_ms
injected).

Boundary (NOT here): route expansion (mission_json -> waypoints/total_steps),
the actual robot execution (cmd/motion to P1) and the running -> done
completion that a P1 status reports. A dispatched task correctly sits at
'running' until that execution wiring lands.
"""
from __future__ import annotations

from typing import Awaitable, Callable, List, Tuple

from xbrain.p3_task.state.machine import (apply_transition,
                                          validate_suspend_fields)
from xbrain.p3_task.state.preconditions import (
    check_v1_type, check_v2_priority, check_v5_mission_parses,
)


# on_transition(task_id, from_state, to_state, reason) -> awaitable.
# from_state is what apply_transition was called with -- the event decision is
# made on the (from, to) pair, never on whether reason is non-empty.
# reason is '' on a
# clean transition, or the failing precondition code on a validate_fail.
OnTransition = Callable[[str, str, str], Awaitable[None]]


def validate_pending(row) -> Tuple[str, str]:
    """Run the pre-expansion preconditions on a pending task. Returns
    (event, reason): ('validate_ok', '') if all pass, else
    ('validate_fail', '<code>: <detail>')."""
    for check in (
        check_v1_type(row.task_type),
        check_v2_priority(row.priority),
        check_v5_mission_parses(row.mission_json),
    ):
        if check is not None:
            return "validate_fail", f"{check.code}: {check.detail}"
    return "validate_ok", ""


async def _dispatch(dao, task_id: str, now_mono_ms: int, started_at: str,
                    boot_id: str, made: List[Tuple[str, str, str]],
                    on_transition: OnTransition) -> None:
    """ready -> running, 走状态机并落 started_at/started_mono.

    抽出来是因为决策树的两个分支(空闲派发 / 抢占后派发)要做完全一样的事;
    写两遍的话, 将来给派发加一步(比如发 cmd/motion/behavior)必然漏掉一处.
    """
    result = apply_transition("ready", "dispatch")
    await dao.dispatch_task(task_id, now_mono_ms,
                            started_at=started_at,
                            started_mono=now_mono_ms / 1000.0,
                            started_boot=boot_id)
    made.append((task_id, "ready", result.to_state))
    await on_transition(task_id, "ready", result.to_state, "")


async def scheduler_tick(conn, dao, *, now_mono_ms: int,
                         on_transition: OnTransition,
                         started_at: str = "",
                         boot_id: str = "") -> List[Tuple[str, str, str]]:
    """One scheduler pass. Returns the transitions made as
    (task_id, from_state, to_state). See module docstring."""
    made: List[Tuple[str, str, str]] = []

    # -- phase 1: validate every pending task --
    rows = await dao.list_by_priority()          # (task_id, priority, seq, state)
    for task_id, _prio, _seq, state in rows:
        if state != "pending":
            continue
        full = await dao.fetch_by_id(task_id)
        if full is None:                          # vanished between calls
            continue
        event, reason = validate_pending(full)
        result = apply_transition("pending", event)
        await dao.update_state(task_id, result.to_state, now_mono_ms)
        made.append((task_id, "pending", result.to_state))
        await on_transition(task_id, "pending", result.to_state, reason)

    # -- phase 1b: yielding 自动恢复扫描 (15 S3.2) --
    #
    # *** 让位对象一进终态, 被抢占的任务自动回 ready, 无需任何外部条件.
    # 状态机逐字: suspended --resume--> ready (yielding: yielded-to task done).
    # 只到 ready, NO 不是直接回 running -- 之后由下面的决策树按优先级正常
    # 派发. 15 S7.3 那六步(快照比对/断点重映射/接入点/重推 route)是
    # ready -> running 那一跳的事, 依赖 task_route_snapshot 与 patrol_progress
    # 与 cmd/motion/route, 三者本期都未建, 所以这里[只做状态归位].
    #
    # *** 排在抢占之前, 顺序不能换.
    # 放在后面的话, 刚恢复到 ready 的任务会在同一拍里被下面的决策树再抢一次
    # (它优先级本来就低于当前 running), 于是 resume->preempt 每拍来回一次,
    # 而每一步都是合法迁移, 日志里看起来像两条任务在正常调度.
    #
    # *** 两种 yielding 的恢复条件不同, 只处理 preempted 这一支.
    # mode_takeover 的条件是 mode.motion_behavior 回 normal(15 S4.1A), 那要
    # 订 state/mode 并由 P2 驱动, 不在调度器的知情范围内 -- 混在这里会让
    # "模式还没交回来"的任务被误判为可恢复. 未建, 不假装(CLAUDE.md 3.2).
    yielding = await dao.list_yielding()
    if yielding:
        rows_y = await dao.list_by_priority()
        # 让位对象 = 当前仍在跑/在等的那些. 全部进终态(即活跃集里没有
        # running 也没有 ready)才算"让位对象已完成".
        # NO 不追踪"具体让给了谁": 那需要在抢占时记下被让位对象的 id, 而
        # tasks 表没有这一列. 用"没有 running"近似是保守的 -- 它只会推迟
        # 恢复(还有别的任务在跑时不恢复), 不会提前恢复.
        busy = any(st == "running" for _i, _p, _s, st in rows_y)
        if not busy:
            for task_id, _p, _sq, reason in yielding:
                if reason != "preempted":
                    continue
                result = apply_transition("suspended", "resume")
                await dao.resume_task(task_id, now_mono_ms)
                made.append((task_id, "suspended", result.to_state))
                await on_transition(task_id, "suspended", result.to_state,
                                    "yielded_to_done")

    # -- phase 2: 15 S6.1 的调度决策树 --
    # Re-read so the tasks just validated to 'ready' are visible (read-your-
    # writes on this connection, still inside the tick's transaction).
    rows2 = await dao.list_by_priority()
    ready = [(tid, p, s) for tid, p, s, state in rows2 if state == "ready"]
    running = [(tid, p, s) for tid, p, s, state in rows2 if state == "running"]
    # priority DESC, submit_seq ASC (15 S6.1). 排一次给下面两个分支共用.
    ready.sort(key=lambda r: (-r[1], r[2]))

    if not running:
        # 决策树第 1 步: 没有 running -> 取 ready 队列最高优先级启动.
        if ready:
            await _dispatch(dao, ready[0][0], now_mono_ms, started_at,
                            boot_id, made, on_transition)
    elif ready:
        # 决策树第 2 步: 有 running, 且 ready 队列里有[更高]优先级 -> 抢占.
        #
        # *** 严格大于, NO 不是 >=.
        # 15 S6.3 表第二行逐字"同优先级任务到达 -> 不抢占, 入队等待". 用 >=
        # 的话两条同优先级任务会互相抢占, 每拍换一次, 谁也跑不完 -- 而每次
        # 抢占都是一次合法的状态迁移, 没有任何东西会报错.
        cur_id, cur_prio, _cur_seq = running[0]
        top_id, top_prio, _top_seq = ready[0]
        if top_prio > cur_prio:
            # 15 S7.2 挂起动作的四步, 顺序不可换:
            #   1. 停止运动 cmd/motion/behavior=hold
            #      -- NO 本期做不到: P3 到 P1 那一跳未建(NEXT.md CLD-2),
            #         P3 全仓不发 cmd/motion/behavior. 与 ES-2(急停挂起)同一
            #         处境, 那里也是只做 DB 写. 不假装做到了(CLAUDE.md 3.2):
            #         链路建好后这里要补上, 且必须在状态写之前.
            #   2. 采样进度并同步落盘
            #      -- NO 本期无进度可采: 进度来自 state/motion/path_progress
            #         (NEXT.md EX-3, 未建), current_step 恒 0.
            #   3. status -> suspended, 同时写 suspend_kind/reason  <-- 做这步
            #   4. 发 event/info/task                                <-- 做这步
            # 3 与 4 是本期能做且必须做的部分: 少了 3, 抢占根本没发生; 少了 4,
            # 操作员看不到"我的任务为什么停了".
            result = apply_transition("running", "suspend")
            validate_suspend_fields(result.to_state, "yielding", "preempted")
            await dao.preempt_task(cur_id, "preempted", now_mono_ms)
            made.append((cur_id, "running", result.to_state))
            await on_transition(cur_id, "running", result.to_state, "preempted")
            # 抢占必须是"挂起"不是"取消"(15 S6.3 末行) -- 那是 U07 断点续跑
            # 的前提. suspend_kind=yielding 的恢复条件是"让位对象一进终态就
            # 自动回 ready"(15 S3.2), 不需要人工干预.
            await _dispatch(dao, top_id, now_mono_ms, started_at,
                            boot_id, made, on_transition)

    # One commit closes the whole tick atomically (see module docstring).
    await conn.commit()
    return made


# Motion result closed set (11 S3.5 relative_move/status terminal values). A
# 'succeeded' completes the task; 'aborted'/'rejected' fail it. 'accepted'/
# 'running' are in-flight and drive no task transition.
_MOTION_TERMINAL = {"succeeded": "complete", "aborted": "fail",
                    "rejected": "fail"}


def compute_duration_sec(started_mono: "float | None",
                         started_boot: "str | None",
                         now_mono_ms: int, boot_id: str) -> "float | None":
    """15 S9.5 的 duration_sec 口径. 跨重启返回 None.

    *** 跨重启必须是 NULL, NO 不得回退用墙钟差值.
    文档逐字: "若终态时的 boot != started_boot, duration_sec 写 NULL, 不得
    回退用墙钟差值充数 -- 那正是本组列要消除的东西". 理由是单调钟只在同一次
    开机内可比(11 CLK-C4): 跨重启的 now_mono - started_mono 是个没有意义的
    数, 而它[看起来完全像个正常时长], 没有任何迹象表明它是错的.
    NULL 的含义是"这次任务跨了重启, 耗时不可知", 上报给甲方的
    summary.duration_sec 也随之为 null(11 S4.4).

    口径是[开始到终态], 不含排队等待 -- 所以基准是 started_mono(派发那一刻)
    而不是 created_ms(入库那一刻).
    """
    if started_mono is None or not started_boot:
        return None                    # 没记开始 -> 无从算起
    if started_boot != boot_id:
        return None                    # 跨重启
    return max(0.0, now_mono_ms / 1000.0 - float(started_mono))


async def apply_motion_result(conn, dao, task_id: str, result: str, *,
                              now_mono_ms: int,
                              on_transition: OnTransition,
                              finished_at: str = "",
                              boot_id: str = "") -> bool:
    """Close a running task on the P1 motion result (11 S3.5). 'succeeded' ->
    running -> done; 'aborted'/'rejected' -> running -> failed. Returns True if
    a transition was made.

    This is the lifecycle-closing half of execution. It is a pure step the
    (future) motion-status subscriber calls; it does NOT subscribe or emit
    cmd/motion -- P1 executing a real path + reporting this status is the
    execution-wiring milestone (P1 today is an ad-hoc-motion MVP). An in-flight
    'accepted'/'running', or a result for a task that is not running, is a
    no-op (not an error: a late status after cancel is legal)."""
    event = _MOTION_TERMINAL.get(result)
    if event is None:
        return False                              # accepted/running: in-flight
    full = await dao.fetch_by_id(task_id)
    if full is None or full.state != "running":
        return False                              # already terminal / cancelled
    transition = apply_transition("running", event)
    # 终态要落 finished_at 与 duration_sec(15 S9.5), update_state 只写 state.
    # 少了这两列: v2.0 S3.3 的 result.summary.duration_sec 无源, 而
    # idx_tasks_finished_at 这个部分索引(WHERE finished_at IS NOT NULL)
    # 永远命中不了任何行 -- 历史任务查询会全表扫.
    await dao.finish_task(
        task_id, transition.to_state, now_mono_ms,
        finished_at=finished_at,
        duration_sec=compute_duration_sec(
            getattr(full, "started_mono", None),
            getattr(full, "started_boot", None),
            now_mono_ms, boot_id))
    await conn.commit()
    await on_transition(task_id, "running", transition.to_state,
                        f"motion:{result}")
    return True
