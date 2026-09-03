"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_events.py
Brief: 11 S6.2 task events -- task state transition -> event/{sev}/task

Description:
The 11 S6.2 `task` row is info/warn, channel normal, and fires on 任务 accept /
reject / start / complete / fail. P3 already drives every task state transition
through the scheduler's on_transition callback (runtime/main_wiring _make_publish),
so this module is the PURE mapping from a (from_state, to_state) transition to the
task event kind + severity -- the wiring emits event/{sev}/task whenever this
returns non-None.

The decision is made on the TRANSITION, not on the reason text. See the comment
above _TRANSITION_EVENT for why: reason is free text written for an operator, and
the old "non-empty reason means rejected" rule turned preemption, auto-resume,
clear_queue, motion results and every operator pause/cancel into false rejections.

Not every transition is an event: the waiting-state shuffles (scheduled -> ready,
blocked -> ready), suspend and resume, and the pre-power-off bookkeeping return
None. Suspend/resume in particular are NOT in S6.2's five kinds -- an operator
watching a pause reads it off the state/task snapshot (state=paused, published at
1 Hz), which is the contracted channel for state; the event stream carries what
happened, not a mirror of the state.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple


TASK_CATEGORY = "task"


#: (from_state, to_state) -> (kind, sev), or None when the transition warrants
#: no 11 S6.2 task event.
#:
#: *** 判别依据是[迁移], NO 不是 reason 是否非空.
#: 原实现写的是 `if reason: return ("rejected","warn")`, 依据是"只有校验失败
#: 会填 reason". 那个前提后来被六个调用点破坏了: 抢占填 "preempted", 自动恢复
#: 填 "yielded_to_done", 清队列填 "clear_queue", 运动结果填 "motion:...",
#: 操作员的暂停/恢复/取消填他自己写的话. 于是这六种全部报成"拒绝" -- 2026-09-03
#: 甲方按一次暂停, 界面上就是一条 "task ... rejected".
#: 根子上的毛病是拿一个[自由文本]字段当闭集判据. reason 非空只说明有人写了句
#: 话, 推不出"这是拒绝". 拒绝有唯一的迁移特征: ("pending","failed").
#:
#: *** 暂停与恢复不发事件, 这是 S6.2 定的.
#: 该类逐字是"任务接受/拒绝/开始/完成/失败", 没有暂停恢复. 操作员要看暂停
#: 状态, 通道是 state/task 快照(state=paused, 1 Hz 在发), 事件流是"发生了什么
#: 值得记一笔", 不是状态镜像. 用户 2026-09-03 确认按 S6.2 办.
#:
#: *** 表必须[覆盖状态机的每一条边], 由 test_task_events 的双向差集判据守.
#: 缺一条就会在运行期抛(见下), 而不是静默不发 -- 静默不发正是 running->done
#: 那条的老毛病: 旧表的键写的是 "succeeded"/"aborted", 这两个状态在
#: machine.py 里根本不存在, 于是任务跑完永远不发"完成"事件. 今天没暴露只是
#: 因为还没有执行器能让任务跑完.
_TRANSITION_EVENT: Dict[Tuple[str, str], Optional[Tuple[str, str]]] = {
    # -- 准入 (pending 出发) --
    ("pending", "ready"):        ("accepted", "info"),   # 校验通过, 入队
    ("pending", "scheduled"):    ("accepted", "info"),   # 受理, 到点再跑
    ("pending", "blocked"):      ("accepted", "info"),   # 受理, 等依赖
    ("pending", "failed"):       ("rejected", "warn"),   # 唯一的"拒绝"
    ("pending", "cancelled"):    ("cancelled", "warn"),
    # -- 等待态之间流转: 内部记账, 操作员从 state/task 就能看到 --
    ("scheduled", "ready"):      None,                   # 到点
    ("scheduled", "blocked"):    None,
    ("scheduled", "cancelled"):  ("cancelled", "warn"),
    ("blocked", "ready"):        None,                   # 依赖解除
    ("blocked", "cancelled"):    ("cancelled", "warn"),
    # -- 执行 --
    ("ready", "running"):        ("started", "info"),
    ("ready", "suspended"):      None,                   # 排队时急停
    ("ready", "cancelled"):      ("cancelled", "warn"),
    ("running", "done"):         ("completed", "info"),
    ("running", "failed"):       ("failed", "warn"),
    ("running", "suspended"):    None,                   # 暂停 / 被抢占
    ("running", "cancelled"):    ("cancelled", "warn"),
    # -- 挂起之后 --
    ("suspended", "ready"):      None,                   # 恢复 / 让位结束
    # needs_review 报 failed, 不是自己造一个新 kind: 云端投影
    # (INTERNAL_TO_V2_TASK_STATE) 已经把 needs_review 映射成 failed, 事件面
    # 跟着它走才不会与快照面互相打架.
    ("suspended", "needs_review"): ("failed", "warn"),
    ("suspended", "failed"):     ("failed", "warn"),     # 恢复放弃
    ("suspended", "cancelled"):  ("cancelled", "warn"),
    # -- 关机前的记账: 任务此前已经终结并发过事件, 再发一条就是重复上报 --
    ("done", "wait_for_power_off"):         None,
    ("failed", "wait_for_power_off"):       None,
    ("cancelled", "wait_for_power_off"):    None,
    ("needs_review", "wait_for_power_off"): None,
}


def task_event_for_transition(from_state: str,
                              to_state: str) -> Optional[Tuple[str, str]]:
    """Return (kind, sev) for one task state transition, or None when it warrants
    no 11 S6.2 task event.

    reason is deliberately NOT a parameter: it is free text written for a human,
    and inferring a closed-set decision from "is it non-empty" is the defect this
    function was rewritten to remove.

    An (from, to) pair the state machine can produce but this table does not
    cover raises: silently returning None would make a newly added edge stop
    reporting without anyone noticing, which is exactly how running->done came
    to emit nothing. The completeness metatest keeps the raise unreachable.
    """
    key = (from_state, to_state)
    if key not in _TRANSITION_EVENT:
        raise KeyError(
            "no 11 S6.2 task event decided for transition %r -> %r; add it to "
            "_TRANSITION_EVENT (every machine.py edge must be covered)"
            % (from_state, to_state))
    return _TRANSITION_EVENT[key]
