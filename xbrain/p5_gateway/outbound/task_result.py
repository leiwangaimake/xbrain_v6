"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_result.py
Brief: state/task 权威终态 message_type=result 与它的一次性判定 (B-5)

Description:
v2.0 R12.4 把终态并进 state/task, 用 message_type 区分, 不另设 task/result.
snapshot 由 CloudProjector 按节律发; result 是[事件驱动]的 -- 一个任务
结束时发一条, 且只发一条.

*** "只发一条"是本文件的全部难点.
p3_task 的 state/task 是[变化即发]的广播, 而同一个终态可能被重复广播
(重连补发, 周期重发, 或者下游多次读到同一行). 每次都发一条 result 的话:
  * Qt 的操作记录里同一个任务出现多条完成
  * 而 v2.0 S8 逐字要求 task event 与 result 用相同 task_id 和结果值,
    审计时对不上
反过来漏发一条 result, 操作员会看到任务永远停在 running -- 他会去中止一个
已经完成的任务.

*** 终态判定按[跃迁]而不是按[当前值].
只看当前值的话, 网关重启后第一次收到 state/task 就会把一个早已完成的任务
当成"刚刚完成"再发一条 result. 所以本文件记住每个 task_id 上一次的 state,
只有 [非终态 -> 终态] 那一次才产出 result.
* 由此得到一个必须接受的后果: 网关重启期间完成的任务不会补 result.
那条信息在 event/{sev}/task 里(可靠面, 断网补发覆盖它), 不在这里补 --
在这里补就要读 task.db 并判断"哪些 Qt 还没收到", 而 Qt 收没收到网关无从
得知. 与其编一个判断, 不如让可靠面去做它本来就该做的事.

Boundaries: 只做终态判定与形状. 不发布, 不读钟(ts 由调用方给).
"""

from __future__ import annotations

import uuid
from typing import Any, Dict, Optional, Tuple

from .state_projection import ProjectionError

#: v2.0 S3.3: result 的 state 闭集. NO 与 snapshot 的七值任务态不是一回事 --
#: snapshot 描述"任务正处在哪", result 描述"它以什么方式结束".
RESULT_STATES = ("done", "failed", "cancelled")

#: 机内任务态 -> v2.0 result state. 只有这三个是终态.
#: 11 S4.4 的 TaskState 十二值里, 其余九个都不产生 result.
TERMINAL_MAP = {
    "completed": "done",
    "done": "done",
    "failed": "failed",
    "cancelled": "cancelled",
    "canceled": "cancelled",       # 机内两种拼写都出现过, 都接
}


def build_result(*, task_id: str, task_type: str, state: str,
                 result_code: int = 0, reason: str = "",
                 completed_count: int = 0, total_count: int = 0,
                 distance_m: Optional[float] = None,
                 # None = 未知. 15 S9.5 与 11 S4.4 逐字要求未知写 NULL,
                 # 0.0 会在 Qt 上显示成"用时 0 秒"(CLAUDE.md 3.1 的
                 # "0.0 冒充已赋值"). 默认值也改成 None: 默认 0.0 的话,
                 # 调用方漏传就自动变成一个假的已知值.
                 duration_sec: Optional[float] = None,
                 started_ts: Optional[float] = None,
                 ended_ts: Optional[float] = None,
                 route_id: Optional[str] = None,
                 route_rev: Optional[int] = None,
                 detail: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """v2.0 S3.3 的 result data.

    *** 失败与取消必须带 reason(逐字"失败/取消必填").
    一条 state=failed 而 reason 为空的 result, 在 Qt 上是"任务失败"四个字
    加一片空白 -- 操作员唯一能做的是重试, 而如果失败原因是围栏外, 重试会
    再失败一次.

    * 成功时 result_code 必须是 0, 失败时必须非 0. 与 ack 那条同一个理由:
    两个字段表达同一件事时它们迟早会不一致.
    """
    if state not in RESULT_STATES:
        raise ProjectionError(
            "result state %r not in the v2.0 S3.3 closed set %s"
            % (state, RESULT_STATES))
    if state == "done" and result_code != 0:
        raise ProjectionError(
            "state=done implies result_code 0; got %d" % result_code)
    if state != "done":
        if result_code == 0:
            raise ProjectionError(
                "state=%s requires a non-zero result_code" % state)
        if not reason:
            raise ProjectionError(
                "state=%s requires a reason (v2.0 S3.3)" % state)
    return {
        "msg_id": uuid.uuid4().hex,
        "message_type": "result",
        "task_id": task_id,
        "task_type": task_type,
        "state": state,
        "result_code": int(result_code),
        "reason": reason,
        "summary": {
            "completed_count": int(completed_count),
            "total_count": int(total_count),
            # distance_m 不适用时为 null(v2.0 逐字). 一个 SET_ALARM_CONFIG
            # 没有里程可言, 填 0 会让 Qt 显示"行进 0 米"而不是"不适用".
            "distance_m": distance_m,
            # None 原样透传: v2.0 S3.3 允许 null, 而 float(None) 会抛,
            # 把整条 result 吃掉 -- 那比少一个字段严重得多(云端就此
            # 不知道任务结束了).
            "duration_sec": (None if duration_sec is None
                             else float(duration_sec)),
            "started_ts": started_ts,
            "ended_ts": ended_ts,
            # 导航任务必填, 其他可 null. NO 不用空字符串代替 null --
            # 空字符串是一个"存在但为空"的 ID, Qt 会拿它去查路径.
            "route_id": route_id,
            "route_rev": route_rev,
        },
        "detail": dict(detail or {}),
    }


class TaskResultTracker:
    """按[跃迁]判定终态, 每个任务只产出一次 result.

    * 记的是 task_id -> 上一次见到的机内 state. 内存增长有界吗: 每条任务
    一个条目, 而任务到终态后条目会被删掉(见 observe) -- 只有从未结束的
    任务会留着, 那与 task.db 里的在途任务数同量级.
    """

    def __init__(self) -> None:
        self._last: Dict[str, str] = {}
        #: 已产出过 result 的 task_id. 与 _last 分开: 一个终态任务的条目
        #: 从 _last 删掉后, 若它的 state/task 被重播, 不能再产一条.
        self._done: set = set()

    def observe(self, task_id: str, state: Any
                ) -> Optional[Tuple[str, str]]:
        """喂一次 state/task 的观察. 返回 (task_id, v2.0 result state)
        当且仅当[这一次是非终态到终态的跃迁且此前没产出过].

        NO 不在这里构造 result -- 构造要的 summary 字段(里程, 时长)本模块
        没有, 由调用方从任务记录里取. 这里只回答"该发了吗".
        """
        if not isinstance(task_id, str) or not task_id:
            return None
        terminal = TERMINAL_MAP.get(state) if isinstance(state, str) else None
        previous = self._last.get(task_id)
        if terminal is None:
            # 非终态: 记下来, 供下一次判跃迁.
            if isinstance(state, str):
                self._last[task_id] = state
            return None
        if task_id in self._done:
            # 重播. 见类文档.
            return None
        if previous is None or TERMINAL_MAP.get(previous) is not None:
            # previous is None: 从没见过这个任务的非终态. 那不是跃迁, 是
            # [重启后的重放] -- 网关刚起来, p3 广播的第一条就带着一个早已
            # 完成的任务. 按当前值判的话会再发一条 result, 而 Qt 十分钟前
            # 就收到过了.
            #
            # *** 这条规则成立有一个前提: 观察点必须在[每一次广播]上, 不能
            # 只在 10 Hz 的 tick 上. 一个 50 ms 内 running -> completed 的
            # 快任务, 在 tick 采样下只会被看到 completed 一次, 于是按本规则
            # 永远发不出 result -- Qt 一直等. 所以 CloudProjector 把
            # observe_task 挂在 state/task 的 Zenoh 回调上(每次广播一次),
            # tick 里那次只是兜底.
            # 这个前提有一条断言盯着(test_cloud_projector 里查 main_wiring
            # 的 observe_task 调用), 因为规则与前提分在两个文件里, 单看
            # 任何一边都看不出依赖.
            #
            # TERMINAL_MAP.get(previous) is not None: 上一次就已是终态,
            # 同样不是跃迁.
            return None
        self._done.add(task_id)
        self._last.pop(task_id, None)
        return task_id, terminal

    def forget(self, task_id: str) -> None:
        """任务记录被清理时调用, 让同 ID 的新任务能再产 result.

        * 现实里 task_id 不复用(t-YYYYMMDD-NNN 是按日序列), 所以这个口子
        今天没有调用者. 保留它是因为 _done 若永不清理就是一个只增不减的
        集合 -- 在一台连续跑几个月的机器上会成为真实的内存增长.
        """
        self._done.discard(task_id)
        self._last.pop(task_id, None)

    def pending(self) -> int:
        """在途任务条目数. 只为可观测."""
        return len(self._last)


__all__ = ["build_result", "TaskResultTracker", "RESULT_STATES",
           "TERMINAL_MAP"]
