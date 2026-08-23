"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: task_control_request.py
Brief: 18 B 类任务控制 -> 11 S7.2 TaskCommand, 发起方解析 task_id

Description:
"先停一下" / "接着走" / "别巡了" 三条语音指令的落点. 它们要发的是 S7.2 的
control 动作(pause / resume / cancel), 而 S7.2 逐字禁止"省略 = 当前任务".

*** 那条禁令禁的是[接收方猜], 不是[发起方解析].
S7.2 给了四条理由, 第一条是要害: 队列是活的 -- 操作员在面板上看到"A 在跑"到
命令抵达之间, A 可能已结束而 B 开始, 简写会暂停错的那条, 且日志里看不出发生过
这件事. 其余三条: 语音要加 0.5-2 s 的 ASR+LLM+确认; 幂等要求重发的 cmd_id
含义不变; 审计要能还原操作员当时指的是哪条.

=> 本模块在[发起方]把"当前任务"解析成一个具体的 task_id, 帧里写死它. 四条
理由全部落地: P3 收到的是一个确定的 id, 若那条任务已经变了, P3 回 E_TASK_STATE
而不是默默暂停另一条; 幂等键是 cmd_id; 审计能看到当时指的是谁.

*** 解析源是 state/task 的 active_task, 不是猜.
拿不到就[口头说没有正在执行的任务], NO 不发帧 -- 发一条 task_id 为空的
control 命令就是把猜的动作推给 P3, 那正是 S7.2 要防的.

*** 五条 B 类里只做得了三条, 另两条如实不接:
  * B10 skip_waypoint -- 18 效果列"P3 路径推进", 而 S7.2 的动作闭集是
    submit / cancel / pause / resume / clear_queue, [没有 skip]. 用 cancel
    代替是错的(那会结束整条任务), 硬凑一个动作更错.
  * B12 stop_follow -- 18 效果列"退出目标导向行为", 这是运动[行为]不是
    任务动作; 它对应 S7.3 ModeCommand 的 set_behavior(NAV-50 三值回 normal),
    走 cmd/mode, 见 mode_request.py.

Boundaries: 只解析与打包. 不判任务状态(那是 P3 的状态机), 不读钟, 不发帧.
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

#: 18 B 类 -> 11 S7.2 action. 只含能表达成 S7.2 动作的三条.
_CONTROL_INTENTS: Dict[str, str] = {
    "pause_task": "pause",
    "resume_task": "resume",
    "cancel_task": "cancel",
}

#: state/task 的 key, 与 P3 的发布端一致(11 S2.2.3).
STATE_TASK_KEY = "state/task"


class TaskControlError(RuntimeError):
    """解析不出要控制哪条任务. 口头说明, NO 不发帧. """


def is_task_control_intent(intent_name: str) -> bool:
    return intent_name in _CONTROL_INTENTS


def active_task_from_state(state: Optional[Mapping[str, Any]]
                           ) -> Optional[Dict[str, Any]]:
    """state 快照里的 active_task, 或 None.

    P3 在没有活动任务时发的是 active_task: null(不是省略该键), 所以 None 与
    "没收到过 state/task" 在这里是同一个答案 -- 两者对操作员的意义相同:
    没有可控制的任务.
    """
    body = (state or {}).get(STATE_TASK_KEY)
    if not isinstance(body, Mapping):
        return None
    active = body.get("active_task")
    return dict(active) if isinstance(active, Mapping) else None


def to_task_control_command(intent_name: str, *,
                            state: Optional[Mapping[str, Any]],
                            cmd_id: str, source: str,
                            reason: str = "") -> Optional[Dict[str, Any]]:
    """B 类控制意图 -> TaskCommand 本体; 不是控制意图则 None.

    抛 TaskControlError 表示"归我管但不知道控制哪条".
    """
    action = _CONTROL_INTENTS.get(intent_name)
    if action is None:
        return None
    active = active_task_from_state(state)
    task_id = (active or {}).get("task_id")
    if not isinstance(task_id, str) or not task_id:
        # NO 不发一条 task_id 为空的 control 命令: 那等于把"猜哪条"推给 P3,
        # 正是 S7.2 禁止的那件事.
        raise TaskControlError("no_active_task")
    payload: Dict[str, Any] = {
        "cmd_id": cmd_id,
        "action": action,
        "task_id": task_id,
        "source": source,
    }
    if reason:
        payload["reason"] = reason
    return payload


def spoken_target(state: Optional[Mapping[str, Any]]) -> str:
    """把"我要操作的是哪条"说给操作员听, 用于确认与回执.

    *** 这不是装饰. S7.2 第一条理由是"队列是活的", 而发起方解析只在操作员
    [能听见解析结果]时才真正安全 -- 说出来他才有机会喊停一条选错的任务.
    """
    active = active_task_from_state(state) or {}
    task_id = active.get("task_id")
    return str(task_id) if task_id else ""
