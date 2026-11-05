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

*** 解析源是 state/task 的 current(11 S4.4 TaskState), 不是猜.
拿不到就[口头说没有正在执行的任务], NO 不发帧 -- 发一条 task_id 为空的
control 命令就是把猜的动作推给 P3, 那正是 S7.2 要防的.

*** 五条 B 类里只做得了三条, 另两条如实不接:
  * B10 skip_waypoint -- 18 效果列"P3 路径推进", 而 S7.2 的动作闭集是
    submit / cancel / pause / resume / clear_queue, [没有 skip]. 用 cancel
    代替是错的(那会结束整条任务), 硬凑一个动作更错. 2026-11-05 起该意图
    已从注册表删除 (99 U129, 用户裁决"不支持, 不需要实现, 删除").
  * B12 stop_follow -- 18 效果列"退出目标导向行为", 这是运动[行为]不是
    任务动作; 它对应 S7.3 ModeCommand 的 set_behavior(NAV-50 三值回 normal),
    走 cmd/mode, 见 mode_request.py.

*** 99 U111 (2026-11-04 整栈实测): "继续"从来没恢复过任何任务.
暂停让当前任务进 suspended, 调度器同一秒拉起队列里的下一条, 于是"继续"
解析到的 current 是那条新任务, P3 回 E_TASK_STATE(它在 running); 急停后
current 为 null, 只能答"没有正在执行的任务". 可恢复的任务从来不在 current,
在 state/task 的 suspended[](11 S4.4) 里. 现在按动作取目标:
  pause  -> current(正在跑的那条);
  resume -> suspended[] 里最近挂起的 passive 条(人工暂停 / 急停 / 低电);
            yielding 不选 -- 它会自己回来, 人工 resume 被契约拒收(S7.2);
  cancel -> current; 没有 current 时同 resume 的选法(暂停之后说"取消任务",
            操作员指的就是刚暂停的那条).
解析结果仍是一个写死的 task_id, S7.2 的四条理由照旧成立.

Boundaries: 只解析与打包. 不判任务状态(那是 P3 的状态机), 不读钟, 不发帧.
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

from xbrain.p4_agent.runtime import turn_labels  # 11 provenance closed sets

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


def current_task_from_state(state: Optional[Mapping[str, Any]]
                            ) -> Optional[Dict[str, Any]]:
    """state 快照里 11 S4.4 TaskState 的 current, 或 None.

    *** 字段名是契约的: 11 S9.5A 逐字"P4/P2 在生成 TaskCommand 时从
    state/task.current.task_id 取值填入". 本函数曾读 active_task, 那是 P3
    占位广播的字段名 -- 两侧一致地偏离契约, 于是谁也没发现, 直到 P3 改发
    真正的 TaskState 才暴露.

    P3 在没有活动任务时发的是 current: null(不是省略该键), 所以 None 与
    "没收到过 state/task" 在这里是同一个答案 -- 两者对操作员的意义相同:
    没有可控制的任务.

    *** 只看 current, NO 不去 queue 里挑一条. 队列里的任务还没开跑,
    对"暂停 / 取消当前任务"这句话来说它不是操作员指的那一条; 替他挑一条
    正是 S7.2 禁止的"把猜哪条推给下游".
    """
    body = (state or {}).get(STATE_TASK_KEY)
    if not isinstance(body, Mapping):
        return None
    current = body.get("current")
    return dict(current) if isinstance(current, Mapping) else None


def latest_passive_suspended(state: Optional[Mapping[str, Any]]
                             ) -> Optional[Dict[str, Any]]:
    """state/task.suspended[] 里最近挂起的 passive 条, 或 None (99 U111).

    按 suspended_ts 取最大; 没有时间的条排最后(拿不准就不抢先).
    """
    body = (state or {}).get(STATE_TASK_KEY)
    items = body.get("suspended") if isinstance(body, Mapping) else None
    if not isinstance(items, list):
        return None
    passive = [i for i in items if isinstance(i, Mapping)
               and i.get("suspend_kind") == "passive"
               and isinstance(i.get("task_id"), str) and i.get("task_id")]
    if not passive:
        return None
    return dict(max(passive, key=lambda i: (
        isinstance(i.get("suspended_ts"), (int, float)),
        i.get("suspended_ts") or 0)))


def control_target(action: str, state: Optional[Mapping[str, Any]]
                   ) -> Optional[Dict[str, Any]]:
    """The task an action means (module docstring, 99 U111), or None."""
    if action == "resume":
        return latest_passive_suspended(state)
    current = current_task_from_state(state)
    if current is not None or action == "pause":
        return current
    return latest_passive_suspended(state)   # cancel with nothing running


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
    if source not in turn_labels.TASK_SOURCES:
        # P3 只认显式表里的来源; 闭集外是调用方 bug.
        raise ValueError("task source %r not in %s"
                         % (source, sorted(turn_labels.TASK_SOURCES)))
    active = control_target(action, state)
    task_id = (active or {}).get("task_id")
    if not isinstance(task_id, str) or not task_id:
        # NO 不发一条 task_id 为空的 control 命令: 那等于把"猜哪条"推给 P3,
        # 正是 S7.2 禁止的那件事.
        raise TaskControlError(
            "no_suspended_task" if action == "resume" else "no_active_task")
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
    active = current_task_from_state(state) or {}
    task_id = active.get("task_id")
    return str(task_id) if task_id else ""
