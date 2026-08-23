"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: system_request.py
Brief: 18 H 类系统指令 -> 11 S7.15 SystemCommand (cmd/system)

Description:
H 类("跑个自检" / "休眠" / "重启系统")的落点. 11 S2.2.3 把 cmd/system 的发布者
列为"云端 / p5_gateway(HMI) / p4_agent(H 类语音, origin:"voice")", 并按 action
把订阅拆给[三个并行订阅者](SYS-1):

    p5_gateway    <- reboot / shutdown / time_sync / generate_report
    p2_core       <- sleep / wake
    p2_core(BIT)  <- run_bit

*** 2026-08-23 之前 H 类被前缀表路由到了 cmd/task.
那张表里 "H": CMD_TASK 与 C 类是同一种错: 照一套假想意图集写的. P3 收到没有顶层
action 的帧就 skip, 于是 H 类 8 条全部落空且两侧都不报错. 本模块把发送侧改到
契约 key 并按 S7.15.1 打报文.

*** 接收端[三处全缺], 这批不建.
全仓没有任何 cmd/system 订阅者(只有 authz/levels.py 的注释提到它). 所以改完路由
之后 H 类[仍然不会生效]-- 但它现在是"发在正确的 key 上, 形状正确, 等一个还没
建的订阅者", 而不是"发在错的 key 上被另一个进程主动丢弃". 这两件事在联调时的
表现完全不同: 前者一接订阅者就通, 后者会让人去查 P3 为什么丢帧.

*** H04 reload_config 不在本 key.
18 逐字"NO 不进 cmd/system", 它走 cmd/config(S7.6) 的 ConfigCommand -- 那是
另一个消息体, 不是本模块的事. 它仍留在原路由并被如实拒绝, 已记 NEXT.

*** origin 是全系统仅有的两道授权边界之一(U23: HMI 不鉴权 => 通道即权限).
语音发出的一律 origin:"voice", NO 不从槽位里取 -- 与 cmd/geo 的 CH-2 同理.

Boundaries: 只打包. 不判 L2/L3 确认令牌(S7.15.5 由执行方签发), 不判通道权限
(S7.15.4 是接收侧的事), 不读钟.
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

#: S7.15.2 action 七值闭集. 本表只列 H 类语音能发起的; shutdown 也在闭集里,
#: 但 18 的 H08 有自己的确认要求, 与 reboot 同属 L2.
_SYSTEM_INTENTS: Dict[str, str] = {
    "run_bit": "run_bit",
    "generate_report": "generate_report",
    "set_time_sync": "time_sync",
    "sleep": "sleep",
    "wake": "wake",
    "reboot": "reboot",
    "shutdown": "shutdown",
}

#: 各 action 会用到的专有参数. S7.15.1 逐字: 未用到的一律不填, 填了也一律忽略
#: (不报错, 进 ack.detail.ignored) -- 所以这里按 action 挑, 不整包透传.
_ACTION_PARAMS: Dict[str, tuple] = {
    "run_bit": ("scope",),
    "generate_report": ("scope", "range", "task_id"),
    "time_sync": ("force_step",),
    "reboot": ("delay_s",),
    "shutdown": ("delay_s",),
}

#: 语音来源恒 voice. origin 是授权边界, 不接受发起方自称.
_VOICE_ORIGIN = "voice"


class SystemRequestError(RuntimeError):
    """这条系统指令翻不出合法的 SystemCommand. """


def is_system_intent(intent_name: str) -> bool:
    return intent_name in _SYSTEM_INTENTS


def to_system_command(intent_name: str, *, slots: Mapping[str, Any],
                      cmd_id: str, source: str,
                      reason: str = "") -> Optional[Dict[str, Any]]:
    """H 类意图 -> SystemCommand 本体; 不是系统指令则 None. """
    action = _SYSTEM_INTENTS.get(intent_name)
    if action is None:
        return None
    payload: Dict[str, Any] = {
        "v": 1,
        "cmd_id": cmd_id,
        "action": action,
        # NO 不从 slots 取: 见模块头注, origin 是授权边界.
        "origin": _VOICE_ORIGIN if source in ("voice", "text") else source,
    }
    for name in _ACTION_PARAMS.get(action, ()):
        value = slots.get(name)
        if value is not None:
            payload[name] = value
    if reason:
        payload["reason"] = reason
    return payload
