"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: mode_request.py
Brief: 18 C 类模式意图 -> 11 S7.3 ModeCommand (p4_agent -> p2_core)

Description:
把语音说出来的模式意图翻成契约的 ModeCommand. 11 S2.2.3 逐字把 cmd/mode 的
发布者列为"云端 / 微信 / HMI / p4_agent", 订阅者 p2_core; 11 S7.3A.1 的裁决
又把分界线写死: A 类(运动原语) / C 类(模式) -> P2, B 类(导航/巡逻) -> P3.

*** 在 2026-08-21 之前, p4_agent 把整个 C 类路由到了 cmd/task.
intent_dispatch 的前缀表里那行是 `"C": CMD_TASK`, 注释写着"C01-C07 (hold,
slow, cancel, look, spin)"-- 那描述的不是 18 册的 C 类, 是照一套假想意图集
写的. 于是"进入喊话模式"发到 cmd/task, P3 因为帧里没有顶层 action 而 skip,
两侧都不报错. 加上 P2 当时根本没订 cmd/mode, 这条链两头都不通.

*** 只翻译六条, 另外两条[故意不接].
按 18 册每一行的效果列逐条核对, C 类里只有六条的效果是"P2 -> ...":

  C01 enter_alarm       -> set_voice_mode(alarm)
  C02 exit_alarm        -> exit_alarm
  C03 enter_broadcast   -> set_voice_mode(broadcast)
  C04 exit_broadcast    -> exit_broadcast
  C05 enter_patrol_mode -> set_voice_mode(dialog)   [18: "P2 -> DIALOG + P3 恢复调度"]
  C07 set_motion_behavior -> set_behavior

不接的两条, 以及为什么不能顺手接:
  * C06 standby -- 18 的效果列是"P3 挂起任务 + P1 hold", 不是 P2. 它落在
    C 类里但效果在别的进程, 需要裁决它到底走哪条 key; 猜一个等于替人做主.
  * C08 query_mode_switch_ok -- 18 标它"查询类(预检)"L0, 操作员问的是
    "现在能切到喊话吗". 把一个提问翻成 ModeCommand 就是[替他切了]--
    这是本模块最容易犯且后果最直接的错.

两条都保持原路由不变并记在 NEXT, NO 不在这里假装支持.

Boundaries: 只做翻译, 不裁决模式规则(那是 P2 的 ModeStateMachine), 不填运动
参数(11 S9.3.2A 的 MO-2: 运动参数一律 P2 填, 不收 P4 的), 不读钟.
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

#: 18 C 类 -> S7.3 action. 值是 (action, voice_mode), voice_mode 仅
#: set_voice_mode 用得上, 其余为 None.
#:
#: *** 按 intent NAME 建表, 不按 id. id 是 18 册的编号, 会随册子改版重排;
#: name 是注册表里的稳定标识, 且 turn_orchestrator 手上拿到的就是 name.
_MODE_INTENTS: Dict[str, tuple] = {
    "enter_alarm": ("set_voice_mode", "alarm"),
    "exit_alarm": ("exit_alarm", None),
    "enter_broadcast": ("set_voice_mode", "broadcast"),
    "exit_broadcast": ("exit_broadcast", None),
    # C05 "进入巡逻模式 / 开始工作": 18 的效果列是"P2 -> DIALOG + P3 恢复
    # 调度". P4 只发得出 P2 那一半 -- 恢复调度是 P2/P3 之间的下游动作, 不是
    # 语音侧再补发一条 cmd/task 能替代的(那会造出第二个真源).
    "enter_patrol_mode": ("set_voice_mode", "dialog"),
    "set_motion_behavior": ("set_behavior", None),
    # A13. It sits in the A class but 11 S7.3 makes set_speed_profile a
    # ModeCommand ACTION, and S7.3.1 (ruling D-04) refused to open a separate
    # MotionCommand for it: "one more top-level key that can raise the speed
    # ceiling" is the worse failure direction, because every such key must
    # then be justified in the S12.1.1 HMI whitelist.
    "set_speed_profile": ("set_speed_profile", None),
}

#: C07 的 behavior 槽闭集 (NAV-50 三种). 闭集外必抛 -- 见 to_mode_command.
BEHAVIOR_VALUES = frozenset({"normal", "follow", "face_target"})

#: A13 profile closed set. U33 DELETED cruise and transit; 18 S3.0 says GBNF
#: and schema validation must both REFUSE those two strings, never map them
#: onto patrol.
PROFILE_VALUES = frozenset({"obstacle_avoid", "patrol"})


class ModeRequestError(RuntimeError):
    """这条模式意图翻不出合法的 ModeCommand. 口头拒绝, 不发帧. """


def is_mode_intent(intent_name: str) -> bool:
    return intent_name in _MODE_INTENTS


def to_mode_command(intent_name: str, *, slots: Mapping[str, Any],
                    cmd_id: str, source: str) -> Optional[Dict[str, Any]]:
    """C 类意图 -> ModeCommand 本体; 不是模式意图则 None.

    None 与抛异常是两件事: None = "这条意图不归我管"(调用方继续走原路由),
    抛 = "归我管但这一条说不清楚"(调用方口头拒绝本轮). 混作一谈会让一条
    填不出槽的模式指令悄悄走成别的 key.
    """
    entry = _MODE_INTENTS.get(intent_name)
    if entry is None:
        return None
    action, voice_mode = entry
    payload: Dict[str, Any] = {
        "cmd_id": cmd_id,
        "action": action,
        # S7.3 的 source 是[通道]. 语音就是 voice, NO 不从槽位里取 --
        # 与 cmd/geo 的 origin 同理(CH-2), 发起方不得自称是别人.
        "source": source,
    }
    if voice_mode is not None:
        payload["voice_mode"] = voice_mode
    if action == "set_speed_profile":
        profile = slots.get("profile")
        if profile not in PROFILE_VALUES:
            raise ModeRequestError(
                "profile %r is not one of %s (U33 removed cruise and transit)"
                % (profile, sorted(PROFILE_VALUES)))
        payload["profile"] = profile
    if action == "set_behavior":
        behavior = slots.get("behavior")
        if not isinstance(behavior, str) or behavior not in BEHAVIOR_VALUES:
            # 闭集外必抛(S13.6): "跟着他走"听成一个不认识的词时, 宁可让
            # 操作员再说一遍, 也不要把机器人切进一个最"接近"的运动行为.
            raise ModeRequestError(
                "behavior %r is not one of %s"
                % (behavior, sorted(BEHAVIOR_VALUES)))
        payload["behavior"] = behavior
    return payload
