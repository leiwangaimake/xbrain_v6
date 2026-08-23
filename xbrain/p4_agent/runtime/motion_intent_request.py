"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: motion_intent_request.py
Brief: 18 A 类运动原语 -> 11 S9.3.2A.3 MotionIntent (p4_agent -> p2_core)

Description:
把"前进三米" / "左转九十度"翻成契约的 MotionIntent. 11 S2.2.3 把 p4_agent 列为
cmd/motion/intent 的[唯一]发布者, p2_core 为订阅者(G-1~G-11 十一道闸门).

*** 2026-08-21 之前 P4 的路由指对了 key, 却发的是自己的 p4_intent_v1 信封 --
既没有 data 包装, 也没有 intent / slots / auth_level / turn_id. 加上 P2 当时
根本没订这个 key, A 类 14 条两头都不通.

*** MI-1: P4 不做任何换算.
data.intent 与 data.slots 必须与 state/voice_turn 的同名字段[逐字一致]
(byte-identical): 轴, 符号, 单位, 限幅, 缺省值一律不碰. 这让"模型输出被谁改过"
成为一个用两条 key 直接比对就能查的事实, 而不用去读 P4 的代码.
=> 本模块只做[搬运与打包], 换算全在 P2(S9.3.2A.4 的轴表).

*** auth_level 恒 "L1", 不是注册表里的 L1a / L1b.
18 S13.1 定 A05-A12 恒 L1; 注册表的 L1a/L1b 是更细的确认级别, 用来决定要不要
口头复述, 与契约字段不是一个量. 透传 L1b 会被 P2 的 G-2 判成 P4 缺陷并落
event/warn/motion -- 看起来只是个字符串, 实则整条指令被拒.

*** MI-5: 语音旁路三条不走本 key.
A01 estop 直达 cmd/estop, A02/A03 走 cmd/chassis/ctrl. 旁路 LLM != 旁路仲裁.
它们在 turn_orchestrator 里更早就被 safety_bypass 截走, 根本到不了这里; 本表
不含它们是第二道保证.

Boundaries: 不换算, 不填运动参数(MO-2: 一律 P2 填), 不判闸门(那是 P2 的
G-1~G-11), 不读钟.
"""
from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

#: S9.3.2A.3 字段表: A05-A12 恒 L1.
_AUTH_LEVEL = "L1"

#: S9.3.2A.4 八值闭集 -> 必填槽位名(None = 无槽位).
#: 键是注册表的 intent NAME, 与契约的 intent 取值逐字相同 -- 这不是巧合,
#: S9.3.2A.3 要求它与 16 S6.7 M1/M2 的 intent 取值逐字一致.
_MOTION_INTENTS: Dict[str, Optional[str]] = {
    "move_forward": "distance_m",
    "move_backward": "distance_m",
    "move_left": "distance_m",
    "move_right": "distance_m",
    "turn_left": "angle_deg",
    "turn_right": "angle_deg",
    "turn_around": None,          # slots = {} 是合法的
    "face_heading": "heading",
}

#: MO-2 的 channel 闭集. P4 只会是语音/文本两种来源之一, 但闭集写全, 因为
#: P2 据此填 relative_move.source.
_SOURCE_TO_CHANNEL = {
    "voice": "mic_local",
    "text": "hmi",
    "cloud": "cloud",
    "wecom": "wecom",
}


class MotionIntentError(RuntimeError):
    """这条运动意图翻不出合法的 MotionIntent. 口头追问, 不发帧. """


def is_motion_intent(intent_name: str) -> bool:
    return intent_name in _MOTION_INTENTS


def to_motion_intent(intent_name: str, *, slots: Mapping[str, Any],
                     cmd_id: str, turn_id: str, source: str,
                     asr_text: str = "") -> Optional[Dict[str, Any]]:
    """A 类意图 -> MotionIntent 信封; 不是运动原语则 None.

    抛 MotionIntentError 表示"归我管但槽位不全".
    *** MI-2: 槽位缺失时[一律不发本 key], 直接 TTS 追问 -- 16 S6.7 的全部
    拒绝分支止步于 P4, 不得下探到 P2. 所以这里抛而不是发一条让 P2 去拒的帧.
    """
    if intent_name not in _MOTION_INTENTS:
        return None
    slot_name = _MOTION_INTENTS[intent_name]
    out_slots: Dict[str, Any] = {}
    if slot_name is not None:
        value = slots.get(slot_name)
        if value is None:
            raise MotionIntentError("missing_slot:%s" % (slot_name,))
        # NO 不做任何换算与限幅(MI-1). 值原样搬过去 -- 越界由 P2 的 G-2/G-3
        # 判, 在这里"顺手" clamp 一下会让 state/voice_turn 与本帧对不上,
        # 而那正是审计比对的依据.
        out_slots[slot_name] = value
    data: Dict[str, Any] = {
        "cmd_id": cmd_id,
        # 唯一的审计缝合点: 没有它, "模型说了什么 -> 机器人做了什么"这条链
        # 断在 P4 出口.
        "turn_id": turn_id,
        "channel": _SOURCE_TO_CHANNEL.get(source, "mic_local"),
        "auth_level": _AUTH_LEVEL,
        "intent": intent_name,
        "slots": out_slots,
    }
    if asr_text:
        # 仅供 P2 审计与 HMI 显示. P2 侧禁止它参与任何判定(字段表逐字),
        # 一旦有人拿它做正则兜底, GBNF 的闭集约束就被绕过了.
        data["asr_text"] = asr_text
    # S3.0 外层信封. P2 的 G-1 对信封做完整校验且不豁免, 所以这里必须包,
    # 不能像 cmd/mode 那样发裸本体.
    return {"v": 1, "src": "p4_agent", "data": data}
