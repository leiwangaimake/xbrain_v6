"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: motion_intent_wiring.py
Brief: P2 的 cmd/motion/intent 接收端 -- G-1..G-11 十一道闸门 (11 S9.3.2A)

Description:
A 类语音运动原语("前进三米" / "左转九十度")的落点. 11 S2.2.3 把 cmd/motion/intent
的发布者列为 p4_agent(唯一), 订阅者 p2_core(G-1~G-11 十一道闸门); S7.3A.1 的裁决
写死了这条路线 P4 -> P2 -> P1, 并给了四条否决走 P3 的理由.

*** 2026-08-21 之前 p2_core 从未订过这个 key.
p2_subscriber.py 里那句订阅是[示例模块], 真跑的 main_wiring 只订 5 个 state
话题加 speak/payload/ptz. 于是 A 类 14 条意图发出去无人接收, 说"前进三米"在
契约上走不到任何执行者, 两侧都不报错.

*** 本模块[只做闸门与换算], 不执行运动.
通过全部闸门后转发一条 cmd/motion/relative_move 给 P1(MO-1: 换新 rm- cmd_id),
真正的速度门 / 围栏裁剪 / RNS 全在 P1, 一字不改.

*** G-10 是本节最容易写错的一条, 契约自己点了名.
S7A.6.3 的 re-arm 触发表逐字写着"新的 cmd/motion/relative_move"-- 也就是说
[P2 转发这个动作本身就是解除软急停缴械的那把钥匙]. 若看到
suspended == "soft_estop" 就先回 E_ARB_DISARMED, U35 的现场表现("喊急停 ->
停住 -> 再喊前进两米 -> 立刻走")会被堵死在 P2, 而 E_ARB_DISARMED 给客户端的
指引"不要重试, 等一条真正的新运动指令"将永远等不到那条指令.
=> 真锁(G-9)预拒, 缴械(G-10)放行.

*** asr_text 禁止参与判定(MI-1 字段表逐字).
一旦拿它做正则兜底, GBNF 的闭集约束就被绕过了 -- 那个字段只供审计与 HMI 显示.

Boundaries: 不填运动参数以外的东西, 不读墙钟(CLK-C1), 不持 Zenoh session
(发布由调用方注入), 不判速度门(P1 的).
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Tuple

from xbrain.common.errors import (
    E_BUSY, E_CAPABILITY, E_LOCKED, E_NO_HEADING, E_SCHEMA, E_UNHEALTHY,
)

_logger = logging.getLogger("xbrain.p2.motion_intent")

CMD_MOTION_INTENT_TOPIC = "cmd/motion/intent"
CMD_MOTION_INTENT_ACK_TOPIC = "cmd/motion/intent/ack"
CMD_RELATIVE_MOVE_TOPIC = "cmd/motion/relative_move"

#: S9.3.2A.4 八值闭集 -> (轴, 符号). 轴换算全在 P2(MI-1: P4 一律不换算).
_INTENT_AXIS: Dict[str, Tuple[str, float]] = {
    "move_forward": ("dx_m", +1.0),
    "move_backward": ("dx_m", -1.0),
    "move_left": ("dy_m", +1.0),
    "move_right": ("dy_m", -1.0),
    "turn_left": ("dyaw_rad", +1.0),
    "turn_right": ("dyaw_rad", -1.0),
    # A11 turn_around: 固定取正(逆时针). +-pi 路程等长, 取定值是为了[可重放]--
    # 同一句话两次执行必须转同一个方向, 否则事故复盘还原不出来.
    "turn_around": ("dyaw_rad", +1.0),
    # A12 face_heading 不走轴换算, 见 MO-3 与 _target_yaw.
    "face_heading": ("target_yaw_rad", +1.0),
}

#: 需要 distance_m 的四个; 需要 angle_deg 的两个. turn_around 无槽位,
#: face_heading 要 heading.
_NEEDS_DISTANCE = frozenset({"move_forward", "move_backward",
                             "move_left", "move_right"})
_NEEDS_ANGLE = frozenset({"turn_left", "turn_right"})
#: G-6: 会改变航向或横移的意图, 都要求 yaw_capable.
_NEEDS_YAW = frozenset({"turn_left", "turn_right", "turn_around",
                        "face_heading", "move_left", "move_right"})
#: G-7: 只有全向底盘能横着走.
_NEEDS_HOLONOMIC = frozenset({"move_left", "move_right"})

#: S9.3.2A.4 八方位 -> ENU 绝对航向(S10.1 REP-105: 正东为 0, 逆时针为正).
_HEADING_RAD: Dict[str, float] = {
    "east": 0.0,
    "northeast": math.pi / 4,
    "north": math.pi / 2,
    "northwest": 3 * math.pi / 4,
    "west": math.pi,
    "southwest": -3 * math.pi / 4,
    "south": -math.pi / 2,
    "southeast": -math.pi / 4,
}

#: MO-2 channel -> relative_move.source.
_CHANNEL_TO_SOURCE = {
    "mic_local": "voice_local",
    "cloud": "voice_cloud",
    "wecom": "voice_wecom",
    "hmi": "text",
}

#: G-8 录制态的三个值(U45).
_TEACH_BUSY_STATES = frozenset({"recording", "pending_name",
                                "pending_overwrite"})


@dataclass(frozen=True)
class MotionLimits:
    """G-3 量程上限.

    *** 没有默认值, 构造期必须注入(CLAUDE.md 3.1).
    一个在代码里带默认值的上限, 是没人看得见也改不动的上限; 而这两个数决定
    "一句话最多能让机器人走多远 / 转多少". 来源 configs/p2_core.yaml 的
    rel_move 段(12 S4.5.5 的 20 m 可配值 + 11 G-3 的 720 度待评审值).
    """
    max_distance_m: float
    max_angle_deg: float


@dataclass(frozen=True)
class GateVerdict:
    """闸门结论. passed=True 时 code/detail 为空. """
    passed: bool
    code: str = ""
    detail: Optional[Dict[str, Any]] = None
    gate: str = ""              # 命中的闸门号, 供日志与事件


def _fail(gate: str, code: str, detail: Dict[str, Any]) -> GateVerdict:
    return GateVerdict(passed=False, code=code, detail=detail, gate=gate)


def parse_intent_envelope(body: Any) -> Dict[str, Any]:
    """S3.0 信封 -> MotionIntent 本体, 或抛 ValueError (G-1).

    *** 信封[完整校验不豁免]: 相对位移是放松型指令, 解析失败按"放行"兜底
    是危险的(同 S9.3.4 E-3). 所以拿不到 data 就抛, NO 不退化成裸本体 --
    这一点与 cmd/mode 不同, 那边接受裸本体是为了让桩发布者能工作, 而这里
    放进来的是会让机器人移动的指令.
    """
    if not isinstance(body, dict):
        raise ValueError("motion intent is not an object")
    data = body.get("data")
    if not isinstance(data, dict):
        raise ValueError("motion intent envelope has no data object")
    return data


def evaluate(cmd: Mapping[str, Any], *, limits: MotionLimits,
             clock: Optional[Mapping[str, Any]] = None,
             health: Optional[Mapping[str, Any]] = None,
             pose: Optional[Mapping[str, Any]] = None,
             robot: Optional[Mapping[str, Any]] = None,
             teach: Optional[Mapping[str, Any]] = None,
             holonomic: Optional[bool] = None) -> GateVerdict:
    """跑 G-2..G-11, 按序先命中先返回 (G-1 在 parse 里).

    每个状态源都可能是 None(该源还没发过话). None 的处理[逐门不同], 见各门
    内的注释 -- 统一按"没收到就放行"或"没收到就拒绝"处理都是错的.
    """
    # -- G-2 闭集与必填 --------------------------------------------------
    intent = cmd.get("intent")
    if intent not in _INTENT_AXIS:
        return _fail("G-2", E_SCHEMA, {"field": "intent"})
    if cmd.get("auth_level") != "L1":
        # A05-A12 恒 L1(18 S13.1). 出现别的值即 P4 缺陷 -- 调用方要同时落
        # event/warn/motion, 这里只给出结论.
        return _fail("G-2", E_SCHEMA, {"field": "auth_level"})
    slots = cmd.get("slots")
    if not isinstance(slots, dict):
        return _fail("G-2", E_SCHEMA, {"field": "slots"})
    if intent in _NEEDS_DISTANCE:
        d = slots.get("distance_m")
        if not _positive_number(d):
            # <= 0 也是 E_SCHEMA: 方向由 intent 决定不进 slots, 所以负数不是
            # "反方向"而是 P4 缺陷.
            return _fail("G-2", E_SCHEMA, {"field": "slots.distance_m"})
    if intent in _NEEDS_ANGLE:
        if not _positive_number(slots.get("angle_deg")):
            return _fail("G-2", E_SCHEMA, {"field": "slots.angle_deg"})
    if intent == "face_heading":
        if slots.get("heading") not in _HEADING_RAD:
            return _fail("G-2", E_SCHEMA, {"field": "slots.heading"})
    # -- G-3 量程 --------------------------------------------------------
    if intent in _NEEDS_DISTANCE:
        dist = float(slots["distance_m"])
        if dist > limits.max_distance_m:
            # detail 必须同时带 field 与 limit(12 S4.5.5 的样例逐字如此) --
            # 只说"超限"而不说上限, 操作员不知道该改成多少.
            return _fail("G-3", E_SCHEMA,
                         {"field": "slots.distance_m",
                          "limit": limits.max_distance_m})
    if intent in _NEEDS_ANGLE:
        ang = float(slots["angle_deg"])
        if ang > limits.max_angle_deg:
            return _fail("G-3", E_SCHEMA,
                         {"field": "slots.angle_deg",
                          "limit": limits.max_angle_deg})
    # -- G-4 时钟 --------------------------------------------------------
    # S1.5.5 把"相对位移"明列在时钟未同步时禁止的动作里.
    # None(还没收到 state/clock) 视为[未同步]: 这一门的失效方向是"时钟没同步
    # 却放行", 后果是位移与录包时间对不上且跨机对齐失效, 所以缺省取保守侧.
    if not (clock or {}).get("ts_sync"):
        return _fail("G-4", E_UNHEALTHY, {"item": "clock"})
    # -- G-5 健康度 ------------------------------------------------------
    if health is not None and not health.get("allow_motion", False):
        # detail.item 要点名[具体失败项](S5.1 闭集), 不说"机器人忙" --
        # 操作员据此才知道去修哪个.
        return _fail("G-5", E_UNHEALTHY,
                     {"item": _first_failing_item(health)})
    # -- G-6 航向 --------------------------------------------------------
    if intent in _NEEDS_YAW:
        if pose is not None and not pose.get("yaw_capable", False):
            # ** 整条拒绝, NO 不做"只执行平移部分"的部分降级(G-6 逐字).
            return _fail("G-6", E_NO_HEADING, {})
    # -- G-7 本体能力 ----------------------------------------------------
    if intent in _NEEDS_HOLONOMIC and holonomic is not True:
        # holonomic 是本体规格. None(配置未落值) 与 False 同样拒绝: 一台可能
        # 不能横着走的车, 不该因为"没查到规格"就试着横着走.
        return _fail("G-7", E_CAPABILITY, {})
    # -- G-8 录制态 (U45) ------------------------------------------------
    if (teach or {}).get("state") in _TEACH_BUSY_STATES:
        # 这道门必须在 P2 拦: 12 S4.7.2 只是把 relative_move 源[抑制],
        # 指令仍会 ACCEPTED -> RUNNING 然后空转到 T-12 的 20 s 超时才报
        # aborted -- 用户要等 20 秒才知道机器人没动.
        return _fail("G-8", E_BUSY, {"reason": "teach_session_active"})
    # -- G-9 真锁 --------------------------------------------------------
    r = robot or {}
    if r.get("hes_lock") is True:
        return _fail("G-9", E_LOCKED, {"reason": "hes_engaged"})
    if r.get("timeout_lock") is True:
        return _fail("G-9", E_LOCKED, {"reason": "cmd_timeout"})
    # -- G-10 软急停缴械: 不拦, 照常转发 ---------------------------------
    # 这里[故意没有代码]. 转发本身就是 re-arm 的钥匙(见模块头注). 写成一道
    # 检查是这一节最容易犯的错, 契约自己点了名.
    # -- G-11 定位降级: 不拦 ---------------------------------------------
    # fix_type == "rtk_float" 时 G-11 标"待评审", 建议不拦: S3.2.1 的
    # E_DEGRADED 限定"不接受新任务", 而本节裁决相对位移[不是任务].
    return GateVerdict(passed=True)


def _positive_number(v: Any) -> bool:
    """正数判定. bool 显式排除: True 是 int, 会被当成 1.0 米走出去. """
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0


#: 11 S5.1 items[].state 闭集里"这一项没在正常工作"的三个值.
#: *** 词表来自 HealthState 枚举, 不是我凭印象写的 -- 第一版写成
#: level in ("fault", "unknown"), 而 "fault" 这个词在全库不存在, level 的取值
#: 是 fatal/degraded/warn, "unknown" 属于 state 不属于 level. 于是提取器恒返回
#: None, ORIN 上真实报文里 8 项全 fatal 却报 item: null -- 一条"永远说不出是
#: 哪一项"的诚实回答, 比编造好, 但同样没用.
_ITEM_NOT_OK = frozenset({"fail", "unknown", "degraded"})


def _first_failing_item(health: Mapping[str, Any]) -> Optional[str]:
    """点名一个真正失败的健康项; 找不到就返回 None 而不是编一个.

    None 会让 ack 里 item 为 null -- 那是"我也不知道是哪一项"的诚实说法,
    比报一个其实是好的项要好: 操作员会照着去修一个没坏的东西.

    *** 优先报 level == fatal 的那一项. 停机时通常好几项同时不 ok(没接底盘
    就是 battery/chassis/cam_rgbd 一起 unknown), 报哪一个都对但只有 fatal
    那一项是"修了才能动"的. 报一个 warn 项会让操作员修完发现还是走不了.
    """
    items = health.get("items")
    if not isinstance(items, dict):
        return None
    fallback = None
    for name, body in items.items():
        if not isinstance(body, dict):
            continue
        if body.get("state") not in _ITEM_NOT_OK:
            continue
        if body.get("level") == "fatal":
            return name
        if fallback is None:
            fallback = name
    return fallback


def to_relative_move(cmd: Mapping[str, Any], *, rm_cmd_id: str,
                     params: Mapping[str, Any]) -> Dict[str, Any]:
    """通过闸门的意图 -> cmd/motion/relative_move 本体.

    rm_cmd_id 由调用方生成 (MO-1: 必须换新 id, NO 不得复用 mi- 那个 --
    两条 key 各自按 S2.3 去重, 共用 id 会让"P4 重发的意图"与"P2 重发的位移"
    互相误判为重复).
    params 是 MO-2 的运动参数, 一律 P2 按配置填, NO 不接受 P4 提供的.
    """
    intent = cmd["intent"]
    slots = cmd.get("slots") or {}
    axis, sign = _INTENT_AXIS[intent]
    body: Dict[str, Any] = {
        "cmd_id": rm_cmd_id,
        "dx_m": 0.0, "dy_m": 0.0, "dyaw_rad": 0.0,
        "source": _CHANNEL_TO_SOURCE.get(cmd.get("channel"), "voice_local"),
        # 回指: 没有 turn_id, state/voice_turn 与本条指令事后对不上,
        # "模型说了什么 -> 机器人做了什么"这条链就断在 P4 出口.
        "turn_id": cmd.get("turn_id"),
    }
    if intent == "face_heading":
        # MO-3: 置 target_yaw_rad 并把 dyaw_rad 置 0, 二者互斥.
        # NO 绝不在这里折算成 dyaw_rad -- 绝对航向要减[当前]yaw, 而 P2 手里
        # 只有 10 Hz state/pose 的快照, 语音链路 0.5-2 s, 机器人若在动, 折算
        # 出来的相对量一定是过期的, 转完必然偏. 折算权留在 P1.
        body["target_yaw_rad"] = _HEADING_RAD[slots["heading"]]
    elif intent == "turn_around":
        body["dyaw_rad"] = math.pi * sign
    elif axis == "dyaw_rad":
        body["dyaw_rad"] = math.radians(float(slots["angle_deg"])) * sign
    else:
        body[axis] = float(slots["distance_m"]) * sign
    body.update(params)
    return body


def motion_intent_ack(cmd_id: str, result: str, code: str = "OK",
                      detail: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """cmd/motion/intent/ack 本体 (S7.7 Ack).

    转发成功时 detail.rm_cmd_id 必须回传 P2 新生成的 id (MO-1) -- P4 靠它把
    后续的 relative_move/status 与本轮对话接上.
    """
    ack: Dict[str, Any] = {"schema": "motion_intent_ack_v1", "cmd_id": cmd_id,
                           "result": result, "code": code}
    if detail is not None:
        ack["detail"] = detail
    return ack
