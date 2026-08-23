"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: mode_wiring.py
Brief: P2 的 cmd/mode 接收端 -- ModeCommand -> 模式机 -> state/mode + ack (11 S7.3)

Description:
把 p2_core 的模式面接到总线上. 11 S2.2.3 逐字把 `xbrain/{rid}/cmd/mode` 的
发布者列为"云端 / 微信 / HMI / p4_agent", 订阅者列为 `p2_core`, 但在
2026-08-21 之前 p2_core 的 main_wiring 只订了 5 个 state 话题加 speak /
payload / ptz -- 这个 key 根本没有消费者.

*** 后果不是"少一个功能", 是 18 册 C 类 8 条指令全部落空.
C01 enter_alarm / C02 exit_alarm / C03 enter_broadcast / C04 exit_broadcast /
C05 enter_patrol_mode / C06 standby / C07 set_motion_behavior /
C08 query_mode_switch_ok 在 18 册的效果列里逐行写着"P2 -> ...", 而 p4_agent
把它们路由到了 cmd/task, P3 因为帧里没有顶层 action 而 skip. 说"进入喊话模式"
在契约上走不到任何执行者, 且两侧都不报错.

*** 零件本来就是齐的, 缺的只是门.
mode/state_machine.py (ModeStateMachine, 三顶层态 + 三 DIALOG 方言 + cmd_id
幂等) 与 mode_actions/dispatch.py (S7.3 六动作闭集 + SP-C1..C3) 都已实现并有
测试; 本模块不重写它们的任何判断, 只做四件事: 解信封 -> 交给 dispatch 做形状与
闭集校验 -> 让 ModeStateMachine 真正换态 -> 发 cmd/mode/ack 与 state/mode.

Boundaries: 不裁决模式规则(那是 state_machine 的), 不碰仲裁域(blocked/self_held
由调用方从仲裁快照传入), 不读墙钟(CLK-C1: 只收单调毫秒). 发布由调用方注入,
本模块自己不持有 Zenoh session.

*** 一个看起来对但会出错的写法: 把 dispatch() 的 accepted 直接当成"模式已切".
dispatch 只做形状与闭集校验(它自己的注释写着 "full validation lives in
BIZ-P2-11 SM"), 真正的换态在 ModeStateMachine.request 里, 且会因仲裁域被占
(blocked) 或最小驻留时间未到(dwell) 而失败. 两者都通过才是"切了", 只看前者
会让 ack 报 accepted 而模式纹丝不动 -- 正是 SP-C3 说的"accepted 不等于已按你
要的档位跑"的同一类错误.
"""
from __future__ import annotations

import json
import logging
from typing import Any, Callable, Dict, FrozenSet, Optional, Tuple

from xbrain.common.errors import E_BUSY, E_INTERNAL, E_SCHEMA
from xbrain.p2_core.mode.state_machine import (
    ModeState, ModeStateMachine, TransitionRequest, TriggerKind,
)
from xbrain.p2_core.mode_actions.dispatch import DispatchResult, dispatch

_logger = logging.getLogger("xbrain.p2.mode")

#: 11 S2.2.3 的两个 key. ack 的订阅者是发起方(p4_agent / HMI / 云端).
CMD_MODE_TOPIC = "cmd/mode"
CMD_MODE_ACK_TOPIC = "cmd/mode/ack"
STATE_MODE_TOPIC = "state/mode"

#: action -> 目标模式态. 只有会换态的动作在表里.
#:
#: *** set_speed_profile 与 reset_profile_lock 不在表内, 这是刻意的:
#: 11 S7.3.1 (原 D-04) 裁决把这两个动作留在 ModeCommand 里, 但它们改的是速度
#: 档位, 不是语音模式 -- 归 dispatch 的 SP-C1..C3 处理, 换态表不该认领它们.
#: 把它们写进这张表会让"慢一点"把机器人从 BROADCAST 踢回 IDLE.
_ACTION_TO_STATE: Dict[str, ModeState] = {
    "exit_broadcast": ModeState.IDLE,
    "exit_alarm": ModeState.IDLE,
}

#: set_voice_mode 的 voice_mode 值 -> 模式态 (11 S7.3: 切换 A/C/E <-> B <-> D).
#: A/C/E 是 DIALOG 的三种方言(本地麦 / 云端 / 微信), 按来源区分而不是三个模式.
_VOICE_MODE_TO_STATE: Dict[str, ModeState] = {
    "dialog": ModeState.DIALOG_A,
    "dialog_a": ModeState.DIALOG_A,
    "dialog_c": ModeState.DIALOG_C,
    "dialog_e": ModeState.DIALOG_E,
    "broadcast": ModeState.BROADCAST,
    "alarm": ModeState.ALARM,
    "idle": ModeState.IDLE,
}


def parse_mode_envelope(raw: bytes) -> Dict[str, Any]:
    """S3.0 信封 -> ModeCommand 本体, 或抛 ValueError.

    同时接受带信封的 {..., "data": {...}} 与裸本体两种形状 -- 与 P2 已有的
    state sink 同一取舍(见 main_wiring 的 _make_state_sink): 桩发布者与未来
    不打信封的生产者都能工作, 不必为此开第二条代码路径.
    """
    body = json.loads(raw.decode("utf-8"))
    if not isinstance(body, dict):
        raise ValueError("mode command is not an object")
    inner = body.get("data")
    return inner if isinstance(inner, dict) else body


def target_state_for(cmd: Dict[str, Any]) -> Optional[ModeState]:
    """这条 ModeCommand 要切到哪个态; None = 它不换态.

    None 不是错误: set_speed_profile / reset_profile_lock 是合法动作, 只是不
    改模式. 调用方据此跳过 ModeStateMachine 而直接回 ack.
    """
    action = cmd.get("action")
    if action == "set_voice_mode":
        wanted = cmd.get("voice_mode")
        if not isinstance(wanted, str):
            return None
        # 闭集外必抛(S13.6): "未知值降级解释"在模式面上意味着一条不认识的
        # 指令把机器人切到某个"最接近"的模式, 而模式决定了麦克风门控与喊话.
        state = _VOICE_MODE_TO_STATE.get(wanted)
        if state is None:
            raise ValueError("voice_mode %r is not a known mode" % (wanted,))
        return state
    return _ACTION_TO_STATE.get(action)


def mode_ack(cmd_id: str, result: str, code: str = "OK",
             detail: Optional[Dict[str, Any]] = None,
             message: str = "") -> Dict[str, Any]:
    """cmd/mode/ack 本体 (11 S7.7 Ack).

    detail.applied 是 AP-1 对 L1 及以上确认级别的强制要求, 且 AP-2 要求它能被
    独立读懂 -- 所以放进去的是结果状态(切到了哪个态), 不是 "ok": true.
    """
    ack: Dict[str, Any] = {"schema": "mode_ack_v1", "cmd_id": cmd_id,
                           "result": result, "code": code}
    if message:
        ack["message"] = message
    if detail is not None:
        ack["detail"] = detail
    return ack


class ModeFace:
    """P2 的 cmd/mode 接收端: 一帧进, 一个 ack 出, 换态时另发 state/mode.

    持有模式机与速度档位状态. 单线程使用(ModeStateMachine 自述 not thread-safe
    by design), 所以 Zenoh 回调必须把帧交到这里之前完成线程切换.
    """

    def __init__(self, machine: Optional[ModeStateMachine] = None,
                 *, publish: Optional[Callable[[str, bytes], None]] = None,
                 arb_snapshot: Optional[Callable[[], Tuple[FrozenSet[str],
                                                           FrozenSet[str]]]] = None,
                 ) -> None:
        self._machine = machine or ModeStateMachine()
        self._publish = publish
        # 速度档位状态, dispatch 的 SP-C3 三字段从这里取. 未标定值不写默认数
        # (CLAUDE.md 3.1): profile 是"当前档", 由后续 set_speed_profile 落定,
        # 这里只记状态不设安全上限.
        self._profile_state: Dict[str, Any] = {
            "profile": None, "locked": False, "max_profile": None}
        # 仲裁快照来源. 未接线时返回两个空集 -- 空集意味着"没有域被占住", 即
        # 不拦截. 这是本模块唯一的宽松默认, 写在这里是因为它必须被看见: 仲裁器
        # 在 MVP 里不跑, 若改成"取不到快照就拒绝", 模式面会整个不可用.
        self._arb_snapshot = arb_snapshot

    @property
    def state(self) -> ModeState:
        return self._machine.state

    def handle_frame(self, raw: bytes, *, now_mono_ms: int) -> Dict[str, Any]:
        """跑完一帧 cmd/mode, 返回要发布的 ack 本体.

        不抛: 一个畸形帧(来自不鉴权的 HMI, U23)不得掀掉 P2 的循环, 而那个循环
        同时在跑健康度与设备域.
        """
        cmd_id = ""
        try:
            cmd = parse_mode_envelope(raw)
            raw_id = cmd.get("cmd_id")
            cmd_id = raw_id if isinstance(raw_id, str) else ""
            return self._apply(cmd, cmd_id, now_mono_ms=now_mono_ms)
        except ValueError as exc:
            _logger.warning("p2 cmd/mode refused: %s", exc)
            return mode_ack(cmd_id, "rejected", E_SCHEMA, {"reason": str(exc)})
        except Exception as exc:              # noqa: BLE001
            _logger.error("p2 cmd/mode failed: %s", exc)
            return mode_ack(cmd_id, "error", E_INTERNAL, {"reason": str(exc)})

    def _apply(self, cmd: Dict[str, Any], cmd_id: str, *,
               now_mono_ms: int) -> Dict[str, Any]:
        # 第一关: 形状与闭集(S7.3 六动作 + SP-C1/SP-C2), 已实现的分派器.
        verdict: DispatchResult = dispatch(cmd, self._profile_state)
        if not verdict.accepted:
            return mode_ack(cmd_id, "rejected", verdict.code,
                            {"reason": verdict.reason})
        target = target_state_for(cmd)
        if target is None:
            # 合法但不换态(速度档位两动作). applied 直接用分派器算出的三字段.
            self._remember_profile(cmd, verdict)
            return mode_ack(cmd_id, "accepted", "OK", {"applied": verdict.applied})
        # 第二关: 真正换态. dispatch 通过不等于切得动 -- 见模块头注.
        blocked, self_held = (self._arb_snapshot() if self._arb_snapshot
                              else (frozenset(), frozenset()))
        result = self._machine.request(
            TransitionRequest(to_state=target, trigger=TriggerKind.CMD,
                              cmd_id=cmd_id),
            blocked_domains=blocked, self_held_domains=self_held)
        if not result.accepted:
            # 码与键名逐字取自 14 S12.1 的 cmd/mode/ack 拒绝样例: E_BUSY +
            # from_mode / to_mode.
            # *** 不是 E_MODE_BLOCKED -- 那个码我一度写了出来, 而 11 S13 的
            # 40 值闭集里没有它(CLAUDE.md 3.5 禁自造码).
            #
            # blocked 与 self_held 都回抄: 14 S5.5 P-1' 与 S12.1 场景 3 都要求
            # 一次报全不短路 -- 两个域被占就说两个, 否则操作员修好一个还是切
            # 不过去, 要来回试.
            # ** blocked[] 的元素是[对象]不是字符串 -- 14 S12.1 的样例逐字是
            # {"domain": "ptz", "holder": "manual_cloud", ...}. 发字符串数组会
            # 让照契约写的消费方 b["domain"] 直接 TypeError, 而这正是本项目
            # 反复出血的那一类"上线帧形状分叉".
            # * 只填 domain 一个键: holder / holder_priority / required_source
            # 要仲裁器快照才有, MVP 里仲裁器不跑. 缺字段是"未知", 编一个
            # holder 出来是"错误" -- 操作员会照着那个名字去找一个并不存在的
            # 占用者. 补全等仲裁器接线, 已记 NEXT.
            # * self_held 反之[就是]字符串数组(同一样例: ["payload_light"]),
            # 两者形状不同是契约本来的样子, 不是这里写歪了.
            return mode_ack(
                cmd_id, "rejected", E_BUSY,
                {"reason": result.reason or "transition_refused",
                 "from_mode": result.from_state.value,
                 "to_mode": result.to_state.value,
                 "blocked": [{"domain": d} for d in sorted(result.blocked)],
                 "self_held": sorted(result.self_held)})
        applied = {"mode": result.to_state.value,
                   "from_mode": result.from_state.value,
                   "changed": result.from_state != result.to_state}
        if verdict.applied:
            applied.update(verdict.applied)
        # 重复 cmd_id 回放的是第一次的结果(模式机自己保证), 结果里如实标出来,
        # 免得发起方把一次回放当成第二次生效.
        result_word = "duplicate" if result.is_replay else "accepted"
        if not result.is_replay:
            self._publish_state(now_mono_ms=now_mono_ms)
        return mode_ack(cmd_id, result_word, "OK", {"applied": applied})

    def _remember_profile(self, cmd: Dict[str, Any],
                          verdict: DispatchResult) -> None:
        """把速度档位动作的结果记回本地状态, 供下一次 SP-C3 三字段取用. """
        applied = verdict.applied or {}
        if "profile_to" in applied:
            self._profile_state["profile"] = applied["profile_to"]
        if "profile_locked" in applied:
            self._profile_state["locked"] = bool(applied["profile_locked"])
        if "max_profile" in applied:
            self._profile_state["max_profile"] = applied["max_profile"]

    def _publish_state(self, *, now_mono_ms: int) -> None:
        """发 state/mode. P2 是这个 key 的唯一发布者(11 S2.2.3).

        只在真的换了态时发 -- 加上 P2 心跳里的周期重发, 就是 S7.10 那套"变更
        即发 + 一个频率下限". 这里不设周期, 周期由调用方的心跳负责.
        """
        if self._publish is None:
            return
        # ** 字段名是 voice_mode, 不是 mode -- 11 S4.3 ModeState 的第一个字段
        # 就是它. P5 的 reader 原本读 d["mode"], 那是在[没有任何 P2 发布者]
        # 的年代两边就着一个不是契约的形状对上的; 现在有真发布者了, 按 S4.3
        # 对齐并同步改了 P5 那一侧.
        # * S4.3 的 ModeState 还有 active_session / motion_behavior /
        # payload{} / ptz{} -- 那些量归 P2 的其他域(payload_wiring /
        # ptz_wiring)所有, 本模块拿不到, 所以[不发]而不是发空壳. 少一个字段
        # 消费方能看出"没有", 发一个假的 payload{} 会让 HMI 显示一个从没发生
        # 过的载荷状态(S7.5 的部分更新语义正是靠这个字段可验证).
        # * 未发 switch_id: 14 S12.1 的样例里有, 但 14 S13 CR-2 逐字记着
        # switching / switch_id "在 11 S4.3 ModeState 中尚不存在", 且当前无任何
        # 消费方读它 -- CLAUDE.md 9.3 禁为将来留口子.
        body = {"schema": "state_mode_v1",
                "voice_mode": self._machine.state.value,
                "mono_ms": now_mono_ms}
        self._publish(STATE_MODE_TOPIC,
                      json.dumps(body, ensure_ascii=False).encode("utf-8"))
