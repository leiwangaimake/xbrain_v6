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
import time
from typing import Any, Callable, Dict, FrozenSet, Optional, Tuple

from xbrain.common.errors import E_BUSY, E_INTERNAL, E_NOT_FOUND, E_SCHEMA
from xbrain.p2_core.mode.state_machine import (
    ModeState,
    ModeStateMachine,
    TransitionRequest,
    TriggerKind,
)
from xbrain.p2_core.mode_actions.dispatch import DispatchResult, dispatch

_logger = logging.getLogger("xbrain.p2.mode")

#: 11 S2.2.3 的两个 key. ack 的订阅者是发起方(p4_agent / HMI / 云端).
CMD_MODE_TOPIC = "cmd/mode"
CMD_MODE_ACK_TOPIC = "cmd/mode/ack"
STATE_MODE_TOPIC = "state/mode"

#: state/mode.exit_reason 的闭集, 逐字取自甲方冻结的 v2.0 S4.3
#: (11 S4.3 v1.2 同步收录, 99 U97). 闭集外的值必抛(CLAUDE.md 3.5).
EXIT_REASONS = ("requested", "timeout", "preempted", "fault", "target_lost",
                "target_left_fence", "manual_cloud", "manual_wecom")

#: ModeCommand.source(11 S7.3 的通道) -> state/mode.source(v2.0 S4.3 的
#: cloud | local | autonomy | system). 微信算远端人员, 归 cloud; HMI 与本地麦
#: 是现场, 归 local. 表外(含缺省)一律 system -- 不猜成某个人.
_SOURCE_TO_V2: Dict[str, str] = {
    "cloud": "cloud", "wecom": "cloud", "hmi": "local", "voice_local": "local",
    "auto_track": "autonomy", "bit": "system",
}


class TransitionAborted(Exception):
    """换态副作用要求[整体拒绝]本次切换(14 S5.7.3 ML-3).

    on_transition 回调在进入新模式的路上失败(例: POST /mode func2 超时)时抛它.
    ModeFace 接住后把模式机回滚到原态, 回 rejected ack, 不发 state/mode --
    旧模式毫发无损, 与 14 S5.5.2 第 4 步失败同一处理路径.
    *** 2026-10-28 之前回调失败只记日志, ack 照样报 accepted, 于是会出现
    "系统说在喊话 / 喇叭其实不响"的半切状态(ML-3 的违反后果原文).
    """

    def __init__(self, code: str, detail: Dict[str, Any]) -> None:
        super().__init__(str(detail.get("reason", code)))
        self.code = code            # 11 S13 闭集码, 由抛出方选定
        self.detail = detail        # 并进 ack.detail, 至少含 reason


def _wall_now() -> float:
    """state/mode.entered_ts 用的墙钟, 单独一处便于测试替换."""
    # WALL-CLOCK-OK(record): v2.0 S4.3 entered_ts 是给人看的进入时刻
    # (显示与审计), 从不参与年龄或超时判定 -- 那些一律单调钟(CLK-C1).
    return time.time()


def exit_reason_for(cmd: Dict[str, Any]) -> str:
    """一条[人发的]退出指令对应的 v2.0 exit_reason.

    * 带 stream_id 的 exit_broadcast 只可能来自甲方 AUDIO_CONTROL(v2.0 S2.5
      的退出协议), 是会话发起方自己收的尾 -> requested.
    * 不带 stream_id: 云端文本 C04 -> manual_cloud, 微信 -> manual_wecom;
      HMI W3 强制退出与本地语音都是现场人员 -> requested.
    """
    if cmd.get("stream_id"):
        return "requested"
    src = cmd.get("source")
    if src == "cloud":
        return "manual_cloud"
    if src == "wecom":
        return "manual_wecom"
    return "requested"

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
                 arb_snapshot: Optional[Callable[
                     [ModeState, ModeState],
                     Tuple[FrozenSet[str], FrozenSet[str]]]] = None,
                 on_transition: Optional[
                     Callable[[ModeState, ModeState], None]] = None,
                 motion_behavior_of: Optional[
                     Callable[[ModeState], str]] = None,
                 ) -> None:
        self._machine = machine or ModeStateMachine()
        self._publish = publish
        # 11 S4.3 motion_behavior 的来源: 模式 -> "hold" / "normal"(99 U99,
        # R-8 (a)). P3 按它让行(15 S4.1A). 由调用方注入是因为映射表在
        # p2_core.yaml mode_motion 段, 本类不读配置; None = 不发该字段
        # (单测与只关心模式机的调用方).
        self._motion_behavior_of = motion_behavior_of
        # 速度档位状态, dispatch 的 SP-C3 三字段从这里取. 未标定值不写默认数
        # (CLAUDE.md 3.1): profile 是"当前档", 由后续 set_speed_profile 落定,
        # 这里只记状态不设安全上限.
        self._profile_state: Dict[str, Any] = {
            "profile": None, "locked": False, "max_profile": None}
        # 仲裁快照来源. 未接线时返回两个空集 -- 空集意味着"没有域被占住", 即
        # 不拦截. 这是本模块唯一的宽松默认, 写在这里是因为它必须被看见: 仲裁器
        # 在 MVP 里不跑, 若改成"取不到快照就拒绝", 模式面会整个不可用.
        self._arb_snapshot = arb_snapshot
        # 换态副作用的挂点(B 模式的收发端要跟着起停). 放回调而不是让
        # ModeFace 自己去碰音频: 本类的边界是"一帧 cmd/mode 进, 一个 ack
        # 出", 把 WS 连接管理塞进来会让它的单测需要一个 payload-service.
        # *** 只在[真的换了态]时调, 见下面的 changed 判断 -- 同态重复下发
        # (甲方连点 start)不该把已经在播的会话重开一次.
        self._on_transition = on_transition
        # 当前 B 模式会话 ID. 由 on_transition 的返回值填, 退出时清回 None.
        # NO 不留上一路的值: 下一次会话若 ack 丢失, state/mode 会拿着旧 ID
        # 继续发, Qt 就把新会话的帧对到旧会话上了.
        self._stream_id: Optional[str] = None
        # 刚结束的那一路 B 会话 ID. 只用于退出指令的幂等判定(v2.0 S2.5:
        # 退出 ack 必须回显原 ID): 会话已因超时退出后, Qt 补发的那条
        # exit_broadcast 带的正是它, 应答 accepted 而不是"查无此会话".
        self._last_stream_id: Optional[str] = None
        # v2.0 S4.3 state/mode 的三个字段(11 S4.3 v1.2, 99 U97). 进程启动即
        # 处于 IDLE, 进入时刻就是现在, 来源是 system.
        self._source = "system"
        self._entered_wall: Optional[float] = _wall_now()
        self._exit_reason: Optional[str] = None

    @property
    def state(self) -> ModeState:
        return self._machine.state

    @property
    def stream_id(self) -> Optional[str]:
        """当前 B 会话 ID(非 B 模式为 None), 供 state/audio 与日志使用."""
        return self._stream_id

    def exit_internal(self, reason: str, *, now_mono_ms: int,
                      trigger: TriggerKind = TriggerKind.TIMEOUT
                      ) -> Dict[str, Any]:
        """P2 自己发起的退出(空闲超时 / 喊话设备故障), 回到 IDLE.

        与人发的 exit_broadcast 走[同一条]换态路径(模式机 + on_transition +
        state/mode), 只是没有发起方, 也不经 dispatch 的形状校验.
        *** 不再合成一帧 {"action": "exit_broadcast"} 去喂 handle_frame:
        那样得到的 exit_reason 只能是"人发的", 云端看不出是超时还是故障
        (v2.0 S4.3 timeout / fault 两个值就是为这个立的), 而 BCT-4 要求
        "触发者是超时而非人工"必须可区分.
        """
        if reason not in EXIT_REASONS:
            raise ValueError("exit_reason %r not in v2.0 S4.3 set" % (reason,))
        return self._transition(
            target=ModeState.IDLE, trigger=trigger, cmd_id="",
            cmd={"source": "system"}, verdict_applied=None,
            exit_reason=reason, now_mono_ms=now_mono_ms)

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
        action = cmd.get("action")
        if action in ("exit_broadcast", "exit_alarm"):
            # 退出指令只退[它自己那个]模式. 原实现把两者都映射到 IDLE, 于是
            # 喊话中收到一条 exit_alarm(例如云端界面上的"解除警戒")也会把
            # 喊话掐掉 -- 退出 B 的路径在 14 S5.6 里是收窄过的, 不该多一条后门.
            own = (ModeState.BROADCAST if action == "exit_broadcast"
                   else ModeState.ALARM)
            refused = self._check_exit(cmd, cmd_id, own)
            if refused is not None:
                return refused
            if self._machine.state != own:
                # 已经不在该模式(例: 超时已自动退出, Qt 随后补发 exit).
                # 幂等应答 accepted, changed=false, 并回显 stream_id
                # (v2.0 S2.5: accepted/duplicate/rejected 都要回显).
                cur = self._machine.state.value
                applied: Dict[str, Any] = {"mode": cur, "from_mode": cur,
                                           "changed": False}
                if action == "exit_broadcast" and cmd.get("stream_id"):
                    applied["stream_id"] = cmd["stream_id"]
                return mode_ack(cmd_id, "accepted", "OK", {"applied": applied})
        return self._transition(
            target=target, trigger=TriggerKind.CMD, cmd_id=cmd_id, cmd=cmd,
            verdict_applied=verdict.applied,
            exit_reason=(exit_reason_for(cmd) if action in
                         ("exit_broadcast", "exit_alarm") else None),
            now_mono_ms=now_mono_ms)

    def _check_exit(self, cmd: Dict[str, Any], cmd_id: str,
                    own: ModeState) -> Optional[Dict[str, Any]]:
        """带 stream_id 的 exit_broadcast 必须指向当前(或刚结束的)那一路.

        v2.0 S2.5 逐字: exit_broadcast 的 stream_id "必须等于要退出的原会话
        stream_id". *** 2026-10-28 之前谁都不核: p5 的注释写"对不上由 p2 拒",
        p2 的 _handle_exit_broadcast 不看这个字段 -- 一条晚到的旧会话退出请求
        会把新开的会话关掉. 对不上 -> E_NOT_FOUND(11 S13.7 "引用的对象不存在"),
        detail 回显请求的 stream_id.
        不带 stream_id 的退出(HMI W3 强制退出 / 云端文本 C04 / 微信)不核:
        它们按定义就是"不管哪一路, 都给我停", 正是 VOI-26 兜底路径的语义.
        """
        sid = cmd.get("stream_id")
        if own != ModeState.BROADCAST or not sid:
            return None
        if self._machine.state == ModeState.BROADCAST:
            if sid == self._stream_id:
                return None
            reason = "stream_id_mismatch"
        elif sid == self._last_stream_id:
            return None                      # 刚结束的那一路, 幂等放行
        else:
            reason = "no_such_session"
        _logger.warning("p2 exit_broadcast refused: %s (got %s, current %s)",
                        reason, sid, self._stream_id)
        return mode_ack(cmd_id, "rejected", E_NOT_FOUND,
                        {"reason": reason, "stream_id": sid,
                         "current_stream_id": self._stream_id})

    def _transition(self, *, target: ModeState, trigger: TriggerKind,
                    cmd_id: str, cmd: Dict[str, Any],
                    verdict_applied: Optional[Dict[str, Any]],
                    exit_reason: Optional[str],
                    now_mono_ms: int) -> Dict[str, Any]:
        """真正换态: 模式机 -> 副作用回调(可整体拒绝) -> 记账 -> state/mode."""
        # dispatch 通过不等于切得动 -- 见模块头注.
        # P-1' on the live arbiters (14 S5.5.3, 99 U99 C1): the snapshot is
        # computed FOR this target from the current mode -- which domains the
        # target needs and which the current mode itself holds (S_self). Until
        # 2026-10-28 it was a no-argument callable nobody wired, so every
        # switch passed the preflight.
        blocked, self_held = (self._arb_snapshot(target, self._machine.state)
                              if self._arb_snapshot
                              else (frozenset(), frozenset()))
        result = self._machine.request(
            TransitionRequest(to_state=target, trigger=trigger,
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
        # *** 回放(重复 cmd_id)不是换态: 模式机回放的是第一次的结果, 其
        # from/to 不同, 原实现据此把副作用再跑一遍 -- 甲方重发一次 start,
        # B 模式的进入动作就再做一次, 分配出第二个 stream_id 而 Qt 手里是第一个.
        changed = (not result.is_replay
                   and result.from_state != result.to_state)
        extra: Dict[str, Any] = {}
        if changed and self._on_transition is not None:
            # 回调可以回一个 dict, 并进 applied. B 模式用它把 stream_id
            # 送出去: 分配方是 p2(会话的持有者), 而 v2.0 S2.5 要求 start 的
            # ack 必须带上它 -- 不并进来的话, p5 拿不到, Qt 也就不知道该往
            # 哪个会话推 PCM.
            # *** 回调抛 TransitionAborted = 整体拒绝(14 ML-3): 模式机回滚到
            # 原态, 回 rejected, 不发 state/mode. 2026-10-28 之前回调失败只记
            # 日志而 ack 照报 accepted, 即 ML-3 原文说的"半切状态".
            try:
                _ret = self._on_transition(result.from_state, result.to_state)
            except TransitionAborted as ab:
                self._machine.rollback(result.from_state, cmd_id=cmd_id,
                                       reason=str(ab.detail.get("reason", "")))
                _logger.warning("p2 mode %s -> %s aborted: %s %s",
                                result.from_state.value, result.to_state.value,
                                ab.code, ab.detail)
                detail = dict(ab.detail)
                detail.update({"from_mode": result.from_state.value,
                               "to_mode": result.to_state.value})
                return mode_ack(cmd_id, "rejected", ab.code, detail)
            if isinstance(_ret, dict):
                extra = _ret
            if result.from_state == ModeState.BROADCAST:
                self._last_stream_id = self._stream_id
            self._record_entry(result.from_state, result.to_state, cmd,
                               exit_reason)
            # *** ack 与 state/mode 要的[不是同一个语义], 不能共用一个值.
            # ack(事件面): 退出时必须[回显]刚结束那一路的 ID --
            #   v2.0 S2.5 逐字"后端不得为退出请求分配新 ID", Qt 靠它确认
            #   自己退的是不是刚才那一路.
            # state/mode(状态面): 退出后必须是 null --
            #   11 S4.3 逐字"broadcast 时必填, 其余模式为 null". 留着旧值的话,
            #   一条 voice_mode=normal 的报文会带着一个早已结束的会话 ID,
            #   Qt 会把下一次会话的帧对到这一路上.
            # => 状态面的值由[结果态]推导, 不抄回调的返回值. 这样它与
            #   voice_mode 不可能打架 -- 两者读的是同一个 result.to_state.
            # *** 2026-09-03 实测抓到: 原实现两面共用, 退出后云端 state/mode
            #   仍带着 audio-gj001-0001. 单测没抓到, 因为测试桩在退出时回的
            #   是 None, 而真回调回的是已结束的 ID -- 桩替被测代码做了事.
            self._stream_id = (extra.get("stream_id")
                               if result.to_state == ModeState.BROADCAST
                               else None)
        applied = {"mode": result.to_state.value,
                   "from_mode": result.from_state.value,
                   "changed": changed}
        if extra:
            applied.update(extra)
        if verdict_applied:
            applied.update(verdict_applied)
        if (cmd.get("action") == "exit_broadcast" and cmd.get("stream_id")
                and "stream_id" not in applied):
            # 退出 ack 回显原 ID(v2.0 S2.5), 回调没给时也补上.
            applied["stream_id"] = cmd["stream_id"]
        # 重复 cmd_id 回放的是第一次的结果(模式机自己保证), 结果里如实标出来,
        # 免得发起方把一次回放当成第二次生效.
        result_word = "duplicate" if result.is_replay else "accepted"
        if not result.is_replay:
            self._publish_state(now_mono_ms=now_mono_ms)
        return mode_ack(cmd_id, result_word, "OK", {"applied": applied})

    def _record_entry(self, from_state: ModeState, to_state: ModeState,
                      cmd: Dict[str, Any], exit_reason: Optional[str]) -> None:
        """换态成功后记 v2.0 S4.3 的 source / entered_ts / exit_reason.

        * 进入 B / D: exit_reason 清为 None(当前模式还没退出).
        * 从 B / D 退出: exit_reason 记退出原因, 一直保留到下一次进入 B / D --
          云端在 normal 态下读到的是"上一次喊话 / 报警是怎么结束的".
        * DIALOG 方言之间的切换不动 exit_reason(v2.0 都是 normal).
        """
        self._source = _SOURCE_TO_V2.get(str(cmd.get("source")), "system")
        self._entered_wall = _wall_now()
        if to_state in (ModeState.BROADCAST, ModeState.ALARM):
            self._exit_reason = None
        elif from_state in (ModeState.BROADCAST, ModeState.ALARM):
            # 进 B / D 以外的态而 exit_reason 未给 = 被别的模式指令顶掉
            # (例: B 中收到 set_voice_mode dialog) -> preempted.
            self._exit_reason = exit_reason or "preempted"

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

    def republish(self, *, now_mono_ms: int) -> None:
        """state/mode 的频率下限(15 S4.1A 逐字"state/mode 到达(事件 + 1 Hz)").

        主循环每秒调一次. *** 2026-10-28 之前只在换态时发: 后起的 P3 / P5
        永远拿不到当前模式, P3 的让行判据(motion_behavior)也就永远等不到
        第一帧 -- 而 p5 的注释还写着"P2 publishes state/mode 10 Hz".
        """
        self._publish_state(now_mono_ms=now_mono_ms)

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
        # stream_id: 11 S4.3(v1.1 新增, 99 U84) -- voice_mode == "broadcast"
        # 时必填, 其余模式为 None. 值由 on_transition 回调在进 B 模式时给出.
        # *** 放在[状态]面而不是只放 ack 里.
        # v2.0 S4.3 要求 state/mode 在 broadcast 时带它(Qt 靠它把 state/audio
        # 的帧对上会话). 只走 cmd/mode/ack 的话, 消费方错过那一条 ack 就再也
        # 拿不到 -- 而 ack 是[事件], 掉一条就没了; 状态面是可重复读的.
        # source / entered_ts / exit_reason: v2.0 S4.3 的三个必填字段
        # (11 S4.3 v1.2 收录, 99 U97). 2026-10-28 之前 p5 读 mode_exit_reason
        # 而全系统没有人写, 云端的 exit_reason 恒为 null -- 人工退出 / 超时 / 
        # 被抢占在 Qt 上看起来一模一样.
        body = {"schema": "state_mode_v1",
                "voice_mode": self._machine.state.value,
                "stream_id": self._stream_id,
                "source": self._source,
                "entered_ts": self._entered_wall,
                "exit_reason": self._exit_reason,
                "mono_ms": now_mono_ms}
        if self._motion_behavior_of is not None:
            # 11 S4.3 motion_behavior: B / D 期间为 mode_motion 配的行为
            # (R-8 (a) 后恒为 hold), 其余为 normal. P3 据此让行 / 恢复
            # (15 S4.1A, 99 U99). 2026-10-28 之前本字段从未发出, P3 也就
            # 从未让行 -- 喊话或报警期间巡逻任务照样标 running.
            body["motion_behavior"] = self._motion_behavior_of(
                self._machine.state)
        self._publish(STATE_MODE_TOPIC,
                      json.dumps(body, ensure_ascii=False).encode("utf-8"))
