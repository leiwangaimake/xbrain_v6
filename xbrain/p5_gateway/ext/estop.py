"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: estop.py
Brief: CHK-0-40 甲方独立急停 Q0 直翻 (cmd/estop -> chassis_relay, <=100ms)

Description:
`xbrain/{rid}/cmd/estop` MUST bypass the normal task pipeline entirely
(R3.3 in docs/MISSON/任务枚举_qt端v2.0.md). It goes:

  Zenoh Q0 receive
    -> validate envelope (in this module)
    -> chassis_relay estop channel (direct forward, no arbiter, no
       queue, no ack from p3)
    -> emit cmd/estop/ack with estop_epoch/applied/recv_mono_ms/
       latency_ms/hes/timeout_lock within 100 ms

Latency budget: robot receives cmd/estop -> ack forwarded within
100 ms; Qt click -> ack landed within 300 ms end-to-end. Both are
measured using RECEIVER MONOTONIC CLOCK (never ts_utc). Any code
that reads ts_utc here to compute latency is a defect.

state/link.estop_path is an INDEPENDENT ok/degraded/down field.
It is not derivable from normal-plane liveness; the two are
different Zenoh sessions on different planes:
  * cmd/estop:      通用面 tcp/<ip>:7447 Q0 dedicated queue
  * state/link:     普通面 same session but different key
The plane-independence discipline is why estop-path down does not
imply cmd/task down and vice versa.

Down debounce: state/link.estop_path flips to 'down' after 3
consecutive missed heartbeats (contract 3 s wall time = ~3
one-second beats). Ack MUST NOT be treated as heartbeat: a single
successful ack does not clear a down mark that came from the
liveness beat, it only advances the last-ack-received timestamp.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

import logging

from xbrain.common.errors import E_SCHEMA

_logger = logging.getLogger("xbrain.p5.estop")


ESTOP_ACTIONS = frozenset({"stop"})


ESTOP_MAX_FORWARD_MS = 100
ESTOP_MAX_E2E_MS = 300


class EstopSchemaError(Exception):
    """Envelope defective; cannot be forwarded."""


@dataclass(frozen=True)
class EstopFrame:
    """Validated, ready to hand to chassis_relay directly."""
    rid: str
    action: str
    reason: str
    ts_utc_sec: float
    seq: int
    src: str
    msg_id: str
    task_id: str


@dataclass(frozen=True)
class EstopAck:
    """cmd/estop/ack payload. All time fields are receiver
    monotonic ms (NEVER ts_utc)."""
    rid: str
    estop_epoch: int
    #: v2.0 S2.3 逐字 "applied 必须为字符串数组" -- 列的是[实际采取]的措施
    #: (样例 ["zero_vel","charge_abort"]), NO 不是一个 bool.
    #: 空数组 = 一项都还没确认生效, 与"确认了但什么都没做"不同; 前者是
    #: 我们现在的真实状态(确认通道 CR-12 cmd/chassis/ctrl/ack 未建).
    applied: Tuple[str, ...]
    recv_mono_ms: int
    latency_ms: int
    hes: str
    timeout_lock: bool

    def to_detail(self) -> Dict[str, Any]:
        """v2.0 S2.3: 这七项在 ack 的 detail 里, 不在顶层."""
        return {
            "result": "accepted",
            "estop_epoch": self.estop_epoch,
            "applied": list(self.applied),
            "recv_mono_ms": self.recv_mono_ms,
            "latency_ms": self.latency_ms,
            "hes": self.hes,
            "timeout_lock": self.timeout_lock,
        }


def validate_and_forward(msg: dict, key_second_segment: str) -> EstopFrame:
    """Fast validate a cmd/estop envelope. Any defect raises
    EstopSchemaError -- the caller must NOT queue or retry; the safe
    action on a defective estop is to LOG and immediately forward a
    'safety-side' HES engaged signal to the chassis to preserve
    fail-safe semantics."""
    if not isinstance(msg, dict):
        raise EstopSchemaError("estop envelope not object")
    rid = msg.get("rid")
    if not isinstance(rid, str) or rid != key_second_segment:
        raise EstopSchemaError(
            f"estop rid mismatch: {rid!r} vs {key_second_segment!r}")
    data = msg.get("data") or {}
    if not isinstance(data, dict):
        raise EstopSchemaError("estop data not object")
    # *** action/reason 在 data.payload 里, NO 不在 data 顶层.
    # v2.0 S2.3 的信封是 {v,rid,ts,seq,src,data:{msg_id,task_id,task_type,
    # payload:{action,reason}}} -- 与普通任务同一层次结构.
    # 本函数原先读 data["action"], 那个拼法对着一个想象的形状: 真报文里
    # 它恒为 None => 每一条合规的急停都会被判 "action not in closed set"
    # 而拒掉. 2026-09-04 终测接线时对着甲方实发的报文才发现 -- 本模块
    # 此前零调用, 所以这个错一直没有机会暴露.
    payload = data.get("payload")
    if not isinstance(payload, dict):
        raise EstopSchemaError("estop payload not object")
    action = payload.get("action")
    if action not in ESTOP_ACTIONS:
        raise EstopSchemaError(
            f"estop action not in closed set: got {action!r}")
    return EstopFrame(
        rid=rid,
        action=action,
        reason=str(payload.get("reason") or ""),
        ts_utc_sec=float(msg.get("ts") or 0.0),
        seq=int(msg.get("seq") or 0),
        src=str(msg.get("src") or ""),
        # msg_id / task_id 同样在 data 里, 不在信封顶层.
        msg_id=str(data.get("msg_id") or ""),
        task_id=str(data.get("task_id") or ""),
    )


def build_ack(frame: EstopFrame,
                recv_mono_ms: int,
                sent_mono_ms: int,
                estop_epoch: int,
                applied: Sequence[str],
                hes: str,
                timeout_lock: bool) -> EstopAck:
    """Assemble the ack. latency_ms uses monotonic diff (R3.3:
    'the judgement uses the robot monotonic-clock field, NOT ts
    subtraction across both ends')."""
    # v2.0 S2.3 只给了样例值 "ok", 没给闭集; 本模块原有 engaged/cleared/
    # unknown 三值. 两边并集在这里放行, 待甲方给出闭集后收窄.
    # *** 本项目当前只能报 unknown: 11 S538/S756 逐字 "硬件急停 HES 完全
    # 不经软件, 软件不可解除" -- 它的状态要由 chassis_relay 报上来, 而那个
    # 进程未编译. 报 "ok" 就是断言一个我们读不到的硬件信号.
    if hes not in ("ok", "engaged", "cleared", "unknown"):
        raise EstopSchemaError(
            f"hes closed set violation: {hes!r}")
    latency_ms = sent_mono_ms - recv_mono_ms
    if latency_ms < 0:
        raise EstopSchemaError(
            f"latency_ms negative: sent={sent_mono_ms} recv={recv_mono_ms}")
    return EstopAck(
        rid=frame.rid,
        estop_epoch=estop_epoch,
        applied=tuple(applied),
        recv_mono_ms=recv_mono_ms,
        latency_ms=latency_ms,
        hes=hes,
        timeout_lock=timeout_lock,
    )


def check_forward_budget(latency_ms: int) -> bool:
    """R3.3: <= 100 ms robot-side forward budget. Returns True if
    within budget."""
    return latency_ms <= ESTOP_MAX_FORWARD_MS


def check_e2e_budget(qt_click_mono_ms: int,
                       ack_received_mono_ms: int) -> bool:
    """R3.3: 300 ms end-to-end budget (Qt click -> ack visible)."""
    return (ack_received_mono_ms - qt_click_mono_ms) <= ESTOP_MAX_E2E_MS


@dataclass
class EstopPathHealth:
    """state/link.estop_path 独立字段 (R3.3).
    Down debounce: >=3 consecutive missed beats -> 'down'.
    A successful ack does NOT clear a 'down' mark; only a resumed
    beat sequence does."""
    consecutive_misses: int = 0
    state: str = "ok"        # ok / degraded / down
    miss_threshold: int = 3

    def on_beat_received(self) -> None:
        """Reset the miss counter; may promote from down back to ok."""
        self.consecutive_misses = 0
        if self.state != "ok":
            self.state = "ok"

    def on_beat_missed(self) -> None:
        """Advance miss counter; may demote to degraded then down."""
        self.consecutive_misses += 1
        if self.consecutive_misses >= self.miss_threshold:
            self.state = "down"
        elif self.consecutive_misses >= 1:
            self.state = "degraded"

    def on_ack_received(self, latency_ms: int) -> None:
        """Ack alone does NOT clear a 'down' beat state (R3.3:
        'It cannot be replaced by a single ack.'). It only records
        the fact of ack delivery for latency stats."""
        # Explicitly do not mutate state.
        _ = latency_ms


def build_estop_ack_detail(msg: dict, rid: str, recv_mono_ms: int, *,
                           sent_mono_ms: int,
                           estop_epoch: int) -> Dict[str, Any]:
    """一条云端 cmd/estop 信封 -> v2.0 S2.3 的 ack detail(七项).

    把 validate_and_forward + build_ack + to_detail 收成一个入口, 让调用方
    不必知道三者的顺序. 校验不过抛 EstopSchemaError -- 调用方[已经转发过
    急停了](fail-safe: 宁可多停一次), 这里抛只影响 detail 能不能带出来.

    applied 恒为空: 见 cloud_wiring._publish_estop_ack 的注释.
    hes 恒为 unknown: 11 S538/S756 逐字"硬件急停 HES 完全不经软件", 它的
    状态要由 chassis_relay 报上来, 而那个进程未编译.
    timeout_lock 恒为 False: 超时锁属 quadruped 的急停状态机(同样未建);
    报 True 会让 Qt 以为机器人被锁在急停里需要人工解除.
    """
    frame = validate_and_forward(msg, rid)
    ack = build_ack(frame,
                    recv_mono_ms=recv_mono_ms,
                    sent_mono_ms=sent_mono_ms,
                    estop_epoch=estop_epoch,
                    applied=(),
                    hes="unknown",
                    timeout_lock=False)
    if not check_forward_budget(ack.latency_ms):
        # 超预算不改 ack 内容(那是实测值), 只记一笔 -- 判据的意义在于
        # 被看见, 把超标的数字改掉等于把判据关掉.
        _logger.warning(
            "p5 estop forward budget exceeded: %d ms > %d ms",
            ack.latency_ms, ESTOP_MAX_FORWARD_MS)
    return ack.to_detail()
