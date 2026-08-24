"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: estop_latch.py
Brief: p1 cmd/estop latch -- 本拍零速的归因锁 + re-arm 状态机 (P1-21, CLD-1a)

Description:
P1-21 逐字: p1_motion 订 cmd/estop, 收到"软急停: 本拍零速 + 落 stop_reason,
NO 不转发(转发是 chassis_relay CR-1 的活)". 全库此前 p1 没订这条 key(批59
CLD-1); 白名单登记了却没接线, 是 P1-21 点名的契约缺口.

*** 本 latch 的[主消费者是 20 Hz ctrl_loop], 不是本文件.
真正的"本拍零速"是 ctrl_loop 每拍读这个 latch -> estop 分支零速 + stop_reason
=soft_estop(见 ctrl_loop.run_one_tick, estop wins over everything). 本文件只
负责把 cmd/estop 落成一个[可观测的锁状态], 并驱动 re-arm 状态机.
* 现状: 20 Hz ctrl_loop 是 skeleton(__main__ heartbeat 不发 cmd_vel, GATED-HW),
所以物理 cmd_vel 零速待整条控制循环激活 -- 那是整个循环的 gate, 不是本 latch
引入的. 订阅 / 落锁 / 归因 / re-arm 都是真实且运行中的.

*** 恢复: 新运动指令即 re-arm, 无需显式解除(14 S3.7 表, U35).
"喊急停 -> 停 -> 喊前进两米 -> 立刻走". 所以 gate_intent 在 latched 时
[清锁并放行]那个新 intent -- 与 p2 的 G-10, three_stops.maybe_rearm 同一
语义. NO 不做"急停后拒绝一切运动"的逻辑: 那会把 U35 的现场行为堵死.

*** 解析 fail-safe(11 S3.0.1, cmd/estop 唯一豁免 v 校验的 key).
一条解析失败的急停不能被丢弃. parse 永不抛, 坏帧照样落锁.

Boundaries: 只维护锁状态与归因. 不发 cmd_vel(ctrl_loop 的活), 不转发到底盘
(chassis_relay 的活), 不做 20 Hz 时序.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Optional

_logger = logging.getLogger(__name__)

#: P1-21 归因. stop_reason 闭集(common/enums)里软急停对应的值.
STOP_REASON_SOFT_ESTOP = "soft_estop"
STOP_REASON_NONE = "none"


def parse_estop_cmd_id(raw: bytes) -> str:
    """从 cmd/estop 字节取 cmd_id. 永不抛(11 S3.0.1 fail-safe).

    cmd_id 是幂等键(同 cmd_id 10 Hz 重发算同一次急停, 11 S2.2.3). 取值顺序
    cmd_id -> msg_id(云端信封) -> origin 回退. 解析失败用固定回退, 同样的
    坏帧重发不至于每帧刷一条日志.
    """
    try:
        body = json.loads(raw.decode("utf-8")) if isinstance(
            raw, (bytes, bytearray)) else raw
    except Exception:                            # noqa: BLE001
        _logger.warning("p1 cmd/estop unparseable; latching anyway (fail-safe)")
        return "estop-unparsed"
    if not isinstance(body, dict):
        return "estop-nonobject"
    return str(body.get("cmd_id") or body.get("msg_id")
               or "estop-%s" % (body.get("origin") or "unknown"))


class P1EstopLatch:
    """p1 侧软急停锁. ctrl_loop 每拍读 is_active(); 新 intent 触发 rearm.

    * 线程: on_estop 在 Zenoh 的 Rust 回调里被调(CLAUDE.md 4.2), 只做纯赋值,
    不发布不 await. gate_intent 在主线程(_on_intent)调. 两者都只碰这几个
    标量字段, 而标量赋值在 GIL 下原子, 与本进程别处的跨线程缓存同一做法.
    """

    def __init__(self) -> None:
        self._active = False
        self._cmd_id: Optional[str] = None
        #: 只为可观测.
        self.latches = 0
        self.rearms = 0

    def on_estop(self, raw: bytes) -> None:
        """收到一条 cmd/estop: 落锁 + 记 cmd_id. RUST 线程, 只赋值.

        幂等: 同 cmd_id 重发不刷计数(10 Hz 重发, 1 s 窗口, 11 S2.2.3).
        """
        cmd_id = parse_estop_cmd_id(raw)
        if self._active and self._cmd_id == cmd_id:
            return                               # 同一次急停的重发, 幂等
        self._active = True
        self._cmd_id = cmd_id
        self.latches += 1
        _logger.warning("p1 soft-estop latched (cmd_id=%s) -> stop_reason=%s; "
                        "20Hz ctrl_loop will zero cmd_vel when active",
                        cmd_id, STOP_REASON_SOFT_ESTOP)

    def is_active(self) -> bool:
        """ctrl_loop 每拍读这个 -> estop 分支零速."""
        return self._active

    def stop_reason(self) -> str:
        """本拍零速的归因(P1-21). 锁着是 soft_estop, 否则 none."""
        return STOP_REASON_SOFT_ESTOP if self._active else STOP_REASON_NONE

    def gate_intent(self) -> bool:
        """一条新 cmd/motion/intent 到达时调. 返回是否放行该 intent.

        *** latched 时: 清锁(re-arm)并放行(14 S3.7 / U35: 新运动指令即钥匙).
        NO 不拒绝新运动 -- 那会堵死"喊急停 -> 喊前进 -> 走". 与 p2 的
        three_stops.maybe_rearm, motion_intent_wiring G-10 同一语义.
        未 latched 时直接放行.
        """
        if not self._active:
            return True
        self._active = False
        self._cmd_id = None
        self.rearms += 1
        _logger.info("p1 soft-estop re-armed by a new motion intent "
                     "(14 S3.7); forwarding it")
        return True


__all__ = ["P1EstopLatch", "parse_estop_cmd_id", "STOP_REASON_SOFT_ESTOP",
           "STOP_REASON_NONE"]
