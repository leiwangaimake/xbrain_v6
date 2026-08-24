"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: estop_wiring.py
Brief: cmd/estop -> domain-1 soft-estop disarm + strobe + state/arb/motion (14 S3.7)

Description:
P2 orders cmd/estop 是 CLD-1 的补线: 全库此前无人订阅 cmd/estop, 三个软件
来源(p4 语音旁路 / HMI 按钮 / 云端)全都发进了空气.

*** P2 [不是]急停的生效路径(14 S3.7 SE-1a 逐字).
急停由 quadruped Tier 1 直接执行; chassis_relay(CR-1) 纯转发到 rt/safety/estop.
P2 订 cmd/estop 只做两件事, 一件都不涉及真正停车:
  1. 域1(motion)缴械 -> state/arb/motion.suspended = "soft_estop"
     p1 读它后本拍零速 hold(P1-21), 那才是软件侧的"停".
  2. 域4 强制红蓝爆闪(SE-1) -- 车停在路中间必须被看见.
NO 域2/3/4/5 一个都不缴械(14 S3.7: "只停车不锁机"). 缴械域4 会把爆闪的
持有者撤掉 -- 车停了反而不闪, 与本意相反.

*** cmd/estop 在本进程只产生 soft_estop.
three_stops 支持三停(soft_estop/hes/cmd_timeout), 但它们经不同通道到达:
soft_estop 走 cmd/estop(本文件); hes 走 state/robot.hes_lock; cmd_timeout
走 20 Hz cmd_vel 年龄. 后两者的接线不在本文件范围, 见 NEXT.

*** 解析必须 fail-safe(11 S3.0.1, cmd/estop 是唯一豁免 v 校验的 key).
一条解析失败的急停[不能被丢弃] -- 丢了就是没停. 所以 parse_estop_frame
永不抛: 解析不出来照样当一次停(宁可多停), 只是 cmd_id 用回退值.

*** 恢复: 新运动指令即可解除, 无需显式解除命令(14 S3.7 表).
只有 soft_estop 能被新指令解除; hes 要 HES 归零+人工 enable, cmd_timeout
要显式 enable. 所以 maybe_rearm 只在 suspended == "soft_estop" 时动作 --
一个不加这个判别的实现会让一条新语音指令把硬件锁也解开.

Boundaries: 只做缴械态的表达与广播. 不转发到底盘(那是 chassis_relay),
不判断急停能不能执行(急停无条件生效). 广播与 emit 由调用方注入(可测).
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any, Callable, Optional

from xbrain.p2_core.three_stops import (StopEvent, StopReason, apply_rearm,
                                        apply_stop)

_logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class EstopFrame:
    """一条解析后的 cmd/estop. cmd_id 是幂等键(同 cmd_id 重发算同一次停)."""
    cmd_id: str
    action: str


def parse_estop_frame(raw: bytes) -> EstopFrame:
    """把 cmd/estop 的字节解析成 EstopFrame. 永不抛(11 S3.0.1 fail-safe).

    *** 解析失败照样当一次停.
    一条坏掉的急停被丢弃 = 没停, 而急停丢不起. 所以任何解析问题都回退到
    一个合法的 stop 帧, 只是 cmd_id 用一个稳定回退值(让同源的坏帧重发仍
    幂等, 不至于每帧刷一条审计).

    cmd_id 取值顺序: 帧的 cmd_id -> msg_id(云端信封用这个) -> origin 回退.
    action 缺失按 stop(急停的默认动作就是停, 缺字段不该让它不停).
    """
    try:
        body = json.loads(raw.decode("utf-8")) if isinstance(
            raw, (bytes, bytearray)) else raw
    except Exception:                            # noqa: BLE001
        # 连 JSON 都不是. 仍然停 -- 用一个固定 cmd_id, 同样的坏帧重发不刷屏.
        _logger.warning("p2 cmd/estop unparseable; stopping anyway (fail-safe)")
        return EstopFrame(cmd_id="estop-unparsed", action="stop")
    if not isinstance(body, dict):
        return EstopFrame(cmd_id="estop-nonobject", action="stop")
    cmd_id = (body.get("cmd_id") or body.get("msg_id")
              or "estop-%s" % (body.get("origin") or "unknown"))
    action = body.get("action") or "stop"
    return EstopFrame(cmd_id=str(cmd_id), action=str(action))


class EstopCoordinator:
    """把 cmd/estop 接到域1 缴械 + 广播 + 爆闪 + re-arm.

    发布与 emit 注入(publish_suspended / emit_event), 所以整条逻辑能在
    无 Zenoh 的测试里跑. arbiter 与 strobe_state 是本进程持有的真对象.
    """

    def __init__(self, motion_arb, strobe_state,
                 emit_event: Callable[[dict], None],
                 publish_suspended: Callable[[int], None]) -> None:
        self._arb = motion_arb
        self._strobe = strobe_state
        self._emit = emit_event
        self._publish = publish_suspended
        #: 只为可观测.
        self.stops = 0
        self.rearms = 0

    def on_estop(self, raw: bytes, now_mono_ms: int) -> None:
        """收到一条 cmd/estop: 缴械域1 + 强制爆闪 + emit + 广播 suspended.

        *** 广播必须在缴械之后, 不能省.
        p1 靠 state/arb/motion.suspended 判零速. 只缴械不广播 = 状态锁在 p2
        进程内, p1 永远读不到, 软急停对运动毫无效果. 这正是"只测构建器看不见
        总线"那类缺陷(批14-16)在急停链路上的样子.
        """
        frame = parse_estop_frame(raw)
        # cmd/estop 在本进程恒 soft_estop(见模块头). hes/cmd_timeout 另有通道.
        apply_stop(StopEvent(StopReason.SOFT_ESTOP, frame.cmd_id, now_mono_ms),
                   self._arb, self._strobe, self._emit)
        self._publish(now_mono_ms)               # 广播 suspended, p1 读它零速
        self.stops += 1

    def maybe_rearm(self, now_mono_ms: int) -> bool:
        """一条新运动指令到达时调用. 仅当处于 soft_estop 缴械时解除.

        *** 只解 soft_estop, NO 不解 hes/cmd_timeout.
        14 S3.7 表: soft_estop 新指令即可解除; hes 要 HES 归零+人工 enable;
        cmd_timeout 要显式 enable. 不加这个判别, 一条新语音指令会把硬件锁
        也解开 -- 而硬件锁是"人必须到现场处理"的那种.

        返回是否真的做了 re-arm(供调用方决定要不要广播).
        """
        if self._arb.suspended() != "soft_estop":
            return False
        apply_rearm("rearm-%d" % now_mono_ms, now_mono_ms,
                    self._arb, self._strobe, self._emit)
        self._publish(now_mono_ms)               # 广播 suspended=null, p1 恢复
        self.rearms += 1
        return True


def suspended_frame(motion_arb, now_mono_ms: int) -> dict:
    """state/arb/motion 的 body(11 S7A.5.1).

    suspended 是必填字段: null | soft_estop | hes | cmd_timeout. p1 只读这个.
    gen 单调递增, 供消费者判"这是不是一次新的缴械/解除".
    """
    return {
        "domain": "motion",
        "suspended": motion_arb.suspended(),     # None 序列化成 JSON null
        "gen": motion_arb.gen(),
        "mono_ms": now_mono_ms,
    }


__all__ = ["EstopFrame", "parse_estop_frame", "EstopCoordinator",
           "suspended_frame"]
