"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: estop_probe.py
Brief: HMI-W5 estop-path health probe state machine (17 S6.3)

Description:
The problem this solves. 17 S6.3 requires the HMI to grey its ESTOP button the
moment the estop path is actually down (NAV-64), not to leave it armed on a dead
link. That needs a real end-to-end probe: P5 pings 1 Hz, the reply RTT and the
consecutive-miss count decide estop_path (ok / degraded / down). Before this the
MVP hard-coded estop_path="ok", which armed the button even with nothing behind
it -- exactly the fail-silent 3.2 forbids.

Which section this follows: 17 S6.3 (link_probe task, 1 Hz):
    pong within RTT threshold        -> "ok"
    pong but RTT over threshold      -> "degraded"
    N consecutive missing pongs      -> "down"
thresholds from hmi.link_rtt_degrade_ms / hmi.link_down_misses.

What it does NOT do, and the boundary. This is the pure STATE MACHINE only --
no zenoh, no clock. The wiring drives it: it calls on_ping_sent() each tick when
it publishes the probe, on_pong() when a reply arrives, and reads estop_path().
Keeping the transport out means the ok/degraded/down logic is unit-tested with
plain numbers, and the monotonic clock (11 CLK-C1) stays the caller's.

Trap this exists to avoid. The probe endpoint is the quadruped estop channel
(17 S6.3), which is GATED-HW today -- no chassis replies. So this MUST start (and
stay) "down" until a real pong arrives, and the button greys honestly. It must
NOT default to "ok": a probe that reports healthy with nothing answering is the
same fail-silent the hard-coded "ok" already was.
"""

from __future__ import annotations

from typing import Optional


class EstopProbe:
    """estop_path state machine driven by ping/pong timing (17 S6.3).

    Starts "down" (misses seeded at the threshold): until a real pong is seen the
    button must be greyed, never armed on faith. rtt_degrade_ms and down_misses
    are injected from config (no defaults here -- 3.1 keeps values in the yaml).
    """

    __slots__ = ("_rtt_degrade_ms", "_down_misses", "_misses", "_rtt_ms",
                 "_awaiting", "_sent_seq", "_sent_mono_ms")

    def __init__(self, rtt_degrade_ms: float, down_misses: int) -> None:
        self._rtt_degrade_ms = rtt_degrade_ms
        self._down_misses = down_misses
        # Seed AT the threshold so estop_path() is "down" before the first pong.
        self._misses = down_misses
        self._rtt_ms: Optional[float] = None
        self._awaiting = False          # a ping is outstanding, no pong yet
        self._sent_seq = 0
        self._sent_mono_ms = 0

    def on_ping_sent(self, seq: int, mono_ms: int) -> None:
        """Record that probe `seq` went out at `mono_ms`. If the PREVIOUS ping is
        still outstanding (never got its pong), that is one missed reply -- count
        it now, capped at the threshold so a long outage does not overflow."""
        if self._awaiting:
            self._misses = min(self._down_misses, self._misses + 1)
        self._awaiting = True
        self._sent_seq = seq
        self._sent_mono_ms = mono_ms

    def on_pong(self, seq: int, mono_ms: int) -> None:
        """A reply arrived. Only the reply to the OUTSTANDING ping counts (a stale
        pong for an older seq is ignored, so a late reply cannot mask a real
        outage). Clears the miss count and records the RTT."""
        if self._awaiting and seq == self._sent_seq:
            self._rtt_ms = max(0.0, float(mono_ms - self._sent_mono_ms))
            self._misses = 0
            self._awaiting = False

    def estop_path(self) -> str:
        """The 17 S6.3 closed value the button reads: down > degraded > ok.

        down wins first (a dead link is not merely slow); then a healthy-but-slow
        link is degraded; only a fresh pong under the RTT threshold is ok."""
        if self._misses >= self._down_misses:
            return "down"
        if self._rtt_ms is not None and self._rtt_ms >= self._rtt_degrade_ms:
            return "degraded"
        return "ok"

    @property
    def rtt_ms(self) -> Optional[float]:
        """Last measured round-trip, or None before the first pong (for the HMI
        latency readout / state/link.latency_ms)."""
        return self._rtt_ms


# --- 11 S8.5 报文形状 (2026-09-27 seq 口径收口) -------------------------
#
# *** 为什么这两个函数在这里而不是留在 runtime/main_wiring.py 的闭包里.
# 它们承载的是本轮修掉的那个缺陷的全部内容 -- "关联号放哪, 从哪读" --
# 而闭包里的代码没有任何单测够得着. 缺陷本身正是因为没人能对着它写一条
# 断言才活了这么久. 提出来之后, 下面两条判据都能在无 zenoh 无钟的条件下跑.
#
# *** 缺陷的形状(实测 2026-09-27, chassis_relay 上机之后):
# p5 发 ping 时 seq 写在[顶层], 收 pong 时也按[顶层] seq 匹配; quadruped 的
# HandlePing 回显的是[信封] seq. 三方看起来一致 -- 直到 chassis_relay 进链:
# RT-C3.e [要求]转发者重建信封并换上自己的计数, 而 relay 在 CR-2/CR-3 两条腿
# 上都要转发本探活. 于是 p5 发出去的号在 RT 侧已经被换掉, 回来的是 relay 的
# 计数, 匹配永远不成立. 现象: pong 以 1 Hz 稳定流动, 两侧进程都健康,
# estop_path 却恒为 down, HMI 的急停按钮永远置灰.
#
# *** 裁决: 三方统一用 data.seq. data 是转发者[逐字节搬运]的部分,
# 唯一能让端到端关联号活着穿过 relay 的地方.

PING_TYPE = "ping"


def build_ping_data(seq: int, mono_ms: int) -> dict:
    """11 S8.5 的 ping 体 -- 放进 S3.0 信封的 data 里的那一层.

    seq 在这里(data 内), NO 不在信封里. 信封 seq 属传输层, RT-C3.e 允许并
    要求转发者改写它; 拿它做端到端关联号, 等于把关联号交给中间人重新编号.
    """
    return {"type": PING_TYPE, "seq": int(seq), "t_mono_ms": int(mono_ms)}


def pong_seq(payload: dict) -> Optional[int]:
    """从一条 pong 里取端到端关联号, 取不到返回 None(调用方按未匹配处理).

    *** 只读 data.seq, NO 不回落到顶层 seq.
    顶层 seq 是最后一跳转发者的计数器, 1 Hz 自增, 与 p5 的 probe_seq 同频
    同量级 -- 它迟早会[偶然相等]. 那一拍会被记成一次成功 RTT, 于是
    estop_path 间歇性地跳成 ok. 一个偶尔为真的匹配比永远不匹配更坏:
    永不匹配是 down(fail-safe, 按钮置灰, 提示用遥控器急停), 偶尔匹配是
    "链路时好时坏"的假象(fail-silent), 而按下去不会停.

    bool 显式排掉: 它是 int 的子类, {"seq": true} 会被当成 1.
    """
    if not isinstance(payload, dict):
        return None
    body = payload.get("data")
    if not isinstance(body, dict):
        return None
    seq = body.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int):
        return None
    return seq
