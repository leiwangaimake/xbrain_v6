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
