"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: b_mode_timer.py
Brief: BIZ-P2-13 -- B-mode IDLE timeout (T-BCAST-IDLE): auto-exit after N s without a valid audio frame

Description:
What this enforces
------------------
User ruling 2026-10-28 (99 U97, D1 of the broadcast gap review): the B-mode
auto-exit follows Party A's frozen v2.0 S8 -- "the backend exits broadcast 300
seconds after the LAST VALID AUDIO FRAME" -- and 00 VOI-26 ("5 minutes without
new audio"). There is NO maximum-duration ceiling any more.

  ~~T-BCAST-MAX: maximum duration counted from entering B, never reset by
  audio frames (14 BCT-2, 11 S1.6.3)~~ -- superseded 2026-10-28. Why: it cut a
  broadcast that was still being spoken (measured on .1.26 2026-10-27: two
  sessions stopped at exactly 300 s while frames were still arriving), and the
  case it was built for -- Party A's process crashes or the link drops so the
  `stop` never comes -- also stops the FRAMES, so an idle timeout ends that
  broadcast too. The one case the idle timeout does not end (a cloud client
  that keeps streaming forever) was accepted by the ruling.

Rules (14 S5.6.1 BCT-1..BCT-6 as revised by U97):
  BCT-1  monotonic clock only (CLK-C1). A wall-clock step from RTK first fix
         or NTP would otherwise expire the timer at once or never.
  BCT-2  the idle reference is max(entry, last ACCEPTED frame). Only a frame
         that passed every accept_chunk check counts: a stream of rejected
         frames (wrong stream_id, bad codec) is not "someone is speaking".
         Heartbeats and repeated `start` commands do not count either.
  BCT-3  on expiry the caller exits B through the mode machine (exit_reason
         timeout) and the speaker must actually go quiet.
  BCT-4  the caller emits event/warn/system with the duration and idle time.
  BCT-5  applies to every way into B, Party A's AUDIO_CONTROL included.
  BCT-6  no re-entry suppression after the exit.

Boundaries: this class holds no clock and no session state; the caller passes
monotonic ms for "now" and for the last accepted frame (the frame time is
written on the Zenoh callback thread into BroadcastSession, read here on the
main loop -- a float assignment, atomic in CPython).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class BModeTimer:
    """Monotonic-clock B-mode idle-timeout tracker.

    idle_timeout_s is mode.b_cast_idle_timeout_s from the resolved snapshot
    (14 S11); 300 s per v2.0 S8.
    """
    idle_timeout_s: float
    started_mono_ms: Optional[int] = None

    def start(self, now_mono_ms: int) -> None:
        """Arm on entering BROADCAST. With no frame yet, idle counts from here:
        an operator who enters B and never speaks is also "idle"."""
        self.started_mono_ms = now_mono_ms

    def stop(self) -> None:
        """Disarm on ANY exit from BROADCAST (command, timeout, fault)."""
        self.started_mono_ms = None

    @property
    def running(self) -> bool:
        """True between start() and stop()."""
        return self.started_mono_ms is not None

    def elapsed_ms(self, now_mono_ms: int) -> int:
        """Ms since entering B (for the BCT-4 event); 0 if not running."""
        if self.started_mono_ms is None:
            return 0
        return max(0, now_mono_ms - self.started_mono_ms)

    def idle_ms(self, now_mono_ms: int,
                last_frame_mono_ms: Optional[int]) -> int:
        """Ms since the later of entry and the last accepted frame.

        A last frame OLDER than the entry (left over from a previous session
        that the caller forgot to clear) must not shorten this session's
        budget, hence the max().
        """
        if self.started_mono_ms is None:
            return 0
        ref = self.started_mono_ms
        if last_frame_mono_ms is not None and last_frame_mono_ms > ref:
            ref = last_frame_mono_ms
        return max(0, now_mono_ms - ref)

    def expired(self, now_mono_ms: int,
                last_frame_mono_ms: Optional[int]) -> bool:
        """True iff running AND idle >= idle_timeout_s (BCT-1: mono ms only)."""
        if self.started_mono_ms is None:
            return False
        return (self.idle_ms(now_mono_ms, last_frame_mono_ms)
                >= int(self.idle_timeout_s * 1000))
