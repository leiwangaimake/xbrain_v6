"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: pose_snap.py
Brief: GWY-P4-25 -- PS-1..PS-6 position snapshot ring buffer + still_1s_ok

Description:
16 S8.6 pose snapshot on intent commit. When P4 dispatches an intent
whose semantics depend on the robot's CURRENT position (e.g., 'record
this fence corner'), the pose_snap block is attached to the intent
envelope so the consumer knows the position at ISSUE time, not at
receive time.

PS-1: ring buffer capacity N poses (typical 100 = 1s @ 100 Hz)
PS-2: pose is EMA + variance over last 1 s
PS-3: still_1s_ok = variance below threshold for a continuous 1 s
       window (else the operator was walking; snapshot might be
       across two different points)
PS-4: snapshot INCLUDES ts_iso for audit, MONOTONIC ms for math
PS-5: if buffer < 1s of history, still_1s_ok=False
PS-6: consumer refuses intent with still_1s_ok=False when the intent
       requires a stable position (recording, waypoint mark)

PoseHistory (99 U109) is the part that is WIRED: the 18 S0.4.1 PS-6
`pose_snap` block cmd/teach carries for a keypoint capture (F06 / F10). It
keeps the last state/pose frames as received (WGS84, the 11 S4.5 fields) and
answers "where was the robot when the operator started speaking" -- not
"where is it now", which after a 2.5-4.8 s LLM turn is up to 9.6 m away (18
S0.4). The ENU Pose / PoseRing above are the older envelope-side draft and
are not on any wire.
"""

from __future__ import annotations

from collections import deque  # bounded history, oldest dropped
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Tuple

#: state/pose frames kept: 10 Hz x 30 s. Longer than any turn (ASR + the T4
#: LLM leg + a queued cloud job), so the first-word moment is still covered.
_HISTORY_LEN = 300  # frames, not a time

#: 11 S12A.8 / 18 PS-6: "still" for a dock is speed below 0.1 m/s for the
#: whole second before the first word (the same numbers P3 uses).
_STILL_SPEED = 0.1  # m/s
_STILL_WINDOW = 1000  # ms
_STILL_MIN_SPAN = 900  # ms; less than this is "less than 1 s" (PS-5)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    heading_deg: float
    mono_ms: int


@dataclass
class PoseRing:
    """Fixed-capacity ring; oldest evicted on write."""
    capacity: int
    _buf: List[Pose] = field(default_factory=list)

    def push(self, p: Pose) -> None:
        self._buf.append(p)
        if len(self._buf) > self.capacity:
            self._buf.pop(0)

    def window(self, since_mono_ms: int) -> List[Pose]:
        return [p for p in self._buf if p.mono_ms >= since_mono_ms]

    def is_still_1s(self, now_mono_ms: int,
                    variance_threshold_meters: float = 0.05) -> bool:
        """PS-3: variance of x/y over last 1 s below threshold."""
        window = self.window(now_mono_ms - 1000)
        # PS-5: less than 1 s of data -> not still.
        if len(window) < 2:
            return False
        span = window[-1].mono_ms - window[0].mono_ms
        if span < 900:    # < 900 ms is still 'less than 1 s'
            return False
        mean_x = sum(p.x for p in window) / len(window)
        mean_y = sum(p.y for p in window) / len(window)
        var_x = sum((p.x - mean_x) ** 2 for p in window) / len(window)
        var_y = sum((p.y - mean_y) ** 2 for p in window) / len(window)
        return (var_x + var_y) ** 0.5 < variance_threshold_meters


@dataclass(frozen=True)
class PoseSnap:
    """PoseSnap block for intent envelope."""
    x: float
    y: float
    heading_deg: float
    mono_ms: int
    still_1s_ok: bool


def _pose_data(frame: Any) -> Optional[Dict[str, Any]]:
    """The 11 S4.5 PoseState fields of a state/pose frame (enveloped or bare)."""
    if not isinstance(frame, dict):
        return None  # not a pose
    data = frame.get("data")  # p1 publishes the 11 S3.0 envelope
    return data if isinstance(data, dict) else frame  # bare for stub publishers


class PoseHistory:
    """The recent state/pose frames, and the 18 PS-6 snapshot taken from them.

    feed() runs on the zenoh callback thread and snapshot() on the turn
    worker; deque.append and list(deque) are each atomic under the GIL, so
    the reader sees a consistent copy without a lock.
    """

    def __init__(self, max_age_ms: int) -> None:
        # The farthest a frame may be from the first-word moment and still
        # stand for it -- the same bound a state reading has before P4 calls
        # it unknown (--query-state-max-age-ms).
        self._max_age_ms = max_age_ms
        self._frames: Deque[Tuple[int, Dict[str, Any]]] = deque(
            maxlen=_HISTORY_LEN)  # (receive mono ms, PoseState data)

    def feed(self, recv_mono_ms: int, frame: Any) -> None:
        """Remember one decoded state/pose frame with its receive time."""
        data = _pose_data(frame)  # unwrap the envelope
        if data is not None:
            self._frames.append((recv_mono_ms, data))  # oldest falls off

    def snapshot(self, t_first_mono_ms: int,
                 now_mono_ms: int) -> Optional[Dict[str, Any]]:
        """The PS-6 `pose_snap` block for a turn whose first word was at
        t_first_mono_ms, or None when no frame near that moment has a position.

        The frame used is the last one received at or before the first word
        (the robot's position as it started speaking); when the history
        starts after it, the first one after. Either must lie within the
        state age bound of the first word.
        """
        frames = list(self._frames)  # a stable copy
        before = [f for f in frames if f[0] <= t_first_mono_ms]  # up to the word
        after = [f for f in frames if f[0] > t_first_mono_ms]  # history began later
        pick = before[-1] if before else (after[0] if after else None)
        if pick is None or abs(pick[0] - t_first_mono_ms) > self._max_age_ms:
            return None  # no pose for that moment
        pose = pick[1]  # PoseState fields
        if pose.get("lat") is None or pose.get("lon") is None:
            return None  # no position to store
        recent = [f for f in before
                  if f[0] >= t_first_mono_ms - _STILL_WINDOW]  # the second before
        span = recent[-1][0] - recent[0][0] if recent else 0  # covered ms
        still = span >= _STILL_MIN_SPAN and all(
            isinstance(p.get("speed_mps"), (int, float))
            and abs(p["speed_mps"]) < _STILL_SPEED
            for _t, p in recent)  # unknown speed is not still
        return {
            "lat": pose.get("lat"), "lon": pose.get("lon"),
            "alt_m": pose.get("alt"),  # PS-6 spells the unit
            "heading_rad": pose.get("heading_rad"),  # ENU, as published
            "heading_valid": bool(pose.get("heading_valid")),
            "heading_level": pose.get("heading_level"),
            "fix_type": pose.get("fix_type"),
            "i_fix": pose.get("i_fix"), "i_heading": pose.get("i_heading"),
            "speed_mps": pose.get("speed_mps"),
            "still_1s_ok": still,  # F10's stillness, judged before the word
            "snap_mono_s": t_first_mono_ms / 1000.0,  # CLOCK_MONOTONIC
            "snap_age_ms": max(0, now_mono_ms - t_first_mono_ms),  # word -> send
        }
