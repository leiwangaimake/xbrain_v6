"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: recording_lock.py
Brief: MOT-PM-30 TR-1/TR-2/TR-3 recording-state exclusive lock

Description:
During geometry_recording (P3 signals P1 via state/task.recording
mirror), P1 SUPPRESSES all five behavior sources except keyboard/
joystick teleop. New delegate requests (nav2_proxy / path_follow)
are REJECTED with E_BUSY + recording_active; in-flight delegate
requests are CANCELLED (in-flight state does not persist across
recording boundary).

TR-1: source suppression is ENTIRE-SOURCE (not per-frame filter).
TR-2: new delegate refused.
TR-3: in-flight delegate cancelled cleanly (no dangling grant).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet


# Sources ALLOWED during recording (the two local teleop channels).
_ALLOWED_DURING_RECORDING: FrozenSet[str] = frozenset({
    "teleop_keyboard", "teleop_joystick",
})


class RecordingRejection(RuntimeError):
    """A request was refused because recording is active."""


@dataclass
class RecordingLock:
    active: bool = False

    def is_source_permitted(self, source: str) -> bool:
        """TR-1: during recording, only the two teleop sources are
        permitted holders. All others get suppressed."""
        if not self.active:
            return True
        return source in _ALLOWED_DURING_RECORDING

    def check_new_delegate(self, source: str) -> None:
        """TR-2: refuse a new delegate request during recording."""
        if not self.active:
            return
        if source not in _ALLOWED_DURING_RECORDING:
            raise RecordingRejection(
                "recording_active: source %r refused "
                "(only keyboard/joystick allowed during recording)"
                % source)
