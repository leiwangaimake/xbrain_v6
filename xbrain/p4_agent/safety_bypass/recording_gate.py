"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: recording_gate.py
Brief: GWY-P4-05 -- U45 recording-state voice-estop suppression

Description:
16 S4.2 U45: while in `geometry_recording` (path/fence recording
session), VOICE estop is SUPPRESSED. Keyboard / handle estop
remain active as the safety baseline.

* Reason: operator holds keyboard, stands next to robot, talks
constantly (announces waypoints, reports back). A voice estop
false-trigger DESTROYS the recording session.

Three MANDATORY actions when suppressing:
  1. Log the suppression (write to commands table with
     route='suppressed', accepted=0, reject_reason='recording_suppressed')
  2. Emit TTS "录制中, 请用手柄急停" so operator knows the button path
  3. Increment counter for telemetry (repeated triggers -> retrain)
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class RecordingState:
    """Current geometry_recording session state."""
    in_recording: bool = False
    voice_estop_suppress_count: int = 0


@dataclass(frozen=True)
class SuppressionRecord:
    """One suppression event (for the commands table)."""
    route: str = "suppressed"
    accepted: int = 0            # DB uses int 0/1 for legacy
    reject_reason: str = "recording_suppressed"
    tts_advice: str = "录制中, 请用手柄急停"


def evaluate(state: RecordingState, bypass_action: str) -> SuppressionRecord | None:
    """Given the safety-bypass matcher fired with `bypass_action`
    ('estop' / 'prone' / 'stand'), decide whether to suppress.

    U45 SUPPRESSES only VOICE ESTOP during recording. prone / stand
    remain executable (they are movement commands, not stops).

    * Keyboard / handle estop bypass this path entirely -- they
    are hardware buttons wired below the voice layer."""
    if not state.in_recording:
        return None
    if bypass_action == "estop":
        state.voice_estop_suppress_count += 1
        return SuppressionRecord()
    return None
