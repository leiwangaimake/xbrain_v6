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

99 U133 (2026-11-05): the recording flag is now the live state/teach session
(turn_orchestrator sets in_recording every turn) -- before that NOTHING set it
and none of this ever fired. The same ruling stops every other control
command while recording ("只能通过云端键盘控制"), so the bypass prone / stand
are suppressed too, with their own advice; the keyboard's space bar is the
stop.
"""

from __future__ import annotations

from dataclasses import dataclass


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
    tts_advice: str = "录制中语音急停不生效, 请用键盘空格键停车"


def evaluate(state: RecordingState, bypass_action: str) -> SuppressionRecord | None:
    """Given the safety-bypass matcher fired with `bypass_action`
    ('estop' / 'prone' / 'stand'), decide whether to suppress.

    U45 suppresses VOICE ESTOP during recording. Since 99 U133 prone /
    stand are suppressed too: they are control commands, and the ruling
    leaves the robot to the cloud keyboard while recording (their advice
    says so instead of pointing at the stop key).

    * Keyboard / handle stops bypass this path entirely -- they never
    reach the voice layer."""
    if not state.in_recording:
        return None
    if bypass_action == "estop":
        state.voice_estop_suppress_count += 1
        return SuppressionRecord()
    return SuppressionRecord(tts_advice=_BUSY_ADVICE)  # prone / stand (U133)


#: 99 U133: what a suppressed prone / stand says (ASCII punctuation).
_BUSY_ADVICE = "正在录制, 录制期间只能用键盘控制机器人. 要结束请说退出录制"
