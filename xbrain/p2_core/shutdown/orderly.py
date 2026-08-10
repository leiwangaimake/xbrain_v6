"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: orderly.py
Brief: INF-DP-10 有序停机 S1..S9 + PWR-S1..S6 + SYS-G1..G4

Description:
Nine-step orderly shutdown sequence:

  S1  ack cloud (5s cap; timeout -> continue anyway, log skipped)
  S2  stop new intent admission
  S3  drain active tasks up to a bounded window
  S4  release ptz / payload / lights
  S5  publish state/mode = 'stopping' and freeze arbitration
  S6  four DB wal_checkpoint(TRUNCATE) + one synchronous=FULL
      (steady state MUST be NORMAL, not FULL -- switching back to
       NORMAL for regular writes stays a separate step)
  S7  motion domain zeros vx/vy/wz and P1 exits LAST
      (PWR-S1: P1 exits AFTER emitting a zero-cmd frame; no more
       frames after that)
  S8  chassis-side reboot handshake if requested
  S9  systemd-notify STOP + process exit

SYS-G1..G4 gates (each refuses shutdown when triggered):
  SYS-G1  active E-stop in progress                  -> E_BUSY
  SYS-G2  active fire/emergency dispatch             -> E_BUSY
  SYS-G3  active teach recording  (NO exemption)     -> E_BUSY
  SYS-G4  active charging + battery < critical       -> E_BUSY

PWR-S1 discipline: P1 must be last-out; PWR-S2 requires the ack
detail + HMI banner to say 'next boot needs one unlock confirm'
(after Orin power-off, timeout_lock is inevitably re-set).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from xbrain.common.errors import E_BUSY


class ShutdownStep(str, Enum):
    S1_CLOUD_ACK = "s1_cloud_ack"
    S2_STOP_ADMIT = "s2_stop_admit"
    S3_DRAIN = "s3_drain"
    S4_RELEASE_PERIPHERALS = "s4_release_peripherals"
    S5_MODE_STOPPING = "s5_mode_stopping"
    S6_DB_CHECKPOINT = "s6_db_checkpoint"
    S7_MOTION_ZERO_P1_EXIT = "s7_motion_zero_p1_exit"
    S8_CHASSIS_HANDSHAKE = "s8_chassis_handshake"
    S9_SYSTEMD_STOP = "s9_systemd_stop"


CLOUD_ACK_MAX_MS = 5_000       # S1 timeout cap
DB_STEADY_SYNC_MODE = "NORMAL"  # NEVER 'FULL' as steady state


@dataclass(frozen=True)
class ShutdownGateVerdict:
    accepted: bool
    code: str = ""
    reason: str = ""


def check_gates(
        estop_active: bool,
        alarm_active: bool,
        teach_recording: bool,
        charging_critical: bool) -> ShutdownGateVerdict:
    """SYS-G1..G4 in fixed order. First trip wins."""
    if estop_active:
        return ShutdownGateVerdict(False, E_BUSY, "SYS-G1 estop_active")
    if alarm_active:
        return ShutdownGateVerdict(False, E_BUSY, "SYS-G2 alarm_active")
    if teach_recording:
        # v0.7.7 ruling: SYS-G3 does NOT get an exemption.
        return ShutdownGateVerdict(False, E_BUSY, "SYS-G3 teach_recording")
    if charging_critical:
        return ShutdownGateVerdict(False, E_BUSY, "SYS-G4 charging_critical")
    return ShutdownGateVerdict(accepted=True)


@dataclass
class ShutdownProgress:
    """State machine tracking one shutdown attempt. steps_completed
    lists what has finished; skipped[] lists non-fatal timeouts."""
    steps_completed: List[str] = field(default_factory=list)
    skipped: List[str] = field(default_factory=list)
    cmd_vel_frames: List[tuple] = field(default_factory=list)    # (mono_ms, vx, vy, wz)
    p1_exited_mono_ms: Optional[int] = None
    hmi_banner: str = ""
    ack_detail: dict = field(default_factory=dict)


def run_s1_cloud_ack(progress: ShutdownProgress,
                      cloud_ack_arrived: bool,
                      wait_ms: int) -> None:
    """S1: attempt cloud ack; if it does not arrive within cap ->
    still continue (log 'skipped' + persist to boot_fail queue)."""
    if not cloud_ack_arrived and wait_ms >= CLOUD_ACK_MAX_MS:
        progress.skipped.append("cloud_ack")
        progress.ack_detail.setdefault("skipped", []).append("cloud_ack")
    progress.steps_completed.append(ShutdownStep.S1_CLOUD_ACK.value)


def run_s6_db_checkpoint(progress: ShutdownProgress,
                          steady_state_sync_mode: str) -> None:
    """S6: after WAL(TRUNCATE) + one synchronous=FULL flush, the
    STEADY state MUST return to NORMAL. Leaving synchronous=FULL
    for normal writes destroys throughput permanently."""
    if steady_state_sync_mode != DB_STEADY_SYNC_MODE:
        raise ValueError(
            f"steady-state synchronous mode must be {DB_STEADY_SYNC_MODE!r}, "
            f"got {steady_state_sync_mode!r}")
    progress.steps_completed.append(ShutdownStep.S6_DB_CHECKPOINT.value)


def run_s7_motion_zero_p1_last(progress: ShutdownProgress,
                                 zero_cmd_mono_ms: int,
                                 p1_exit_mono_ms: int) -> None:
    """S7 + PWR-S1: P1 emits a final zero cmd_vel, then exits. No
    cmd_vel frames may be emitted AFTER p1_exit_mono_ms."""
    # Record the zero frame at zero_cmd_mono_ms.
    progress.cmd_vel_frames.append((zero_cmd_mono_ms, 0.0, 0.0, 0.0))
    progress.p1_exited_mono_ms = p1_exit_mono_ms
    if p1_exit_mono_ms < zero_cmd_mono_ms:
        raise ValueError(
            f"P1 exit_ms={p1_exit_mono_ms} must be >= zero_cmd_ms="
            f"{zero_cmd_mono_ms} (PWR-S1: zero first, exit last)")
    progress.steps_completed.append(
        ShutdownStep.S7_MOTION_ZERO_P1_EXIT.value)


def any_cmd_vel_after_p1_exit(progress: ShutdownProgress) -> bool:
    """PWR-S1 gate: no frame may be emitted after P1 has exited."""
    if progress.p1_exited_mono_ms is None:
        return False
    return any(
        mono > progress.p1_exited_mono_ms
        for mono, *_rest in progress.cmd_vel_frames)


def assert_pwr_s2_banner(progress: ShutdownProgress) -> None:
    """PWR-S2 explicit: ack.detail + HMI banner must mention the
    next-boot unlock. Silent omission would leave the operator
    surprised by BLOCKED at next power-up."""
    banner_ok = ("unlock" in progress.hmi_banner.lower()
                 or "解锁" in progress.hmi_banner
                 or "next boot" in progress.hmi_banner.lower())
    ack_ok = any(
        "unlock" in str(v).lower() or "解锁" in str(v)
        for v in progress.ack_detail.values()
        if isinstance(v, (str, list)))
    if not banner_ok:
        raise ValueError(
            f"PWR-S2: HMI banner must mention next-boot unlock, got "
            f"{progress.hmi_banner!r}")
    if not ack_ok:
        raise ValueError(
            f"PWR-S2: ack.detail must include unlock hint, got "
            f"{progress.ack_detail!r}")
