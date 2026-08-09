"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: shutdown.py
Brief: MOT-RD-8 RNS suppression + shutdown 5 branches + RNS-A3 arbiter discipline

Description:
20 §10 RNS shutdown branches. RNS returns None (no candidate) in
FIVE distinct scenarios:
  S1 grid_stale                grid_age_ms > threshold
  S2 rtk_degraded              can't compensate world<->robot frame
  S3 fence_edge                inside D_slow of fence
  S4 recording_mode            geometry_recording active
  S5 rns_module_dead           internal module fault

RNS-A3: RNS output is CANDIDATE, not command. Whether it holds
the arbiter is decided by P1's source arbiter based on priority
and other sources' activity. RNS MUST NOT try to force itself into
the holder position.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class RnsShutdownReason(str, Enum):
    GRID_STALE = "grid_stale"
    RTK_DEGRADED = "rtk_degraded"
    FENCE_EDGE = "fence_edge"
    RECORDING_MODE = "recording_mode"
    MODULE_DEAD = "module_dead"


@dataclass(frozen=True)
class ShutdownDecision:
    """None-return reason. Emitted for observability."""
    shutdown: bool
    reason: str = ""


def evaluate_shutdown(
    grid_age_ms: int,
    grid_stale_threshold_ms: int,
    rtk_degraded: bool,
    inside_fence_edge: bool,
    recording_active: bool,
    module_healthy: bool,
) -> ShutdownDecision:
    """Check all 5 branches in fixed order; first hit wins."""
    if not module_healthy:
        return ShutdownDecision(True, RnsShutdownReason.MODULE_DEAD.value)
    if recording_active:
        return ShutdownDecision(True, RnsShutdownReason.RECORDING_MODE.value)
    if inside_fence_edge:
        return ShutdownDecision(True, RnsShutdownReason.FENCE_EDGE.value)
    if rtk_degraded:
        return ShutdownDecision(True, RnsShutdownReason.RTK_DEGRADED.value)
    if grid_age_ms > grid_stale_threshold_ms:
        return ShutdownDecision(True, RnsShutdownReason.GRID_STALE.value)
    return ShutdownDecision(False)
