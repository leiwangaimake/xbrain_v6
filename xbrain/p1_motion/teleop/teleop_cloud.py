"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: teleop_cloud.py
Brief: MOT-PM-22 teleop_cloud (550) exit-layer veto + zero-vy + link-down zero-vel

Description:
teleop_cloud is priority 550, ABOVE nav2_proxy (600 - typo? Actually
550 sits BELOW nav2_proxy=600). Wait -- per 12 S4.1 the order is:
teleop local sources 750/800 (highest teleop) then rns 700 then
nav2_proxy 600 then teleop_cloud 550. teleop_cloud is DELIBERATELY
below Nav2 because it is a REMOTE and slow channel; cloud latency
must not preempt local nav authority.

Three exit-layer rules DIFFERENT from local teleop:
  * wz clipped to wz_blind (bounded even for cloud tel-op that has
    no local situational awareness); LOCAL teleop's wz limited but
    NOT clipped to wz_blind -- the variant catches merging the two
    paths into one limiter
  * vy REJECTED (cloud has no side awareness of obstacles); local
    teleop may issue vy
  * cloud link down (last frame > 1 s ago) -> ZERO velocity IMMEDIATELY
    (not gradual decel; fail-safe direction)
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CloudTeleopFrame:
    vx_mps: float
    vy_mps: float
    wz_radps: float
    arrived_mono_ms: int


class CloudTeleopReject(RuntimeError):
    """A cloud teleop frame violated a hard constraint."""


def clamp_and_check(
    frame: CloudTeleopFrame,
    now_mono_ms: int,
    obstacle_avoid_max_mps: float = 0.5,
    wz_blind_radps: float = 0.5,
    link_timeout_ms: int = 1000,
) -> tuple:
    """Apply cloud-specific rules; return (vx, vy, wz).

    * vy != 0 -> reject entire frame (cloud lacks side-awareness)
    * vx clamped to obstacle_avoid.max_mps (no fast cloud motion)
    * wz CLIPPED to wz_blind (not limited: cloud is blind)
    * link stale -> zero-vel

    Variant: any impl that lets cloud vy > 0 through, or clips
    local-teleop wz to wz_blind (which is a cloud-only rule),
    breaks the separation."""
    # Link-down check FIRST.
    if now_mono_ms - frame.arrived_mono_ms > link_timeout_ms:
        return (0.0, 0.0, 0.0)

    if frame.vy_mps != 0.0:
        raise CloudTeleopReject(
            "cloud vy=%f rejected; cloud has no side-obstacle awareness"
            % frame.vy_mps)

    vx = max(-obstacle_avoid_max_mps,
             min(obstacle_avoid_max_mps, frame.vx_mps))
    wz = max(-wz_blind_radps, min(wz_blind_radps, frame.wz_radps))
    return (vx, 0.0, wz)
