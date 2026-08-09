"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: pose_assembly.py
Brief: MOT-PM-16 odometry -> state/pose assembly (PS-1..PS-6)

Description:
Every 20 Hz tick the P1 loop produces TWO messages that carry the
same motion snapshot: rt/motion/cmd_vel.gate (published to
chassis_relay) and state/pose.motion (published to Zenoh general
plane). PS-4 requires these two to be BYTE-FOR-BYTE identical in
the shared fields. Achieved by building ONE MotionSnapshot object
and serialising it twice; any implementation that computes the
fields twice invites drift where a health-factor injected between
the two calls silently changes only one side.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class MotionSnapshot:
    """One tick's motion snapshot. Serialised into BOTH cmd_vel.gate
    and state/pose.motion; the two carry identical values."""
    vx_mps: float
    wz_radps: float
    speed_factor: float
    limiter: str            # closed-set limiter value
    heading_deg: float
    h_factor: float         # heading confidence [0, 1]
    rtk_factor: float       # RTK confidence [0, 1]
    gen: int                # arbiter grant generation


def to_cmd_vel_gate(snap: MotionSnapshot) -> dict:
    """Serialise for rt/motion/cmd_vel.gate. Same fields as
    to_pose_motion; guaranteed identical by construction."""
    return {
        "vx": snap.vx_mps,
        "wz": snap.wz_radps,
        "speed_factor": snap.speed_factor,
        "limiter": snap.limiter,
        "heading_deg": snap.heading_deg,
        "h_factor": snap.h_factor,
        "rtk_factor": snap.rtk_factor,
        "gen": snap.gen,
    }


def to_pose_motion(snap: MotionSnapshot) -> dict:
    """Serialise for state/pose.motion. Same shape as to_cmd_vel_gate.
    Identity is not a coincidence; it is required by PS-4."""
    return {
        "vx": snap.vx_mps,
        "wz": snap.wz_radps,
        "speed_factor": snap.speed_factor,
        "limiter": snap.limiter,
        "heading_deg": snap.heading_deg,
        "h_factor": snap.h_factor,
        "rtk_factor": snap.rtk_factor,
        "gen": snap.gen,
    }
