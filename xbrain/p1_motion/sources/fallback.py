"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: fallback.py
Brief: MOT-PM-29 fence_guard(1000) veto-only + hold(100) always-alive

Description:
Two special sources on the P1 arbiter:

  fence_guard  priority 1000, VETO-ONLY -- highest priority but
               NEVER produces motion; its 'output' is only the
               ability to veto other sources. Structural guard:
               if fence_guard EVER produced non-zero vx or wz, that
               would violate its contract.

  hold         priority 100, ALWAYS-ALIVE -- lowest priority, output
               is fixed zero velocity. Ensures the domain 1 holder
               is always defined; without hold, an ARB-idle domain
               would leave chassis without cmd_vel.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FenceGuardOutput:
    """Fence guard OUTPUT is ONLY veto flags; never (vx, wz)."""
    veto_forward: bool = False
    veto_lateral: bool = False
    veto_reverse: bool = False


class FenceGuardContractError(RuntimeError):
    """fence_guard tried to produce non-veto motion."""


def build_fence_guard(veto_forward: bool = False,
                       veto_lateral: bool = False,
                       veto_reverse: bool = False) -> FenceGuardOutput:
    """Construct fence_guard output. Non-veto fields do not exist;
    this function has no way to produce vx / wz."""
    return FenceGuardOutput(
        veto_forward=veto_forward,
        veto_lateral=veto_lateral,
        veto_reverse=veto_reverse,
    )


def hold_output() -> tuple:
    """hold(100) output is FIXED zero-vel."""
    return (0.0, 0.0)
