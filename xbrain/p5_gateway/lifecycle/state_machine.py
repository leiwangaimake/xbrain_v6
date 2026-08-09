"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: state_machine.py
Brief: GWY-P5-18 P5 lifecycle SM + S8/S9 failure map

Description:
17 S8 lifecycle states (transitions listed below):

  starting  -> minimal (W-1 window)
  minimal   -> full
  full      -> degraded  (any recoverable failure)
  degraded  -> full      (auto-recover)
  full      -> stopping  (SystemCommand{poweroff})
  degraded  -> stopping  (same)
  stopping  -> stopped

W-1 window: minimal mode allows only /api/health + link probe +
event drain. All new HMI sessions rejected. This is the observation
window during Stage 4 boot.

17 S9 failure map:
  db_writer_stuck -> degraded  (rest of the process keeps running)
  cloud_disconnect -> degraded
  hmi_disconnect -> stay in current state (per-session)
  disk_full -> degraded + emit health_critical
"""

from __future__ import annotations

from enum import Enum


class GatewayState(str, Enum):
    STARTING = "starting"
    MINIMAL = "minimal"
    FULL = "full"
    DEGRADED = "degraded"
    STOPPING = "stopping"
    STOPPED = "stopped"


ALLOWED: dict = {
    GatewayState.STARTING: {GatewayState.MINIMAL},
    GatewayState.MINIMAL:  {GatewayState.FULL, GatewayState.STOPPING},
    GatewayState.FULL:     {GatewayState.DEGRADED, GatewayState.STOPPING},
    GatewayState.DEGRADED: {GatewayState.FULL, GatewayState.STOPPING},
    GatewayState.STOPPING: {GatewayState.STOPPED},
    GatewayState.STOPPED:  set(),
}


class InvalidGatewayTransition(Exception):
    pass


def transition(from_state: GatewayState,
                 to_state: GatewayState) -> GatewayState:
    if to_state not in ALLOWED[from_state]:
        raise InvalidGatewayTransition(
            f"{from_state.value!r} -> {to_state.value!r} not allowed")
    return to_state


def minimal_mode_allows(operation: str) -> bool:
    """W-1: only /api/health, event drain, and link probe permitted
    while in minimal mode."""
    return operation in {"api_health", "event_drain", "link_probe"}
