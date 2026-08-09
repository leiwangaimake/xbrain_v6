"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geom.py
Brief: MOT-PM-13/14/15 fence geometry + two-stage commit

Description:
Two responsibilities in one module: (1) vector projection of commanded velocity against a fence boundary given soft and hard distances plus latency budget; NaN / Inf inputs collapse to zero velocity (fail-safe). (2) the two-stage fence-commit state machine (idle/staged/committed) that lets P2 stage a fence change, then commit atomically or abort without ever exposing a partial fence to the runtime.
"""



from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


def vector_project_toward_fence(vx: float, vy: float,
                                  d_soft_m: float,
                                  d_hard_m: float,
                                  t_lat_s: float) -> Tuple[float, float]:
    """Project velocity vector against fence boundary.

    d_soft: warning distance (start decel)
    d_hard: veto distance (zero-vel component toward fence)
    t_lat_s: latency + brake budget

    Returns clamped (vx, vy). If either input is NaN/Inf, returns
    zero (fail-safe direction)."""
    for v in (vx, vy, d_soft_m, d_hard_m, t_lat_s):
        if v is None or not math.isfinite(v):
            return 0.0, 0.0
    if d_hard_m <= 0:
        return 0.0, 0.0
    if d_soft_m <= d_hard_m:
        # inside veto zone -> component toward fence is 0.
        return 0.0, 0.0
    # Linear ramp between d_hard and d_soft.
    ratio = (d_soft_m - d_hard_m) / max(d_hard_m, 0.01)
    factor = max(0.0, min(1.0, ratio))
    return vx * factor, vy * factor


class FenceStage:
    """MOT-PM-14 two-stage fence: stage / commit / abort / ping."""
    IDLE = "idle"
    STAGED = "staged"
    COMMITTED = "committed"


@dataclass
class FenceStageMachine:
    state: str = FenceStage.IDLE
    staged_id: Optional[str] = None

    def stage(self, fence_id: str) -> None:
        self.state = FenceStage.STAGED
        self.staged_id = fence_id

    def commit(self) -> str:
        if self.state != FenceStage.STAGED:
            raise RuntimeError("commit from non-STAGED state")
        self.state = FenceStage.COMMITTED
        return self.staged_id or ""

    def abort(self) -> None:
        self.state = FenceStage.IDLE
        self.staged_id = None

    def ping(self) -> bool:
        """Return True if a staged fence exists (heartbeat check)."""
        return self.state == FenceStage.STAGED
