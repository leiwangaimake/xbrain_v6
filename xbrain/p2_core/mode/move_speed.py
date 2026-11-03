"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: move_speed.py
Brief: The robot's move speed for forward / back / strafe -- default 0.2 m/s, voice-adjustable in steps, an explicit value kept across restarts (99 U106)
Description:
What problem this solves
------------------------
Field test 2026-11-03: forward / back ran at 0.2 m/s and strafe at 0.15 m/s,
both set as environment variables in the start script, so changing them meant
editing a script and restarting p1. At 0.15 the chassis strafed at only 0.035-
0.07 m/s. User ruling 2026-11-03 (99 U106): forward, back and both strafes use
ONE move speed, 0.2 m/s by default, set by cloud voice / text like the volume
and the turn speed ("移动速度快一点 / 慢一点 / 最快 / 最慢"); an explicit value
is persisted and can be set again any time; the numbers live in a config file.

The numbers (all from p2_core.yaml move_speed, none in code)
------------------------------------------------------------
  default_mps  0.2  user ruling 2026-11-03.
  min_mps      0.1  user ruling 2026-11-03; must be strictly above both
  max_mps      0.5  linear control deadzones (common.spec.min_cmd_vx_mps /
  step_mps     0.1  min_cmd_vy_mps), else P2 refuses to start.

The discipline (default, steps, clamping, persistence, the deadzone check) is
stepped_setting.SteppedSetting, shared with the turn speed.

Where it is used: ModeFace applies the set_move_speed action and publishes
snapshot() in state/mode; the relmove builder fills MO-2 max_speed_mps with
effective(); p1's basic-motion executor moves at it.
"""
from __future__ import annotations  # postponed annotations

from xbrain.p2_core.mode.stepped_setting import LEVELS, SteppedSetting  # the shared discipline


class MoveSpeedSetting(SteppedSetting):
    """Default or explicit move speed (forward / back / strafe), persisted across P2 restarts."""

    def __init__(self, path: str, min_mps: float, max_mps: float, step_mps: float,
                 default_mps: float, deadzone_mps: float) -> None:
        """All from the resolved p2_core.move_speed block; deadzone_mps is the larger of
        the two linear control deadzones (the caller takes the max). A bad range
        is a config error."""
        super().__init__(path, key="move_speed_mps", unit="mps", label="move speed",
                         min_v=min_mps, max_v=max_mps, step=step_mps,
                         default=default_mps, deadzone=deadzone_mps)


__all__ = ["MoveSpeedSetting", "LEVELS"]
