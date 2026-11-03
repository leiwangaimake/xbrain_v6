"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: turn_speed.py
Brief: The robot's in-place turn speed -- a configured default, voice-adjustable in steps, an explicit value kept across restarts (99 U104)
Description:
What problem this solves
------------------------
Field test 2026-10-29 (R019): "左转90度 / 180度" turned short, left turns
especially. The p1 log shows every finished turn within 1.5 degrees; the bad
ones were stopped by the stall guard, many after ~5 degrees, because the
fixed 0.3 rad/s sits on the chassis' "twists but does not step" threshold
(0.2 only twists, measured 2026-09-15). User ruling 2026-10-30 (99 U104): turn
at the maximum by default; "机器人转速快一点 / 慢一点 / 最快 / 最慢" adjusts it,
never below the twist threshold; an explicitly set value stays, across
restarts, exactly like the device volume (99 U100 continued). User ruling
2026-11-03 after a field run (U104 continued 3): default 0.6, range 0.4-0.8,
step 0.1 -- the default is no longer the maximum.

The numbers (all from p2_core.yaml turn_speed, none in code)
------------------------------------------------------------
  max_radps      0.8  the vendor manuals give no maximum yaw rate; 0.6 was the
                      interim until the 2026-11-03 field run (0.8 tracked
                      103-104%, overshoot up to 7.4 deg on 180 deg).
  default_radps  0.6  user ruling 2026-11-03 (tracked 90-103%, overshoot
                      1.9-3.6 deg on 180 deg).
  min_radps      0.4  above the twist threshold (0.2 twists, 0.3 still stalls).
  step_radps     0.1  one "快一点 / 慢一点".

Rules
-----
  * nothing set -> default_radps; an explicit value is persisted and
    restored at the next start;
  * a stored value outside [min, max] (the config moved since) is CLAMPED
    into range with a warning, never trusted above max;
  * arithmetic runs on integer milli-rad/s, so ten "快一点" never drift to
    0.6000000001 and miss the max;
  * the file is written new + fsync + rename + directory fsync (this robot
    loses power uncleanly); an unreadable file falls back to the default.
  * the default is its own config key, default_radps (user ruling
    2026-11-03, 99 U104 continued 3: 0.6 inside [0.4, 0.8]); ~~the default is
    max_radps~~ (U104 until 2026-11-03).

  deadzone    common.spec.min_cmd_wz_radps (2026-11-03, engineering value
              pending measurement); min_radps must be strictly above it.

The discipline itself (default, steps, clamping, persistence, the deadzone
check) is stepped_setting.SteppedSetting, shared with the move speed (99
U106) since 2026-11-03; this module only names the turn speed's numbers.

Where it is used: ModeFace applies the set_turn_speed action and publishes
snapshot() in state/mode; the relmove builder fills MO-2 max_yaw_rate_radps
with effective().
"""
from __future__ import annotations  # postponed annotations

from xbrain.p2_core.mode.stepped_setting import LEVELS, SteppedSetting  # the shared discipline


class TurnSpeedSetting(SteppedSetting):
    """Default-max or explicit turn speed, persisted across P2 restarts."""

    def __init__(self, path: str, min_radps: float, max_radps: float, step_radps: float,
                 default_radps: float, deadzone_radps: float) -> None:
        """All six from the resolved p2_core.turn_speed block; a bad range is a config error."""
        super().__init__(path, key="turn_speed_radps", unit="radps", label="turn speed",
                         min_v=min_radps, max_v=max_radps, step=step_radps,
                         default=default_radps,  # 2026-11-03: its own key, no longer max
                         deadzone=deadzone_radps)


__all__ = ["TurnSpeedSetting", "LEVELS"]
