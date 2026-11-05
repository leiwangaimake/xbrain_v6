"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: roles.py
Brief: The name -> role rule for waypoints: 返航终点站 is home, 充电桩 / 充电站 is dock (99 U119)

Description:
The user ruled on 2026-11-05 (99 U119) that the operator names the two role
points: a waypoint named exactly 返航终点站 is THE return-home point, and a
waypoint whose name contains 充电桩 or 充电站 is a charging point. P3 stores
the type from this rule; P4 uses it to answer "返航" without a home point and
to recognise a named dock goto. Moved here from p3_task/geo/waypoint_kind.py
the same day (99 U128) so both processes read one rule.

Traps:
  * Matching 返航终点 as a substring would make 返航终点站2 or 旧返航终点站 a
    home too; the ruling names one exact point, so equality is used.
  * The dock rule is a substring on purpose: 1号充电桩, 东门充电站 are all docks.
"""

from __future__ import annotations

from typing import Optional  # plain values

#: The exact name of the return-home point (99 U119, the user's words).
HOME_WAYPOINT_NAME = "返航终点站"
#: Name fragments that make a waypoint a charging point (99 U119).
DOCK_NAME_KEYWORDS = ("充电桩", "充电站")


def is_dock_name(name: str) -> bool:
    """True when the name declares a charging point (substring rule)."""
    return any(k in name for k in DOCK_NAME_KEYWORDS)  # 1号充电桩, 东门充电站


def waypoint_type_for_name(name: str) -> Optional[str]:
    """'home' / 'dock' when the name says so (99 U119), else None -- the
    caller keeps whatever type it would have used."""
    if name == HOME_WAYPOINT_NAME:  # the one named point, equality only
        return "home"  # 15 S9.5.1 home row
    if is_dock_name(name):  # 充电桩 / 充电站 anywhere
        return "dock"  # a charging point
    return None  # the name says nothing about the type
