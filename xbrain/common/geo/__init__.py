"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Cross-process geo naming rules -- which waypoint names carry a role (99 U119 / U128)

Description:
P3 derives a waypoint's type from its name when it is created or renamed
(99 U119: exactly 返航终点站 -> home, a name containing 充电桩 / 充电站 ->
dock), and P4 has to apply the SAME rule before it sends anything (99 U128:
a spoken "返航" with no home point is answered at once, a named "去1号充电站"
is a dock goto). Two copies of the rule would drift the first time somebody
adds an alias on one side only, so the one copy lives here, in the shared
layer, like xbrain/common/fence does for the fence primitives.

What it does NOT do: no database access and no lookup -- callers own their
catalogue (P3 reads geo.db, P4 reads state/geo/manifest).
"""

from xbrain.common.geo.roles import (  # noqa: F401  (re-export, the public surface)
    DOCK_NAME_KEYWORDS,
    HOME_WAYPOINT_NAME,
    is_dock_name,
    waypoint_type_for_name,
)
