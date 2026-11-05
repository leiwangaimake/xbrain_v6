"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: waypoint_kind.py
Brief: Waypoint type from its name (home / dock) and the return-home target lookup (99 U119)

Description:
What problem this solves. 15 S9.5.1 defines the return_home destination as
"the waypoints row with type = 'home' (smallest num when several)", but no
capture path ever wrote anything but type = 'poi': voice F06 hard-coded it
(teach/runtime.py) and cmd/geo create defaulted to it (ingest/geo_object.py).
So no home point could exist, and the link-loss return_home of 11 S4.6.4 /
15 S11.2 was dispatched into `running` with nothing pushed -- measured
2026-11-04: rh-7899-0 held `running` for hours and blocked every task below
priority 95.

The user ruled on 2026-11-05 (99 U119) that the operator names these points:
  - a waypoint named exactly 返航终点站 is THE return-home point (type home);
  - a waypoint whose name contains 充电桩 or 充电站 is a charging point
    (type dock), e.g. 1号充电桩.
The type is derived from the NAME when the point is created or renamed (the
user chose "save-time by name" over "runtime string match": a type column
survives a later display rename only if the rule is re-applied, which rename
does here too). Runtime lookups then read the type column, as 15 S9.5.1 says.

Return target (U119, the user's rule 4 falling back to rule 3):
  1. the active home point (smallest num, then oldest id);
  2. none -> the active dock point nearest the robot (straight-line distance
     from the last known pose; no pose -> the oldest dock);
  3. none of either -> None. The caller fails the task with E_NOT_FOUND
     (15 S9.5.1 "无 home 行时任务前置校验 E_NOT_FOUND"), never leaves it
     running.

2026-11-05 (99 U128): the name rule itself moved to xbrain/common/geo (P4
applies it too) and is re-exported here; the dock choice is now by
TRAVEL distance (route-aware, the B01 plan of 99 U127) rather than straight
line; and the dock fallback is explicit (allow_dock) -- the link-loss
return keeps it (U119 rule 4), a spoken "返航" does not (U128: with no home
point it is answered "not defined", never sent to a dock silently).

What it does NOT do. It does not decide WHEN to return (link loss is
lifecycle/link_loss.py, low battery is the charge trigger), it does not push
anything (runtime/route_push_runtime.push_goto_for_task does, with the
returned geo_id as params.waypoint_id), and it does not enforce "a dock point
must lie on a route" (that is the low-battery batch, not built yet).

Traps:
  - Matching 返航终点 as a substring would make 返航终点站2 or 旧返航终点站
    a home too; the ruling names one exact point, so equality is used.
  - Reading type = 'home' rows without state = 'active' AND tombstone = 0
    sends the robot to a deleted or disabled point. (tombstone = 0 is
    redundant today: the delete applier writes state = 'deleted' and
    tombstone = 1 together, so dropping it is an EQUIVALENT mutant no test
    can kill -- kept as a guard in case the two ever drift apart.)
"""

from __future__ import annotations

import math  # haversine when no route plan is configured
from typing import Optional, Sequence, Tuple  # plain values

from xbrain.common.geo import (  # noqa: F401  (re-exported for existing importers)
    DOCK_NAME_KEYWORDS,
    HOME_WAYPOINT_NAME,
    waypoint_type_for_name,
)
from xbrain.p3_task.route.route_plan import RouteGeom, plan_goto  # travel distance (U127)

#: Mean Earth radius for the straight-line distance (metres).
_EARTH_R_M = 6371000.0


def renamed_waypoint_type(new_name: str, current_type: str) -> Optional[str]:
    """The type a waypoint must take after a rename to new_name, or None for
    "leave the column alone". A rename INTO a rule name sets it; a rename OUT
    of one (a home / dock whose new name no longer says so) drops back to poi
    so the old role does not linger under an unrelated name."""
    by_name = waypoint_type_for_name(new_name)  # what the new name says
    if by_name is not None:  # the new name carries a role
        return by_name if by_name != current_type else None  # set only on change
    if current_type in ("home", "dock"):  # the role came from the old name
        return "poi"  # no longer named as one
    return None  # gate / poi / ... untouched


def _distance_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Great-circle distance between two (lat, lon) in degrees, metres."""
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))  # to radians
    h = (math.sin((la2 - la1) / 2.0) ** 2
         + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2.0) ** 2)  # haversine
    return 2.0 * _EARTH_R_M * math.asin(min(1.0, math.sqrt(h)))  # clamp rounding


async def nearest_dock(geo_conn, pose: Optional[Tuple[float, float]],
                       routes: Sequence[RouteGeom],
                       membership_m: Optional[float]) -> Optional[str]:
    """The active dock point the robot reaches by the SHORTEST TRAVEL
    distance (99 U128, B09 "去充电站" with no name and the U119 fallback).

    Travel distance = the U127 goto plan's length (route-aware) when
    membership_m is configured; straight-line distance otherwise (tests that
    predate U127). No pose -> the oldest dock (a lookup must not invent a
    position). None when there is no dock point at all."""
    cur = await geo_conn.execute(
        "SELECT geo_id, rtk_lat, rtk_lon FROM waypoints WHERE type='dock' "
        "AND state='active' AND tombstone=0 ORDER BY id")  # every live dock, oldest first
    docks = list(await cur.fetchall())  # small table
    if not docks:  # no dock point defined
        return None  # caller decides what that means
    if pose is None:  # no position to measure from
        return str(docks[0][0])  # oldest, deterministic

    def cost(d) -> float:
        target = (float(d[1]), float(d[2]))  # the dock's position
        if membership_m is None:  # no route plan configured
            return _distance_m(pose, target)  # straight line
        return plan_goto(pose, target, routes, membership_m).cost_m  # what the robot would drive

    return str(min(docks, key=cost)[0])  # shortest travel


async def resolve_return_target(geo_conn, pose: Optional[Tuple[float, float]], *,
                                allow_dock: bool,
                                routes: Sequence[RouteGeom],
                                membership_m: Optional[float]
                                ) -> Optional[Tuple[str, str]]:
    """(geo_id, 'home' | 'dock') for a return_home task, or None.

    home first (15 S9.5.1: smallest num, NULL num last, then oldest); then,
    ONLY when allow_dock, the dock with the shortest travel distance
    (nearest_dock). allow_dock has no default on purpose: the link-loss
    return passes True (U119 rule 4), a spoken return passes False (U128) --
    a default would let a new caller inherit the wrong one silently."""
    cur = await geo_conn.execute(
        "SELECT geo_id FROM waypoints WHERE type='home' AND state='active' "
        "AND tombstone=0 ORDER BY (num IS NULL), num, id LIMIT 1")  # 15 S9.5.1
    row = await cur.fetchone()  # at most one
    if row is not None:  # a home point exists
        return str(row[0]), "home"  # rule 4: go home
    if not allow_dock:  # a spoken return: home or nothing (U128)
        return None  # caller fails it with E_NOT_FOUND
    dock = await nearest_dock(geo_conn, pose, routes, membership_m)  # U119 fallback
    return (dock, "dock") if dock is not None else None  # or nothing at all
