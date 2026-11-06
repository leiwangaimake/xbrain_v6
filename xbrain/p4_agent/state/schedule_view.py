"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: schedule_view.py
Brief: 99 U144 -- the scheduled tasks as P4 can speak them: list, name, same-minute clash, time read back

Description:
The problem. A timed task ("下午3点去1号测试点") could not be cancelled by
voice: "取消任务" acts on the current or the latest suspended task (11 S7.2,
99 U111), never on one still waiting for its time. And nothing stopped two
timed tasks at the same minute. The user's rulings of 2026-11-06: timed tasks
must be cancellable / deletable by voice, and only one may be scheduled per
time -- a second is refused with a spoken reason.

What it does. Pure functions over the state P4 already caches:
  * scheduled_items(task_state, manifest_items, now, tz) -> the queue entries
    in state "scheduled" (P3 adds not_before_ts / target_waypoint_id / target_route_id to
    each queue item since 99 U144), each with a speakable time and label,
    earliest first;
  * clash(items, epoch) -> the item already in that MINUTE, or None;
  * at_minute(items, epoch) -> the items in the minute a spoken time names;
  * spoken_epoch(epoch, now, tz) -> "今天15点" / "明天8点30分".

Boundaries. It reads no clock (now is passed in) and sends nothing; P3 is
still the authority -- it refuses a same-minute submit from any channel
(E_BUSY schedule_conflict). This module is P4's pre-check, so the operator
hears the reason instead of "好的" followed by a refusal.

Traps:
  * Comparing seconds. A spoken time names a minute; "下午3点" said twice is
    the same slot even when one arrived through a delay form a few seconds
    off. Epoch minutes and local minutes coincide (zone offsets are whole
    minutes), so floor(epoch / 60) is the slot.
  * Naming by id. The operator never said "w-1"; the label resolves ids to
    the catalogue names, and says "目标" when a name is gone.
"""
from __future__ import annotations  # postponed annotations

from datetime import datetime  # local time read back
from typing import Any, Dict, List, Mapping, Optional  # plain value types
from zoneinfo import ZoneInfo  # the site zone

#: The type words of a label; the target name is appended where it applies.
_TYPE_WORD = {"goto": "去", "charge": "去", "patrol": "沿", "return_home": "返航"}


def _minute(epoch: float) -> int:
    """The minute slot of an epoch time (see Traps)."""
    return int(float(epoch) // 60)  # whole minutes since the epoch


def _local(epoch: float, tz: Optional[str]) -> datetime:
    """The epoch time in the site zone (the system zone when unset)."""
    if tz:
        return datetime.fromtimestamp(float(epoch), ZoneInfo(tz))  # site zone
    return datetime.fromtimestamp(float(epoch))  # system zone, as P4 schedules


def spoken_epoch(epoch: float, now: float, tz: Optional[str]) -> str:
    """"今天15点" / "明天8点30分" / "11月9日15点" for a scheduled time."""
    at, today = _local(epoch, tz), _local(now, tz)  # both in the site zone
    days = (at.date() - today.date()).days  # 0 today, 1 tomorrow, ...
    day = {0: "今天", 1: "明天", 2: "后天"}.get(days, "%d月%d日" % (at.month, at.day))
    return day + "%d点" % at.hour + ("%d分" % at.minute if at.minute else "")


def _label(item: Mapping[str, Any], names: Mapping[str, str]) -> str:
    """What the task does, in words ("去1号测试点", "沿1号路径巡逻", "返航")."""
    kind = item.get("type")  # 11 S4.4 queue type
    if kind == "return_home":
        return "返航"  # no target to name
    if kind == "patrol":
        route = names.get(item.get("target_route_id") or "")  # the route name
        return "沿%s巡逻" % route if route else "再巡逻"  # B04 has no route yet
    if kind == "charge" and not item.get("target_waypoint_id"):
        return "去充电站"  # nearest dock, picked when due
    target = names.get(item.get("target_waypoint_id") or "", "目标")  # a gone name
    return _TYPE_WORD.get(kind, "执行") + target  # goto / named dock / other


def scheduled_items(task_state: Any, manifest_items: Any, now: float,
                    tz: Optional[str]) -> List[Dict[str, Any]]:
    """The scheduled queue entries, earliest first, each as
    {task_id, epoch, when, label}. Entries without a time are skipped (a
    "scheduled" task always has one; one without cannot be named)."""
    body = task_state.get("data", task_state) if isinstance(task_state, Mapping) else None
    queue = body.get("queue") if isinstance(body, Mapping) else None  # 11 S4.4
    names = {str(i.get("geo_id")): str(i.get("name")) for i in (manifest_items or [])
             if isinstance(i, Mapping) and i.get("geo_id")}  # id -> spoken name
    out: List[Dict[str, Any]] = []
    for item in queue if isinstance(queue, list) else []:
        if not isinstance(item, Mapping) or item.get("state") != "scheduled":
            continue  # only tasks still waiting for their time
        nb = item.get("not_before_ts")  # 99 U144 field
        if isinstance(nb, bool) or not isinstance(nb, (int, float)):
            continue  # cannot be named by time
        out.append({"task_id": item.get("task_id"), "epoch": float(nb),
                    "when": spoken_epoch(nb, now, tz), "label": _label(item, names)})
    return sorted(out, key=lambda i: i["epoch"])  # earliest first


def clash(items: List[Dict[str, Any]], epoch: float) -> Optional[Dict[str, Any]]:
    """The scheduled item in the same minute as epoch, or None (99 U144)."""
    return next((i for i in items if _minute(i["epoch"]) == _minute(epoch)), None)


def at_minute(items: List[Dict[str, Any]], epoch: float) -> List[Dict[str, Any]]:
    """Every scheduled item in the minute epoch names (at most one, by the
    one-per-minute rule; a list so a legacy duplicate is not hidden)."""
    return [i for i in items if _minute(i["epoch"]) == _minute(epoch)]


__all__ = ["scheduled_items", "clash", "at_minute", "spoken_epoch"]
