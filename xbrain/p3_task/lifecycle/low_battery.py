"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: low_battery.py
Brief: 99 U146 low-battery return -- judge state/power, pick the dock / home, build the task and the cloud alarms

Description:
The user's low-battery rule (2026-11-05 ruling 3, restated 2026-11-06, 99
U146), replacing 15 S8.2 / S8.2A's 30 % CHG-10/11 trigger and its
"critical -> park in place" tier:

  * soc_pct below trigger_soc_pct (20) -> ONE voice alarm and a cloud alarm
    every cloud_alarm_period_s (2 s) for as long as it lasts; the robot keeps
    its normal speed (the U117 x0.3 slow-down is removed);
  * target: the nearest waypoint whose name carries 充电桩 / 充电站 (type
    dock) AND that lies within common.geo.route_membership_m of some route
    (the rule that a dock must be on a route, 99 U146); none -> the waypoint
    named 返航终点站 (type home); neither -> no task, the alarm says so;
  * the cloud alarm closes ONLY when the chassis reports charging (any
    battery's `charging` true, or `charge == "charging"`; 99 U125);
  * if remain_mile_km cannot cover the drive to the target -> the alarm says
    unreachable (still driving toward it);
  * at or below stop_soc_pct (5) -> stop and lie down;
  * at boot, no home point -> one cloud report naming what is missing.

This module is the PURE half: LowBatteryMonitor turns successive state/power
bodies into episode events, pick_target chooses the destination from geo.db,
build_low_battery_row the task row. The wiring (runtime/main_wiring.py)
publishes, inserts and suspends.

What it does NOT do: drive (the injected charge / return_home task is
dispatched by the scheduler and pushed like any goto), dock (arrival at the
waypoint ends the task; the chassis's own charging flag ends the alarm), or
read a clock (now_s is passed in, monotonic, CLK-C1).

Traps:
  * Closing the alarm when soc rises above 20 %. The rule closes it on the
    chassis's CHARGING report only; a soc that wobbles over the line without
    charging would silence an alarm nothing has resolved.
  * Picking a dock off every route. The ruling makes on-route a condition of
    being a dock target: such a point is skipped, never driven to straight.
  * Treating a missing remain_mile_km as zero (an unreachable alarm for a
    chassis that simply did not report); missing means "not judged".
"""
from __future__ import annotations  # postponed annotations

import json  # mission_json
from dataclasses import dataclass  # the picked target
from typing import Any, List, Mapping, Optional, Sequence, Tuple  # plain value types

from xbrain.common.geo.goto_plan import RouteGeom, plan_goto  # travel distance (U127)
from xbrain.common.geo.polyline import distance_to_route_m  # on-route test
from xbrain.p3_task.dao.tasks_dao import TaskRow  # the injected task
from xbrain.p3_task.state.machine import InvalidTransition, apply_transition, validate_suspend_fields  # the graph

#: 11 S4.4 suspend pair for the 5 % stop (15 S8.2A CR-6: every energy-related
#: suspension is passive / low_battery).
LB_SUSPEND_KIND = "passive"  # never resumes by itself
LB_SUSPEND_REASON = "low_battery"  # the 11 S4.4 closed-set reason
#: The injected task's priority: the charge lane, as the link-loss return (95,
#: 15 S6.3 "充电触发(95)"): it preempts patrols and gotos.
LB_PRIORITY = 95  # charge lane priority


class LowBatteryConfigError(ValueError):  # refused at construction
    """A threshold missing, not positive, or out of order (CLAUDE.md 3.1)."""


def _num(v: Any) -> Optional[float]:  # tolerant numeric read
    """A real number or None."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):  # bools are not numbers here
        return None  # absent / malformed
    return float(v)  # usable


def is_charging(power: Mapping[str, Any]) -> bool:  # U125 charging criterion
    """99 U125: any battery's charging flag, or the six-state charge field."""
    if power.get("charge") == "charging":  # 11 S4.1 six-state field
        return True  # the chassis says so
    packs = power.get("list")  # 13 BAT-1 array (quadruped rt_payloads)
    if isinstance(power.get("batteries"), Mapping):  # tolerated wrapper shape
        packs = power["batteries"].get("list", packs)  # nested list form
    return any(isinstance(b, Mapping) and b.get("charging") is True  # any pack charging counts
               for b in (packs if isinstance(packs, list) else []))


class LowBatteryMonitor:  # the episode state machine
    """One low-battery episode at a time.

    update() returns the events of this frame, in order, from:
      "low"      the episode starts (soc < trigger, not charging) -- once;
      "stop"     soc <= stop threshold during the episode -- once;
      "alarm"    a cloud alarm is due (every period while the episode lasts);
      "charging" the chassis reports charging: the episode (and the alarm)
                 ends -- once.
    """

    def __init__(self, trigger_soc_pct: Any, stop_soc_pct: Any,
                 alarm_period_s: Any) -> None:
        vals = {"low_battery.trigger_soc_pct": trigger_soc_pct,  # names for the error text
                "low_battery.stop_soc_pct": stop_soc_pct,
                "low_battery.cloud_alarm_period_s": alarm_period_s}
        for name, v in vals.items():  # every key required, positive
            if _num(v) is None or float(v) <= 0.0:  # missing / null / <= 0
                raise LowBatteryConfigError("%s must be a positive number, got %r" % (name, v))
        if float(stop_soc_pct) >= float(trigger_soc_pct):  # 5 must sit below 20
            raise LowBatteryConfigError(  # out of order
                "low_battery.stop_soc_pct %r must be below trigger_soc_pct %r"
                % (stop_soc_pct, trigger_soc_pct))
        self._trigger = float(trigger_soc_pct)  # soc below this starts an episode
        self._stop = float(stop_soc_pct)  # soc at / below this stops the robot
        self._period = float(alarm_period_s)  # cloud alarm cadence
        self.low = False  # an episode is in progress
        self.stopped = False  # the 5 % stop has been done this episode
        self._last_alarm: Optional[float] = None  # when the last alarm went out

    def update(self, power: Optional[Mapping[str, Any]], now_s: float) -> List[str]:  # one frame in, events out
        """Events for this state/power body (None: nothing received -> none)."""
        if not isinstance(power, Mapping):  # no frame: nothing to say
            return []  # chassis silent: no judgement
        soc = _num(power.get("soc_pct"))  # min over present packs, or None
        out: List[str] = []  # events of this frame
        if self.low and is_charging(power):  # the only way an episode ends
            self.low, self.stopped, self._last_alarm = False, False, None  # reset for the next episode
            return ["charging"]
        if soc is None:  # unknown soc
            return out  # no reading: keep the state, judge nothing new
        if not self.low and soc < self._trigger and not is_charging(power):
            self.low = True  # a new episode
            out.append("low")
        if self.low and not self.stopped and soc <= self._stop:  # the last 5 %
            self.stopped = True  # last 5 %: stop and lie down, once
            out.append("stop")
        if self.low and (self._last_alarm is None or now_s - self._last_alarm >= self._period):  # due by the cadence
            self._last_alarm = now_s  # cadence measured from the last alarm
            out.append("alarm")
        return out  # in order: low, stop, alarm


@dataclass(frozen=True)
class Target:  # one destination
    """Where a low-battery return goes."""
    geo_id: str  # the waypoint
    name: str  # as the operator named it
    kind: str  # "dock" | "home"
    lat: float  # its position
    lon: float


def on_route(point: Tuple[float, float], routes: Sequence[RouteGeom],  # U127 membership, inclusive
             membership_m: float) -> bool:
    """True when point lies within membership_m of any route polyline (the
    U127 rule, inclusive; 99 U146 makes it a condition of a dock target)."""
    return any(distance_to_route_m(point, r.points, r.closed) <= membership_m  # any route close enough
               for r in routes if r.points)


async def list_role_points(geo_conn, wtype: str) -> List[Tuple[str, str, float, float]]:  # home / dock rows
    """(geo_id, name, lat, lon) of every live waypoint of this type, oldest first."""
    cur = await geo_conn.execute(  # live rows only
        "SELECT geo_id, name, rtk_lat, rtk_lon FROM waypoints WHERE type=? "
        "AND state='active' AND tombstone=0 ORDER BY (num IS NULL), num, id", (wtype,))
    return [(str(g), str(n), float(la), float(lo)) for g, n, la, lo in await cur.fetchall()]  # plain tuples


async def pick_target(geo_conn, pose: Optional[Tuple[float, float]],  # dock first, then home
                      routes: Sequence[RouteGeom], membership_m: float) -> Optional[Target]:
    """The nearest ON-ROUTE dock by travel distance, else the home point, else None."""
    docks = [d for d in await list_role_points(geo_conn, "dock")
             if on_route((d[2], d[3]), routes, membership_m)]  # off-route docks never qualify
    if docks:  # some dock qualifies
        if pose is None:
            best = docks[0]  # no position: the oldest, deterministic
        else:
            best = min(docks, key=lambda d: plan_goto(pose, (d[2], d[3]), routes,
                                                      membership_m).cost_m)  # what it would drive
        return Target(best[0], best[1], "dock", best[2], best[3])
    homes = await list_role_points(geo_conn, "home")  # 返航终点站 (15 S9.5.1 order)
    if homes:  # a home point exists
        h = homes[0]
        return Target(h[0], h[1], "home", h[2], h[3])
    return None  # nothing to go to: the alarm says so


def unreachable(power: Mapping[str, Any], pose: Optional[Tuple[float, float]],  # remain_mile_km vs the drive
                target: Optional[Target], routes: Sequence[RouteGeom],
                membership_m: float) -> Optional[bool]:
    """True when remain_mile_km cannot cover the drive to target; None when it
    cannot be judged (no remain_mile_km, no pose, no target)."""
    remain_km = _num(power.get("remain_mile_km"))  # chassis estimate (U125)
    if remain_km is None or pose is None or target is None:
        return None  # not judged, never "unreachable"
    need_m = plan_goto(pose, (target.lat, target.lon), routes, membership_m).cost_m  # metres the robot would drive
    return remain_km * 1000.0 < need_m  # metres the battery has vs metres to drive


def build_low_battery_row(task_id: str, submit_seq: int, target: Target, *,  # charge or return_home
                          soc_pct: Optional[float], now_mono_ms: int,
                          created_at: str) -> TaskRow:
    """The injected task: charge to that dock, or return_home to the home point."""
    mission = {"source": "auto", "reason": "low_battery", "soc_pct": soc_pct,  # why, and where to
               "params": {"waypoint_id": target.geo_id}}  # the dispatcher reads params
    return TaskRow(  # one pending row
        task_id=task_id, task_type="charge" if target.kind == "dock" else "return_home",
        state="pending", priority=LB_PRIORITY, submit_seq=submit_seq,
        mission_json=json.dumps(mission, ensure_ascii=False, separators=(",", ":")),
        total_steps=1, current_step=0, step_status_json="[]",  # one step: go there
        created_ms=now_mono_ms, updated_ms=now_mono_ms,
        source="charge",  # the charge lane (15 S4.2.1)
        trace_id="lowbatt-" + task_id, resume_policy="continue", created_at=created_at)


async def suspend_running_for_low_battery(dao, conn, now_mono_ms: int  # ES-2 shaped suspension
                                          ) -> Optional[Tuple[str, str, str]]:
    """The 5 % stop: suspend the running task (passive / low_battery); same
    transaction discipline as ES-2 (estop_suspend.py)."""
    rows = await dao.list_by_priority()  # (task_id, prio, seq, state)
    running = [tid for tid, _p, _s, state in rows if state == "running"]  # the running one
    if not running:
        return None  # nothing running: the lie-down still happens
    task_id = running[0]  # at most one (scheduler invariant)
    to_state = apply_transition("running", "suspend").to_state  # the graph agrees
    validate_suspend_fields(to_state, LB_SUSPEND_KIND, LB_SUSPEND_REASON)  # CR-8 pair
    await conn.execute("BEGIN IMMEDIATE")  # same lock timing as every P3 write
    try:
        await dao.suspend_task(task_id, LB_SUSPEND_KIND, LB_SUSPEND_REASON, now_mono_ms)  # the CR-8 pair written together
        await conn.commit()  # never leave the implicit transaction open
    except Exception:
        await conn.rollback()  # nothing half-applied
        raise
    return task_id, to_state, LB_SUSPEND_REASON  # for the publish


async def close_low_battery_task(dao, conn, task_id: Optional[str], now_mono_ms: int  # charging ends it
                                 ) -> Optional[Tuple[str, str, str]]:
    """Charging started: the low-battery task has done its job -> cancelled.

    Left alone it stays suspended (the 5 % stop suspends it) or running, and a
    later "继续" would resume the latest suspended task -- this one -- and
    drive a charging robot back towards the dock (found on the robot,
    2026-11-06, 99 U146). Already terminal / unknown id -> None, nothing done.
    Returns (task_id, from_state, to_state) for the publish."""
    if not task_id:  # no task was injected (no target)
        return None  # nothing to close
    states = {tid: st for tid, _p, _s, st in await dao.list_by_priority()}  # live rows only
    frm = states.get(task_id)  # its state, None when already terminal
    if frm is None:
        return None  # finished on its own (arrived) or cancelled by voice
    try:
        to_state = apply_transition(frm, "cancel").to_state  # the graph agrees
    except InvalidTransition:
        return None  # a state with no cancel arrow: leave it
    await conn.execute("BEGIN IMMEDIATE")  # same lock timing as every P3 write
    try:
        await conn.execute(  # cancelled; the CR-8 pair cleared like any cancel (task_apply)
            "UPDATE tasks SET state=?, suspend_kind=NULL, suspend_reason=NULL, "
            "updated_ms=? WHERE task_id=?", (to_state, now_mono_ms, task_id))
        await conn.commit()  # never leave the implicit transaction open
    except Exception:
        await conn.rollback()  # nothing half-applied
        raise
    return task_id, frm, to_state  # for the publish


__all__ = ["LowBatteryMonitor", "close_low_battery_task", "LowBatteryConfigError", "Target", "pick_target",  # the module's API
           "unreachable", "build_low_battery_row", "suspend_running_for_low_battery",
           "is_charging", "on_route", "list_role_points", "LB_PRIORITY"]
