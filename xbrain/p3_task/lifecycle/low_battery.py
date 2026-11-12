"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: low_battery.py
Brief: 99 U146/U157 low-battery return -- judge state/power, pick the dock / home, build the task, the cloud alarms and the held set

Description:
The user's low-battery rule (2026-11-05 ruling 3, restated 2026-11-06, 99
U146), replacing 15 S8.2 / S8.2A's 30 % CHG-10/11 trigger and its
"critical -> park in place" tier, with the end and resume rules of 99 U157
(user ruling 2026-11-10):

  * soc_pct below trigger_soc_pct (20) -> ONE voice alarm and a cloud alarm
    every cloud_alarm_period_s (2 s) for as long as the episode lasts; the
    robot keeps its normal speed (the U117 x0.3 slow-down is removed);
  * target: the nearest waypoint whose name carries 充电桩 / 充电站 (type
    dock) AND that lies within common.geo.route_membership_m of some route
    (the rule that a dock must be on a route, 99 U146); none -> the waypoint
    named 返航终点站 (type home); neither -> no task, the alarm says so;
  * the chassis reports charging (any battery's `charging` true, or
    `charge == "charging"`; 99 U125) -> the episode does NOT end (U157): it
    turns into the charging phase, whose 2 s alarm says "charging" instead of
    "low battery", and the injected task is cancelled (it has arrived);
  * the episode ends -- and only then may the tasks it interrupted resume by
    themselves -- when soc_pct reaches charge_full_soc_pct (100) in the
    charging phase, or on a battery swap (a pack slot observed empty during
    the episode for at least swap_absent_min_s is present again; judged on the
    soc of that moment) with soc_pct at or above swap_resume_soc_pct (80). A swap that leaves soc_pct between trigger and swap_resume ends the
    alarm but the interrupted tasks wait for a person (U157 ruling 1);
  * unplugged before full: below the trigger -> back to the low phase (a new
    return task is injected); otherwise the interrupted phase, alarmed every
    2 s, until charging resumes, a swap, or full (U157 ruling 2);
  * restart protection, two layers: the episode record in task.db memory, and
    charging_below_full() read straight off the chassis -- while charging below
    full no preempted task resumes by itself, episode or not;
  * a cold swap (2026-11-11 ruling): restored in the low phase, the first soc
    reading is at / above the trigger, not charging, and at least
    cold_swap_min_rise_pct above the soc the record holds -> treated as a swap,
    same 80 % split;
  * a hot swap the presence rule missed (99 U171, user ruling 2026-11-12):
    in the low or interrupted phase, not charging, soc at / above the
    trigger and at least cold_swap_min_rise_pct above the lowest not-charging
    soc of this episode -> treated as a swap, same 80 % split. Measured
    2026-11-11 21:40-21:41: a hot swap read 14 -> 100 in one step with no
    `present: false` frame long enough, and the episode kept alarming every
    2 s at 100 % until 22:51. The base resets to the current soc on every
    charging frame, so what a charger adds is never read as a swap;
  * if remain_mile_km cannot cover the drive to the target -> the alarm says
    unreachable (still driving toward it);
  * at or below stop_soc_pct (5) -> stop and lie down;
  * at boot, no home point -> one cloud report naming what is missing.

This module is the PURE half: LowBatteryMonitor turns successive state/power
bodies into episode events, pick_target chooses the destination from geo.db,
build_low_battery_row the task row, the episode record round-trips through
task.db memory. The wiring (runtime/main_wiring.py) publishes, inserts,
suspends and passes the held set to the scheduler (schedule/driver.py).

What it does NOT do: drive (the injected charge / return_home task is
dispatched by the scheduler and pushed like any goto), dock (arrival at the
waypoint ends the task; the chassis's own charging flag starts the charging
phase), stand the robot up after a charge (an operator does, U157 ruling 4),
or read a clock (now_s is passed in, monotonic, CLK-C1).

Traps:
  * Closing the episode when soc rises above 20 % on its own. A soc that
    wobbles over the line without charging would silence an alarm nothing has
    resolved; the swap rule therefore also needs a pack to have gone absent
    and come back (99 U154), which a wobble never produces.
  * Ending the episode when charging STARTS (the U146 rule until 2026-11-10).
    The scheduler resumes a preempted task as soon as nothing is running, so
    the moment the return task arrived the interrupted task drove the robot
    away from the dock at 19 % (field log 2026-11-10 20:18:33). The held set
    is what stops that; it must outlive a P3 restart, hence the memory row.
  * Requiring the swapped pack's own level to be high. soc_pct is the minimum
    over the PRESENT packs (13 V-68), so with two packs the episode closes
    only once neither present pack is below the threshold -- swapping one of
    two low packs leaves the other one low, and the episode stays.
  * Picking a dock off every route. The ruling makes on-route a condition of
    being a dock target: such a point is skipped, never driven to straight.
  * Treating a missing remain_mile_km as zero (an unreachable alarm for a
    chassis that simply did not report); missing means "not judged".
"""
from __future__ import annotations  # postponed annotations

import json  # mission_json
import math  # isfinite: NaN / inf are not values
from dataclasses import dataclass  # the picked target
from typing import Any, Dict, List, Mapping, Optional, Sequence, Set, Tuple  # plain value types

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
    """A finite real number or None. NaN / inf count as absent: a NaN alarm
    period passed every `<= 0` check and then silenced the alarm for good
    (`now - last >= nan` is never true), and a NaN soc compares False with
    every threshold (2026-11-11 audit)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):  # bools are not numbers here
        return None  # absent / malformed
    if not math.isfinite(float(v)):
        return None  # NaN / inf: not a usable value
    return float(v)  # usable


def _packs(power: Mapping[str, Any]) -> List[Mapping[str, Any]]:  # 13 BAT-1 per-pack entries
    """The per-pack entries of a state/power body (13 BAT-1 `list`), [] when absent."""
    packs = power.get("list")  # 13 BAT-1 array (quadruped rt_payloads)
    if isinstance(power.get("batteries"), Mapping):  # tolerated wrapper shape
        packs = power["batteries"].get("list", packs)  # nested list form
    return [b for b in (packs if isinstance(packs, list) else []) if isinstance(b, Mapping)]


def is_charging(power: Mapping[str, Any]) -> bool:  # U125 charging criterion
    """99 U125: any battery's charging flag, or the six-state charge field."""
    if power.get("charge") == "charging":  # 11 S4.1 six-state field
        return True  # the chassis says so
    return any(b.get("charging") is True for b in _packs(power))  # any pack charging counts


#: The episode phases (99 U157). idle = no episode.
PHASE_IDLE, PHASE_LOW, PHASE_CHARGING, PHASE_INTERRUPTED = "idle", "low", "charging", "interrupted"
_PHASES = (PHASE_IDLE, PHASE_LOW, PHASE_CHARGING, PHASE_INTERRUPTED)  # closed set


def charging_below_full(power: Optional[Mapping[str, Any]], charge_full_soc_pct: float,
                        stable_charging: Optional[bool] = None) -> bool:
    """The chassis-side resume guard (99 U157, the plan's restart protection):
    True while the chassis reports charging and soc_pct is below the full
    threshold (or unknown). While True the scheduler resumes no preempted task
    by itself -- whatever the episode record says. It is the layer that does
    not depend on P3 memory: a charge started by hand (no episode), or a
    record lost before it was written, still keeps a charging robot on the
    dock. No frame -> False (nothing is known to be charging).

    stable_charging is the debounced flag (ChargingDebouncer, 15 S8.5 CD-1).
    Either the raw bit OR the debounced flag blocks: a one-frame drop of the
    raw bit while charging must not lift the guard for that frame (2026-11-11
    audit), and a fresh raw true blocks at once (blocking is the safe side)."""
    if not isinstance(power, Mapping):
        return False
    if not (is_charging(power) or stable_charging is True):
        return False
    soc = _num(power.get("soc_pct"))
    return soc is None or soc < float(charge_full_soc_pct)


class ChargingDebouncer:  # 15 S8.5 CD-1 for the low-battery rule (99 U157)
    """The chassis charging flag, debounced (15 S8.5 CD-1, adopted for U157 by
    the user 2026-11-11): charging counts once the raw flag has held true for
    plug_confirm_s, and stops counting once it has held false for
    unplug_confirm_s. `stable` is None until either has happened -- right after
    a start nothing is known yet, and a False there would read a charging robot
    as unplugged. Plug-in moments and a loose contact make the raw bit flicker;
    judged on one frame, a flicker cancelled the return task (false charging)
    or dropped the charge into "interrupted" and lifted the resume guard for a
    frame (false unplug)."""

    def __init__(self, plug_confirm_s: float, unplug_confirm_s: float) -> None:
        self._plug = float(plug_confirm_s)  # raw true this long -> charging
        self._unplug = float(unplug_confirm_s)  # raw false this long -> not charging
        self.stable: Optional[bool] = None  # unknown until confirmed either way
        self.raw = False  # the last raw reading
        self._since: Optional[float] = None  # when the raw value last changed

    def update(self, raw: bool, now_s: float) -> Optional[bool]:
        """Feed one raw reading; returns the debounced value (None = unknown)."""
        if self._since is None or raw != self.raw:
            self.raw, self._since = raw, now_s  # a change restarts the clock
        held_for = now_s - self._since
        if raw and self.stable is not True and held_for >= self._plug:
            self.stable = True
        elif not raw and self.stable is not False and held_for >= self._unplug:
            self.stable = False
        return self.stable


class LowBatteryMonitor:  # the episode state machine
    """One low-battery episode at a time, in phases (99 U157):

        idle --soc < trigger, not charging--> low
        low / interrupted --charging--> charging
        charging --unplugged, soc < trigger--> low      (a new return task)
        charging --unplugged, soc >= trigger--> interrupted
        interrupted --soc < trigger--> low
        charging / interrupted --soc >= charge_full--> idle   "charge_full"
        any episode phase --swap, soc >= swap_resume--> idle  "swapped"
        any episode phase --swap, trigger <= soc < swap_resume--> idle  "swapped_partial"

    update() returns the events of this frame, in order, from:
      "low"             the low phase starts (episode start, or back from
                        charging / interrupted below the trigger);
      "stop"            soc <= stop threshold in the low phase -- once per
                        low phase;
      "charging"        the charging phase starts;
      "interrupted"     unplugged at or above the trigger, before full;
      "alarm"           a cloud alarm is due (every period while an episode
                        lasts; the wiring words it by `phase`);
      "charge_full"     ends: full while charging (or unplugged at full);
      "swapped"         ends: a swap with soc >= swap_resume;
      "swapped_partial" ends: a swap with trigger <= soc < swap_resume --
                        the interrupted tasks wait for a person.
    """

    def __init__(self, trigger_soc_pct: Any, stop_soc_pct: Any,
                 alarm_period_s: Any, charge_full_soc_pct: Any,
                 swap_resume_soc_pct: Any, swap_absent_min_s: Any,
                 cold_swap_min_rise_pct: Any, plug_confirm_s: Any,
                 unplug_confirm_s: Any) -> None:
        vals = {"low_battery.trigger_soc_pct": trigger_soc_pct,  # names for the error text
                "low_battery.stop_soc_pct": stop_soc_pct,
                "low_battery.cloud_alarm_period_s": alarm_period_s,
                "low_battery.charge_full_soc_pct": charge_full_soc_pct,
                "low_battery.swap_resume_soc_pct": swap_resume_soc_pct,
                "low_battery.swap_absent_min_s": swap_absent_min_s,
                "low_battery.cold_swap_min_rise_pct": cold_swap_min_rise_pct,
                "low_battery.plug_confirm_s": plug_confirm_s,
                "low_battery.unplug_confirm_s": unplug_confirm_s}
        for name, v in vals.items():  # every key required, finite and positive (CLAUDE.md 3.1)
            if _num(v) is None or float(v) <= 0.0:  # missing / null / NaN / inf / <= 0
                raise LowBatteryConfigError("%s must be a positive number, got %r" % (name, v))
        check_thresholds(trigger_soc_pct, stop_soc_pct, charge_full_soc_pct,
                         swap_resume_soc_pct)  # the order 5 < 20 < 80 <= 100 <= 100
        self._trigger = float(trigger_soc_pct)  # soc below this starts an episode
        self._stop = float(stop_soc_pct)  # soc at / below this stops the robot
        self._period = float(alarm_period_s)  # cloud alarm cadence
        self._full = float(charge_full_soc_pct)  # the charging alarm ends here (U157)
        self._swap_resume = float(swap_resume_soc_pct)  # a swap this high resumes at once (U157)
        self._absent_min = float(swap_absent_min_s)  # a slot empty this long, then filled = a swap
        self._cold_rise = float(cold_swap_min_rise_pct)  # boot soc this far above the record = a cold swap
        self.charging = ChargingDebouncer(float(plug_confirm_s), float(unplug_confirm_s))  # CD-1
        self.phase = PHASE_IDLE  # no episode
        self.stopped = False  # the 5 % stop has been done in this low phase
        self._last_alarm: Optional[float] = None  # when the last alarm went out
        self._absent_since: Dict[int, float] = {}  # slot -> monotonic s it was first seen empty
        self._swap_pending = False  # a slot was refilled after a real absence; judge on the next soc
        self._restored_soc: Optional[float] = None  # the record's soc; set by restore() until judged
        self._restored = False  # the episode came back from memory; first soc not judged yet
        self.last_cold_swap = False  # the last swap end was judged cold (detail.cold_swap)
        # 99 U171: the lowest not-charging soc of this episode, what a swap's
        # rise is measured from; reset to the current soc while charging.
        self._rise_base: Optional[float] = None  # None: no episode, or no soc seen yet

    @property
    def low(self) -> bool:
        """An episode is in progress (any phase but idle)."""
        return self.phase != PHASE_IDLE

    def restore(self, phase: str, last_soc: Optional[float] = None) -> None:
        """Re-enter a persisted phase after a P3 restart (the memory row).
        The alarm cadence starts over; the 5 % latch and the swap history do
        not survive (a stop is re-judged on the next frame). last_soc is the
        soc the record holds -- what the cold-swap rise is measured from; None
        (no soc in the record) means no cold swap can be judged."""
        if phase not in _PHASES:
            raise ValueError("unknown low-battery phase %r" % (phase,))
        self.phase = phase
        self.stopped, self._last_alarm = False, None
        self._absent_since, self._swap_pending = {}, False
        self._restored = True  # the first frame with a soc is judged for a cold swap
        self._restored_soc = _num(last_soc)
        self._rise_base = self._restored_soc  # U171 measures from the record too

    def snapshot(self) -> Dict[str, Any]:
        """Every mutable fact, so the wiring can roll a frame back when its
        handler failed (the next frame then raises the same events again)."""
        return {"phase": self.phase, "stopped": self.stopped, "last_alarm": self._last_alarm,
                "absent_since": dict(self._absent_since), "swap_pending": self._swap_pending,
                "restored": self._restored, "restored_soc": self._restored_soc,
                "last_cold_swap": self.last_cold_swap, "rise_base": self._rise_base,  # U171 state too
                "charging": (self.charging.stable, self.charging.raw, self.charging._since)}

    def rollback(self, snap: Mapping[str, Any]) -> None:
        """Undo update() back to a snapshot() (2026-11-11 audit: a failed "low"
        handler left the monitor in the low phase with no return task and no
        held set, and nothing ever retried it)."""
        self.phase, self.stopped, self._last_alarm = snap["phase"], snap["stopped"], snap["last_alarm"]
        self._absent_since, self._swap_pending = dict(snap["absent_since"]), snap["swap_pending"]
        self._restored, self._restored_soc = snap["restored"], snap["restored_soc"]
        self.last_cold_swap = snap["last_cold_swap"]
        self._rise_base = snap["rise_base"]  # a rolled-back frame re-judges the same rise
        self.charging.stable, self.charging.raw, self.charging._since = snap["charging"]

    def _enter(self, phase: str) -> None:  # a phase change inside an episode
        """Switch phase; the next alarm goes out at once, worded for it."""
        self.phase, self._last_alarm = phase, None
        if phase != PHASE_LOW:
            self.stopped = False  # the 5 % pause lifts once on the charger (U146)

    def _end_episode(self) -> None:  # back to idle, ready for the next one
        """Reset every per-episode fact."""
        self.phase, self.stopped, self._last_alarm = PHASE_IDLE, False, None
        self._absent_since, self._swap_pending = {}, False  # presence history belongs to one episode
        self._restored, self._restored_soc = False, None
        self._rise_base = None  # the next episode measures from its own start

    def _track_presence(self, power: Mapping[str, Any], now_s: float) -> None:  # 99 U154 swap evidence
        """Record when a slot went empty; a slot that is filled again after
        being empty for at least swap_absent_min_s is a swap, judged on the
        soc of that moment (the next frame with a soc). An absence shorter than
        that is a presence glitch and is forgotten (2026-11-11 audit: one
        glitch frame used to latch a swap for the rest of the episode). A pack
        without a boolean `present` says nothing either way."""
        for pos, b in enumerate(_packs(power)):
            slot = b.get("index") if isinstance(b.get("index"), int) else pos  # 13 BAT-1 slot index
            if b.get("present") is False:
                self._absent_since.setdefault(slot, now_s)  # pulled out, or empty to begin with
            elif b.get("present") is True and slot in self._absent_since:
                if now_s - self._absent_since.pop(slot) >= self._absent_min:
                    self._swap_pending = True  # a pack went in where there was none, for real

    def update(self, power: Optional[Mapping[str, Any]], now_s: float) -> List[str]:  # one frame in, events out
        """Events for this state/power body (None: nothing received -> none)."""
        if not isinstance(power, Mapping):  # no frame: nothing to say
            return []  # chassis silent: no judgement
        soc = _num(power.get("soc_pct"))  # min over present packs, or None
        raw = is_charging(power)
        stable = self.charging.update(raw, now_s)  # CD-1: None until confirmed
        charging = stable is True  # phase changes act on the debounced flag only
        out: List[str] = []  # events of this frame
        if self.phase == PHASE_IDLE:
            # an episode starts only when NOT charging is confirmed and the raw
            # bit agrees: right after a start (stable None) a charging robot at
            # 15 % must not get a return task that the charge then cancels
            if soc is None or soc >= self._trigger or raw or stable is not False:
                return out  # no episode starts
            self.phase = PHASE_LOW  # a new episode
            self._rise_base = soc  # U171: the rise of a later swap is measured from here
            self._track_presence(power, now_s)  # a slot already empty now counts when it is filled
            out.append("low")
        else:
            self._track_presence(power, now_s)
            cold = False
            if self._restored and soc is not None and stable is not None:
                # 99 U157 cold swap (user ruling 2026-11-11): restored in the
                # low phase, the first reading is at / above the trigger, not
                # charging, AND at least cold_swap_min_rise_pct above the soc
                # the record holds. The rise is what tells a pack swapped while
                # P3 was down from a soc that wobbled 19 -> 20 across a restart
                # (audit 2026-11-11). Found live: 8 % at power-off, 69 % single
                # pack at boot, and the restored episode kept alarming.
                self._restored = False  # judged once, on the first soc with the charge known
                cold = (self.phase == PHASE_LOW and stable is False and soc >= self._trigger
                        and self._restored_soc is not None
                        and soc - self._restored_soc >= self._cold_rise)
            hot = False
            if self._swap_pending and soc is not None:
                self._swap_pending = False  # judged on the soc of the refill, once
                hot = soc >= self._trigger
            rise = False  # 99 U171: a large rise without charging is a swap
            if soc is not None:  # no soc this frame: nothing to measure, base kept
                if raw or stable is not False or self.phase == PHASE_CHARGING:  # any hint of charging
                    self._rise_base = soc  # on the charger (or not yet known): a rise from here is the charge
                else:  # confirmed not charging: a rise here came from a new pack
                    rise = (self._rise_base is not None  # a base to measure from
                            and self.phase in (PHASE_LOW, PHASE_INTERRUPTED)  # not while charging
                            and soc >= self._trigger  # out of the low band, as hot / cold require
                            and soc - self._rise_base >= self._cold_rise)  # 10 points: not a wobble
                    self._rise_base = soc if self._rise_base is None else min(self._rise_base, soc)  # the episode low
            if hot or cold or rise:  # a swap (99 U154 / U157 / U171)
                full = soc >= self._swap_resume  # U157: high enough to resume at once
                self.last_cold_swap = cold and not hot  # for the cleared report
                self._end_episode()
                return ["swapped" if full else "swapped_partial"]
            if soc is not None and soc >= self._full and self.phase in (
                    PHASE_CHARGING, PHASE_INTERRUPTED):  # U157: full ends the charging alarm
                self._end_episode()
                return ["charge_full"]
            if charging and self.phase != PHASE_CHARGING:
                self._enter(PHASE_CHARGING)  # on the charger: the alarm now says charging
                out.append("charging")
            elif stable is False and self.phase == PHASE_CHARGING and soc is not None:
                if soc < self._trigger:  # unplugged and still low: drive back (U157 ruling 2)
                    self._enter(PHASE_LOW)
                    out.append("low")
                else:  # unplugged before full
                    self._enter(PHASE_INTERRUPTED)
                    out.append("interrupted")
            elif (stable is False and self.phase == PHASE_INTERRUPTED and soc is not None
                    and soc < self._trigger):
                self._enter(PHASE_LOW)  # drained again after an interrupted charge
                out.append("low")
        if (self.phase == PHASE_LOW and not self.stopped and soc is not None
                and soc <= self._stop):  # the last 5 %
            self.stopped = True  # last 5 %: stop and lie down, once
            out.append("stop")
        if self._last_alarm is None or now_s - self._last_alarm >= self._period:  # due by the cadence
            self._last_alarm = now_s  # cadence measured from the last alarm
            out.append("alarm")
        return out  # in order: phase change, stop, alarm


def check_thresholds(trigger: Any, stop: Any, full: Any, swap_resume: Any) -> None:
    """The order the rule needs: stop < trigger < swap_resume <= full <= 100.
    Shared by the monitor and the __main__ config check (one rule, one place).
    A full above 100 could never be reached and the charging alarm would never
    end; a swap_resume at or below the trigger would make the partial-swap
    branch unreachable."""
    stop, trigger, full, swap_resume = float(stop), float(trigger), float(full), float(swap_resume)
    if not stop < trigger:
        raise LowBatteryConfigError(
            "low_battery.stop_soc_pct %r must be below trigger_soc_pct %r" % (stop, trigger))
    if not trigger < swap_resume <= full <= 100.0:
        raise LowBatteryConfigError(
            "low_battery needs trigger_soc_pct %r < swap_resume_soc_pct %r <= "
            "charge_full_soc_pct %r <= 100" % (trigger, swap_resume, full))


#: task.db memory key of the running episode (99 U157): the phase, the held
#: task ids and the injected task, so a P3 restart mid-charge neither forgets
#: to hold the interrupted tasks nor raises a second return task.
EPISODE_MEMORY_KEY = "low_battery_episode"


def encode_episode(phase: str, held: Set[str], task_id: Optional[str],
                   soc: Optional[float] = None) -> bytes:
    """The memory row value (JSON, ids sorted so equal records compare equal).
    soc is the last whole-percent reading (the cold-swap rise is measured from
    it); None when unknown."""
    return json.dumps({"phase": phase, "held": sorted(held), "task_id": task_id,
                       "soc": _num(soc)}, separators=(",", ":")).encode("utf-8")


def decode_episode(raw: Any) -> Optional[Tuple[str, Set[str], Optional[str], Optional[float]]]:
    """(phase, held, task_id, soc) of a stored record; None for idle / absent /
    unreadable. An unreadable row is not trusted to hold anything. soc is None
    for a record written before it was stored (no cold swap is judged then)."""
    if raw is None:
        return None
    try:
        rec = json.loads(raw if isinstance(raw, str) else bytes(raw).decode("utf-8"))
    except (ValueError, UnicodeDecodeError, TypeError):
        return None
    if not isinstance(rec, dict) or rec.get("phase") not in _PHASES or rec.get("phase") == PHASE_IDLE:
        return None
    held = {str(t) for t in rec.get("held") or [] if isinstance(t, str)}
    tid = rec.get("task_id") if isinstance(rec.get("task_id"), str) else None
    return rec["phase"], held, tid, _num(rec.get("soc"))


async def live_task_ids(dao) -> Set[str]:
    """Every non-terminal task id: what an episode holds when it starts."""
    return {tid for tid, _p, _s, _st in await dao.list_by_priority()}


async def park_held_for_manual(dao, conn, held: Set[str], now_mono_ms: int) -> List[str]:
    """U157 ruling 1 (a swap below swap_resume): the alarm ends, but the tasks
    the episode interrupted must not resume by themselves. A preempted task
    (yielding / preempted) is re-tagged passive / low_battery -- the pair that
    "never resumes by itself" (15 S8.2A CR-6) -- so the scheduler's yielding
    scan passes it by and it waits in history for 继续. Returns the ids
    re-tagged."""
    to_state = "suspended"
    validate_suspend_fields(to_state, LB_SUSPEND_KIND, LB_SUSPEND_REASON)  # CR-8 pair
    parked: List[str] = []
    for tid in sorted(held):
        full = await dao.fetch_by_id(tid)
        if full is None or full.state != "suspended" or full.suspend_kind != "yielding":
            continue  # running / waiting / already passive: nothing to re-tag
        parked.append(tid)
    if not parked:
        return parked
    await conn.execute("BEGIN IMMEDIATE")  # same lock timing as every P3 write
    try:
        for tid in parked:
            await conn.execute(
                "UPDATE tasks SET suspend_kind=?, suspend_reason=?, updated_ms=? "
                "WHERE task_id=? AND state='suspended'",
                (LB_SUSPEND_KIND, LB_SUSPEND_REASON, now_mono_ms, tid))
        await conn.commit()
    except Exception:
        await conn.rollback()  # nothing half-applied
        raise
    return parked


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


async def suspend_running_for_low_battery(dao, conn, now_mono_ms: int,  # ES-2 shaped suspension
                                          return_task_id: Optional[str] = None
                                          ) -> Optional[Tuple[str, str, str]]:
    """The 5 % stop: suspend the running task; same transaction discipline as
    ES-2 (estop_suspend.py). The return task itself is parked passive /
    low_battery (it never resumes; charging cancels it). Any OTHER task -- the
    episode began at or below 5 % in one frame, before the return task could
    preempt it -- is suspended yielding / preempted, so it resumes at the
    episode's end like every interrupted task (2026-11-11 audit: it was parked
    passive and never came back although the end said auto_resume)."""
    rows = await dao.list_by_priority()  # (task_id, prio, seq, state)
    running = [tid for tid, _p, _s, state in rows if state == "running"]  # the running one
    if not running:
        return None  # nothing running: the lie-down still happens
    task_id = running[0]  # at most one (scheduler invariant)
    to_state = apply_transition("running", "suspend").to_state  # the graph agrees
    is_return = return_task_id is not None and task_id == return_task_id
    kind, reason = ((LB_SUSPEND_KIND, LB_SUSPEND_REASON) if is_return
                    else ("yielding", "preempted"))
    validate_suspend_fields(to_state, kind, reason)  # CR-8 pair
    await conn.execute("BEGIN IMMEDIATE")  # same lock timing as every P3 write
    try:
        if is_return:
            await dao.suspend_task(task_id, kind, reason, now_mono_ms)  # the CR-8 pair written together
        else:
            await dao.preempt_task(task_id, reason, now_mono_ms)  # yielding, as a preemption
        await conn.commit()  # never leave the implicit transaction open
    except Exception:
        await conn.rollback()  # nothing half-applied
        raise
    return task_id, to_state, reason  # for the publish


async def close_low_battery_task(dao, conn, task_id: Optional[str], now_mono_ms: int  # the return task's end
                                 ) -> Optional[Tuple[str, str, str]]:
    """The injected return task has done its job -> cancelled. Called when the
    charging phase starts, and again at the episode's end (full / swap) in case
    it is still live (99 U157; until 2026-11-10 only on charging, which then
    also ended the episode).

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
           "is_charging", "on_route", "list_role_points", "LB_PRIORITY", "check_thresholds",
           "EPISODE_MEMORY_KEY", "encode_episode", "decode_episode", "live_task_ids",
           "park_held_for_manual", "charging_below_full", "PHASE_IDLE", "PHASE_LOW", "PHASE_CHARGING", "PHASE_INTERRUPTED"]
