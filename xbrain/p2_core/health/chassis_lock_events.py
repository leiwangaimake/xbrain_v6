"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: chassis_lock_events.py
Brief: event/warn/estop estop.locked -- repeated every period_s while the chassis is locked (99 U196)

Description:
What problem this solves. In the 2026-11-13 stress test the chassis was still
locked (nobody had unlocked it after the boot) when the first goto went out.
P1 commanded a turn for about 140 s, the chassis did not move, the task stayed
"running", and nothing anywhere said why: state/robot carried the lock, but no
process turned it into an event, so the HMI event list, record.db and the cloud
stayed silent. User ruling 2026-11-13: report the chassis lock state as an
event, once every 3 s while it lasts, and stop as soon as it is unlocked.

Rules (pure, no I/O, no clock -- the caller passes now_mono_s):
  * locked = hes_lock OR timeout_lock, the two Tier 1 locks of 11 S9.12.2,
    read from state/robot exactly like the HMI lock button
    (p5 data_readers.chassis_lock_group). The derived `locked` field is NOT
    read (11 S4.1 marks it compat-only). A soft estop is not a lock (U35);
  * the first locked frame reports at once, then one report per period_s;
  * unlocked (both locks false) ends the spell: nothing more is sent, and the
    next lock reports at once again;
  * unknown -- no state/robot yet, a frame missing either lock field, or the
    last frame older than max_age_s -- reports nothing and leaves the spell as
    it was. "Locked" is a claim about the chassis, and a chassis nobody has
    heard from is not known to be locked.

What it does NOT do: it does not unlock (only cmd/chassis/ctrl enable can,
11 S9.3.3), does not judge health, and does not publish (main_wiring puts the
frame on event/warn/estop). The one-shot estop.timeout_lock / estop.hes_engaged
types of 11 S6.2 are a separate thing and still have no producer.
"""
from __future__ import annotations  # postponed annotations

from typing import Any, Dict, Mapping, Optional  # signatures

#: detail.type of these events (the HMI event list keys its wording on it).
EVENT_TYPE = "estop.locked"  # one constant, read by the HMI eventText

#: The two Tier 1 lock fields of state/robot (11 S4.1 / S9.12.2).
LOCK_FIELDS = ("hes_lock", "timeout_lock")


def _positive(name: str, value: Any) -> float:
    """A required positive number; bool is refused by name (True would read as 1)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:  # CLAUDE.md 3.1
        raise ValueError("health.chassis_lock_event.%s must be a positive number, got %r"
                         % (name, value))  # the key path, so the operator knows what to fix
    return float(value)  # seconds


def lock_state(robot: Optional[Mapping[str, Any]], rx_mono_s: Optional[float],
               now_mono_s: float, max_age_s: float) -> Optional[Dict[str, Any]]:
    """state/robot -> {locked, hes_lock, timeout_lock, hes}, or None when unknown."""
    if not isinstance(robot, Mapping) or rx_mono_s is None:  # never heard from
        return None  # unknown
    if now_mono_s - rx_mono_s > max_age_s:  # the cache outlived its source
        return None  # unknown, not "still locked"
    hes_lock, timeout_lock = robot.get("hes_lock"), robot.get("timeout_lock")  # the two locks
    if not isinstance(hes_lock, bool) or not isinstance(timeout_lock, bool):  # a field missing
        return None  # never default a lock to False (or True)
    return {"locked": hes_lock or timeout_lock, "hes_lock": hes_lock,
            "timeout_lock": timeout_lock, "hes": robot.get("hes")}  # hes says WHY it will not clear


class ChassisLockReporter:
    """One locked spell at a time: report on entry, then every period_s."""

    def __init__(self, period_s: Any, max_age_s: Any) -> None:
        """Both from health.chassis_lock_event.*; missing / non-positive / bool raise."""
        self._period_s = _positive("period_s", period_s)  # user ruling: 3 s
        self._max_age_s = _positive("max_age_s", max_age_s)  # state/robot freshness bound
        self._since: Optional[float] = None  # monotonic s the spell was first seen
        self._last: Optional[float] = None   # monotonic s of the last report

    def observe(self, robot: Optional[Mapping[str, Any]], rx_mono_s: Optional[float],
                now_mono_s: float) -> Optional[Dict[str, Any]]:
        """One pass of the health loop -> the report due now, or None."""
        st = lock_state(robot, rx_mono_s, now_mono_s, self._max_age_s)  # what the chassis says
        if st is None:  # unknown: say nothing, keep the spell
            return None
        if not st["locked"]:  # unlocked: the spell is over
            self._since = self._last = None  # the next lock reports at once
            return None
        if self._since is None:  # a new spell
            self._since = now_mono_s  # locked_s counts from here
        if self._last is not None and now_mono_s - self._last < self._period_s:  # not due yet
            return None
        self._last = now_mono_s  # the next one is period_s from here
        st["locked_s"] = round(now_mono_s - self._since, 1)  # how long it has been locked
        return st  # {locked, hes_lock, timeout_lock, hes, locked_s}


def chassis_lock_event(report: Mapping[str, Any], *, eid: str, ts: float,
                       src: str = "p2_core") -> Dict[str, Any]:
    """The 11 S6.1 Event for one report. `ts` is the display stamp only."""
    # reason uses the G-9 vocabulary (11 S7.x E_LOCKED): a pressed hardware
    # stop is named first, since it is the lock an unlock alone cannot clear.
    reason = "hes_engaged" if report.get("hes_lock") else "cmd_timeout"  # which lock holds it
    detail: Dict[str, Any] = {  # what the HMI and the cloud read
        "type": EVENT_TYPE, "reason": reason,  # which lock
        "hes_lock": report.get("hes_lock"), "timeout_lock": report.get("timeout_lock"),
        "hes": report.get("hes"), "locked_s": report.get("locked_s")}  # how long
    # The title is constant on purpose: the HMI ring folds exact repeats (same
    # cat, sev, title, detail.type) into one entry with a count.
    return {"eid": eid, "sev": "warn", "cat": "estop", "channel": "normal",  # 99 U196
            "code": "CHASSIS_LOCKED",  # stable machine key
            "title": "chassis locked",  # English, log / record title
            "detail": detail, "src": src, "ts": ts}  # ts: display only
