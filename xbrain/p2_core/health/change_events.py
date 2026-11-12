"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: change_events.py
Brief: event/{warn|fault}/health -- one event per health item state change (11 S6.2 health row)

Description:
What problem this solves. 11 S6.2 lists the `health` event category verbatim as
"健康度变化" (sev warn / fault, channel normal) and P2 is the health owner (11
S5.1: it publishes health/summary), yet nothing emitted those events: an item
that went from ok to fail (the speaker, the camera) showed only as a changed
field in a 1 Hz summary, so the HMI event list, record.db and the cloud never
heard of it. User ruling 2026-11-12 (the HMI event list): every device health
change is recorded, one entry per change.

Rules (pure, no I/O, no clock -- the caller passes now_mono_s):
  * baseline: the first KNOWN state of an item (ok / warn / degraded / fail) is
    its baseline and is NOT reported -- the stack starting up is not news;
  * an item that has never been known (compute, gpu, dla today: no source) never
    gets a baseline, so it never reports;
  * after its baseline an item reports every change, including to and from
    unknown (a camera whose feed stops is news);
  * a new state must persist for hold_s before it is reported (a single 1 Hz
    sample of a flapping item is not two events); going back to the reported
    state inside the hold cancels the pending change;
  * sev: a change TO fail is `fault`, every other change `warn` -- the S6.2 row
    allows exactly these two, so a recovery is `warn` too (detail.to says ok).

What it does NOT do: it does not decide health (aggregate.py does) and does not
publish (main_wiring puts the frame on event/{sev}/health). detail.device is the
item name, which is the key the cloud projection maps to its fault source
(p5 state_projection _DEVICE_TO_FAULT_SOURCE).
"""
from __future__ import annotations  # postponed annotations

from typing import Any, Dict, List, Mapping, Optional, Tuple  # signatures

#: States that can serve as a baseline. unknown is reported once an item has a
#: baseline, but never becomes one -- see the module docstring.
KNOWN_STATES = frozenset({"ok", "warn", "degraded", "fail"})  # aggregate.py's known set

#: detail.type of these events (the HMI event list keys its wording on it).
EVENT_TYPE = "health.item_changed"  # one constant, read by the HMI eventText


class HealthChangeTracker:
    """Per-item reported state + one pending change, judged with a hold."""

    def __init__(self, hold_s: float) -> None:
        """hold_s from health.change_event_hold_s; null / negative / bool raise."""
        # bool is an int subclass: True would silently mean a 1 s hold.
        if isinstance(hold_s, bool) or not isinstance(hold_s, (int, float)) or hold_s < 0:  # CLAUDE.md 3.1
            raise ValueError("hold_s must be a number >= 0, got %r" % (hold_s,))  # p2 refuses to start
        self._hold_s = float(hold_s)  # seconds a new state must last
        self._reported: Dict[str, str] = {}  # item -> last reported (or baseline) state
        self._pending: Dict[str, Tuple[str, float]] = {}  # item -> (new state, since)

    def observe(self, items: Mapping[str, Any], now_mono_s: float) -> List[Dict[str, Any]]:
        """One health/summary `items` mapping -> the changes due now, each
        {device, from, to, level, reason}."""
        out: List[Dict[str, Any]] = []  # changes that held long enough this call
        for name, body in items.items():  # every item of this summary
            state = body.get("state") if isinstance(body, Mapping) else None  # tolerate junk
            if not isinstance(state, str):  # a malformed item says nothing
                continue  # never raise inside the 1 Hz publish loop
            baseline = self._reported.get(name)  # what the list last heard
            if baseline is None:  # never known before
                if state in KNOWN_STATES:  # unknown is never a baseline
                    self._reported[name] = state  # first known state: baseline, silent
                continue  # nothing to report yet
            if state == baseline:  # back to (or still at) the reported state
                self._pending.pop(name, None)  # a flap inside the hold is nothing
                continue  # no change
            pending = self._pending.get(name)  # a change already under way?
            if pending is None or pending[0] != state:  # new target -> the hold restarts
                self._pending[name] = (state, now_mono_s)  # start (or restart) the hold
                pending = self._pending[name]  # the hold just started
            if now_mono_s - pending[1] >= self._hold_s:  # held long enough (monotonic)
                out.append({"device": name, "from": baseline, "to": state,  # one entry
                            "level": body.get("level"), "reason": body.get("detail")})  # why
                self._reported[name] = state  # the next change is measured from here
                self._pending.pop(name, None)  # reported: nothing pending
        return out  # usually empty


def health_change_event(change: Mapping[str, Any], *, eid: str, ts: float,
                        src: str = "p2_core") -> Dict[str, Any]:
    """The 11 S6.1 Event for one change. `ts` is the display stamp only."""
    # 11 S6.2 health row allows warn / fault only: fail is fault, the rest warn.
    sev = "fault" if change.get("to") == "fail" else "warn"  # a recovery is warn too
    detail: Dict[str, Optional[Any]] = {  # what the HMI and the cloud read
        "type": EVENT_TYPE, "device": change.get("device"),  # device = item name
        "from": change.get("from"), "to": change.get("to"),  # the transition
        "level": change.get("level"), "reason": change.get("reason")}  # fatal/optional, why
    return {"eid": eid, "sev": sev, "cat": "health", "channel": "normal",  # S6.2 row
            "code": "HEALTH_ITEM_CHANGED",  # stable machine key
            "title": "health %s %s -> %s" % (change.get("device"), change.get("from"),  # English
                                             change.get("to")),  # log / record title
            "detail": detail, "src": src, "ts": ts}  # ts: display only
