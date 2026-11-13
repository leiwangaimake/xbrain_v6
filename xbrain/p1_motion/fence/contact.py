"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: contact.py
Brief: 99 U186 -- boundary contact (撞线) on the activity area / a forbid zone: start, every N s, end

Description:
User ruling 2026-11-13: "活动区域, 禁止区域撞线要播报和上报事件且要持续, 直到撞线状态
消失"; the repeat period defaults to 3 s and is configurable. episodes.py says
when an excursion BEGINS (soft_enter / hard_clip once per episode, E-3) --
deliberately not repeated. This tracker is the other thing the user asked for:
while the robot is held at the boundary, keep saying so.

"Contact" (撞线) = a boundary polygon (allow / forbid) whose zero cap is active
this tick: PolyDistance.hard_now -- the robot is at or past the EFFECTIVE
boundary (nominal minus the fix-quality inset), now or at the predicted point
(12 S7.2 / S7.3). Not the soft band, where the robot only slows.

Per polygon:
  contact_start (warn)  hard_now begins
  contact_hold  (warn)  every repeat_s while it lasts
  contact_end   (info)  released: d_eff >= release_m, or hard_now off for
                        release_s in a row (hysteresis, so a robot sitting on
                        the line does not toggle start / end every tick)
All three of one contact share an episode id (E-2), the counter increments
after the end. A rev change, or a polygon leaving the set, ends its contact.

Recording (P1 knows it from state/teach): during a FENCE recording the robot
is driven along boundaries on purpose, so a contact is told once (the start,
detail.recording = "fence") and never repeated; during a ROUTE recording the
start / holds carry detail.recording = "route" so P4 words them as "the route
hit the boundary, drive back" (the keyboard is stopped there, nav_tick).

dedup: dedup_window_s 0 on every row, with the tick's rising wall ts -- the
p5 merge then keeps each hold as its own row (a non-zero window would fold
the repeats into one and the cloud would see a single event, the opposite of
"持续").

No geometry this tick (enforcement degraded, no fence) freezes the state: an
unknown position is reported as neither in nor out of contact.
"""

from __future__ import annotations  # postponed annotations

from dataclasses import dataclass  # event rows
from typing import Dict, List, Optional  # annotations

from xbrain.p1_motion.fence.clip import FenceEval  # the tick's per-polygon numbers

#: the boundary roles -- the only ones a contact exists for
_BOUNDARY_ROLES = ("allow", "forbid")


@dataclass(frozen=True)
class ContactEvent:
    """One contact row; the publisher adds eid / ts / src (event/{sev}/fence)."""
    kind: str                 # contact_start | contact_hold | contact_end
    severity: str             # warn | warn | info
    poly_id: str
    poly_name: str
    role: str                 # allow | forbid
    episode_id: int
    recording: Optional[str]  # None | "route" | "fence" (what the keyboard is doing)
    d_eff_m: Optional[float]  # distance to the effective boundary this tick

    @property
    def dedup_key(self) -> str:
        """One key per contact; window 0 keeps every hold its own row."""
        return "fence:contact:%s:%d" % (self.poly_id, self.episode_id)

    dedup_window_s = 0  # see the module docstring


class ContactTracker:
    """Level to edge for boundary contact. observe() once per nav tick."""

    def __init__(self, repeat_s: float, release_m: float, release_s: float) -> None:
        # the three p1_motion.yaml fence.contact_* values (99 U186), injected:
        # no defaults here (they are behaviour the user set, 3 s by default)
        self._repeat_s = float(repeat_s)  # hold period
        self._release_m = float(release_m)  # distance that ends a contact at once
        self._release_s = float(release_s)  # time off the line that ends it
        self._rev: Optional[int] = None  # the fence set the state belongs to
        self._on: Dict[str, Dict] = {}  # poly_id -> {ep, last, off_since, name, role}
        self._episode: Dict[str, int] = {}  # poly_id -> next episode id

    def _end(self, pid: str, st: Dict, d_eff: Optional[float],
             recording: Optional[str]) -> ContactEvent:
        """The end row of one contact; frees the polygon and moves its episode on."""
        self._on.pop(pid, None)  # no longer in contact
        self._episode[pid] = st["ep"] + 1  # E-2: next contact, next id
        return ContactEvent("contact_end", "info", pid, st["name"], st["role"],
                            st["ep"], recording, d_eff)

    def observe(self, fev: Optional[FenceEval], *, now_s: float, rev: Optional[int],
                recording: Optional[str]) -> List[ContactEvent]:
        """This tick's contact rows (usually none). now_s is monotonic seconds."""
        out: List[ContactEvent] = []  # rows to publish
        if rev != self._rev:  # a new fence set: old contacts end
            for pid, st in list(self._on.items()):  # every open contact
                out.append(self._end(pid, st, None, recording))  # closed with the old set
            self._rev = rev  # the state belongs to the new set now
        if fev is None or not fev.per_poly:  # no geometry: frozen, see the docstring
            return out
        seen = set()  # polygons present this tick
        for p in fev.per_poly:  # every polygon's numbers
            if p.role not in _BOUNDARY_ROLES:  # warning / speed zones have no line
                continue
            seen.add(p.poly_id)  # present
            st = self._on.get(p.poly_id)  # its open contact, if any
            if p.hard_now:  # held at the effective boundary this tick
                if st is None:  # contact begins
                    ep = self._episode.get(p.poly_id, 0)  # this contact's id
                    self._on[p.poly_id] = {"ep": ep, "last": now_s, "off_since": None,
                                           "name": p.name, "role": p.role}
                    out.append(ContactEvent("contact_start", "warn", p.poly_id, p.name,
                                            p.role, ep, recording, p.d_eff_m))
                else:  # still in contact
                    st["off_since"] = None  # back on the line: the release clock restarts
                    if recording != "fence" and now_s - st["last"] >= self._repeat_s:
                        st["last"] = now_s  # next hold one period from now
                        out.append(ContactEvent("contact_hold", "warn", p.poly_id, st["name"],
                                                st["role"], st["ep"], recording, p.d_eff_m))
            elif st is not None:  # off the line: released at a distance or after a while
                if p.d_eff_m >= self._release_m:  # clearly away
                    out.append(self._end(p.poly_id, st, p.d_eff_m, recording))
                elif st["off_since"] is None:  # just left it: start the clock
                    st["off_since"] = now_s
                elif now_s - st["off_since"] >= self._release_s:  # stayed off long enough
                    out.append(self._end(p.poly_id, st, p.d_eff_m, recording))
        for pid, st in list(self._on.items()):  # a polygon gone from the set
            if pid not in seen:  # switched off / deleted while in contact
                out.append(self._end(pid, st, None, recording))
        return out


__all__ = ["ContactEvent", "ContactTracker"]
