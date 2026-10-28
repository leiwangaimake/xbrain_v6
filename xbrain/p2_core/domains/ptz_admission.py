"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptz_admission.py
Brief: Domain-5 (PTZ) admission on the live arbiter -- manual hold with idle release, lower mode holders yield at once

Description:
What problem this solves
------------------------
The PTZ controller (ptz/ptz_controller.py, 99 U98) shipped with its own
domain-5 admission because the arbiter did not run yet: it compared the
requester's priority with the CURRENT MOTION only, and the domain was free
again the instant a jog ended. state/arb/ptz therefore always said "idle"
while an operator was steering, and B / D mode (14 S5.5.1 table A: domain 5
= auto_track) had nothing to hold. Since 99 U99 C1 the four P2 arbiters run
in ArbHub; this adapter puts the controller's admission on the ptz arbiter.

The rules (14 S4.4, S4.4.2; user ruling 2026-10-28)
---------------------------------------------------
  * a manual command (cloud / HMI / WeCom / local voice -> manual_cloud, the
    controller's SOURCE_IDS) requests the domain, or renews it when it already
    holds it. A strictly higher holder refuses it (E_BUSY, holder named); an
    equal one is the same source (manual_cloud is the only 700) and renews.
  * the manual hold is NOT released when a motion ends: 14 S4.4.2 "manual
    idle beyond T_idle (30 s) -> released automatically", T_idle =
    arbiter.domains.ptz.manual_idle_release_s. tick() does that release.
  * a lower holder (auto_track 500 / preset_patrol 300) is preempted and its
    ack is given HERE, on its behalf. That is only correct because neither
    moves the head in this phase: the body is held still in B / D (99 U90
    R-8 (a)) and set_track / patrol answer E_NOT_IMPLEMENTED, so there is no
    motion of theirs to stop first. When a tracker that really drives the
    head lands, it must ack itself and this shortcut must go.
  * admission without acquisition (refocus, 11 S7.4.4 "admission only, not a
    holder") checks the holder and takes nothing.

What it does NOT do: it does not decide whether a MODE needs domain 5 (the
user ruled 2026-10-28 that domain 5 never blocks entering B / D; main_wiring
leaves it out of the P-1' preflight and re-requests auto_track later), and it
does not touch the camera (the controller stops the head on a holder change).
"""

from __future__ import annotations  # postponed annotations

import threading  # admit (PTZ worker) and tick (main loop) run on two threads
from typing import Any, Callable, Dict, Optional, Tuple  # type hints only

DOMAIN = "ptz"  # 11 S7A domain 5


class PtzArbAdmission:
    """The controller's domain-5 gate, backed by ArbHub's ptz arbiter."""

    def __init__(self, hub, now_ms: Callable[[], int],  # hub = domains.hub.ArbHub
                 idle_release_s: float) -> None:
        """idle_release_s is arbiter.domains.ptz.manual_idle_release_s from
        the resolved snapshot -- no default (CLAUDE.md 3.1 spirit)."""
        self._hub = hub  # the live arbiters
        self._now = now_ms  # monotonic ms (CLAUDE.md 3.4)
        self._idle_ms = int(round(float(idle_release_s) * 1000))  # 14 S4.4.2 T_idle
        self._lock = threading.Lock()  # guards _held / _last_use
        self._held: Optional[Tuple[str, str]] = None  # (source, req_id) this adapter holds
        self._last_use = 0  # mono ms of the last admitted command

    def admit(self, source_id: str, *, acquire: bool) -> Optional[Dict[str, Any]]:
        """None = admitted; otherwise the 11 S7A.0 E_BUSY detail."""
        now = self._now()  # one timestamp for the whole decision
        with self._lock:  # one admission at a time
            h, gen = self._hub.holder_gen(DOMAIN)  # current holder + gen, one read
            mine = (h is not None and self._held is not None
                    and (h.source_id, h.req_id) == self._held)  # this adapter's own hold
            if h is not None and not mine:  # someone else holds the head
                want = self._hub.priority_of(DOMAIN, source_id)  # requester's priority
                if want is None or h.priority >= want:  # higher (or equal other) holder wins
                    return {"domain": DOMAIN, "holder": h.source_id,
                            "holder_priority": h.priority, "gen": gen}  # 11 S7A.0 E_BUSY detail shape
            if not acquire:  # refocus: admission only
                return None  # admitted, nothing taken
            self._last_use = now  # restarts the idle clock
            if mine and h.source_id == source_id:  # already ours: keep the lease alive
                self._hub.renew(DOMAIN, h.source_id, h.req_id, now)  # 30 s lease pushed out
                return None  # admitted
            rid = self._hub.new_req_id(DOMAIN, source_id)  # a fresh attempt
            g = self._hub.request(DOMAIN, source_id, now, req_id=rid)  # free -> granted, lower -> preempt
            res = getattr(g.result, "value", g.result)  # enum or plain string
            if res == "queued" and h is not None:  # a lower mode source holds it
                # Acked on its behalf: auto_track / preset_patrol drive no
                # motion this phase (see the module docstring).
                self._hub.ack_preempt(DOMAIN, h.source_id, h.req_id, now)  # promotes us at once
            if not self._hub.holds(DOMAIN, source_id, rid):  # neither granted nor promoted
                self._hub.cancel(DOMAIN, source_id, rid, now)  # leave no ghost waiter
                cur, gen = self._hub.holder_gen(DOMAIN)  # who has it now, and the gen
                return {"domain": DOMAIN,
                        "holder": cur.source_id if cur else None,
                        "holder_priority": cur.priority if cur else None,
                        "gen": gen}  # refused, 11 S7A.0 shape
            self._held = (source_id, rid)  # ours until idle
            return None  # admitted

    def gen(self) -> int:  # for the controller's own E_BUSY detail
        """The ptz domain gen now (11 S7A.2), so a refusal the controller
        makes itself carries the 11 S7A.0 gen field too."""
        return self._hub.holder_gen(DOMAIN)[1]  # the gen half

    def tick(self) -> bool:  # P2 main loop, before ArbHub.tick
        """14 S4.4.2: release the manual hold after T_idle without a command.
        True on the pass it released (for the caller's log)."""
        now = self._now()  # one timestamp
        with self._lock:  # vs a concurrent admit
            if self._held is None:  # nothing held
                return False  # nothing to do
            src, rid = self._held  # our hold
            if not self._hub.holds(DOMAIN, src, rid):  # lost it (revoked / preempted)
                self._held = None  # tidy only: admit re-requests whenever the holder is gone
                return False  # nothing to release
            if now - self._last_use < self._idle_ms:  # still in use
                self._hub.renew(DOMAIN, src, rid, now)  # lease follows the idle clock, not the other way
                return False  # keep holding
            self._hub.release(DOMAIN, src, rid, "done", now)  # idle: give it back
            self._held = None  # free
            return True  # released


__all__ = ["PtzArbAdmission", "DOMAIN"]  # public surface
