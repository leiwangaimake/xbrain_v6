"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: comm_events.py
Brief: 11 S4.6.8 comm events -- cloud-link level transition -> event/{sev}/comm

Description:
P5 owns the cloud-link state (11 S4.6, LinkStateMachine), so it is the producer of
the comm events 11 S4.6.8 defines: every cloud-link level transition emits one
event/{sev}/comm (channel=normal, U18a). This module is the PURE mapping from a
(prev_level -> new_level) transition to the event kind + severity, so the wiring
just diffs the level each heartbeat and publishes what this returns.

The 11 S4.6.8 table:
  cloud_degraded  warn   L0 -> L1
  cloud_down      warn   L1 -> L2
  cloud_lost      alarm  L2 -> L3
  cloud_up        info   drop back to L0 (hysteresis passed); MUST carry the total
                         disconnected_s of the outage + link_epoch
  link_timer_reset  warn LNK-4 (P5 restart) -- needs a persisted "ran before" marker,
                         deferred (see NEXT.md)
  rtb_triggered   alarm  L3 fired return_home -- needs P3's energy decision (action /
                         reason_detail) + the deterministic task_id, deferred

Levels escalate MONOTONICALLY within one outage (disconnected_s only grows), so each
of L1/L2/L3 is entered once and fires once. cloud_up carries the outage duration from
BEFORE the reset (disconnected_s is 0 the instant we return to L0), so the caller
passes the previous tick's disconnected_s.

NOTE the cloud_up sev is info but comm's channel is normal (U18a), so cloud_up is
best-effort -- fine, because a resumed link means the live path is back anyway. The
alarm-sev ones (cloud_lost) still ride channel=normal per S6.2 ("断网时本就发不出
去, 补发优先级无意义"); sev and channel are orthogonal (U18a).
"""

from __future__ import annotations

from typing import Optional, Tuple


COMM_CATEGORY = "comm"


def comm_event_for_level(prev_level: Optional[int], new_level: int,
                         prev_disconnected_s: float,
                         link_epoch: int) -> Optional[Tuple[str, str, dict]]:
    """Return (kind, sev, detail) for a cloud-link level transition, or None when the
    transition warrants no comm event (11 S4.6.8). prev_level is None on the first
    observation (no transition). A multi-level jump (e.g. 0->2, possible if evaluate
    is called sparsely) fires for the level REACHED, not each crossed -- 1 Hz makes
    single-step the norm and the reached level is the operationally-relevant one."""
    if prev_level is None or new_level == prev_level:
        return None
    if new_level > prev_level:
        if new_level == 1:
            kind, sev = "cloud_degraded", "warn"
        elif new_level == 2:
            kind, sev = "cloud_down", "warn"
        elif new_level == 3:
            kind, sev = "cloud_lost", "alarm"
        else:
            return None
        return (kind, sev, {"kind": kind, "level": new_level})
    # new_level < prev_level: the only downward event is the return to L0 (an outage
    # ending). Intermediate downward steps do not occur -- L0 is reached directly
    # once the hysteresis window passes.
    if new_level == 0:
        return ("cloud_up", "info",
                {"kind": "cloud_up",
                 "disconnected_s": prev_disconnected_s,
                 "link_epoch": link_epoch})
    return None
