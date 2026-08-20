"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: aggregate.py
Brief: Build the 11 S5.1 HealthSummary from the live state P2 can observe

Description:
P2 is the publisher of health/summary (11 S2.2: 1 Hz, consumed by the HMI, the
cloud and by P3 for task admission -- and, since the recording batch, by the
S12A.3 arming check 4). This module holds the per-item state and shapes the
message; the wiring feeds it and publishes.

*** The rule that shapes everything here: an item with no source is UNKNOWN, and
it says so in its detail.

Six of the nineteen items have no producer in this build -- there is no
perception process (cam_rgbd, lidar), no calibrated threshold for compute / gpu
/ dla, and no storage policy wired. Reporting those as `ok` would be the exact
fail-silent shape CLAUDE.md 3.2 catalogues: a health summary that says
everything is fine is indistinguishable from one that is not looking. So they
stay unknown, and each carries a detail naming what is missing, which is what
turns "the robot will not move" into "the robot will not move BECAUSE nothing
reports the camera".

That has a visible consequence and it is the correct one: with cam_rgbd unknown,
14 S8.3 gives allow_motion=false, so P3 refuses to arm a recording. Without an
obstacle-avoidance sensor the robot may not be driven, teleop included.

What each item's state is DERIVED from is deliberately one function per source
(state_from_pose, state_from_clock, ...), pure and separately testable: the
derivations are where a wrong reading turns into a wrong permission, and they
are easier to argue about one at a time than inside the loop that calls them.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple

from xbrain.p2_core.health.factor import FactorConfig, compute_factor
from xbrain.p2_core.health.items import ITEMS, HealthState, kind_of, level_of

#: Items with no producer in this build, and what is missing for each. The text
#: reaches the HMI and the ack of a refused recording, so it names the missing
#: THING rather than saying "unknown".
NO_SOURCE_DETAIL: Dict[str, str] = {
    "cam_rgbd": "no perception process publishes camera health",
    "lidar": "no perception process publishes lidar health",
    "compute": "thermal/memory thresholds not calibrated",
    "gpu": "no gpu health source",
    "dla": "not_used",
    "storage": "no storage health source",
}

#: 11 S3.2.1 fix quality -> rtk item state. rtk_fixed is the only quality the
#: autonomous modes are specified against; the lower ones are a real degradation
#: rather than a failure, because U34 keeps a teleop escape available on them.
_FIX_STATE = {
    "rtk_fixed": HealthState.OK,
    "rtk_float": HealthState.DEGRADED,
    "dgps": HealthState.DEGRADED,
    "single": HealthState.DEGRADED,
}


@dataclass
class _Entry:
    state: HealthState = HealthState.UNKNOWN
    detail: Optional[str] = None
    since_mono: float = 0.0


@dataclass
class HealthAggregator:
    """Holds the current state of every S5.1A item and builds the summary.

    since_mono is CLK-C1 monotonic and is set when an item CHANGES state (11
    S5.1: it drives the 14 S8.4 recover_hold_s hysteresis and the HMI's
    "degraded for 4m12s"). Re-observing the same state does not reset it --
    a flapping source would otherwise make a long-standing degradation look
    like it had just appeared.
    """
    entries: Dict[str, _Entry] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ITEMS:
            self.entries.setdefault(name, _Entry(
                detail=NO_SOURCE_DETAIL.get(name)))

    def observe(self, item: str, state: HealthState, *, now_mono_s: float,
                detail: Optional[str] = None) -> None:
        """Record one item's state. Unknown item names raise: a producer and
        this table disagreeing must not be absorbed silently."""
        entry = self.entries[item]
        if entry.state != state:
            entry.since_mono = now_mono_s
        entry.state = state
        entry.detail = detail if detail is not None else entry.detail

    def states(self) -> Dict[str, HealthState]:
        return {name: e.state for name, e in self.entries.items()}

    def build_summary(self, cfg: FactorConfig) -> Dict[str, Any]:
        """The 11 S5.1 HealthSummary payload."""
        states = self.states()
        out = compute_factor(states, cfg)
        items: Dict[str, Any] = {}
        for name, entry in sorted(self.entries.items()):
            body: Dict[str, Any] = {"state": entry.state.value,
                                    "level": level_of(name).value,
                                    "kind": kind_of(name).value}
            if entry.detail:
                body["detail"] = entry.detail
            if entry.since_mono:
                body["since_mono"] = round(entry.since_mono, 3)
            items[name] = body
        return {"schema": "health_summary_v1",
                "overall": _overall(states),
                "speed_factor": round(out.speed_factor, 3),
                "allow_motion": out.allow_motion,
                "max_profile": out.max_profile,
                "items": items}


def _overall(states: Mapping[str, HealthState]) -> str:
    """11 S5.1 overall: ok | degraded | fatal.

    fatal when a fatal-level item has failed; degraded when anything is less
    than ok -- including unknown. An unknown item is NOT rolled up as ok: the
    summary would then read `ok` on a machine whose monitors are all silent.
    """
    from xbrain.p2_core.health.items import HealthLevel

    worst = "ok"
    for name, state in states.items():
        if state == HealthState.FAIL and level_of(name) == HealthLevel.FATAL:
            return "fatal"
        if state != HealthState.OK:
            worst = "degraded"
    return worst


# -- per-source derivations -------------------------------------------------

def state_from_pose(pose: Optional[Mapping[str, Any]]
                    ) -> Tuple[Tuple[HealthState, str], Tuple[HealthState, str]]:
    """(rtk, heading) states from a state/pose body.

    No pose at all is FAIL, not unknown: P1 publishes it at 10 Hz whenever it is
    running, so its absence means the positioning chain is down -- which is a
    determinate answer, not a missing one.
    """
    if not pose:
        return ((HealthState.FAIL, "no state/pose"),
                (HealthState.FAIL, "no state/pose"))
    fix = pose.get("fix_type")
    if fix is None:
        rtk = (HealthState.FAIL, "no fix")
    else:
        rtk = (_FIX_STATE.get(fix, HealthState.DEGRADED), str(fix))
    # 11 S3.3: heading_valid is the SOLE downstream criterion (H-1); level 1 is
    # the dual-antenna solution, 2 is COG, 3 is none.
    if pose.get("heading_valid"):
        level = pose.get("heading_level")
        heading = ((HealthState.OK, "L1 dual_antenna") if level == 1
                   else (HealthState.DEGRADED, "L%s %s" % (
                       level, pose.get("heading_source"))))
    else:
        heading = (HealthState.FAIL, "heading invalid")
    return rtk, heading


def state_from_clock(clock: Optional[Mapping[str, Any]]
                     ) -> Tuple[HealthState, str]:
    """clock item from state/clock. CLK-A3 makes an absent ClockStatus
    fail-safe (sync=false), so absence and desync are the same answer here."""
    if not clock:
        return HealthState.FAIL, "no state/clock"
    if clock.get("sync"):
        return HealthState.OK, "synced via %s" % (clock.get("source") or "?")
    return HealthState.FAIL, "not synced"


def state_from_robot(robot: Optional[Mapping[str, Any]]
                     ) -> Tuple[HealthState, str]:
    """chassis item from state/robot (published by chassis_relay, CR-4).

    Absent is UNKNOWN rather than FAIL, and the asymmetry with pose above is
    deliberate: chassis_relay is a C++ process that does not exist in this
    build, so its silence means "not wired", not "the chassis dropped". Calling
    it FAIL would be equally wrong in the other direction -- it would forbid
    motion for a reason that is not evidence.
    """
    if not robot:
        return HealthState.UNKNOWN, "no state/robot (chassis_relay not wired)"
    if robot.get("hes"):
        return HealthState.FAIL, "hardware e-stop engaged"
    link = robot.get("chassis_link")
    if link is False:
        return HealthState.FAIL, "chassis link down"
    return HealthState.OK, "linked"


def state_from_power(power: Optional[Mapping[str, Any]], *,
                     critical_soc_pct: Optional[float] = None
                     ) -> Tuple[HealthState, str]:
    """battery item from state/power (chassis_relay, CR-5).

    critical_soc_pct has NO default here. 11 S5.1A marks it U-BIT-1, awaiting
    the operator, and a guessed threshold on the item that forbids motion is
    precisely what CLAUDE.md 3.1 forbids: too low and the robot strands itself,
    too high and it refuses to work. Without it the SoC is reported as a detail
    and the state stays degraded-not-failed -- visible, and not acted on.
    """
    if not power:
        return HealthState.UNKNOWN, "no state/power (chassis_relay not wired)"
    soc = power.get("soc_pct")
    if not isinstance(soc, (int, float)):
        return HealthState.UNKNOWN, "no soc_pct in state/power"
    if critical_soc_pct is None:
        return HealthState.DEGRADED, "soc %.1f%%, critical threshold not set" % soc
    if soc <= critical_soc_pct:
        return HealthState.FAIL, "soc %.1f%% at or below critical" % soc
    return HealthState.OK, "soc %.1f%%" % soc


def state_from_link(link: Optional[Mapping[str, Any]]
                    ) -> Tuple[HealthState, str]:
    """network item from state/link. TSK-20 / U36: losing the cloud does not
    stop the current task, so this is warn-level and never blocks anything --
    it is reported so the operator knows why the cloud view went stale."""
    if not link:
        return HealthState.UNKNOWN, "no state/link"
    level = link.get("level")
    if level in (None, "L0"):
        return HealthState.OK, "cloud link up"
    return HealthState.DEGRADED, "cloud link %s" % level


def refresh_health(agg: HealthAggregator, state_cache: Mapping[str, Any], *,
                   now_mono_s: float,
                   device_states: Optional[Mapping[str, Optional[bool]]] = None,
                   critical_soc_pct: Optional[float] = None) -> None:
    """Feed one round of observations into the aggregator.

    Called once per publish tick from the P2 loop. Every derivation runs every
    tick rather than on-change: the sources are last-value caches, so a source
    that stops publishing must be able to age into a worse state, and a
    change-driven update would freeze the last good reading forever.
    """
    rtk, heading = state_from_pose(state_cache.get("pose"))
    agg.observe("rtk", rtk[0], now_mono_s=now_mono_s, detail=rtk[1])
    agg.observe("heading", heading[0], now_mono_s=now_mono_s,
                detail=heading[1])
    for item, derived in (
            ("clock", state_from_clock(state_cache.get("clock"))),
            ("chassis", state_from_robot(state_cache.get("robot"))),
            ("battery", state_from_power(state_cache.get("power"),
                                         critical_soc_pct=critical_soc_pct)),
            ("network", state_from_link(state_cache.get("link")))):
        agg.observe(item, derived[0], now_mono_s=now_mono_s, detail=derived[1])
    # Device liveness (mic / ptz / payload_*) from the SW-12 bridge. None means
    # never sampled -- left as whatever it was, which for an unplumbed device is
    # the UNKNOWN it started in.
    for device, is_up in (device_states or {}).items():
        if is_up is None or device not in ITEMS:
            continue
        agg.observe(device,
                    HealthState.OK if is_up else HealthState.FAIL,
                    now_mono_s=now_mono_s,
                    detail="link up" if is_up else "link down")
