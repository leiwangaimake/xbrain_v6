"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: episodes.py
Brief: 11 S9A.9 fence events from FenceEval transitions -- episodes, dedup keys, E-1/E-2/E-3

Description:
The clip (clip.py) says what is true this tick; the cloud and the HMI need
to know when it CHANGED. This tracker turns consecutive FenceEval values into
the 11 S9A.9 event rows for constraining polygons:
  soft_enter (warn)   d_eff first < margin_soft_eff
  hard_clip  (warn)   first outward-component clip in this episode (E-3: not
                      repeated while clipping continues)
  breach     (alarm)  d_nom < 0, the NOMINAL boundary crossed
  recovered  (info)   back to d_nom > 0
  soft_exit  (info)   back out of the soft band -- ends the episode
  fence_clip_degenerate (alarm)  projection had no inward direction
  fence_degraded / fence_restored (alarm / info)  enforcement left / regained
                      full (set-level, dedup fence:degraded:{reason} 300 s)

Episode (E-2): one counter per polygon; every event of one excursion carries
the same id, soft_exit is the last one and the counter increments after it,
so the cloud can stitch soft_enter -> hard_clip -> breach -> recovered ->
soft_exit into a single process. A breach that starts without a soft_enter
(position jump) still uses the current counter. Channels are NOT decided
here (E-1: p5 derives channel from the category / kind table); dedup keys
are the S9A.9 strings so p5's 60 s / 300 s windows work unchanged.

Not done here: zone_enter / zone_exit (zones.py, warning polygons, point
in polygon); fence_lost / fence_changed / fence_stage_failed (the holder
and the ack path own those). A rev change resets the per-polygon memory:
ids may be reused across sets and a stale 'in soft band' flag would swallow
the first soft_enter of the new geometry.

Looks right, is wrong: emitting hard_clip on every clipped tick. 20 Hz along
a fence is a thousand events a minute (EVT-14); E-3 says once per episode.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from xbrain.p1_motion.fence.clip import ENFORCEMENT_FULL, FenceEval


@dataclass(frozen=True)
class FenceEvent:
    """One 11 S9A.9 row. poly_id / role are "" for set-level kinds."""
    kind: str
    severity: str
    poly_id: str
    poly_name: str
    role: str
    episode_id: int
    dedup_key: str
    d_nom_m: Optional[float] = None
    d_eff_m: Optional[float] = None


class FenceEpisodeTracker:
    """Per-polygon soft / breach / clip memory + episode counters, and the
    set-level enforcement memory. Driven from the 20 Hz thread only."""

    def __init__(self) -> None:
        self._rev: Optional[int] = None
        self._soft: Dict[str, bool] = {}
        self._breach: Dict[str, bool] = {}
        self._clipped: Dict[str, bool] = {}
        self._episode: Dict[str, int] = {}
        self._enforcement: Optional[str] = None
        self._degenerate = False

    def _reset_polys(self) -> None:
        self._soft.clear()
        self._breach.clear()
        self._clipped.clear()
        self._episode.clear()
        self._degenerate = False

    def observe(self, ev: FenceEval, *, rev: Optional[int]) -> List[FenceEvent]:
        """The events this tick, in the E-2 order soft_enter -> hard_clip ->
        breach / recovered -> soft_exit per polygon, then the degenerate row.
        mutant: skip the episode increment on soft_exit -> every excursion
        shares episode 0 and the cloud cannot separate them ->
        test_exit_ends_the_episode red."""
        out: List[FenceEvent] = []
        if rev != self._rev:
            self._reset_polys()
            self._rev = rev
        prev = self._enforcement
        if prev is not None and prev != ev.enforcement:
            if ev.enforcement != ENFORCEMENT_FULL:
                out.append(FenceEvent("fence_degraded", "alarm", "", "", "", 0,
                                      "fence:degraded:%s" % ev.degrade_reason))
            elif prev != ENFORCEMENT_FULL:
                out.append(FenceEvent("fence_restored", "info", "", "", "", 0,
                                      "fence:restored"))
        self._enforcement = ev.enforcement
        if ev.enforcement != ENFORCEMENT_FULL or ev.margin_soft_eff_m is None:
            # nothing is being judged: the per-polygon memory stays as it was
            # (a fix dropout mid-episode must not fabricate a soft_exit).
            return out
        msoft = ev.margin_soft_eff_m
        for pd in ev.per_poly:
            pid = pd.poly_id
            ep = self._episode.get(pid, 0)
            soft_now = pd.d_eff_m < msoft
            breach_now = pd.d_nom_m < 0.0
            clip_now = pd.hard_now and ev.clipped
            was_soft = self._soft.get(pid, False)
            was_breach = self._breach.get(pid, False)
            if soft_now and not was_soft:
                out.append(FenceEvent("soft_enter", "warn", pid, pd.name, pd.role, ep,
                                      "fence:soft:%s:%d" % (pid, ep), pd.d_nom_m, pd.d_eff_m))
            if clip_now and not self._clipped.get(pid, False):
                out.append(FenceEvent("hard_clip", "warn", pid, pd.name, pd.role, ep,
                                      "fence:clip:%s:%d" % (pid, ep), pd.d_nom_m, pd.d_eff_m))
                self._clipped[pid] = True
            if breach_now and not was_breach:
                out.append(FenceEvent("breach", "alarm", pid, pd.name, pd.role, ep,
                                      "fence:breach:%s:%d" % (pid, ep), pd.d_nom_m, pd.d_eff_m))
            elif was_breach and not breach_now:
                out.append(FenceEvent("recovered", "info", pid, pd.name, pd.role, ep,
                                      "fence:recovered:%s:%d" % (pid, ep), pd.d_nom_m, pd.d_eff_m))
            self._breach[pid] = breach_now
            if was_soft and not soft_now:
                out.append(FenceEvent("soft_exit", "info", pid, pd.name, pd.role, ep,
                                      "fence:soft_exit:%s:%d" % (pid, ep), pd.d_nom_m, pd.d_eff_m))
                self._episode[pid] = ep + 1          # E-2: the excursion is over
                self._clipped[pid] = False           # E-3: next episode may clip again
            self._soft[pid] = soft_now
        if ev.degenerate and not self._degenerate:
            pid = ev.poly_id or ""
            out.append(FenceEvent("fence_clip_degenerate", "alarm", pid, ev.poly_name,
                                  ev.role or "", self._episode.get(pid, 0),
                                  "fence:degenerate:%s" % pid, ev.d_nom_m, ev.d_eff_m))
        self._degenerate = ev.degenerate
        return out


__all__ = ["FenceEvent", "FenceEpisodeTracker"]
