"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: link_state.py
Brief: 11 S4.6 LinkState -- cloud-link state machine (disconnected_s + L0..L3)

Description:
P5 is the sole authority for cloud-link state (11 S7.1A): it is the only process
holding the cloud socket, so it alone can say how long the cloud has been out of
contact. That answer (level 0..3 + disconnected_s) is the ONE judge for return-to-
base (NFR-12 / TSK-20..22) and the reconnect signal for the event backfill; every
other process reads `level`, never recomputes it (LNK-6). This module is that pure
state machine (11 S4.6.3), driven only by cloud-rx timestamps + injected thresholds
so it tests against a synthetic monotonic timeline with no cloud.

The six mandates it enforces (11 S4.6.3 LNK-1..6), each easy to get subtly wrong:
  LNK-1  CLOCK_MONOTONIC only. disconnected_s is NEVER a wall-clock subtraction --
         the outage is exactly when the wall clock is most likely unsynced (NTP is
         on the same dead link). now_mono is injected; no clock is read here.
  LNK-2  the outage timer starts at last_rx_mono (last time we HEARD the cloud), not
         at the moment we declared it down -- else every outage under-counts by one
         degraded_s window.
  LNK-3  hysteresis: after rx resumes, the link must be continuously reachable for
         stable_s before disconnected_s clears. Without it a link that is up 1 s
         every 4 min keeps the return-to-base timer alive forever -- and "up now,
         down now" is the COMMON campus-network failure, not an edge case. A flap
         does NOT reset down_since, so disconnected_s keeps growing across flaps.
  LNK-4  no cross-restart continuation. A fresh process re-starts the timer from its
         own boot (gw_start_mono); consumers see gw_start_mono change and know the
         timer reset. (Labelling the reason 'gateway_restart' vs 'never_connected'
         needs a persisted "I ran before" marker -- not wired yet; a fresh boot is
         never_connected per LNK-5, and gw_start_mono still surfaces the restart.)
  LNK-5  cold start that never connected is NOT 'up'. It is reason=never_connected
         with down_since = gw_start, so the L2/L3 limits are in force exactly when
         they matter most -- a robot that never reached the cloud must not run as if
         supervised.
  LNK-6  level is decided HERE only; consumers read it, they do not re-judge by
         seconds (or P2 thinks connected while P3 is returning home).

The reconnect edge (rx resumed after an outage) is surfaced as .reconnected on the
snapshot so the wiring fires ONE event backfill per reconnect -- this subsumes the
interim LinkReconnectDetector. transport_error / router_down reasons need lower-level
zenoh signals not wired yet; a gap-based outage is reason=heartbeat_timeout.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# reason closed set (11 S4.6.2). transport_error / router_down need signals P5 does
# not have yet; the observable outage is heartbeat_timeout.
REASON_OK = "ok"
REASON_NEVER = "never_connected"
REASON_TIMEOUT = "heartbeat_timeout"
REASON_RESTART = "gateway_restart"

CLOUD_UP = "up"
CLOUD_DEGRADED = "degraded"
CLOUD_DOWN = "down"


@dataclass(frozen=True)
class LinkThresholds:
    """The 11 S4.6.2 thresholds, injected (never defaulted in code, CLAUDE.md 3.1):
    degraded_s L0->L1, down_s L1->L2, rtb_s L2->L3 (None = L3 disabled, the fail-safe
    while TSK-21 rtb_s is undefined), stable_s the LNK-3 hysteresis window."""

    degraded_s: float
    down_s: float
    rtb_s: Optional[float]
    stable_s: float

    def __post_init__(self) -> None:
        for name in ("degraded_s", "down_s", "stable_s"):
            v = getattr(self, name)
            if v is None or v <= 0:
                raise ValueError(f"{name} must be > 0, got {v!r}")
        if self.down_s <= self.degraded_s:
            raise ValueError("down_s must exceed degraded_s")
        if self.rtb_s is not None and self.rtb_s <= self.down_s:
            raise ValueError("rtb_s, when set, must exceed down_s")


@dataclass(frozen=True)
class LinkSnapshot:
    """One evaluate() result -- the fields P5 publishes on state/link (11 S4.6.2)
    plus `reconnected`, the once-per-outage edge that drives the event backfill."""

    cloud_link: str
    level: int
    disconnected_s: float
    to_next_level_s: Optional[float]
    reason: str
    last_rx_mono: float
    link_epoch: int
    gw_start_mono: float
    reconnected: bool


class LinkStateMachine:
    """The 11 S4.6.3 algorithm. Feed cloud-rx timestamps via on_cloud_rx() (from the
    Zenoh callback thread) and call evaluate(now_mono) at 1 Hz; read the snapshot."""

    def __init__(self, thresholds: LinkThresholds, gw_start_mono: float) -> None:
        self._th = thresholds
        self._gw_start = gw_start_mono
        # LNK-5 cold start: never connected, NOT up. The outage clock starts at boot
        # so L2/L3 limits engage if the cloud is never reached.
        self._last_rx_mono = gw_start_mono
        self._down_since_mono: Optional[float] = gw_start_mono
        self._reconnect_mono: Optional[float] = None
        self._link_epoch = 0
        self._reason = REASON_NEVER
        self._cloud_link = CLOUD_DOWN
        self._level = 2
        self._disconnected_s = 0.0
        # Armed so the FIRST cloud contact backfills whatever was persisted before
        # the cloud was reachable (e.g. events from before a restart).
        self._backfill_pending = True
        # LNK-5: until the FIRST cloud message arrives, a small gap must NOT read as
        # reachable -- a just-booted P5 has a tiny gap but has heard nothing, and
        # reporting 'up' there would silently disable the L2/L3 limits.
        self._ever_rx = False

    def on_cloud_rx(self, now_mono: float) -> None:
        """Any cloud message (heartbeat pong / cmd / event ack / recon rsp) refreshes
        the outage clock (11 S4.6.3 step 1). If we were down, remember WHEN rx
        resumed (reconnect_mono) but do NOT clear yet -- LNK-3 hysteresis clears it
        only after stable_s. A lone float write is safe under the GIL."""
        self._ever_rx = True
        self._last_rx_mono = now_mono
        if self._cloud_link != CLOUD_UP and self._reconnect_mono is None:
            self._reconnect_mono = now_mono

    def evaluate(self, now_mono: float) -> LinkSnapshot:
        """1 Hz evaluation (11 S4.6.3 step 2). Recompute level / cloud_link /
        disconnected_s and return the snapshot. Sets reconnected=True on the single
        tick the link first becomes reachable again after an outage."""
        th = self._th
        gap = now_mono - self._last_rx_mono
        reconnected = False

        # LNK-5: not reachable until the cloud has been heard from at least once.
        if self._ever_rx and gap < th.degraded_s:
            # Reachable right now.
            in_window = (self._down_since_mono is not None
                         and self._reconnect_mono is not None
                         and (now_mono - self._reconnect_mono) < th.stable_s)
            # Fire the backfill the first tick we are reachable after an outage,
            # whether we enter the observation window or clear straight through.
            if self._backfill_pending:
                reconnected = True
                self._backfill_pending = False
            if in_window:
                # LNK-3 observation window: still 'degraded', keep accumulating so a
                # flap-then-recover cannot reset the return-home timer.
                self._level = 1
                self._cloud_link = CLOUD_DEGRADED
                self._disconnected_s = now_mono - self._down_since_mono
            else:
                # Stable long enough (or was never down) -> up, outage over.
                self._level = 0
                self._cloud_link = CLOUD_UP
                self._reason = REASON_OK
                self._disconnected_s = 0.0
                self._down_since_mono = None
                self._reconnect_mono = None
        else:
            # Unreachable right now.
            if self._down_since_mono is None:
                # A NEW outage: start the clock at last-heard (LNK-2), bump the epoch
                # (RTB idempotency), and arm the backfill for when it comes back.
                self._down_since_mono = self._last_rx_mono
                self._link_epoch += 1
                self._backfill_pending = True
                if self._reason != REASON_RESTART:
                    self._reason = REASON_TIMEOUT
            self._reconnect_mono = None
            self._disconnected_s = now_mono - self._down_since_mono
            if self._disconnected_s < th.down_s:
                self._level = 1
            elif th.rtb_s is None or self._disconnected_s < th.rtb_s:
                self._level = 2
            else:
                self._level = 3
            self._cloud_link = CLOUD_DEGRADED if self._level == 1 else CLOUD_DOWN

        return self._snapshot(now_mono, reconnected)

    def _snapshot(self, now_mono: float, reconnected: bool) -> LinkSnapshot:
        return LinkSnapshot(
            cloud_link=self._cloud_link,
            level=self._level,
            disconnected_s=round(self._disconnected_s, 3),
            to_next_level_s=self._to_next_level(),
            reason=self._reason,
            last_rx_mono=self._last_rx_mono,
            link_epoch=self._link_epoch,
            gw_start_mono=self._gw_start,
            reconnected=reconnected,
        )

    def _to_next_level(self) -> Optional[float]:
        """Seconds to the next level for the HMI countdown (11 S4.6.2). null at L0
        (nothing counting down) and L3 / rtb_s unset (nowhere further to go)."""
        th = self._th
        if self._level == 1:
            return round(th.down_s - self._disconnected_s, 3)
        if self._level == 2 and th.rtb_s is not None:
            return round(th.rtb_s - self._disconnected_s, 3)
        return None
