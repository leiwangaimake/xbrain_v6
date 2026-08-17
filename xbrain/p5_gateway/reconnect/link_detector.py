"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: link_detector.py
Brief: cloud-link reconnect edge detector -> one backfill per reconnect (17 S3.5.2)

Description:
17 S3.5.2 runs a backfill pass on the cloud link DISCONNECTED/DEGRADED -> CONNECTED
transition. Without something that FIRES on that edge, the BackfillRunner never
runs in the live loop (before this it was only called from a test), so events that
piled up during an outage are never re-sent -- zenoh does NOT buffer for an offline
subscriber, so those events are lost to the cloud until a backfill replays them.

This is deliberately MINIMAL and does ONE safe thing: detect that cloud receive
traffic (event/ack or recon/rsp) resumed after a silence, and report that single
down->up edge so the caller can trigger_backfill(). Triggering an extra backfill is
idempotent (replayed events are deduped by eid at the cloud, 11 S8.4), so a false
positive is harmless -- which is why this can run on a crude liveness signal.

What it deliberately does NOT do (and must not, to avoid a fail-silent stand-in):
  * It does NOT compute or publish state/link.cloud_link. That field is the 11 S4.6
    LinkState machine (level / disconnected_s / link_epoch / stable_s hysteresis),
    which return-to-base (NFR-12 / TSK-20) reads for real decisions. Publishing a
    half-baked 2-value cloud_link here would mis-drive that timer (LK-5). When S4.6
    lands, feed its link_epoch bump into note_reconnect() instead of this liveness.
  * It does NOT mark events delivered. Delivery marking stays conservative
    (DeliveryMarker), per S3.5.1 "宁可重发一条".

Liveness only: a cloud message seen within `timeout_s` means up; the FIRST one
after a gap longer than that is the reconnect. Starts DOWN, so the first cloud
contact after boot is itself an edge (backfill whatever accumulated before contact).
"""

from __future__ import annotations

from typing import Optional


class LinkReconnectDetector:
    """Report the cloud-link down->up edge from receive-liveness (interim signal;
    see module docstring for why it is intentionally minimal)."""

    def __init__(self, timeout_s: float, start_up: bool = False) -> None:
        if timeout_s <= 0:
            raise ValueError(f"timeout_s must be > 0, got {timeout_s}")
        self._timeout = timeout_s
        # Start down: the first cloud contact after boot is an edge, so anything
        # persisted before the cloud was reachable gets a backfill.
        self._up = start_up
        self._last_rx: Optional[float] = None

    def note_cloud_rx(self, now_mono: float) -> None:
        """Record that a cloud message (ack / recon rsp) just arrived. Called from
        the Zenoh callback thread; a lone float write is safe under the GIL."""
        self._last_rx = now_mono

    def poll(self, now_mono: float) -> bool:
        """Recompute up/down from liveness and return True on a down->up edge (the
        caller then triggers exactly one backfill). Idempotent within an 'up'
        stretch: returns True only on the transition, not while it stays up."""
        up = (self._last_rx is not None
              and (now_mono - self._last_rx) <= self._timeout)
        edge = up and not self._up
        self._up = up
        return edge

    @property
    def is_up(self) -> bool:
        return self._up
