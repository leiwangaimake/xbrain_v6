"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: device_health_bridge.py
Brief: p2 device liveness -> 11 S6.2 device_offline/device_online events

Description:
p2_core owns the device links (mic via arecord, payload via PayloadDomain ->
payload-service GZH-2 sockets, ptz via PtzDomain ONVIF), so it is where a device
dropping is first observable. This bridge turns that into the 11 S6.2 events added
2026-08-17: it holds one DeviceLivenessMonitor per device and, on a confirmed
transition, builds the event (build_device_event) and hands it to an injected emit
callback (the p2 GEN publisher onto event/{sev}/{cat}). P5 then persists it and
backfills it to the cloud (SW-12).

The liveness SIGNAL is fed by the wiring per device: observe(device_id, is_up).
is_up=None means 'unknown this tick' (no device/endpoint) and feeds nothing -- so
a device whose liveness is not yet plumbed (payload/ptz until their clients expose
a reachability check, GATED-HW) simply emits no events, while MIC (arecord thread
alive/dead, real today) drives real offline/online. The debounce lives in the
monitor: a flap does not flood the cloud.

Reuses the pure helpers in p5_gateway.event.device_events (build_device_event +
DeviceLivenessMonitor) -- they carry no p5 runtime dependency, they are the
producer-side event shape (a shared-location move is a later cleanup).
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

from xbrain.p5_gateway.event.device_events import (
    DeviceLivenessMonitor, build_device_event,
)

_logger = logging.getLogger("xbrain.p2.device_health")


class DeviceHealthBridge:
    """Per-device liveness -> device event, via an injected emit callback."""

    def __init__(self, rid: str, emit: Callable[[dict], None],
                 now_iso: Callable[[], str], eid_gen: Callable[[str, bool], str],
                 down_threshold: int = 3, up_threshold: int = 2) -> None:
        self._rid = rid
        self._emit = emit
        self._now_iso = now_iso
        self._eid_gen = eid_gen
        self._down_thr = down_threshold
        self._up_thr = up_threshold
        self._monitors: dict = {}

    def register(self, device_id: str) -> None:
        """Add a monitor for one 11 S5.1A device id. Its emit builds the event and
        forwards it. Registering twice is a no-op (idempotent wiring)."""
        if device_id in self._monitors:
            return
        self._monitors[device_id] = DeviceLivenessMonitor(
            device_id, emit=self._on_transition,
            down_threshold=self._down_thr, up_threshold=self._up_thr)

    def observe(self, device_id: str, is_up: Optional[bool]) -> None:
        """One liveness sample for a device. None = unknown this tick (feed
        nothing); True/False advance the debounce. Unknown never fabricates a
        transition, so an unplumbed device stays silent (not a false 'online')."""
        if is_up is None:
            return
        mon = self._monitors.get(device_id)
        if mon is None:
            return
        mon.observe(is_up)

    def _on_transition(self, device_id: str, offline: bool) -> None:
        """A monitor confirmed a transition -> build + emit the event."""
        try:
            ev = build_device_event(
                device_id, offline, rid=self._rid,
                eid=self._eid_gen(device_id, offline),
                detected_at=self._now_iso(), created_at=self._now_iso(),
                ts=0.0, src="p2_core")
            self._emit(ev)
            _logger.info("device %s %s emitted", device_id,
                         "offline" if offline else "online")
        except Exception as exc:  # noqa: BLE001 -- a bad emit must not kill p2
            _logger.warning("device event emit failed (%s): %s",
                            type(exc).__name__, exc)
