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
        # Per-device OFFLINE detail addendum (reason [+ socket]) per 11 S6.2. Kept
        # here, not in build_device_event, because it is a wiring fact (which link /
        # failure mode this device has), not intrinsic to the event shape.
        self._offline_detail: dict = {}
        # Devices whose liveness has actually been SAMPLED at least once.
        # DeviceLivenessMonitor starts in the reported-up state on purpose (so a
        # device that is fine at boot emits no spurious online event), which
        # means reported_up cannot distinguish "up" from "never looked". The
        # health summary must make that distinction -- reporting an unsampled
        # device as ok is the fail-silent shape -- so the fact of having been
        # sampled is recorded here rather than inferred from the monitor.
        self._sampled: set = set()

    def register(self, device_id: str,
                 offline_detail: Optional[dict] = None) -> None:
        """Add a monitor for one 11 S5.1A device id. offline_detail is merged into
        the detail of the OFFLINE event only (the 11 S6.2 'reason'/'socket'
        evidence -- e.g. ptz reason=onvif_unreachable, payload socket=8519); the
        online event stays {type, device}, since a failure 'reason' is meaningless
        on a recovery. Registering twice is a no-op (idempotent wiring)."""
        if device_id in self._monitors:
            return
        if offline_detail:
            self._offline_detail[device_id] = dict(offline_detail)
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
        self._sampled.add(device_id)
        mon.observe(is_up)

    def states(self) -> dict:
        """{device_id: True | False | None} -- up, down, or never sampled.

        None is what keeps an unplumbed device out of the health summary as
        UNKNOWN instead of ok. GATED-HW devices (payload / ptz until their
        clients expose a reachability check) live in that state for now.
        """
        return {dev: (mon.reported_up if dev in self._sampled else None)
                for dev, mon in self._monitors.items()}

    def _on_transition(self, device_id: str, offline: bool) -> None:
        """A monitor confirmed a transition -> build + emit the event."""
        try:
            # 11 S6.2: attach reason/socket on the OFFLINE event only (evidence of
            # what went down); the paired online carries just {type, device}.
            extra = self._offline_detail.get(device_id) if offline else None
            ev = build_device_event(
                device_id, offline, rid=self._rid,
                eid=self._eid_gen(device_id, offline),
                detected_at=self._now_iso(), created_at=self._now_iso(),
                ts=0.0, src="p2_core", extra_detail=extra)
            self._emit(ev)
            _logger.info("device %s %s emitted", device_id,
                         "offline" if offline else "online")
        except Exception as exc:  # noqa: BLE001 -- a bad emit must not kill p2
            _logger.warning("device event emit failed (%s): %s",
                            type(exc).__name__, exc)
