"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: device_events.py
Brief: build + debounce the 11 S6.2 device_offline/device_online events (batch 6)

Description:
The three device-offline events added to 11 S6.2 on 2026-08-17 (payload / mic /
ptz) share one shape, so their producers should not each hand-roll it. This module
is that shared piece:

  build_device_event   assemble the S6.1 Event for a device transition. cat comes
                       from the device id (payload_* -> payload, mic -> voice,
                       ptz -> ptz), detail.type is device_offline|device_online,
                       detail.device is the S5.1A device id. sev is warn (offline)
                       / info (online). channel is NOT set here -- the P5 pipeline
                       derives it from cat+detail (both -> alarm, paired, E-1).
  DeviceLivenessMonitor  a tiny per-device up/down state machine with DEBOUNCE:
                       a link must be seen down for `down_threshold` consecutive
                       checks before an offline event fires (and up for
                       `up_threshold` before online), so a flapping socket does
                       not flood the cloud with offline/online pairs.

Where cloud visibility stands (Q-P5-8, CLOSED -- decision A, 2026-08-17): the cloud
broadens its real-time subscription beyond event/alarm/** to the warn/fault segments
(exact wildcard finalised with the operator at SW-4). A device_offline is sev=warn,
so once the cloud subscribes event/warn/** it arrives LIVE via the producer's direct
put (P5 is not a relay, 17 S3.5.0) -- no producer change here. Its channel=alarm +
need_ack=1 make delivery guaranteed (ack, or backfill on a genuine disconnect). NOTE
the disconnect safety net is still incomplete: the reconnect->trigger_backfill wiring
and recon (17 S3Y.3) are not yet wired into the live loop (separate follow-up).
"""

from __future__ import annotations

from typing import Callable, Optional


# 11 S5.1A device id -> 11 S6.2 event category. The four payload sub-devices all
# roll up to the payload category; mic is the voice category's device; ptz is its
# own. An id outside this map raises (closed-set discipline, 11 S13.6).
DEVICE_CATEGORY: dict = {
    "payload_speaker": "payload",
    "payload_siren": "payload",
    "payload_strobe": "payload",
    "payload_light": "payload",
    "mic": "voice",
    "ptz": "ptz",
}


class UnknownDevice(Exception):
    """device_id is not in the 11 S5.1A device set -> no category to emit under."""


def build_device_event(device_id: str, offline: bool, *, rid: str, eid: str,
                       detected_at: str, created_at: str, ts: float,
                       src: str, extra_detail: Optional[dict] = None) -> dict:
    """Assemble the record.db ev dict for a device transition. sev = warn (offline)
    / info (online); channel is left for the pipeline to derive (cat+detail ->
    alarm). extra_detail carries producer specifics (reason, socket, ...)."""
    cat = DEVICE_CATEGORY.get(device_id)
    if cat is None:
        raise UnknownDevice(
            f"{device_id!r} not in {sorted(DEVICE_CATEGORY)}")
    dtype = "device_offline" if offline else "device_online"
    detail = {"type": dtype, "device": device_id}
    if extra_detail:
        detail.update(extra_detail)
    return {
        "eid": eid, "rid": rid, "cat": cat,
        "sev": "warn" if offline else "info",
        "title": f"{device_id} {dtype}",
        "detail": detail, "src": src,
        "ts": ts, "ts_sync": 0,
        "detected_at": detected_at, "created_at": created_at,
    }


class DeviceLivenessMonitor:
    """Per-device up/down debounce. Feed it observations; it fires the emit
    callback exactly once per confirmed transition (offline once, online once),
    never on a flap that reverses before the threshold."""

    def __init__(self, device_id: str, emit: Callable[[str, bool], None],
                 down_threshold: int = 3, up_threshold: int = 2,
                 start_up: bool = True) -> None:
        if device_id not in DEVICE_CATEGORY:
            raise UnknownDevice(device_id)
        if down_threshold < 1 or up_threshold < 1:
            raise ValueError("thresholds must be >= 1")
        self._device_id = device_id
        self._emit = emit
        self._down_thr = down_threshold
        self._up_thr = up_threshold
        # Reported state (what the cloud last heard). Start 'up' so a device that
        # is healthy from boot emits nothing (no spurious online at startup).
        self._reported_up = start_up
        self._down_streak = 0
        self._up_streak = 0

    def observe(self, is_up: bool) -> Optional[bool]:
        """One liveness sample. Returns True if an OFFLINE event fired, False if an
        ONLINE event fired, None if no transition (still debouncing or unchanged).
        The emit callback is invoked with (device_id, offline_bool) on a fire."""
        if is_up:
            self._up_streak += 1
            self._down_streak = 0
            if not self._reported_up and self._up_streak >= self._up_thr:
                self._reported_up = True
                self._emit(self._device_id, False)   # online
                return False
        else:
            self._down_streak += 1
            self._up_streak = 0
            if self._reported_up and self._down_streak >= self._down_thr:
                self._reported_up = False
                self._emit(self._device_id, True)     # offline
                return True
        return None

    @property
    def reported_up(self) -> bool:
        return self._reported_up
