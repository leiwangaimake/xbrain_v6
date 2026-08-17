"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: channel_map.py
Brief: 11 S6.2 channel derivation -- category (+ detail.type/kind) -> normal|alarm

Description:
The backfill channel is NOT the producer's free choice: 11 S6.2 hardcodes it per
category, and the producer must apply that fixed value ("发送方不得临时决定",
17 S3.3). This module is the single in-code encoding of that mapping, so the
pipeline derives channel here and no producer can put an event on the wrong
backfill cursor -- a stuck-alarm-state bug (E-1) or a lost recovery.

Why channel matters (17 S3.5): the two backfill cursors (normal / alarm) are
physically separate and rate-limited independently. An alarm-worthy event on the
normal cursor arrives late (or is dropped) after a reconnect; a recovery on the
wrong cursor from its breach leaves the cloud stuck in alarm state forever. So
the paired events (breach/recovered, fence_degraded/fence_restored, and the
device_offline/device_online pair added 2026-08-17) MUST ride the SAME channel --
alarm -- exactly as 11 S9A.9 E-1 requires.

Most categories are single-channel; a few are per-detail:
  * fence   default alarm, but soft_enter/soft_exit/hard_clip/fence_changed/
            fence_stage_failed ride normal (11 S6.2 fence sub-table).
  * estop   default alarm, but estop.hes_cleared / estop.unlock ride normal
            (11 S6.2 estop note).
  * payload/ptz/voice  default normal, but device_offline + device_online ride
            alarm together (paired, E-1).

The category->default map is asserted COMPLETE against EVENT_CATEGORY by the
metatest: a category added to the closed set without a channel here fails the
test, so it can never fall through to a silent default (3.2).
"""

from __future__ import annotations

from typing import Optional

from xbrain.common.enums import EVENT_CATEGORY


# Per-category DEFAULT channel (11 S6.2). Every value in EVENT_CATEGORY must have
# an entry here (metatest-enforced). Categories with per-detail exceptions list
# their default here and the exceptions in _DETAIL_OVERRIDES below.
CATEGORY_CHANNEL: dict = {
    "intrusion": "alarm",
    "fence": "alarm",          # sub-kinds override to normal
    "speed_limit": "normal",
    "mode_change": "normal",
    "arbitration": "normal",
    "task": "normal",
    "charging": "normal",
    "health": "normal",        # 11 S6.2: even sev=fault stays normal channel
    "bit": "normal",
    "chassis": "alarm",
    "estop": "alarm",          # sub-types override to normal
    "comm": "normal",          # disconnected -> cannot send anyway, backfill moot
    "rtk": "alarm",
    "geo": "normal",
    "teach": "normal",
    "teleop": "normal",
    "perception": "normal",
    "motion": "normal",
    "payload": "normal",       # device_offline/online override to alarm
    "data": "normal",
    "system": "normal",
    "ptz": "normal",           # device_offline/online override to alarm
    "voice": "normal",         # device_offline/online override to alarm
}


# (category, detail_type) -> channel, ONLY where it differs from the category
# default. detail_type is Event.detail.type OR Event.detail.kind (fence uses
# kind, estop/device use type). A pair (offline/online) shares the same channel.
_DETAIL_OVERRIDES: dict = {
    # fence: default alarm; these five ride normal (11 S6.2 fence sub-table).
    ("fence", "soft_enter"): "normal",
    ("fence", "soft_exit"): "normal",
    ("fence", "hard_clip"): "normal",
    ("fence", "fence_changed"): "normal",
    ("fence", "fence_stage_failed"): "normal",
    # estop: default alarm; the two "cleared/unlocked" info events ride normal.
    ("estop", "estop.hes_cleared"): "normal",
    ("estop", "estop.unlock"): "normal",
    # device offline/online (2026-08-17): default-normal categories, but a device
    # outage + its recovery ride alarm together (E-1: never leave cloud stuck).
    ("payload", "device_offline"): "alarm",
    ("payload", "device_online"): "alarm",
    ("ptz", "device_offline"): "alarm",
    ("ptz", "device_online"): "alarm",
    ("voice", "device_offline"): "alarm",
    ("voice", "device_online"): "alarm",
}


class ChannelDerivationError(Exception):
    """cat was not a known event category, so no channel can be derived. Raising
    (not defaulting) keeps an off-contract category out of the event stream --
    11 S13.6 closed-set discipline, not a silent 'normal'."""


def detail_type_of(detail: Optional[dict]) -> Optional[str]:
    """The sub-event key a category may switch channel on: detail.type first
    (estop/rtk/device), else detail.kind (fence). None when the detail carries
    neither -- then the category default applies."""
    if not detail:
        return None
    return detail.get("type") or detail.get("kind")


def derive_channel(cat: str, detail: Optional[dict] = None) -> str:
    """The 11 S6.2 channel for this event. A (cat, detail_type) override wins over
    the category default; an unknown cat raises. This is the ONLY place channel is
    decided -- the pipeline calls it and overwrites any producer-supplied channel,
    so S3.3 ("发送方不得临时决定") holds by construction."""
    if cat not in EVENT_CATEGORY:
        raise ChannelDerivationError(f"unknown event category: {cat!r}")
    dt = detail_type_of(detail)
    if dt is not None:
        override = _DETAIL_OVERRIDES.get((cat, dt))
        if override is not None:
            return override
    return CATEGORY_CHANNEL[cat]
