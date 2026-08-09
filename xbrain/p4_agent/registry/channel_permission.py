"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: channel_permission.py
Brief: GWY-P4-24 -- CH-18-1 origin-letter -> channel permission

Description:
18 S13.1 tags every intent with an origin letter (A/B/C/D/E/...):
  A local voice        (mic origin, on-machine)
  B cloud voice        (LAN cloud ASR)
  C cloud text (HMI)
  D cloud text (wecom)
  E remote voice       (wecom voice channel)
  ...

CH-18-1: an intent's origin letter determines which channels are
ALLOWED to submit it. Three default rules govern the mapping.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, FrozenSet


# Origin letter -> set of channel keys that are ALLOWED to submit
# intents of this origin. Missing letter -> deny all channels.
_ORIGIN_TO_CHANNELS: Dict[str, FrozenSet[str]] = {
    "A": frozenset({"local_mic"}),
    "B": frozenset({"cloud_voice"}),
    "C": frozenset({"hmi_text", "cloud_text"}),
    "D": frozenset({"wecom_text"}),
    "E": frozenset({"wecom_voice"}),
    "F": frozenset({"local_mic", "hmi_text"}),   # F = shared local
    "G": frozenset({"local_mic", "hmi_text",
                     "cloud_text", "cloud_voice",
                     "wecom_text", "wecom_voice"}),   # G = all channels
    "H": frozenset({"cloud_text"}),               # H = ops
    "I": frozenset({"local_mic", "hmi_text"}),    # I = interactive
    "J": frozenset({"local_mic", "hmi_text",
                     "cloud_text"}),               # J = wide
}


def is_channel_allowed(origin_letter: str, channel: str) -> bool:
    """CH-18-1: return True iff channel is in the origin's allowed
    set. Unknown origin -> False (deny)."""
    allowed = _ORIGIN_TO_CHANNELS.get(origin_letter)
    if allowed is None:
        return False
    return channel in allowed


def allowed_channels(origin_letter: str) -> FrozenSet[str]:
    return _ORIGIN_TO_CHANNELS.get(origin_letter, frozenset())
