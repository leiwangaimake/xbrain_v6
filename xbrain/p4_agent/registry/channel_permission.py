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
from typing import Any, Dict, FrozenSet, Mapping, Optional, Tuple

from xbrain.common.errors import E_CHANNEL_DENIED


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


# --- GWY-P4-42 (32.J) text-command channel admission --------------------

# cmd/voice_text carries a coarse channel token (11 S8.7.5): cloud | wecom |
# hmi. Map it to the CH-18-1 channel key. These are all TEXT channels (the
# text entry never carries a voice channel), so each maps to its *_text key.
_VOICE_TEXT_CHANNEL_MAP: Dict[str, str] = {
    "cloud": "cloud_text",
    "wecom": "wecom_text",
    "hmi": "hmi_text",
}


class ChannelAdmissionError(RuntimeError):
    """The cmd/voice_text channel token is not in the closed set."""


def channel_admission(
    intent_id: str,
    slots: Mapping[str, Any],
    channel: str,
) -> Tuple[bool, str]:
    """Decide whether a text command may be submitted over `channel`.

    Returns (allowed, code): code is '' when allowed, else E_CHANNEL_DENIED.

    channel is the cmd/voice_text token (cloud/wecom/hmi); an unknown token
    raises (closed set), never silently denies -- a typo in the gateway
    should surface, not look like a permission failure.

    Scope (deliberate): this enforces the H03f override (16 S3233) --
    H03 set_time_sync with force_step==true is allowed ONLY over the cloud
    channel (not A/T/E/H -> E_CHANNEL_DENIED). It does NOT apply the general
    per-intent CH-18-1 origin gate here, because an intent's origin letter
    (18 S13.1) is NOT its id-class prefix (A04's id-prefix 'A' is the motion
    class, not the local-voice origin) and that per-intent origin tag is not
    yet loaded into the registry. Guessing origin from the id prefix would
    wrongly deny legitimate commands (e.g. a G query over HMI). The general
    gate stays in is_channel_allowed, which takes the origin letter
    EXPLICITLY from the caller -- wire it once 18 S13.1 origins are in the
    registry. Recorded, not silently skipped.
    """
    key = _VOICE_TEXT_CHANNEL_MAP.get(channel)
    if key is None:
        raise ChannelAdmissionError(
            "channel %r not in %s"
            % (channel, sorted(_VOICE_TEXT_CHANNEL_MAP)))

    # H03f: force_step==true -> cloud only (16 S3233 / 18 S10.2A).
    if intent_id == "H03" and slots.get("force_step") is True:
        if channel != "cloud":
            return (False, E_CHANNEL_DENIED)
        return (True, "")

    return (True, "")
