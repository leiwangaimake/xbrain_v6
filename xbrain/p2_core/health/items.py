"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: items.py
Brief: BIZ-P2-18 -- 19 health items + closed-set severity levels

Description:
11 S5.1A defines the health item closed set. Each item has:
  * name (closed set)
  * level (fatal | degraded | warn)
  * state (ok | degraded | fail | unknown)

BIZ-P2-18 spec: 19 items (LID-1 merged, ai_svc pending; see 11 v0.6
Q-P2-8 closed). Names below reflect the merged state.
"""

from __future__ import annotations

from enum import Enum
from typing import FrozenSet


class HealthLevel(str, Enum):
    """11 S5.1A level column."""
    FATAL = "fatal"           # this item down = system unusable
    DEGRADED = "degraded"     # this item down = reduced capability
    WARN = "warn"             # this item down = advisory


class HealthState(str, Enum):
    OK = "ok"
    DEGRADED = "degraded"
    FAIL = "fail"
    UNKNOWN = "unknown"


# 11 S5.1A health item closed set (19 after LID-1 merge). Each item's
# level per doc. Additions require 11 doc change; removals same.
ITEM_LEVELS = {
    # RT-plane critical
    "chassis":         HealthLevel.FATAL,
    "cam_rgbd":        HealthLevel.FATAL,
    "lidar":           HealthLevel.DEGRADED,     # LID-1 merged
    "estop":           HealthLevel.FATAL,
    "clock":           HealthLevel.DEGRADED,
    "rtk":             HealthLevel.DEGRADED,
    "ptz":             HealthLevel.DEGRADED,
    "mic":             HealthLevel.DEGRADED,
    # payload family (four rows per 14 S8.5)
    "payload_svc":     HealthLevel.DEGRADED,
    "payload_speaker": HealthLevel.DEGRADED,
    "payload_siren":   HealthLevel.DEGRADED,
    "payload_strobe":  HealthLevel.DEGRADED,
    "payload_light":   HealthLevel.DEGRADED,
    # link + config + persistence
    "state_link":      HealthLevel.DEGRADED,
    "config_freeze":   HealthLevel.FATAL,
    "persistence":     HealthLevel.DEGRADED,
    # BIT auxiliary
    "ptz_home":        HealthLevel.WARN,
    "speech_preset":   HealthLevel.WARN,
    "ai_svc":          HealthLevel.DEGRADED,
}


HEALTH_ITEMS: FrozenSet[str] = frozenset(ITEM_LEVELS.keys())


def level_of(item: str) -> HealthLevel:
    """Return the level for a known item; raise KeyError on unknown."""
    return ITEM_LEVELS[item]


def is_fatal(item: str) -> bool:
    """BIT-G1: skip_items / non_blocking_items may NOT contain any
    fatal item. Helper for that check."""
    return ITEM_LEVELS.get(item) == HealthLevel.FATAL
