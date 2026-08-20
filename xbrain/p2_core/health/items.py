"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: items.py
Brief: BIZ-P2-18 -- the 11 S5.1A health item table (name, kind, level, roles)

Description:
11 S5.1A is the specification table for health items: which items exist, what
kind each is, how severe its failure is, whether it counts toward speed_factor,
and whether its failure forbids motion. This module is that table in code.

*** REBUILT 2026-08-20. The previous table also held nineteen items, and twelve
of them matched the contract. The other seven did not exist in S5.1A at all
(estop, payload_svc, state_link, config_freeze, persistence, ptz_home, ai_svc)
while seven contract items were missing (heading, compute, battery, gpu, dla,
storage, network). Two of the missing ones -- compute and battery -- are among
the FIVE whose failure forbids motion, so the aggregate could not refuse motion
for an overheating computer or a flat battery: the items were not there to fail.
Several levels were wrong in the same direction (rtk and clock were degraded
where the contract says fatal).

It went unnoticed because the test asserted "these safety-critical names are
present" rather than set equality. A presence check passes on any superset, so
it stayed green through a set that shared twelve members with the contract.
test_health_items_match_the_contract now extracts S5.1A and compares both
directions, the same shape as the closed-set metatest.

Three role columns, not one. 14 S8.2 computes speed_factor over one subset and
allow_motion over another, and the two are NOT the same set: rtk and heading are
fatal yet drive neither, because their motion constraint is expressed through
i_fix / i_heading and the 11 S3.2.1 hard speed caps instead -- forbidding motion
on them outright would zero-speed a teleop escape, which U34 forbids. Storing
the roles per item (rather than as three lists, or as the counts 7/12/5 that 14
explicitly forbids implementations from referencing) keeps each item's row
readable next to the contract's.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, FrozenSet


class HealthLevel(str, Enum):
    """11 S5.1A level column -- how bad this item's failure is."""
    FATAL = "fatal"           # boot BIT blocks on it
    DEGRADED = "degraded"     # failure limits capability
    WARN = "warn"             # advisory only


class HealthState(str, Enum):
    """11 S5.1 items[].state."""
    OK = "ok"
    WARN = "warn"             # off-nominal but inside tolerance (BIT-20)
    DEGRADED = "degraded"
    FAIL = "fail"
    UNKNOWN = "unknown"


class HealthKind(str, Enum):
    """11 S5.1 items[].kind. device = is the thing there and talking;
    cap = does a capability hold (clock sync, compute headroom, disk,
    battery, heading, preset library, network). The HMI and the BIT
    announcement group by this, because the two fail differently: a device
    failure removes one function, a capability failure removes the premise
    of a whole family of them."""
    DEVICE = "device"
    CAP = "cap"


@dataclass(frozen=True)
class HealthItem:
    """One S5.1A row."""
    kind: HealthKind
    level: HealthLevel
    #: 14 S8.2 step 0: does this item's factor_i take part in speed_factor?
    counts_in_speed_factor: bool
    #: 14 S8.2 step 3: does state=fail on this item set allow_motion=false?
    drives_allow_motion: bool


#: The 11 S5.1A table, transcribed row for row. Adding or removing a row is a
#: contract change (S5.1A: 新增需评审) and the metatest will fail until 11
#: agrees. ai_svc is deliberately absent: S5.1A records it as pending, and the
#: contract states the row is not merged yet.
ITEMS: Dict[str, HealthItem] = {
    # Fatal, but driving NEITHER aggregate: their motion constraint is the
    # i_fix / i_heading hard caps of 11 S3.2.1 (HL-2). Counting them here as
    # well would apply the same restriction twice -- 14 S8.2 says so in as many
    # words, and it is why the teleop escape survives an RTK loss.
    "rtk":             HealthItem(HealthKind.DEVICE, HealthLevel.FATAL,
                                  False, False),
    "heading":         HealthItem(HealthKind.CAP, HealthLevel.FATAL,
                                  False, False),
    # The five that forbid motion when they fail.
    "chassis":         HealthItem(HealthKind.DEVICE, HealthLevel.FATAL,
                                  True, True),
    "clock":           HealthItem(HealthKind.CAP, HealthLevel.FATAL,
                                  True, True),
    "compute":         HealthItem(HealthKind.CAP, HealthLevel.FATAL,
                                  True, True),
    "battery":         HealthItem(HealthKind.CAP, HealthLevel.FATAL,
                                  True, True),
    "cam_rgbd":        HealthItem(HealthKind.DEVICE, HealthLevel.FATAL,
                                  True, True),
    # Count toward speed_factor without forbidding motion.
    "lidar":           HealthItem(HealthKind.DEVICE, HealthLevel.DEGRADED,
                                  True, False),
    "gpu":             HealthItem(HealthKind.CAP, HealthLevel.DEGRADED,
                                  True, False),
    # Neither: their failure removes a function, not motion.
    "dla":             HealthItem(HealthKind.CAP, HealthLevel.WARN,
                                  False, False),
    "storage":         HealthItem(HealthKind.CAP, HealthLevel.DEGRADED,
                                  False, False),
    "ptz":             HealthItem(HealthKind.DEVICE, HealthLevel.DEGRADED,
                                  False, False),
    "mic":             HealthItem(HealthKind.DEVICE, HealthLevel.DEGRADED,
                                  False, False),
    "payload_speaker": HealthItem(HealthKind.DEVICE, HealthLevel.DEGRADED,
                                  False, False),
    "payload_siren":   HealthItem(HealthKind.DEVICE, HealthLevel.DEGRADED,
                                  False, False),
    "payload_strobe":  HealthItem(HealthKind.DEVICE, HealthLevel.DEGRADED,
                                  False, False),
    "payload_light":   HealthItem(HealthKind.DEVICE, HealthLevel.WARN,
                                  False, False),
    "speech_preset":   HealthItem(HealthKind.CAP, HealthLevel.DEGRADED,
                                  False, False),
    "network":         HealthItem(HealthKind.CAP, HealthLevel.WARN,
                                  False, False),
}


#: 11 S5.1A "BIT 专有项": produced by a one-shot boot action, NOT a continuously
#: monitored quantity, so they are part of BitReport.items and must NOT appear
#: in HealthSummary.items. The contract gives the reason: a one-shot result
#: parked in the health summary is a field that never updates again -- the
#: operator reads ptz_home: ok and does not know it is three days old.
BIT_ONLY_ITEMS: Dict[str, HealthItem] = {
    "ptz_home":        HealthItem(HealthKind.DEVICE, HealthLevel.WARN,
                                  False, False),
}


#: Backwards-compatible view used by the aggregate and the restrict matrix.
ITEM_LEVELS = {name: item.level for name, item in ITEMS.items()}
HEALTH_ITEMS: FrozenSet[str] = frozenset(ITEMS)
#: BitReport.items = the health closed set PLUS the BIT-only items (S5.1A).
BIT_REPORT_ITEMS: FrozenSet[str] = frozenset(ITEMS) | frozenset(BIT_ONLY_ITEMS)


def level_of(item: str) -> HealthLevel:
    """The level for a known item; KeyError on an unknown name.

    Raising rather than defaulting: an unrecognised item reaching the aggregate
    means the table and its producer disagree, and a default level would make
    that disagreement invisible for as long as the item stays healthy.
    """
    return ITEMS[item].level


def kind_of(item: str) -> HealthKind:
    return ITEMS[item].kind


def counts_in_speed_factor(item: str) -> bool:
    return ITEMS[item].counts_in_speed_factor


def drives_allow_motion(item: str) -> bool:
    return ITEMS[item].drives_allow_motion


def is_fatal(item: str) -> bool:
    """10 S5.4.4 assertion L: bit.quick.skip_items / non_blocking_items may not
    contain a fatal item -- that pairing was a way to switch off a safety
    self-check from YAML, with nothing guarding it."""
    return ITEMS.get(item, BIT_ONLY_ITEMS.get(item)) is not None and (
        ITEMS.get(item, BIT_ONLY_ITEMS.get(item)).level == HealthLevel.FATAL)
