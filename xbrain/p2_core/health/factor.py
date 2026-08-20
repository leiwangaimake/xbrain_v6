"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: factor.py
Brief: BIZ-P2-19 -- speed_factor / allow_motion / max_profile (14 S8.2, S8.3)

Description:
Aggregates the S5.1A item states into the triple P1 consumes:

  allow_motion (bool)   -- may the robot move at all
  speed_factor (0..1)   -- the health multiplier on the speed gate
  max_profile (str)     -- the highest admissible profile

*** REBUILT 2026-08-20 against 14 S8.2. The previous implementation differed
from the specified algorithm in three ways, each changing the answer:

  1. speed_factor was a PRODUCT of the per-item factors; 14 S8.2 step 2 says
     min, and says why: a product compounds several mild degradations into an
     unreasonable value (0.7 cubed is 0.34), where min stays predictable and
     explainable to whoever is standing next to a slow robot.
  2. state=unknown multiplied 0.5 unconditionally. 14 S8.2 answer 1 rules that
     LEVEL WINS: a warn-level item is 1.0 in every state. dla, network and
     payload_light are necessarily unknown before boot completes, so the old
     rule dropped the speed of every machine by half at every startup, with
     nothing on site to explain it.
  3. every item was multiplied in, including rtk and heading. 14 S8.2 step 0
     filters to the items S5.1A marks as counting -- rtk and heading are
     constrained by the i_fix / i_heading caps of 11 S3.2.1 instead, and
     counting them here applies the same limit twice (12 S6.6 multiplies
     h_factor by i_factor afterwards).

And one that changed nothing yet but would have: allow_motion was returned True
even when max_profile came out "none", i.e. "you may move, at no profile".
14 S8.3 closes that: if not even obstacle_avoid is admissible, allow_motion is
false.

The numbers 7 / 12 / 5 (how many items are in each role) are deliberately NOT
written here. 14 S8.2 states in as many words that they are the current value of
a derived quantity, not the contract, and that implementations must not
reference them; the contract is the per-item role columns in S5.1A, which
items.py carries.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping

from xbrain.p2_core.health.items import (
    HealthLevel, HealthState, counts_in_speed_factor, drives_allow_motion,
    level_of,
)


@dataclass(frozen=True)
class FactorConfig:
    """The 14 S8.2 step-1 factor table, from p2_core.yaml health.factors."""
    fatal_degraded: float       # fatal item, state degraded
    degraded_fail: float        # degraded item, state fail
    degraded_degraded: float    # degraded item, state degraded
    unknown: float              # fatal/degraded item, state unknown


@dataclass(frozen=True)
class FactorOutput:
    allow_motion: bool
    speed_factor: float
    max_profile: str            # "obstacle_avoid" | "patrol" | "none"


def factor_for(item: str, state: HealthState, cfg: FactorConfig) -> float:
    """14 S8.2 step 1: factor_i from (state, level).

    The table, with the two answers S8.2 added for cases v0.2 left undefined:
      * level=warn  -> 1.0 in EVERY state, unknown included (answer 1).
      * fatal+warn  -> 1.0: warn means off-nominal but within tolerance
        (BIT-20), and within tolerance is not a reason to slow down (answer 2).
      * fatal+fail  -> handled in step 3, not here; such an item has already
        been filtered out, so its factor is 1.0 (answer 3).
    """
    level = level_of(item)
    if level == HealthLevel.WARN:
        return 1.0
    if state in (HealthState.OK, HealthState.WARN):
        return 1.0
    if state == HealthState.UNKNOWN:
        return cfg.unknown
    if level == HealthLevel.FATAL:
        # DEGRADED (FAIL is step 3's, and reaches here only for a fatal item
        # that does not drive allow_motion -- answer 3 gives it 1.0).
        return cfg.fatal_degraded if state == HealthState.DEGRADED else 1.0
    # DEGRADED level.
    return cfg.degraded_fail if state == HealthState.FAIL \
        else cfg.degraded_degraded


def compute_factor(item_states: Mapping[str, HealthState],
                   cfg: FactorConfig) -> FactorOutput:
    """The 14 S8.2 / S8.3 aggregate.

    item_states may be partial: an item nobody reported is treated as UNKNOWN
    rather than skipped. Skipping it would make a missing monitor look like a
    healthy one -- the aggregate would improve when a health source died.
    """
    states: Dict[str, HealthState] = {}
    for item in _all_items():
        states[item] = item_states.get(item, HealthState.UNKNOWN)
    # Step 3 first, because it short-circuits: any allow_motion-driving item in
    # FAIL forbids motion outright.
    blocking = [i for i, s in states.items()
                if s == HealthState.FAIL and drives_allow_motion(i)]
    if blocking:
        return FactorOutput(allow_motion=False, speed_factor=0.0,
                            max_profile="none")
    # Steps 0 + 1 + 2: min over the participating items only.
    participating = [factor_for(i, s, cfg) for i, s in states.items()
                     if counts_in_speed_factor(i)]
    speed_factor = min(participating) if participating else 1.0
    # 14 S8.3: both profiles require cam_rgbd (U54-a made their require lists
    # identical), so max_profile is effectively binary. UNKNOWN does not admit
    # a profile: "I do not know whether the camera is there" is not a basis for
    # driving with obstacle avoidance.
    cam = states.get("cam_rgbd", HealthState.UNKNOWN)
    if cam in (HealthState.OK, HealthState.WARN, HealthState.DEGRADED):
        return FactorOutput(allow_motion=True,
                            speed_factor=max(0.0, min(1.0, speed_factor)),
                            max_profile="patrol")
    # Not even obstacle_avoid is admissible -> allow_motion false (S8.3).
    # This is the case an ORIN with no camera lands in, and it is the correct
    # answer there: without the obstacle-avoidance sensor the robot may not be
    # driven, including under teleop during a recording.
    return FactorOutput(allow_motion=False, speed_factor=0.0,
                        max_profile="none")


def _all_items():
    """The S5.1A closed set. Imported lazily so the item table stays the single
    place the membership is defined."""
    from xbrain.p2_core.health.items import ITEMS
    return ITEMS.keys()
