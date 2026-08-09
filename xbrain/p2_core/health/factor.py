"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: factor.py
Brief: BIZ-P2-19 -- speed_factor / allow_motion / max_profile computation

Description:
Aggregates the 19 health item states into a single
cmd/motion/factor message that P1 subscribes:

  * allow_motion (bool)  -- any FATAL item in fail state -> False
  * speed_factor (0..1)  -- product of per-item degrade factors
  * max_profile (str)    -- highest allowed profile given health

The factor table comes from p2_core.yaml.health.factors:
  fatal_degraded: 0.3   (a fatal item in DEGRADED state)
  degraded_fail:  0.5   (a degraded item in FAIL state)
  degraded_degraded: 0.7 (a degraded item in DEGRADED state)
  unknown:        0.5   (any item in UNKNOWN state)

★ Publish rate is 1 Hz stable (14 S2.3 P-2). If P1 does not receive
health/factor for 3 s -> P1 downgrades; for 10 s -> P1 stops.

★ No hot-swap of mode: FM-3 verbatim 'health downgrade must NOT
auto-switch mode' -- it only lowers speed_factor / disallows motion.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, FrozenSet, Mapping

from xbrain.p2_core.health.items import HealthLevel, HealthState, level_of


@dataclass(frozen=True)
class FactorConfig:
    """From p2_core.yaml.health.factors."""
    fatal_degraded: float       # 0.3
    degraded_fail: float        # 0.5
    degraded_degraded: float    # 0.7
    unknown: float              # 0.5


@dataclass(frozen=True)
class FactorOutput:
    """The cmd/motion/factor message."""
    allow_motion: bool
    speed_factor: float
    max_profile: str            # "obstacle_avoid" | "patrol" | "none"


def compute_factor(
    item_states: Mapping[str, HealthState],
    cfg: FactorConfig,
) -> FactorOutput:
    """Compute the (allow_motion, speed_factor, max_profile) triple.

    * Any FATAL item in FAIL -> allow_motion=False, factor=0, profile=none
    * Any item state contributes a multiplier per FactorConfig
    * max_profile downgrades from patrol -> obstacle_avoid based on
      cam_rgbd state (which both profiles require per p2_core.yaml)
    """
    # Fatal-fail short-circuit.
    for item, state in item_states.items():
        if state == HealthState.FAIL and level_of(item) == HealthLevel.FATAL:
            return FactorOutput(
                allow_motion=False,
                speed_factor=0.0,
                max_profile="none",
            )

    # Multiply per-item factors.
    factor = 1.0
    for item, state in item_states.items():
        try:
            lvl = level_of(item)
        except KeyError:
            continue     # unknown item name; skip (not our concern here)
        if state == HealthState.UNKNOWN:
            factor *= cfg.unknown
            continue
        if lvl == HealthLevel.FATAL and state == HealthState.DEGRADED:
            factor *= cfg.fatal_degraded
        elif lvl == HealthLevel.DEGRADED and state == HealthState.FAIL:
            factor *= cfg.degraded_fail
        elif lvl == HealthLevel.DEGRADED and state == HealthState.DEGRADED:
            factor *= cfg.degraded_degraded

    # max_profile: both profiles require cam_rgbd (per p2_core.yaml).
    # If cam_rgbd is not OK, no profile is admissible.
    cam = item_states.get("cam_rgbd", HealthState.UNKNOWN)
    if cam not in (HealthState.OK, HealthState.DEGRADED):
        profile = "none"
    else:
        # Both profiles pass cam_rgbd check; pick patrol by default
        # (higher speed cap).
        profile = "patrol"

    return FactorOutput(
        allow_motion=True,
        speed_factor=max(0.0, min(1.0, factor)),
        max_profile=profile,
    )
