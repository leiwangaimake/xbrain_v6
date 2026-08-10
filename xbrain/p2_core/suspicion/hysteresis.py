"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: hysteresis.py
Brief: CHK-1-14 target-suspicion σ hysteresis band (CAL-12 §9A.15)

Description:
9A.15 (CAL-12) defines the target-suspicion band width by:
    h = k_h * wpos_sigma_m + h_min_m

Bigger localisation uncertainty (wpos_sigma_m) -> wider band.
Zero k_h -> band collapses to h_min -> a target hovering at the
boundary would flip in/out every tick (siren + strobe cycling).

** THE DIRECTION MATTERS -- this rule was previously written
   BACKWARDS in the docs and must be tested with a specific
   variant. **

Enter (become suspicious):
    dist_inside > -h AND held for enter_persist_s
    ('dist_inside' is signed distance from the boundary; positive
     means inside the region. So `> -h` means "either inside,
     OR outside by less than h meters". That's the WIDER
     entry gate, which the (correct) design uses.)

Exit (stop being suspicious):
    dist_inside < -h AND held for exit_persist_s
    (asymmetric hysteresis: leaving requires bigger margin,
     not the same +h boundary used for entering. Symmetric ± h
     would let a target at the edge chatter across the state
     line.)

Configuration check: exit_persist_s >= 4 * enter_persist_s
(deep hysteresis; enter fast, exit slow).

L2/L3 downgrade (§9A.15.3):
  L1  normal
  L2  wpos_valid == False (heading L2 lost) -> use distance +
      dwell time only, drop zone-vs-zone rules
  L3  headingL3 lost -> ALL region rules disabled, retain
      non-region rules; detail.zone_rules_disabled = True in
      the emitted event (required field)
"""

from __future__ import annotations

from dataclasses import dataclass


class HysteresisConfigError(Exception):
    pass


@dataclass(frozen=True)
class HysteresisConfig:
    """All FOUR fields required at construction (CLAUDE.md 3.1)."""
    k_h: float
    h_min_m: float
    enter_persist_s: float
    exit_persist_s: float

    def __post_init__(self) -> None:
        if self.k_h < 0:
            raise HysteresisConfigError(f"k_h must be >= 0, got {self.k_h}")
        if self.h_min_m <= 0:
            raise HysteresisConfigError(
                f"h_min_m must be > 0, got {self.h_min_m} (zero band = "
                f"chatter across boundary)")
        if self.enter_persist_s <= 0:
            raise HysteresisConfigError(
                f"enter_persist_s must be > 0, got {self.enter_persist_s}")
        if self.exit_persist_s < 4 * self.enter_persist_s:
            raise HysteresisConfigError(
                "exit_persist_s (%r) must be >= 4 * enter_persist_s (%r); "
                "asymmetric hysteresis with enter fast, exit slow"
                % (self.exit_persist_s, self.enter_persist_s))


def band_width_m(wpos_sigma_m: float, cfg: HysteresisConfig) -> float:
    """h = k_h * wpos_sigma_m + h_min_m."""
    return cfg.k_h * wpos_sigma_m + cfg.h_min_m


def entering_threshold(cfg: HysteresisConfig,
                        wpos_sigma_m: float) -> float:
    """Enter fires when dist_inside > -h -- so the threshold is -h."""
    return -band_width_m(wpos_sigma_m, cfg)


def exiting_threshold(cfg: HysteresisConfig,
                       wpos_sigma_m: float) -> float:
    """Exit fires when dist_inside < -h -- same numerical boundary
    as entering_threshold, but the CROSSING DIRECTION differs and
    the persist times differ. The 'symmetric ±h' bug the CHK-1-14
    variant guards is confusing the CROSSING SEMANTIC, not the
    numeric boundary."""
    return -band_width_m(wpos_sigma_m, cfg)


def is_entering(dist_inside_m: float, wpos_sigma_m: float,
                  cfg: HysteresisConfig) -> bool:
    """One-sample entry check (persist is caller's responsibility)."""
    return dist_inside_m > entering_threshold(cfg, wpos_sigma_m)


def is_exiting(dist_inside_m: float, wpos_sigma_m: float,
                 cfg: HysteresisConfig) -> bool:
    """One-sample exit check."""
    return dist_inside_m < exiting_threshold(cfg, wpos_sigma_m)


# --- §9A.15.3 L1/L2/L3 downgrade -----------------------------------

DEGRADE_LEVELS = ("L1", "L2", "L3")


def zone_rules_active(level: str) -> bool:
    """L3 disables ALL region/zone rules; L1/L2 keep them."""
    if level not in DEGRADE_LEVELS:
        raise HysteresisConfigError(
            f"unknown degrade level {level!r}; use one of {DEGRADE_LEVELS}")
    return level != "L3"


def event_detail_for_level(level: str, heading_src: str) -> dict:
    """§9A.15.3: when downgraded, event MUST carry both heading_src
    and zone_rules_disabled. Missing either would let the operator
    not know which regime the event was captured under."""
    if level not in DEGRADE_LEVELS:
        raise HysteresisConfigError(f"unknown level {level!r}")
    return {
        "heading_src": heading_src,
        "zone_rules_disabled": (level == "L3"),
    }
