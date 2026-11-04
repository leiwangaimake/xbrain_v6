"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: classify.py
Brief: RTK tiers + perception health + obstacle dispatch (P2.6/P3 -- 20 S3.2.1/S5)

Description:
This slice (P2.6/P2.4): the RTK quality tiers (RNS-N-6) and perception-health
speed limiting. The obstacle-class dispatch (RNS-N-7, class_map, static/dynamic,
velocity_frame==raw refusal) lands in P3 -- it shares this file because both are
"decide what the world means before acting", but they are separable and the
dispatch depends on classify config not present this slice.

RTK tiers (RNS-N-6, S3.2.1) -- the three-way, failure directions written in:
  FIXED   -> normal
  FLOAT   -> slow (rtk_float_g) + enlarge arrival radius (arrival_radius_float_
             scale): dm-level error may never reach the old radius, or reach it
             early.
  SINGLE/no-fix -> STOP and report rtk_unreliable (A-RTK-1): "where am I" is no
             longer trustworthy; continuing is blind driving toward an absolute
             goal, which is meaningless. NOT slow-and-continue (A-RTK-2 reverse).
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

from .types import T_CLASS_NAMES, NavFailReason, NavFailure


class RtkTier(str, Enum):
    """RTK fix quality, mapped to the three RNS behaviors (S3.2.1). Values match
    the fix_type semantics upstream; RNS only cares about these three tiers."""
    FIXED = "fixed"
    FLOAT = "float"
    SINGLE = "single"      # single / no-fix / no solution -> stop


def rtk_stop_and_report(tier: RtkTier) -> Optional[NavFailure]:
    """A-RTK-1: SINGLE/no-fix -> stop and report rtk_unreliable. FIXED/FLOAT do
    not fail here (FLOAT is slow-continue, handled by the caps). mutant: return
    None for SINGLE (keep driving) -> blind driving -> reddens."""
    if tier == RtkTier.SINGLE:
        return NavFailure(reason=NavFailReason.RTK_UNRELIABLE,
                          detail={"fix": tier.value})
    return None


def rtk_speed_factor(tier: RtkTier, rtk_float_g: float) -> float:
    """The speed coefficient for the RTK tier (S3.2.1 / S8.1A). FIXED = 1.0 (no
    limit), FLOAT = rtk_float_g (< 1). SINGLE never reaches here (it stopped)."""
    if tier == RtkTier.FLOAT:
        return rtk_float_g
    return 1.0


def rtk_arrival_radius(tier: RtkTier, base_radius_m: float,
                       float_scale: float) -> float:
    """FLOAT enlarges the arrival radius (S3.2.1): dm-level error would otherwise
    never satisfy the base radius."""
    if tier == RtkTier.FLOAT:
        return base_radius_m * float_scale
    return base_radius_m


def health_speed_capped(invalid_pixel_ratio: Optional[float],
                        invalid_ratio_limit: Optional[float]) -> bool:
    """RNS-I-2 / S3.1.8: invalid_pixel_ratio over the limit forces a speed cap --
    the only cover for "both semantic and geometry are blind" (glass/water where
    both channels say no obstacle). invalid_ratio_limit null -> conservatively
    capped (unusable threshold -> worst case, CLAUDE.md 3.1)."""
    # 11 v2.3 row 3: a null RATIO (no depth statistic this heartbeat) is the
    # same direction -- quality unknown is capped, never read as healthy.
    # mutant: return False on a null ratio -> a blind heartbeat drives -> red.
    if invalid_ratio_limit is None or invalid_pixel_ratio is None:
        return True
    return invalid_pixel_ratio > invalid_ratio_limit


# ── obstacle dispatch (P3 -- 20 S5.1/S5.1.1/S5.5) ─────────────────────────────
# The class_map maps the open perception vocabulary to a behavior class
# (20 S5.1.1). person is NOT here -- it is locked to person_stop in code
# (config.PERSON_BEHAVIOR); a config person mapping is a startup failure
# (A-CLS-4, config.assert_person_locked). Unmapped classes -> block + slow.
UNMAPPED_BEHAVIOR = "block"     # 20 S5.1.1: unmapped -> block+slow (never dropped)
PERSON_BEHAVIOR = "person_stop"
IGNORE_BEHAVIOR = "ignore"      # 20 S5.1.1 v1.35: airborne only, explicit rows


def behavior_class(class_name: str, class_map: dict) -> str:
    """Map a perception class_name to its behavior class (20 S5.1.1). person is
    locked in code (never read from class_map). Unmapped -> block (conservative,
    never dropped, never traverse). A-CLS-4 reverse: unmapped MUST be block."""
    if class_name == "person":
        return PERSON_BEHAVIOR
    mapped = class_map.get(class_name)
    if mapped is None:
        return UNMAPPED_BEHAVIOR
    return mapped


def effective_behavior(class_name: str, semantic_status: str, class_map: dict):
    """The behavior class RNS actually acts on for one tracked object, plus
    whether the object may ever join the STATIC pile (20 S5.1.1 v1.35).
    Returns (behavior | None, allow_static):
      - None: DROP. The traversable-segmentation class is the T channel, not
        an object (11 S3.1B.2 v2.1, A-CLS-5). Folding it through the
        unmapped->block default would turn the whole walkable area into a
        wall.
      - person: person_stop whatever its status.
      - proxy (semantic_status): the MORE conservative of the mapping and
        block -- ignore/traverse collapse to block, hazard stays hazard,
        vehicle/animal keep their mapping but never enter the static pile.
      - confirmed: the mapping as configured; unmapped -> block.
    99 U113 (20 S3.0 WP-1) removed the confidence branch that lived here
    ("below min_confidence -> block, person immune", A-CLS-7): an object
    below world.object_min_confidence never reaches this function now --
    world.keep_confident drops it, person included. Keeping a second
    threshold here would be two rules for one number.
    mutants: return block for the T class (A-CLS-5 red); return ignore for
    a proxy bird."""
    if class_name in T_CLASS_NAMES:
        return None, False
    if class_name == "person":
        return PERSON_BEHAVIOR, False
    proxy = semantic_status == "proxy"
    beh = behavior_class(class_name, class_map)
    if proxy:
        if beh in (IGNORE_BEHAVIOR, "traverse"):
            beh = UNMAPPED_BEHAVIOR
        return beh, False
    return beh, True


def is_static(velocity_mps: float, dwell_s: float,
              v_static_thresh: float, t_static_dwell: float) -> bool:
    """Static criterion = speed threshold AND dwell time (20 S5.5, A-CLS-2/3).
    BOTH conditions: speed alone lets estimator noise read a parked car as
    moving (robot waits forever); threshold alone lets a just-started car (still
    slow) read as static (robot detours, car moves). velocity is the
    ego-removed speed magnitude; dwell_s is how long it has held under threshold.
    99 U112: no runtime caller since only a person waits (20 S5.5 v1.45);
    kept, with its tests, for a vehicle dynamic rule to be wired back to."""
    return abs(velocity_mps) < v_static_thresh and dwell_s >= t_static_dwell


def dispatch_dynamic(behavior: str, velocity_mps: float, dwell_s: float,
                     v_static_thresh: float, t_static_dwell: float) -> bool:
    """Is this obstacle in the DYNAMIC pile (20 S5.1)? person is ALWAYS dynamic
    (person_stop, never threaded/detoured -- A-CLS-1). Every other class is
    dynamic UNTIL it passes the static criterion. mutant: make a static person
    join the static pile -> person enters candidate generation -> A-CLS-1 red.
    99 U112: source.py no longer calls this -- a non-person is the static pile
    outright (20 S5.1 v1.45); kept for a vehicle dynamic rule."""
    if behavior == PERSON_BEHAVIOR:
        return True   # person is always dynamic; never a detour candidate
    return not is_static(velocity_mps, dwell_s, v_static_thresh, t_static_dwell)


def usable_velocity(velocity_frame: str, velocity_mps: float,
                    raw_policy: str) -> Optional[float]:
    """velocity_frame==raw refusal (20 S3.1.5, A-FUS-7). raw velocity has NOT had
    ego motion removed, so a static tree reads as -v_ego and the whole world looks
    moving. This phase's only legal raw_policy is 'reject' -> return None (motion
    state unknown -> conservative). ego_removed -> the velocity is usable.
    99 U112: no runtime caller (velocity decides nothing once only a person
    waits); kept for a vehicle dynamic rule."""
    if velocity_frame == "raw":
        if raw_policy != "reject":
            raise ValueError("raw_velocity_policy must be 'reject' this phase "
                             "(20 S3.1.5); got %r" % raw_policy)
        return None   # cannot judge motion from raw velocity
    return velocity_mps
