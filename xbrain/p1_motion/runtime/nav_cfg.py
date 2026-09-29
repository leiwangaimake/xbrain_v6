"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: nav_cfg.py
Brief: resolved p1_motion.yaml + rns.yaml snapshots -> the NavConfig the 20 Hz loop is built from

Description:
The loop's parameters have three sources and one rule. Sources: the p1
snapshot's geo / nav / relative_move / timeouts_ms sections (configs/
p1_motion.yaml, values expanded by the freeze line from common.*), and the
RNS module snapshot (resolved/rns.yaml, 12 S12.0A). Rule: nothing here has a
default -- a null or missing leaf is a NavConfigError that NAMES THE KEY, and
the caller then starts p1 WITHOUT the navigation loop (the pre-P7.2 state:
no cmd_vel is published, the chassis stays in timeout_lock, which is the safe
state -- __main__.main_loop's own reasoning). This is CLAUDE.md 3.1 applied
to the loop as a whole: common.spec.max_wz_radps is null until V-01 is
measured, so production p1 refuses to navigate and says why, while the dev
fixture (tests/fixtures/overrides.py) supplies a value and the dev stack runs.

Both snapshots are read through xbrain.common.config.resolved (never the
source, 10 S5.4.1); this module only walks the trees it is handed, so it is
testable with plain dicts.

What it does NOT do: it does not run the RNS startup assertions (RnsSource
does, on construction), does not validate ranges beyond "positive number"
(the freeze line's assertion G owns spec ranges), and does not read clocks.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

from xbrain.p1_motion.fence.clip import FIX_WITH_FENCE, FenceClipError, FenceConstants
from xbrain.p1_motion.nav.relmove_intake import RelMoveLimits
from xbrain.p1_motion.path.local_frame import (
    LocalFrame,
    LocalFrameError,
    frame_from_config,
)
from xbrain.p1_motion.rotation.rcg import RotationConfigError, RotationLimits


class NavConfigError(ValueError):
    """A loop parameter is missing / null / off-type; message names the key."""


@dataclass(frozen=True)
class NavConfig:
    frame: LocalFrame
    fence: FenceConstants          # 12 S7 clip constants (p1 fence.* refs, 11 S9A.6 M-01)
    speed_up_hold_ms: int          # 12 S6.7 T_up (speed_gate.hysteresis.speed_up_hold_s, in ms)
    d_up_margin_m: float           # 12 S6.2 rise margin (speed_gate.hysteresis.d_up_margin_m)
    max_vx_mps: float
    max_wz_radps: float
    holonomic: bool
    v_nom_mps: float
    v_obstacle_avoid_mps: float
    relmove: RelMoveLimits
    gnss_dead_ms: int
    health_degrade_ms: int
    health_dead_ms: int
    rns: Dict[str, Any]           # the whole {"rns": {...}} tree for RnsSource
    r_eff_m: float                # rns.geometry.r_eff_m (12 S6A; D2 assertion base)
    rot_limits: RotationLimits    # 12 S12 rotation_clearance (step 6b constants)
    # The TRUE body radius RCG-1 and r_check are evaluated on. None when no
    # loaded tree carries it -- see _read_r_robot for why that is the standing
    # state and why nothing here substitutes a number for it.
    r_robot_m: Optional[float]


def _walk(tree: Mapping[str, Any], dotted: str) -> Any:
    """Leaf by dotted path; missing or null -> NavConfigError naming the key.
    mutant: return None for a missing leaf -> a null max_wz_radps becomes a
    NavConfig with None and the loop divides by it later -> red."""
    cur: Any = tree
    for part in dotted.split("."):
        if not isinstance(cur, Mapping) or part not in cur:
            raise NavConfigError("p1_motion.yaml key %r missing" % dotted)
        cur = cur[part]
    if cur is None:
        raise NavConfigError(
            "p1_motion.yaml key %r is null -- uncalibrated, refusing the "
            "navigation loop (CLAUDE.md 3.1)" % dotted)
    return cur


def _nonneg(tree: Mapping[str, Any], dotted: str) -> float:
    """A finite number >= 0 (an inset of 0.0 is legal, a null leaf is not --
    _walk already names a null leaf)."""
    v = _walk(tree, dotted)
    if isinstance(v, bool) or not isinstance(v, (int, float)) \
            or not math.isfinite(v) or v < 0.0:
        raise NavConfigError("p1_motion.yaml key %r must be a finite non-negative "
                             "number, got %r" % (dotted, v))
    return float(v)


def _pos(tree: Mapping[str, Any], dotted: str) -> float:
    v = _walk(tree, dotted)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0:
        raise NavConfigError("p1_motion.yaml key %r must be a positive number, got %r"
                             % (dotted, v))
    return float(v)


def _pos_int(tree: Mapping[str, Any], dotted: str) -> int:
    v = _walk(tree, dotted)
    if isinstance(v, bool) or not isinstance(v, int) or v <= 0:
        raise NavConfigError("p1_motion.yaml key %r must be a positive int, got %r"
                             % (dotted, v))
    return v


def _bool(tree: Mapping[str, Any], dotted: str) -> bool:
    v = _walk(tree, dotted)
    if not isinstance(v, bool):
        raise NavConfigError("p1_motion.yaml key %r must be a bool, got %r" % (dotted, v))
    return v


def _read_r_robot(rns_tree: Mapping[str, Any]) -> Optional[float]:
    """The TRUE r_robot for RCG-1, or None when no tree carries it.

    12 S12's rotation_clearance block states that r_robot is NOT redefined
    there: its single definition lives in the RNS inflation section, and a
    private P1 copy would be flagged as a suspected duplicate by the freeze
    line's assertion B. So it is read from the RNS tree, at the path 12 S12
    names, and nowhere else.

    Returning None when that key is absent is the whole point of this helper.
    configs/rns.yaml carries rns.geometry.r_eff_m but no inflation section, and
    r_eff_m is NOT a substitute: 12 S6A.4.1 iron rule (1) forbids r_eff from
    reaching r_check, because doing so writes "not known" as "known 0.5" --
    the fail-open RCG-1 exists to name. Nor is the doc's calibrated figure
    copied in here: CLAUDE.md iron rule 3 forbids filling a calibration value
    to make something run, and a number typed in at this layer would be
    indistinguishable downstream from one that came off the machine.

    The consequence is that RCG-1 refuses until the key lands, which is
    precisely what 12 S6A.3.3 specifies for an uncalibrated body.

    mutant: return cfg r_eff_m when the inflation key is missing -> RCG-1
    passes on an unmeasured body and r_check is built from it -> the
    r_robot-uncalibrated criterion goes green with no calibration.
    """
    infl = rns_tree.get("rns", {})
    if not isinstance(infl, Mapping):
        return None
    infl = infl.get("inflation")
    if not isinstance(infl, Mapping):
        return None
    v = infl.get("r_robot_m")
    if v is None or isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    return float(v)


def _rotation_limits(p1_tree: Mapping[str, Any]) -> RotationLimits:
    """12 S12 rotation_clearance -> RotationLimits, or NavConfigError by key.

    Every leaf goes through _walk, so a missing or null one is reported with
    its dotted path and nothing is defaulted (CLAUDE.md 3.1). Range and type
    checks are RotationLimits' own __post_init__ rather than a second copy
    here; its error is re-raised as a NavConfigError so the caller has one
    exception type to handle for the whole config.

    wz_blind_radps is read like every other leaf as of 2026-09-29. It was
    hard-coded None before that, and the reason it was is worth keeping: 12 S12
    kept the key out of this block and reused 11 S3.1.5.6
    free_space.blind.wz_blind_radps, then warned that a ${common.*} reference
    would be rejected as unresolved because the key was not in any shared table.
    What changed is that the key now HAS a value -- configs/common.yaml carries
    common.motion.free_space.blind.wz_blind_radps: 0.3 (user ruling, 2026-09-29)
    -- so the reference resolves and 10 S5.4.2 R-2's only requirement (the path
    starts with common.) is met. refs.resolve looks the path up in the merged
    tree; there is no allow-list of reachable common.* paths, which is what the
    old warning assumed.

    Reading it rather than passing None is not cosmetic. 12 S6A.3.3's RCG-3
    correction makes the blind clamp the STANDING path on this machine (the
    rear of the ring is never observed), so a None here would degrade every
    spin_like tick to a veto -- the robot could not turn at all, which is the
    availability failure the correction exists to fix.

    A missing or null leaf still refuses by name (_walk), same as the other
    twelve. RotationLimits keeps wz_blind_radps Optional and apply_rotation_
    permit keeps 12 S12 landing plan (2) -- cannot get the clamp value, then do
    not let it through, never on a guessed one -- because that is a modelled
    state, not a defaulted one; it is simply no longer reachable from a
    well-formed snapshot.

    mutant: default it to 0.3 here when the leaf is absent -> the clamp value
    stops being traceable to a file and CLAUDE.md 3.1 is broken for a safety
    param -> test_missing_rotation_key_names_itself[wz_blind_radps] red.
    """
    try:
        return RotationLimits(
            margin_rot_m=_walk(p1_tree, "rotation_clearance.margin_rot_m"),
            r_self_mask_m=_walk(p1_tree, "rotation_clearance.r_self_mask_m"),
            rot_occ_max=_walk(p1_tree, "rotation_clearance.rot_occ_max"),
            rot_unknown_max_cells=_walk(
                p1_tree, "rotation_clearance.rot_unknown_max_cells"),
            rot_unknown_ratio_max=_walk(
                p1_tree, "rotation_clearance.rot_unknown_ratio_max"),
            grid_age_max_ms=_walk(p1_tree, "rotation_clearance.grid_age_max_ms"),
            recheck_ticks=_walk(p1_tree, "rotation_clearance.recheck_ticks"),
            wz_eps_radps=_walk(p1_tree, "rotation_clearance.wz_eps_radps"),
            k_rot=_walk(p1_tree, "rotation_clearance.k_rot"),
            r_robot_fallback_m=_walk(
                p1_tree, "rotation_clearance.r_robot_fallback_m"),
            ped_speed_mps=_walk(p1_tree, "rotation_clearance.ped_speed_mps"),
            allow_visual_override=_walk(
                p1_tree, "rotation_clearance.allow_visual_override"),
            wz_blind_radps=_walk(p1_tree, "rotation_clearance.wz_blind_radps"))
    except RotationConfigError as exc:
        raise NavConfigError(str(exc)) from exc


def build_nav_config(p1_tree: Mapping[str, Any], rns_tree: Mapping[str, Any]) -> NavConfig:
    """Two resolved trees -> NavConfig, or NavConfigError naming the first
    offending key. rns_tree must carry the "rns" section (12 S12.0A)."""
    try:
        frame = frame_from_config(_walk(p1_tree, "geo.enu_origin"))
    except LocalFrameError as exc:
        raise NavConfigError(str(exc)) from exc
    if not isinstance(rns_tree, Mapping) or not isinstance(rns_tree.get("rns"), Mapping):
        raise NavConfigError("resolved rns.yaml has no 'rns' section (12 S12.0A)")
    v_nom = _pos(p1_tree, "nav.v_nom_mps")
    v_oa = _pos(p1_tree, "nav.v_obstacle_avoid_mps")
    try:
        # v_profile_max = the patrol tier (11 S9A.6 (3) d_stop(v_profile_max));
        # the degraded teleop cap = the obstacle_avoid tier (11 S3.2.1 0.5 m/s
        # manual cap; U54 made the two rows one).
        fence = FenceConstants(
            brake_k=_pos(p1_tree, "fence.brake_k"),
            brake_a_mps2=_pos(p1_tree, "fence.brake_a_mps2"),
            t_lat_s=_pos(p1_tree, "fence.t_lat_s"),
            soft_margin_min_m=_pos(p1_tree, "fence.soft_margin_min_m"),
            predict_dt_s=_pos(p1_tree, "fence.predict_dt_s"),
            margin_by_fix={fix: _nonneg(p1_tree, "fence.margin_by_fix.%s" % fix)
                           for fix in FIX_WITH_FENCE},
            projection_iters=_pos_int(p1_tree, "fence.projection_iters"),
            v_profile_max_mps=v_nom, teleop_cap_degraded_mps=v_oa)
    except FenceClipError as exc:
        raise NavConfigError(str(exc)) from exc
    hold_s = _pos(p1_tree, "speed_gate.hysteresis.speed_up_hold_s")
    return NavConfig(
        frame=frame,
        fence=fence,
        speed_up_hold_ms=int(round(hold_s * 1000.0)),
        d_up_margin_m=_pos(p1_tree, "speed_gate.hysteresis.d_up_margin_m"),
        max_vx_mps=_pos(p1_tree, "nav.max_vx_mps"),
        max_wz_radps=_pos(p1_tree, "nav.max_wz_radps"),
        holonomic=_bool(p1_tree, "nav.holonomic"),
        v_nom_mps=v_nom,
        v_obstacle_avoid_mps=v_oa,
        relmove=RelMoveLimits(
            max_distance_m=_pos(p1_tree, "relative_move.max_distance_m"),
            max_yaw_rad=_pos(p1_tree, "relative_move.max_yaw_rad"),
            pure_rotation_eps_m=_pos(p1_tree, "relative_move.pure_rotation_eps_m"),
            default_timeout_s=_pos(p1_tree, "relative_move.default_timeout_s"),
            abort_on_obstacle=_bool(p1_tree, "relative_move.abort_on_obstacle")),
        gnss_dead_ms=_pos_int(p1_tree, "timeouts_ms.gnss"),
        health_degrade_ms=_pos_int(p1_tree, "timeouts_ms.health_degrade"),
        health_dead_ms=_pos_int(p1_tree, "timeouts_ms.health_dead"),
        rns={"rns": dict(rns_tree["rns"])},
        r_eff_m=_pos(rns_tree, "rns.geometry.r_eff_m"),
        rot_limits=_rotation_limits(p1_tree),
        r_robot_m=_read_r_robot(rns_tree),
    )
