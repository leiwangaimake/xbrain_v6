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

from dataclasses import dataclass
from typing import Any, Dict, Mapping

from xbrain.p1_motion.nav.relmove_intake import RelMoveLimits
from xbrain.p1_motion.path.local_frame import LocalFrame, LocalFrameError, frame_from_config


class NavConfigError(ValueError):
    """A loop parameter is missing / null / off-type; message names the key."""


@dataclass(frozen=True)
class NavConfig:
    frame: LocalFrame
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


def build_nav_config(p1_tree: Mapping[str, Any], rns_tree: Mapping[str, Any]) -> NavConfig:
    """Two resolved trees -> NavConfig, or NavConfigError naming the first
    offending key. rns_tree must carry the "rns" section (12 S12.0A)."""
    try:
        frame = frame_from_config(_walk(p1_tree, "geo.enu_origin"))
    except LocalFrameError as exc:
        raise NavConfigError(str(exc)) from exc
    if not isinstance(rns_tree, Mapping) or not isinstance(rns_tree.get("rns"), Mapping):
        raise NavConfigError("resolved rns.yaml has no 'rns' section (12 S12.0A)")
    return NavConfig(
        frame=frame,
        max_vx_mps=_pos(p1_tree, "nav.max_vx_mps"),
        max_wz_radps=_pos(p1_tree, "nav.max_wz_radps"),
        holonomic=_bool(p1_tree, "nav.holonomic"),
        v_nom_mps=_pos(p1_tree, "nav.v_nom_mps"),
        v_obstacle_avoid_mps=_pos(p1_tree, "nav.v_obstacle_avoid_mps"),
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
    )
