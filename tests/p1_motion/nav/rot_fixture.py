"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rot_fixture.py
Brief: shared 12 S12 rotation_clearance constants for the NavTick test stacks

Description:
NavTick requires a RotationLimits (12 S2.2 step 6b) with no None-means-skip
shape, because 12 S6A.7 RC-D7 refuses the rotation permit any off switch and a
constructor that ran without it when the block was absent would be that switch
under another name. Every NavTick test stack therefore has to supply one, and
one factory here keeps the six of them from drifting apart -- a per-file copy
would let one stack quietly relax a bound and still look like the others.

The values are the 12 S12 rotation_clearance block, read from the real
configs/p1_motion.yaml at import so the tests cannot pass against numbers the
robot does not use. margin_rot_m is the one leaf that arrives as a ${common.*}
reference in the source file (the freeze line expands it, and the freeze line's
MR-1 asserts it equals common.safety.d_safe_m), so it is resolved here from
configs/safety/brake.yaml -- the same single source, not a literal retyped.

What this file is NOT for: it does not decide anything the permit judges.
r_robot stays a per-test argument because the criteria differ on exactly that
value.

wz_blind_radps used to default to None here, and that default was correct for
as long as nav_cfg hard-coded None. It no longer is: as of 2026-09-29 the key
carries a value (configs/common.yaml, user ruling) and nav_cfg reads it, so
None would now be a shape the robot never runs -- and a misleading one, since
12 S6A.3.3's RCG-3 correction makes the clamp the STANDING path rather than an
exception. The default therefore resolves the same reference the config file
does. A test that wants 12 S12 landing plan (2) passes wz_blind_radps=None.
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import yaml

from xbrain.p1_motion.rotation.rcg import RotationLimits

_ROOT = Path(__file__).resolve().parents[3]
_P1 = yaml.safe_load((_ROOT / "configs" / "p1_motion.yaml").read_text(encoding="utf-8"))
_BRAKE = yaml.safe_load(
    (_ROOT / "configs" / "safety" / "brake.yaml").read_text(encoding="utf-8"))
_COMMON = yaml.safe_load(
    (_ROOT / "configs" / "common.yaml").read_text(encoding="utf-8"))

# The block as written in configs/p1_motion.yaml. KeyError here is the point:
# if somebody removes the section, every NavTick test fails at import with the
# section name rather than passing against a fabricated default.
_RC = _P1["rotation_clearance"]

# margin_rot_m is "${common.safety.d_safe_m}" in the source file. The freeze
# line expands it; here it is read from the L3 safety layer that defines it, so
# the two can never disagree without the assertion that guards them (MR-1)
# disagreeing too.
_D_SAFE = _BRAKE["common"]["safety"]["d_safe_m"]

# Same treatment for the clamp: p1_motion.yaml holds
# "${common.motion.free_space.blind.wz_blind_radps}", and 11 S3.1.5.6 defines
# it in the L1 common layer. Reading it there is how the fixture stays a
# reference rather than becoming a second source of the number.
_WZ_BLIND = _COMMON["common"]["motion"]["free_space"]["blind"]["wz_blind_radps"]

# Sentinel so "caller said None on purpose" stays distinguishable from "caller
# said nothing". A plain None default cannot express both, and the difference
# is the whole point here: None is the 12 S12 landing-plan-(2) degrade and must
# be reachable, while the absent case must give the production value.
_UNSET = object()


def rot_limits(*, wz_blind_radps: object = _UNSET) -> RotationLimits:
    """The production rotation_clearance block, with the clamp overridable.

    Omitting wz_blind_radps gives the value the robot runs with (the L1 key).
    Passing None explicitly gives 12 S12 landing plan (2) -- cannot get the
    clamp value, so do not let it through -- which is still a modelled state
    and still has its own test, it is just no longer the default shape.
    """
    clamp: Optional[float] = (_WZ_BLIND if wz_blind_radps is _UNSET
                              else wz_blind_radps)   # type: ignore[assignment]
    return RotationLimits(
        margin_rot_m=_D_SAFE,
        r_self_mask_m=_RC["r_self_mask_m"],
        rot_occ_max=_RC["rot_occ_max"],
        rot_unknown_max_cells=_RC["rot_unknown_max_cells"],
        rot_unknown_ratio_max=_RC["rot_unknown_ratio_max"],
        grid_age_max_ms=_RC["grid_age_max_ms"],
        recheck_ticks=_RC["recheck_ticks"],
        wz_eps_radps=_RC["wz_eps_radps"],
        k_rot=_RC["k_rot"],
        r_robot_fallback_m=_RC["r_robot_fallback_m"],
        ped_speed_mps=_RC["ped_speed_mps"],
        allow_visual_override=_RC["allow_visual_override"],
        wz_blind_radps=clamp)
