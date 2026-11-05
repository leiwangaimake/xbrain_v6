"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: velocity.py
Brief: teleop axes -> a body velocity at the voice-set move / turn speed (99 U134)

Description:
What problem this solves. A teleop frame carries NORMALISED axes (11 S12A.9.2:
"axes 是归一化量, 不是 m/s"), and until 2026-11-05 nothing in P1 turned them into
a velocity: the arbitration winner suspended RNS and the robot stood still
under every teleop input. The user ruled (99 U134) that the cloud keyboard
drives "at the speed set by voice, or its default, exactly as the basic
motions do" -- the two values P2 owns (move speed, 99 U106; turn speed,
U104) and already publishes on state/mode as move_speed.mps and
turn_speed.radps (11 S4.3).

The rule, one place for every teleop device:
    vx = axes.vx * move_speed     vy = axes.vy * move_speed (holonomic only)
    wz = axes.wz * turn_speed     each axis clamped to [-1, 1] first
The result is a CANDIDATE: NavTick still sends it through the speed gate,
the rotation permit and the fence clip (S12A.9.2: teleop and autonomy share
one limiting chain), so a speed set by voice is a ceiling, never a promise.

No speed known -> no velocity (None). P1 must have heard state/mode at least
once; a teleop that guessed a speed would be CLAUDE.md 3.1's default in
another form, and its failure direction (the robot drives at a number nobody
set) is the wrong one.

Boundary: which source drives is teleop/state.py; what the tick does with the
candidate is nav/nav_tick.py; where the speeds come from is the state/mode
subscriber in runtime/nav_wiring.py.

Looks right but is wrong:
  * scaling by profile.max_mps (the S12A.9.2 sketch) -- that is 2.0 m/s on
    patrol, four times the voice default of 0.5, and not what the operator
    set;
  * passing vy through on a non-holonomic base -- the chassis drops it
    silently and the operator sees "y does nothing"; zeroed here and said so
    by the gate attribution instead.
"""
from __future__ import annotations  # postponed annotations

from dataclasses import dataclass  # an immutable value
from typing import Any, Dict, Optional, Tuple  # plain types

from xbrain.p1_motion.sources.arbiter_p1 import BehaviorSource  # the P1 source names


@dataclass(frozen=True)  # value object
class TeleopSpeeds:  # what P2 says now
    """move_speed (m/s) and turn_speed (rad/s) from the latest state/mode."""
    move_mps: float  # forward / back / strafe
    turn_radps: float  # in-place turn


#: Which P1 behaviour source a teleop device drives as. The cloud keyboard is
#: the local keyboard family (99 U134: "完全开放" -- no cloud-only limits).
_SOURCE_FOR_DEVICE = {
    "gamepad": BehaviorSource.TELEOP_JOYSTICK,  # 750
    "keyboard_local": BehaviorSource.TELEOP_KEYBOARD,  # 800
    "keyboard_hmi": BehaviorSource.TELEOP_KEYBOARD,  # HMI keys
    "virtual_stick": BehaviorSource.TELEOP_KEYBOARD,  # HMI stick, same family
    "keyboard_cloud": BehaviorSource.TELEOP_KEYBOARD,  # 99 U134
}


def source_for(device: str) -> BehaviorSource:
    """The P1 source a teleop device drives as; KeyError for an unknown one
    (the tracker already refuses off-set devices, so this is a bug)."""
    return _SOURCE_FOR_DEVICE[device]


def _positive(v: Any) -> Optional[float]:
    """v as a positive float, or None (bool / non-number / <= 0)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0.0:
        return None  # not a usable speed
    return float(v)  # usable


def speeds_from_mode(body: Any) -> Optional[TeleopSpeeds]:
    """TeleopSpeeds from a state/mode body (11 S4.3 move_speed.mps /
    turn_speed.radps), or None when either is absent or not positive."""
    if not isinstance(body, dict):
        return None  # not a body
    move = body.get("move_speed") if isinstance(body.get("move_speed"), dict) else {}
    turn = body.get("turn_speed") if isinstance(body.get("turn_speed"), dict) else {}
    mps, radps = _positive(move.get("mps")), _positive(turn.get("radps"))  # current values
    if mps is None or radps is None:
        return None  # P2 has not published both
    return TeleopSpeeds(move_mps=mps, turn_radps=radps)  # usable


def _axis(axes: Dict[str, Any], key: str) -> float:
    """One normalised axis clamped to [-1, 1]; anything not a number is 0."""
    v = axes.get(key, 0.0)  # absent = not pushed
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return 0.0  # garbage drives nothing
    return max(-1.0, min(1.0, float(v)))  # clamped


def teleop_velocity(axes: Dict[str, Any], speeds: Optional[TeleopSpeeds], *,
                    holonomic: bool) -> Optional[Tuple[float, float, float]]:
    """(vx, vy, wz) for one tick, or None when no speed is known yet."""
    if speeds is None:
        return None  # never guess a speed (module docstring)
    vx = _axis(axes, "vx") * speeds.move_mps  # forward / back
    vy = _axis(axes, "vy") * speeds.move_mps if holonomic else 0.0  # strafe, holonomic only
    wz = _axis(axes, "wz") * speeds.turn_radps  # turn, left positive
    return vx, vy, wz  # a candidate, gated by NavTick
