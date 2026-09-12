"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_relmove_intake.py
Brief: relative_move -> goto translation and the 12 S4.5.2 reject codes

Description:
The translation's one geometric fact (G = pose + R(yaw) * (dx, dy)) and each
reject row of 12 S4.5.2 that this phase evaluates get one test; codes are
compared against the exported constants, never literals. The achieved-delta
helper (RM-2) is checked in the start body frame.
"""
from __future__ import annotations

import math

import pytest

from xbrain.common.errors import E_CAPABILITY, E_DEGRADED, E_NO_HEADING, E_SCHEMA
from xbrain.p1_motion.nav.relmove_intake import (RelMoveLimits, RelMoveReject,
                                                 achieved_body_delta,
                                                 translate_relative_move)

pytestmark = pytest.mark.no_device

LIM = RelMoveLimits(max_distance_m=20.0, max_yaw_rad=6.2832,
                    pure_rotation_eps_m=0.02, default_timeout_s=20.0,
                    abort_on_obstacle=True)


def _body(dx=1.0, dy=0.0, dyaw=0.0, **kw):
    b = {"cmd_id": "rm-1", "dx_m": dx, "dy_m": dy, "dyaw_rad": dyaw,
         "max_speed_mps": 0.5, "max_yaw_rate_radps": 0.5, "source": "voice_local"}
    b.update(kw)
    return b


def _go(body, *, pose=(0.0, 0.0), yaw=0.0, heading_valid=True, holonomic=True):
    return translate_relative_move(body, pose_xy=pose, yaw_rad=yaw,
                                   heading_valid=heading_valid,
                                   holonomic=holonomic, limits=LIM)


def _code(body, **kw):
    with pytest.raises(RelMoveReject) as ei:
        _go(body, **kw)
    return ei.value.code, ei.value.detail


def test_endpoint_rotates_by_yaw():
    """"前进 1 米" facing north (yaw pi/2) lands one metre NORTH.
    mutant: add (dx, dy) without the rotation -> lands east -> red."""
    g = _go(_body(dx=1.0), pose=(2.0, 3.0), yaw=math.pi / 2)
    assert g.endpoint_xy[0] == pytest.approx(2.0, abs=1e-9)
    assert g.endpoint_xy[1] == pytest.approx(4.0, abs=1e-9)
    g = _go(_body(dx=0.0, dy=1.0), pose=(0.0, 0.0), yaw=0.0)     # 左移 1 米
    assert g.endpoint_xy == pytest.approx((0.0, 1.0), abs=1e-9)


def test_defaults_come_from_limits_when_absent():
    g = _go(_body())
    assert g.timeout_s == 20.0 and g.abort_on_obstacle is True and g.source == "voice_local"
    g = _go(_body(timeout_s=5.0, abort_on_obstacle=False))
    assert g.timeout_s == 5.0 and g.abort_on_obstacle is False


def test_ty1_target_yaw_and_dyaw_are_exclusive():
    code, detail = _code(_body(dyaw=0.5, target_yaw_rad=1.0))
    assert code == E_SCHEMA and detail["field"] == "target_yaw_rad"


def test_range_limits_name_field_and_limit():
    code, detail = _code(_body(dx=25.0))
    assert code == E_SCHEMA and detail == {"field": "dx_m", "limit": 20.0}


def test_lateral_needs_holonomic():
    assert _code(_body(dx=0.0, dy=1.0), holonomic=False)[0] == E_CAPABILITY


def test_no_fix_is_degraded():
    assert _code(_body(), pose=None) == (E_DEGRADED, {"item": "no_fix"})


def test_invalid_heading_is_no_heading():
    assert _code(_body(), heading_valid=False)[0] == E_NO_HEADING


def test_pure_rotation_is_nav2_spin_boundary():
    assert _code(_body(dx=0.0, dyaw=1.5708)) == (E_CAPABILITY, {"item": "nav2_spin"})


def test_translation_with_rotation_is_heading_goal_boundary():
    assert _code(_body(dx=1.0, dyaw=0.3)) == (E_CAPABILITY, {"item": "heading_goal"})


@pytest.mark.parametrize("body", [
    "x", {"dx_m": 1.0, "dy_m": 0.0, "dyaw_rad": 0.0},
    _body(dx="1"), _body(dx=float("nan")), _body(dx=True), _body(timeout_s=0.0),
])
def test_shape_defects_are_schema(body):
    assert _code(body)[0] == E_SCHEMA


def test_achieved_delta_in_start_body_frame():
    g = _go(_body(dx=2.0), pose=(1.0, 1.0), yaw=math.pi / 2)
    dx, dy = achieved_body_delta(g, (1.0, 2.5))
    assert dx == pytest.approx(1.5, abs=1e-9) and dy == pytest.approx(0.0, abs=1e-9)


def test_zero_move_without_rotation_is_a_goto_to_the_current_pose():
    g = _go(_body(dx=0.0, dy=0.0), pose=(3.0, 4.0), yaw=1.0)
    assert g.endpoint_xy == pytest.approx((3.0, 4.0))


def test_allow_motion_false_is_unhealthy():
    """12 S4.5.2 row 3. mutant: drop the check -> accepted -> red."""
    from xbrain.common.errors import E_UNHEALTHY
    with pytest.raises(RelMoveReject) as ei:
        translate_relative_move(_body(), pose_xy=(0.0, 0.0), yaw_rad=0.0, heading_valid=True,
                                holonomic=True, limits=LIM, allow_motion=False)
    assert ei.value.code == E_UNHEALTHY
