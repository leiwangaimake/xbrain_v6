"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_host_gate.py
Brief: host speed gate -- vetoes, the 11 S3.4 worked example, axis rules

Description:
The gate has three testable faces. Vetoes: each of estop / health / rtk /
heading / perception-dead zeroes everything and names its 11 S9.6.5 row.
Attribution: the 11 S3.4 worked example (profile 2.0, f 2.0, spec 2.0, h 0.6,
i 1.0 -> v_max 1.20, limiter health, limiter_all [health, free_space, profile,
spec]) is reproduced verbatim -- argmin-of-terms would report free_space and
fail it. Axes: f caps vx > 0 only; reverse and lateral use the f-less
ceiling; a non-holonomic chassis never emits vy.
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.nav.health_factor import HealthView
from xbrain.p1_motion.nav.host_gate import (apply_gate, attribute, compute_gate,
                                            forward_d_free)

pytestmark = pytest.mark.no_device

OK = HealthView(1.0, True, "patrol", "ok", 100)


def _gate(**kw):
    args = dict(v_nom_mps=2.0, spec_max_vx_mps=2.0, f_free_mps=6.0, health=OK,
                i_fix=1.0, i_heading=1.0, heading_valid=True, estop=False,
                perception_dead=False)
    args.update(kw)
    return compute_gate(**args)


def test_forward_sector_min_skips_unknown_bins():
    d = [6.0] * 181
    d[90] = None
    d[95] = 1.5
    d[0] = 0.1            # outside the +/-20 bin sector
    assert forward_d_free(d) == 1.5
    assert forward_d_free([None] * 181) is None


@pytest.mark.parametrize("kw, limiter", [
    (dict(estop=True), "estop"),
    (dict(health=HealthView(0.0, False, None, "never", None)), "health"),
    (dict(i_fix=None), "rtk"),
    (dict(i_fix=0.0), "rtk"),
    (dict(heading_valid=False), "heading"),
    (dict(perception_dead=True), "free_space"),
    (dict(f_free_mps=None), "free_space"),
])
def test_vetoes_zero_everything_and_name_the_row(kw, limiter):
    """mutant: drop the perception_dead veto -> a robot with no perception
    frame ever drives on the bare follow spine -> the free_space rows red."""
    g = _gate(**kw)
    assert g.veto and g.v_max_fwd == 0.0
    assert apply_gate(g, 1.0, 0.2, 0.3, True) == (0.0, 0.0, 0.0)
    assert attribute(g, 1.0)[0] == limiter


def test_estop_outranks_every_other_veto():
    g = _gate(estop=True, i_fix=None, heading_valid=False, perception_dead=True)
    assert attribute(g, 0.0)[0] == "estop"


def test_attribution_matches_11_s34_example():
    """11 S3.4: v_max 1.20 = 2.00 x 0.60 x 1.00; limiter health; limiter_all
    [health, free_space, profile, spec] (delta 0.80 first, then the three
    tied m/s terms in closed-set order).
    mutant: attribute to the tied-min term instead of the largest delta ->
    free_space -> red."""
    g = _gate(health=HealthView(0.6, True, "patrol", "ok", 100))
    assert g.v_max_fwd == pytest.approx(1.2)
    limiter, all_ = attribute(g, 1.8)
    assert limiter == "health"
    assert all_ == ("health", "free_space", "profile", "spec")


def test_free_space_dominates_when_f_cuts():
    g = _gate(f_free_mps=2.0)          # f = 0.5 < profile/spec 2.0
    assert g.v_max_fwd == pytest.approx(0.5)
    limiter, all_ = attribute(g, 1.0)
    assert limiter == "free_space" and all_ == ("free_space",)


def test_none_only_when_unclipped_and_factors_unity():
    g = _gate()
    assert attribute(g, 1.0) == ("none", ())
    assert attribute(g, 2.5)[0] == "free_space"    # three-way tie -> closed-set order (8 < 12 < 13)
    g = _gate(i_fix=0.4)
    assert attribute(g, 0.1)[0] == "rtk"           # factor < 1 -> never "none"


def test_i_factor_attributes_to_smaller_of_fix_and_heading():
    assert attribute(_gate(i_fix=0.4, i_heading=0.9), 1.0)[0] == "rtk"
    assert attribute(_gate(i_fix=0.9, i_heading=0.4), 1.0)[0] == "heading"


def test_f_caps_forward_only_reverse_and_lateral_use_free_ceiling():
    """front blocked (f = 0) must not pin a backup or the body-shield vy.
    mutant: clamp vx < 0 with v_max_fwd -> the backup dies -> red."""
    g = _gate(f_free_mps=1.0, v_nom_mps=1.0)
    assert g.v_max_fwd == 0.0 and g.v_max_free == 1.0
    assert apply_gate(g, 0.8, 0.0, 0.0, True) == (0.0, 0.0, 0.0)
    assert apply_gate(g, -0.3, 0.2, 0.4, True) == (-0.3, 0.2, 0.4)
    assert apply_gate(g, -3.0, 5.0, 0.0, True) == (-1.0, 1.0, 0.0)


def test_non_holonomic_never_emits_vy():
    g = _gate()
    assert apply_gate(g, 0.5, 0.3, 0.0, False) == (0.5, 0.0, 0.0)


def test_max_profile_downgrade_is_attributed_to_health():
    """11 S3.6 max_profile == obstacle_avoid: the tier cap is a HEALTH cut
    (11 S9.6.5 row 3), not a profile cut. mutant: keep the tied term named
    profile -> red."""
    g = compute_gate(v_nom_mps=0.5, spec_max_vx_mps=2.0, f_free_mps=6.0, health=OK,
                     i_fix=1.0, i_heading=1.0, heading_valid=True, estop=False,
                     perception_dead=False, profile_downgraded=True)
    assert g.v_max_fwd == pytest.approx(0.5)
    limiter, all_ = attribute(g, 1.0)
    assert limiter == "health" and all_ == ("health",)
