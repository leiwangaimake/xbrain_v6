"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_band_hysteresis.py
Brief: 12 S6.2 hysteresis table -- instant drop, margin + hold to rise, per-band timers, gap memory, NavTick integration

Description:
The rise/drop rules of the f term with the U54 numbers (3.5 m / 3.0 s on
top, 2.3 m and 1.75 m by the same rule below), the per-band timer behaviour
(one hold from 1 m to 5 m reaches 2.0 directly; a staged retreat stops at
0.5 until 3.5 m has its own 3 s), an unknown reading keeping the band but
breaking every timer, and the same rules seen through NavTick: a RNS goto
in open ground starts in the 0.5 band when the first clearance says so and
is not allowed to speed up before the hold.
"""
from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from tests.p1_motion.rns.scenes import healthy_status, snapshot, uniform_free
from xbrain.p1_motion.gate.speed_gate import BANDS, BandHysteresis, SpeedGateError, band_of
from xbrain.p1_motion.nav.health_factor import HealthView
from xbrain.p1_motion.nav.nav_tick import NavInputs, NavTick, NavTickConfigError
from xbrain.p1_motion.nav.relmove_intake import RelMoveLimits, translate_relative_move
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.sources.arbiter_p1 import P1Arbiter
from xbrain.p1_motion.sources.rns_avoid import RnsAvoidSource

pytestmark = pytest.mark.no_device

_ROOT = Path(__file__).resolve().parents[3]
_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))
OK = HealthView(1.0, True, "patrol", "ok", 100)
LIM = RelMoveLimits(20.0, 6.2832, 0.02, 20.0, True)


def _h():
    return BandHysteresis(3000, 0.5)


def test_table_and_thresholds_are_the_u54_numbers():
    assert BANDS == ((3.0, 2.0), (1.8, 0.5), (1.25, 0.2), (0.0, 0.0))
    assert [band_of(d) for d in (5.0, 3.0, 2.99, 1.8, 1.79, 1.25, 1.24, -1.0)] == [0, 0, 1, 1, 2, 2, 3, 3]
    h = _h()
    assert [h.rise_threshold_m(k) for k in range(3)] == [3.5, 2.3, 1.75]


def test_drop_is_instant_and_rise_needs_margin_and_hold():
    """mutant: promote on the raw band -> red."""
    h = _h()
    assert h.update(5.0, 0) == 2.0                 # first reading enters raw
    assert h.update(2.9, 50) == 0.5                # below 3.0: down NOW
    assert h.update(3.4, 100) == 0.5               # raw says 2.0 but < 3.5: no rise
    for t in range(150, 3100, 50):
        assert h.update(3.6, t) == 0.5             # 3.6 held < 3 s
    assert h.update(3.6, 3150) == 2.0              # 3.6 held >= 3 s (150 -> 3150)


def test_timer_breaks_when_the_clearance_dips_below_the_threshold():
    """mutant: keep the timer across a dip -> red."""
    h = _h()
    h.update(2.0, 0)
    for t in range(50, 2050, 50):
        h.update(3.6, t)
    assert h.update(3.4, 2100) == 0.5              # dip below 3.5 restarts
    assert h.update(3.6, 5000) == 0.5              # 3.6 again, only 0 s so far
    assert h.update(3.6, 8100) == 2.0              # 5000 -> 8100 >= 3 s


def test_lower_bands_extrapolate_the_same_rule():
    h = _h()
    assert h.update(1.0, 0) == 0.0                 # stop band
    for t in range(50, 3050, 50):
        assert h.update(1.7, t) == 0.0             # raw 0.2 band but < 1.75
    assert h.update(1.8, 3100) == 0.0              # 1.8 >= 1.75, timer starts now
    assert h.update(1.8, 6150) == 0.2              # band 2 earned (3100 -> 6150); 2.3 never met
    h2 = _h()
    h2.update(1.0, 0)
    for t in range(50, 3100, 50):
        h2.update(1.8, t)                          # >= 1.75 (band 2) but < 2.3 (band 1)
    assert h2.update(1.8, 3100) == 0.2             # only one band up


def test_one_hold_reaches_the_top_when_every_threshold_was_met_all_along():
    """mutant: promote one band per hold -> red (2.0 would take 9 s)."""
    h = _h()
    h.update(1.0, 0)
    for t in range(50, 3050, 50):
        assert h.update(5.0, t) == 0.0
    assert h.update(5.0, 3100) == 2.0


def test_staged_retreat_stops_at_the_band_whose_own_hold_is_done():
    h = _h()
    h.update(1.0, 0)
    for t in range(50, 3050, 50):
        h.update(2.5, t)                           # >= 2.3 for 3 s, < 3.5
    assert h.update(3.6, 3100) == 0.5              # band 1 earned; band 0 timer just started
    assert h.update(3.6, 6050) == 0.5
    assert h.update(3.6, 6150) == 2.0              # 3100 -> 6150 >= 3 s


def test_unknown_reading_keeps_the_band_and_breaks_the_timers():
    """mutant: an unknown reading re-enters at the raw band -> red."""
    h = _h()
    h.update(2.0, 0)                               # 0.5 band
    for t in range(50, 2050, 50):
        h.update(3.6, t)
    assert h.update(None, 2100) is None            # no f term this tick
    assert h.band == 1                             # memory kept
    assert h.update(5.0, 2150) == 0.5              # back: still 0.5, timers restarted
    assert h.update(5.0, 5100) == 0.5
    assert h.update(5.0, 5200) == 2.0
    g = _h()
    g.update(5.0, 0)
    g.update(None, 50)
    assert g.update(1.0, 100) == 0.0               # coming back slower drops at once


def test_parameters_are_refused_when_null():
    for hold, margin in ((None, 0.5), (0, 0.5), (3000, None), (3000, 0.0), (True, 0.5)):
        with pytest.raises(SpeedGateError):
            BandHysteresis(hold, margin)


# --- through NavTick -------------------------------------------------------

def _stack(hold_ms=3000):
    cfg = copy.deepcopy(_CFG)
    rns = RnsSource(cfg=cfg, r_eff_m=0.5)
    src = RnsAvoidSource(rns, cfg["rns"])
    tick = NavTick(src, P1Arbiter(dwell_ms=200), v_nom_mps=2.0, wz_max_rps=1.2,
                   spec_max_vx_mps=2.0, holonomic=True,
                   speed_up_hold_ms=hold_ms, d_up_margin_m=0.5)
    goal = translate_relative_move(
        {"cmd_id": "rm-1", "dx_m": 15.0, "dy_m": 0.0, "dyaw_rad": 0.0},
        pose_xy=(0.0, 0.0), yaw_rad=0.0, heading_valid=True, holonomic=True, limits=LIM)
    src.load_goto(goal, 1000)
    return tick


def _inp(now, clearance):
    return NavInputs(now_mono_ms=now, pose_xy=(0.0, 0.0), yaw_rad=0.0, heading_valid=True,
                     i_fix=1.0, i_heading=1.0,
                     perception=snapshot(uniform_free(clearance, t_capture_mono_ms=now,
                                                      t_seg_mono_ms=now - 10),
                                         None, healthy_status(t_publish_mono_ms=now)),
                     health=OK, estop=False, teleop_active=False)


def test_nav_tick_holds_the_slow_band_until_the_rise_is_earned():
    """The gate's free_space term follows the band memory, not the raw
    clearance: open ground after a 2.5 m start stays at 0.5 for the hold
    (limiter free_space), then releases to the nominal 2.0."""
    tick = _stack()
    out = None
    for k in range(6):
        out = tick.run(_inp(1000 + 50 * k, 2.5))    # 0.5 band
    assert out.v_max == pytest.approx(0.5) and out.limiter == "free_space"
    for k in range(6, 60):                          # 6.0 m for 2.7 s: still held
        out = tick.run(_inp(1000 + 50 * k, 6.0))
    assert out.v_max == pytest.approx(0.5) and out.limiter == "free_space"
    for k in range(60, 70):                         # past the 3 s hold
        out = tick.run(_inp(1000 + 50 * k, 6.0))
    assert out.v_max == pytest.approx(2.0) and out.limiter != "free_space"


def test_nav_tick_refuses_a_null_hysteresis_leaf():
    cfg = copy.deepcopy(_CFG)
    src = RnsAvoidSource(RnsSource(cfg=cfg, r_eff_m=0.5), cfg["rns"])
    with pytest.raises(NavTickConfigError):
        NavTick(src, P1Arbiter(), v_nom_mps=2.0, wz_max_rps=1.2, spec_max_vx_mps=2.0,
                holonomic=True, speed_up_hold_ms=None, d_up_margin_m=0.5)
