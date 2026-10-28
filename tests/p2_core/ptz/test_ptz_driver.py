"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptz_driver.py
Brief: PTZ driver -- voice E-intents plan the right legs, primitives send one ONVIF call each

Description:
Since 2026-10-28 (Qt contract v0.2, 99 U98) the driver no longer sleeps
through a pulse: plan() turns an E-intent into timed legs (velocity, ms) that
ptz_controller runs, and move() / stop() are single ONVIF calls. These cases
pin the sign of each direction, the pulse archive per amount, the speed
levels, the scan shapes, and that a blocked or slot-less intent plans
nothing -- plus that plan() itself never touches the camera (the property
that makes every pulse interruptible). onvif_client is stubbed; no device.
Each case names the mutant it kills (CLAUDE.md 3.3).
"""
from __future__ import annotations

import pytest

import xbrain.p2_core.ptz.onvif_client as oc
from xbrain.p2_core.ptz.ptz_driver import PtzDriver, PtzDriverConfig

pytestmark = pytest.mark.no_device


@pytest.fixture
def driver(monkeypatch):
    """A driver whose ONVIF calls are recorded instead of sent."""
    calls = []
    monkeypatch.setattr(oc, "get_profile_token", lambda s, *a, **k: "tok")
    monkeypatch.setattr(oc, "ptz_continuous",
                        lambda s, p, t, **kw: calls.append(("move", kw)))
    monkeypatch.setattr(oc, "ptz_stop",
                        lambda s, p, t, **kw: calls.append(("stop", kw)))
    d = PtzDriver(PtzDriverConfig(host="h", user="u", pwd="p"))
    d._calls = calls  # recorded camera calls
    return d


def _vel(plan, i=0):
    """Velocity of leg i."""
    return plan.legs[i][0]


# -- E01 move: direction sign + pulse archive ----------------------------

def test_move_left_negative_pan(driver):
    """MUTATION: flip the left sign -> pan > 0 -> red."""
    plan = driver.plan("E01", {"direction": "left", "amount": "normal"})
    assert plan.kind == "legs" and len(plan.legs) == 1
    pan, tilt, zoom = _vel(plan)
    assert pan < 0 and tilt == 0.0 and zoom == 0.0     # left = pan negative


def test_move_up_positive_tilt(driver):
    """MUTATION: swap the up/down vectors -> tilt < 0 -> red."""
    pan, tilt, _zoom = _vel(driver.plan("E01", {"direction": "up"}))
    assert tilt > 0 and pan == 0.0


def test_move_amount_picks_pulse(driver):
    """MUTATION: ignore the amount slot -> both legs 1000 ms -> red."""
    small = driver.plan("E01", {"direction": "left", "amount": "small"})
    large = driver.plan("E01", {"direction": "left", "amount": "large"})
    assert small.legs[0][1] == 350 and large.legs[0][1] == 2800


def test_move_no_direction_dropped(driver):
    """No direction -> nothing planned, counted as dropped.
    MUTATION: default the direction -> a 'legs' plan -> red."""
    plan = driver.plan("E01", {"amount": "normal"})
    assert plan.kind == "drop" and driver.calls_dropped == 1


def test_move_bad_amount_dropped(driver):
    """An amount outside the archive is dropped, never mapped to a guess.
    MUTATION: fall back to 'normal' -> 'legs' -> red."""
    assert driver.plan("E01", {"direction": "left", "amount": "huge"}).kind == "drop"


# -- E06 zoom ------------------------------------------------------------

def test_zoom_in_positive(driver):
    """MUTATION: flip the zoom sign -> red."""
    pan, _tilt, zoom = _vel(driver.plan("E06", {"zoom_dir": "in"}))
    assert zoom > 0 and pan == 0.0


def test_zoom_out_negative(driver):
    """Zoom out is the negative velocity."""
    assert _vel(driver.plan("E06", {"zoom_dir": "out"}))[2] < 0


# -- E05 / E08 stop ------------------------------------------------------

def test_stop_intents(driver):
    """MUTATION: drop E08 from the stop set -> 'drop' -> red."""
    assert driver.plan("E05", {}).kind == "stop"
    assert driver.plan("E08", {}).kind == "stop"


# -- E09 speed changes the leg magnitude ---------------------------------

def test_speed_fast_larger_magnitude(driver):
    """MUTATION: ignore the speed level -> equal magnitudes -> red."""
    driver.plan("E09", {"level": "slow"})
    slow = abs(_vel(driver.plan("E01", {"direction": "right"}))[0])
    driver.plan("E09", {"level": "fast"})
    fast = abs(_vel(driver.plan("E01", {"direction": "right"}))[0])
    assert fast > slow


def test_speed_up_down_steps(driver):
    """Steps clamp at both ends. MUTATION: no clamp -> IndexError -> red."""
    driver.plan("E09", {"level": "slow"})
    driver.plan("E09", {"level": "up"})
    assert driver._speed_level == "normal"
    driver.plan("E09", {"level": "up"})
    driver.plan("E09", {"level": "up"})               # clamp at fast
    assert driver._speed_level == "fast"
    for _ in range(4):
        driver.plan("E09", {"level": "down"})         # clamp at slow
    assert driver._speed_level == "slow"


# -- E07 scan shapes -----------------------------------------------------

def test_scan_orbit_is_one_long_pan(driver):
    """E07 orbit is one long single-direction pan, not the sweep.
    MUTATION: route orbit through the sweep -> three legs -> red."""
    plan = driver.plan("E07", {"scan_mode": "orbit", "direction": "right"})
    assert len(plan.legs) == 1 and _vel(plan)[0] > 0   # one pan, rightward
    assert plan.legs[0][1] >= 8000                      # a long (~full) turn


def test_scan_sweep_is_three_legs(driver):
    """Sweep = right, left (double length), right."""
    plan = driver.plan("E07", {"scan_mode": "sweep"})
    signs = [v[0] > 0 for v, _ms in plan.legs]
    assert signs == [True, False, True]
    assert plan.legs[1][1] == 2 * plan.legs[0][1]


# -- planning never touches the camera; primitives send exactly one call --

def test_plan_sends_nothing(driver):
    """The property that makes every pulse interruptible: planning is pure.
    MUTATION: send the first move from plan() -> recorded call -> red."""
    for intent, slots in (("E01", {"direction": "left"}), ("E06", {"zoom_dir": "in"}),
                          ("E07", {"scan_mode": "orbit"}), ("E05", {})):
        driver.plan(intent, slots)
    assert driver._calls == []


def test_move_is_one_continuous_move_with_three_axes(driver):
    """11 PTZ-C5: one ContinuousMove carries pan, tilt and zoom.
    MUTATION: send pan/tilt and zoom as two calls -> two records -> red."""
    driver.move(0.5, -0.5, 0.25)
    assert driver._calls == [("move", {"pan": 0.5, "tilt": -0.5, "zoom": 0.25})]


def test_stop_passes_axis_groups(driver):
    """stop(pantilt, zoom) maps 1:1 onto the ONVIF Stop flags."""
    driver.stop(False, True)
    assert driver._calls == [("stop", {"pantilt": False, "zoom": True})]


def test_blocked_intent_dropped_not_faked(driver):
    """E02 home (T-PTZ-1 blocked): nothing planned, nothing sent."""
    assert driver.plan("E02", {}).kind == "drop"
    assert driver._calls == [] and driver.calls_dropped == 1
