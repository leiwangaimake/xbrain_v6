"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptz_driver.py
Brief: PTZ driver maps E-intents to ONVIF move/zoom/stop (device stubbed)

Description:
Tests that the PTZ driver turns each E-intent into the right ONVIF calls
with the right direction sign, pulse length, and speed -- with onvif_client
and time.sleep stubbed so no device or real delay is needed. Each carries a
mutation guard per CLAUDE.md 3.3.
"""
from __future__ import annotations

import pytest

import xbrain.p2_core.ptz.onvif_client as oc
from xbrain.p2_core.ptz.ptz_driver import PtzDriver, PtzDriverConfig

pytestmark = pytest.mark.no_device


@pytest.fixture
def driver(monkeypatch):
    calls = []
    monkeypatch.setattr(oc, "get_profile_token", lambda s, *a, **k: "tok")
    monkeypatch.setattr(oc, "ptz_continuous",
                        lambda s, p, t, **kw: calls.append(("move", kw)))
    monkeypatch.setattr(oc, "ptz_stop",
                        lambda s, p, t, **kw: calls.append(("stop", kw)))
    # No real pulse delay in tests.
    monkeypatch.setattr("xbrain.p2_core.ptz.ptz_driver.time.sleep",
                        lambda s: None)
    d = PtzDriver(PtzDriverConfig(host="h", user="u", pwd="p"))
    d._calls = calls
    return d


def _moves(d):
    return [kw for kind, kw in d._calls if kind == "move"]


# -- E01 move: direction sign + pulse (function E01) ---------------------

def test_move_left_negative_pan(driver):
    driver.handle("E01", {"direction": "left", "amount": "normal"})
    m = _moves(driver)[-1]
    assert m["pan"] < 0 and m["tilt"] == 0.0          # left = pan negative
    # a Stop follows the move (bounded jog).
    assert driver._calls[-1][0] == "stop"


def test_move_up_positive_tilt(driver):
    driver.handle("E01", {"direction": "up", "amount": "normal"})
    m = _moves(driver)[-1]
    assert m["tilt"] > 0 and m["pan"] == 0.0


def test_move_amount_picks_pulse(driver, monkeypatch):
    """MUTATION guard: amount -> pulse archive (350/1000/2800). Capture the
    slept duration to prove small != large."""
    slept = []
    monkeypatch.setattr("xbrain.p2_core.ptz.ptz_driver.time.sleep",
                        lambda s: slept.append(s))
    driver.handle("E01", {"direction": "left", "amount": "small"})
    driver.handle("E01", {"direction": "left", "amount": "large"})
    assert slept[0] == 0.35 and slept[1] == 2.8       # 350ms vs 2800ms


def test_move_no_direction_dropped(driver):
    before = driver.calls_made
    driver.handle("E01", {"amount": "normal"})        # no direction
    assert driver.calls_made == before
    assert driver.calls_dropped >= 1


# -- E06 zoom ------------------------------------------------------------

def test_zoom_in_positive(driver):
    driver.handle("E06", {"zoom_dir": "in", "amount": "normal"})
    m = _moves(driver)[-1]
    assert m["zoom"] > 0 and m["pan"] == 0.0


def test_zoom_out_negative(driver):
    driver.handle("E06", {"zoom_dir": "out", "amount": "normal"})
    assert _moves(driver)[-1]["zoom"] < 0


# -- E05/E08 stop --------------------------------------------------------

def test_stop_intents(driver):
    driver.handle("E05", {})
    assert driver._calls[-1][0] == "stop"
    driver.handle("E08", {})
    assert driver._calls[-1][0] == "stop"


# -- E09 speed changes the move magnitude --------------------------------

def test_speed_fast_larger_magnitude(driver):
    driver.handle("E09", {"level": "slow"})
    driver.handle("E01", {"direction": "right", "amount": "normal"})
    slow_pan = abs(_moves(driver)[-1]["pan"])
    driver.handle("E09", {"level": "fast"})
    driver.handle("E01", {"direction": "right", "amount": "normal"})
    fast_pan = abs(_moves(driver)[-1]["pan"])
    assert fast_pan > slow_pan                        # fast jogs faster


def test_speed_up_down_steps(driver):
    driver.handle("E09", {"level": "slow"})
    driver.handle("E09", {"level": "up"})
    assert driver._speed_level == "normal"
    driver.handle("E09", {"level": "up"})
    assert driver._speed_level == "fast"
    driver.handle("E09", {"level": "up"})             # clamp at fast
    assert driver._speed_level == "fast"


# -- blocked intents are not driven (E_CAPABILITY handled upstream) ------

def test_scan_orbit_is_one_long_pan(driver, monkeypatch):
    """E07 orbit ('环视一周') is one long single-direction pan, not the
    3-leg sweep. MUTATION: routing orbit through the sweep path would emit
    three moves with alternating sign."""
    slept = []
    monkeypatch.setattr("xbrain.p2_core.ptz.ptz_driver.time.sleep",
                        lambda s: slept.append(s))
    driver.handle("E07", {"scan_mode": "orbit", "direction": "right"})
    moves = _moves(driver)
    assert len(moves) == 1 and moves[0]["pan"] > 0     # one pan, rightward
    assert slept[0] >= 8.0                              # a long (~full) turn


def test_scan_sweep_is_three_legs(driver):
    driver.handle("E07", {"scan_mode": "sweep", "direction": "left"})
    moves = _moves(driver)
    assert len(moves) == 3                              # right, left, right


def test_blocked_intent_dropped_not_faked(driver):
    before_made = driver.calls_made
    driver.handle("E02", {})                          # home: T-PTZ-1 blocked
    assert driver.calls_made == before_made           # no ONVIF call
    assert driver.calls_dropped >= 1
