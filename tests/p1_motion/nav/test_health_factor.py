"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_health_factor.py
Brief: HealthFactorSlot timeout ladder (11 S3.6) and body validation

Description:
The ladder has four rungs (never / ok / degraded / dead) and the two that
matter for safety are the ends: never received must forbid motion (a p1 that
starts before p2's grant must not drive), and dead must forbid motion. The
degraded rung must take the LOWER of 0.3 and the last value (12 S3.3 monotone
non-increasing ceilings). Off-shape bodies are refused, not clamped.
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.nav.health_factor import (DEGRADED_SPEED_FACTOR,
                                                HealthFactorError,
                                                HealthFactorSlot,
                                                parse_health_factor)

pytestmark = pytest.mark.no_device

DEGRADE, DEAD = 3000, 10000


def _slot():
    return HealthFactorSlot(DEGRADE, DEAD)


def _body(sf=1.0, allow=True, mp="patrol"):
    return {"speed_factor": sf, "allow_motion": allow, "max_profile": mp,
            "reason": "none", "detail_ref": "health/summary"}


def test_never_received_forbids_motion():
    """mutant: return allow_motion=True on 'never' -> red."""
    v = _slot().view(5000)
    assert v.state == "never"
    assert v.allow_motion is False
    assert v.speed_factor == 0.0


def test_fresh_message_passes_through():
    s = _slot()
    s.on_message(_body(0.8, True, "patrol"), rx_mono_ms=1000)
    v = s.view(1500)
    assert (v.state, v.speed_factor, v.allow_motion, v.max_profile) == ("ok", 0.8, True, "patrol")
    assert v.age_ms == 500


def test_degraded_takes_the_lower_factor():
    """3 s without -> 0.3, but never ABOVE the last received value.
    mutant: use the last value instead of min(0.3, last) -> red."""
    s = _slot()
    s.on_message(_body(0.9), rx_mono_ms=1000)
    assert s.view(1000 + DEGRADE).speed_factor == DEGRADED_SPEED_FACTOR
    s.on_message(_body(0.1), rx_mono_ms=1000)
    assert s.view(1000 + DEGRADE).speed_factor == 0.1
    assert s.view(1000 + DEGRADE).state == "degraded"


def test_dead_forbids_motion_even_if_last_allowed():
    s = _slot()
    s.on_message(_body(1.0, True), rx_mono_ms=1000)
    v = s.view(1000 + DEAD)
    assert v.state == "dead"
    assert v.allow_motion is False
    assert v.speed_factor == 0.0


def test_last_allow_motion_false_is_kept_while_fresh():
    s = _slot()
    s.on_message(_body(1.0, False), rx_mono_ms=1000)
    assert s.view(1100).allow_motion is False


@pytest.mark.parametrize("body", [
    _body(sf=1.7), _body(sf=-0.1), _body(sf="1"), _body(sf=True),
    _body(allow="true"), _body(mp="cruise"), _body(mp=None), "nope",
])
def test_off_shape_body_is_refused_and_counted(body):
    s = _slot()
    with pytest.raises(HealthFactorError):
        s.on_message(body, rx_mono_ms=1000)
    assert s.rejected == 1 and s.received == 0
    assert s.view(1000).state == "never"


def test_parse_returns_typed_triple():
    assert parse_health_factor(_body(1, True, "obstacle_avoid")) == (1.0, True, "obstacle_avoid")


@pytest.mark.parametrize("deg, dead", [(0, 100), (100, 100), (200, 100), (True, 100), (1.5, 100)])
def test_ctor_refuses_bad_thresholds(deg, dead):
    with pytest.raises(HealthFactorError):
        HealthFactorSlot(deg, dead)
