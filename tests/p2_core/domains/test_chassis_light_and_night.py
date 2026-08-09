"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chassis_light_and_night.py
Brief: domains tests -- chassis light and night

Description:
BIZ-P2-27 + P2-28 -- chassis light no-merge + night_patrol RE-7.
"""


import pytest

from xbrain.p2_core.domains.chassis_light import (
    CHASSIS_LIGHT_PATTERNS, ChassisLightCommand,
    guard_no_merge, is_payload_light_key,
)
from xbrain.p2_core.suspicion.night_patrol import (
    NightPatrolCfg, effective_speed_limit_enabled, should_emit_advisory,
)


pytestmark = pytest.mark.no_device


# --- chassis light: no-merge with payload light ---

def test_chassis_light_command_accepts_all_patterns():
    for p in CHASSIS_LIGHT_PATTERNS:
        ChassisLightCommand(pattern=p)


def test_chassis_light_command_rejects_out_of_set():
    with pytest.raises(ValueError):
        ChassisLightCommand(pattern="strobe")


def test_chassis_light_rejects_bad_rgb():
    with pytest.raises(ValueError):
        ChassisLightCommand(pattern="solid", color_rgb=(300, 0, 0))


def test_payload_light_key_detected():
    """Guard identifies payload-light-related keys."""
    assert is_payload_light_key("cmd/payload/light")
    assert is_payload_light_key("state/payload_light")
    assert not is_payload_light_key("rt/chassis/light")
    assert not is_payload_light_key("cmd/chassis/light")


def test_guard_no_merge_rejects_payload_key():
    """VARIANT (spec P2-27): chassis-light path publishing on payload
    light key = merged-channel defect."""
    with pytest.raises(ValueError) as ei:
        guard_no_merge("cmd/payload/light")
    assert "merged" in str(ei.value).lower() or "BIZ-P2-27" in str(ei.value)


def test_guard_no_merge_accepts_chassis_key():
    guard_no_merge("rt/chassis/light")   # must not raise
    guard_no_merge("cmd/chassis/light")


# --- night_patrol RE-7 ---

def test_night_patrol_disabled_forces_speed_limit_off():
    """RE-7 rule 2: enabled=false overrides speed_limit_enabled=true."""
    cfg = NightPatrolCfg(enabled=False, speed_limit_enabled=True)
    assert effective_speed_limit_enabled(cfg) is False


def test_night_patrol_enabled_respects_config():
    cfg = NightPatrolCfg(enabled=True, speed_limit_enabled=True)
    assert effective_speed_limit_enabled(cfg) is True
    cfg2 = NightPatrolCfg(enabled=True, speed_limit_enabled=False)
    assert effective_speed_limit_enabled(cfg2) is False


def test_night_patrol_disabled_suppresses_advisory():
    """RE-7 rule 3: no spam events when night_patrol is off."""
    cfg = NightPatrolCfg(enabled=False)
    assert should_emit_advisory(cfg, now_hour=3) is False
