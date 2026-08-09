"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: d_class.py
Brief: GWY-P4-28 -- D-class light/volume routing PL-1..PL-6 + D01 dual-channel

Description:
18 D-class covers payload light + volume commands. Routing:
  PL-1  brightness value in [1..100] -> D02 set_brightness
  PL-2  brightness 0 -> D07 (turn off, distinct intent)
  PL-3  brightness > 100 -> reject E_RANGE
  PL-4  red/blue mode value in [1..16] -> D18 set_pattern
  PL-5  red/blue mode = 0 -> reject (0 is 'off' which is D07)
  PL-6  volume [0..100] -> D14 set_volume

D01 which dual-channel: 'turn on the lights' can mean payload
searchlight (D01 which=searchlight) OR chassis light (D01 which=
chassis). The `which` slot MUST be filled; missing = E_SCHEMA.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class DRangeError(RuntimeError):
    """PL-3 / PL-5 range violation."""


def route_brightness(value: int) -> str:
    """PL-1/2/3: choose the intent id based on brightness value."""
    if value == 0:
        return "D07_lights_off"
    if 1 <= value <= 100:
        return "D02_set_brightness"
    raise DRangeError(
        "PL-3: brightness %d out of range [0..100]" % value)


def route_redblue_pattern(value: int) -> str:
    """PL-4/5: choose based on pattern value."""
    if value == 0:
        raise DRangeError(
            "PL-5: red/blue pattern 0 is 'off'; use D07_lights_off instead")
    if 1 <= value <= 16:
        return "D18_set_pattern"
    raise DRangeError(
        "PL-4: red/blue pattern %d out of range [1..16]" % value)


def route_volume(value: int) -> str:
    """PL-6: 0..100 -> D14 set_volume."""
    if not (0 <= value <= 100):
        raise DRangeError(
            "PL-6: volume %d out of range [0..100]" % value)
    return "D14_set_volume"


class LightWhich(str, Enum):
    SEARCHLIGHT = "searchlight"
    CHASSIS = "chassis"


class SchemaError(RuntimeError):
    """D01 missing `which` slot."""


def route_lights_on(which: str) -> str:
    """D01 dual-channel: which is REQUIRED."""
    if not which:
        raise SchemaError(
            "D01 lights_on requires `which` slot (missing = ambiguous)")
    try:
        w = LightWhich(which)
    except ValueError as exc:
        raise SchemaError(
            "D01 which=%r not in %s"
            % (which, [x.value for x in LightWhich])) from exc
    return "D01_" + w.value
