"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: chassis_light.py
Brief: BIZ-P2-27 -- rt/chassis/light channel (separate from payload light)

Description:
Chassis (M20S 底盘) has its OWN light channel exposed on Zenoh key
`rt/chassis/light`. This is DIFFERENT from `cmd/chassis/light`
(which flows chassis-side up through quadruped) and DIFFERENT from
the payload light domain (14 §4.3 domain 4 lives on the payload
device via GZH-2, not on the chassis).

Separation rule (BIZ-P2-27 spec):
  * chassis lights and payload lights MUST NEVER be merged into a
    single channel / single message
  * a change to chassis lights uses a different key + different
    codec + different owner than payload lights

This module publishes rt/chassis/light. It is a thin wrapper because
the payload of the message is the chassis's own light command (which
quadruped defines); P2 just relays operator requests here without
transformation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import FrozenSet


# Chassis light patterns known to M20S. Closed set; adding requires
# quadruped side change too.
CHASSIS_LIGHT_PATTERNS: FrozenSet[str] = frozenset({
    "off", "solid", "breathing", "flash_slow", "flash_fast",
})


@dataclass(frozen=True)
class ChassisLightCommand:
    """rt/chassis/light message shape."""
    pattern: str                  # closed set
    color_rgb: tuple = (255, 255, 255)   # 0..255 per channel

    def __post_init__(self) -> None:
        if self.pattern not in CHASSIS_LIGHT_PATTERNS:
            raise ValueError(
                "pattern %r not in %s"
                % (self.pattern, sorted(CHASSIS_LIGHT_PATTERNS)))
        if len(self.color_rgb) != 3:
            raise ValueError("color_rgb must be (r, g, b)")
        for c in self.color_rgb:
            if not (0 <= c <= 255):
                raise ValueError("color_rgb components must be 0..255")


def is_payload_light_key(key: str) -> bool:
    """The payload-light-related Zenoh keys. If a chassis-light
    codepath ever touches one of these it's a merged-channel bug."""
    return key.startswith("cmd/payload/light") or \
           key.startswith("state/payload_light")


def guard_no_merge(intended_key: str) -> None:
    """Guard called at chassis-light publish time. If the intended
    Zenoh key names the payload-light surface it's a merged-channel
    defect."""
    if is_payload_light_key(intended_key):
        raise ValueError(
            "chassis light publish must NEVER target %r "
            "(payload light channel) -- keep them separate per BIZ-P2-27"
            % intended_key)
