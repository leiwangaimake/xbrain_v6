"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: handlers.py
Brief: MOT-PM-27 P1 failure table + event/{severity}/motion emission

Description:
12 S11 enumerates P1 failure classes with severity + response:
  cmd_age_too_high      severity=warn   -> SAFE_STOP transition
  fence_lost            severity=fault  -> zero-vel + fault event
  rns_module_dead       severity=warn   -> disable RNS source
  quadruped_link_down   severity=fault  -> SAFE_STOP transition
  perception_fail       severity=warn   -> shed obstacle_avoid profile

Each hit emits ONE event/{severity}/motion with a detail dict; the
kind is a closed set per row.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, FrozenSet


class MotionFailKind(str, Enum):
    CMD_AGE_TOO_HIGH = "cmd_age_too_high"
    FENCE_LOST = "fence_lost"
    RNS_MODULE_DEAD = "rns_module_dead"
    QUADRUPED_LINK_DOWN = "quadruped_link_down"
    PERCEPTION_FAIL = "perception_fail"


_KIND_SEVERITY = {
    MotionFailKind.CMD_AGE_TOO_HIGH: "warn",
    MotionFailKind.FENCE_LOST: "fault",
    MotionFailKind.RNS_MODULE_DEAD: "warn",
    MotionFailKind.QUADRUPED_LINK_DOWN: "fault",
    MotionFailKind.PERCEPTION_FAIL: "warn",
}


@dataclass(frozen=True)
class MotionFailEvent:
    kind: str
    severity: str
    detail: Dict[str, Any]


def build_event(kind: MotionFailKind, detail: Dict[str, Any]) -> MotionFailEvent:
    """Assemble one event/{severity}/motion event. Severity comes
    from the fixed table above; caller cannot override."""
    return MotionFailEvent(
        kind=kind.value,
        severity=_KIND_SEVERITY[kind],
        detail=detail,
    )
