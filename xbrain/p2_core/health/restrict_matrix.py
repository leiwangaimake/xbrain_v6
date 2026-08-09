"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: restrict_matrix.py
Brief: BIZ-P2-20 -- health item x restricted-function matrix (FM-1..FM-3)

Description:
11 S5.1D / 14 S8 restrict matrix. For each combination of (item, state)
what functions become UNAVAILABLE.

FM-1 exceptions:
  rtk=degraded (rtk_float) -> refuse NEW tasks with E_DEGRADED;
  other degraded items only lower speed_factor / frame rate,
  NEVER disable functions.

FM-2: 'today broken' (fail) is DIFFERENT from 'never had it'
(capability). Rejection codes distinguish:
  * item fail  -> E_UNHEALTHY + detail.item
  * capability -> E_CAPABILITY (a different error entirely)

FM-3: health downgrade NEVER auto-switches mode. voice_mode stays
what it was; only the acquire path (e.g., asr_local cannot hold
domain 3 while mic=fail) reflects the fault.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Mapping

from xbrain.common.errors import E_DEGRADED, E_UNHEALTHY
from xbrain.p2_core.health.items import HealthState


@dataclass(frozen=True)
class RestrictDecision:
    """Whether a specific operation is allowed given health state."""
    allowed: bool
    code: str = "OK"           # E_UNHEALTHY / E_DEGRADED / E_CAPABILITY / OK
    detail_item: str = ""


def check_new_task_admission(item_states: Mapping[str, HealthState]) -> RestrictDecision:
    """FM-1: rtk=degraded refuses NEW tasks."""
    rtk = item_states.get("rtk", HealthState.UNKNOWN)
    if rtk == HealthState.DEGRADED:
        return RestrictDecision(
            allowed=False, code=E_DEGRADED,
            detail_item="rtk",
        )
    # Fatal-fail items handled elsewhere (factor.py returns
    # allow_motion=False; task admission checks factor too).
    return RestrictDecision(allowed=True)


def check_asr_local_admission(item_states: Mapping[str, HealthState]) -> RestrictDecision:
    """asr_local source cannot hold domain 3 when mic=fail. FM-3:
    does NOT change voice_mode."""
    mic = item_states.get("mic", HealthState.UNKNOWN)
    if mic == HealthState.FAIL:
        return RestrictDecision(
            allowed=False, code=E_UNHEALTHY,
            detail_item="mic",
        )
    return RestrictDecision(allowed=True)


def check_ptz_command(item_states: Mapping[str, HealthState]) -> RestrictDecision:
    """FM-2: PTZ down -> E_UNHEALTHY (device broke), NOT E_CAPABILITY
    (which would say the robot never had a PTZ)."""
    ptz = item_states.get("ptz", HealthState.UNKNOWN)
    if ptz == HealthState.FAIL:
        return RestrictDecision(
            allowed=False, code=E_UNHEALTHY,
            detail_item="ptz",
        )
    return RestrictDecision(allowed=True)


def check_time_window_rules_active(item_states: Mapping[str, HealthState]) -> bool:
    """RE-3a: time_window rules require clock=OK."""
    return item_states.get("clock", HealthState.UNKNOWN) == HealthState.OK
