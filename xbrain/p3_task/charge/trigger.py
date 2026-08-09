"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: trigger.py
Brief: BIZ-P3-15 charge triggers CR-1..CR-8 + §8.2A critical stop-in-place

Description:
15 §8.2 defines eight independent conditions that request charging:

  CR-1 SoC below soc_low_pct  -> normal charge request
  CR-2 SoC below soc_urgent_pct -> pre-empt everything
  CR-3 SoC below soc_critical_pct -> §8.2A stop in place, DO NOT drive
  CR-4 explicit user command
  CR-5 scheduled patrol block ended with SoC < soc_recover_pct
  CR-6 return_home task specifies auto-charge
  CR-7 dock_admission gate cleared and a charge task was previously
       suspended waiting for this dock
  CR-8 idle timer > idle_charge_after_s

CR-3 (§8.2A) is special: the robot must NOT try to drive to a dock.
Driving from critical SoC to any dock might not make it; the safe
fallback is stop-in-place and emit HEALTH_CRITICAL for a human.

All thresholds come from configs (no defaults, per CLAUDE.md §3.1).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ChargeDecision:
    triggered: bool
    trigger_code: str = ""
    stop_in_place: bool = False


@dataclass(frozen=True)
class ChargeThresholds:
    """All fields required at construction (no defaults). Caller
    reads them from configs and passes them in explicitly. Missing
    values -> None -> AttributeError at first use."""
    soc_low_pct: float
    soc_urgent_pct: float
    soc_critical_pct: float
    soc_recover_pct: float
    idle_charge_after_s: float


def evaluate(soc_pct: float,
              user_requested: bool,
              patrol_block_ended: bool,
              return_home_auto: bool,
              dock_admission_ok_for_suspended: bool,
              idle_seconds: float,
              thr: ChargeThresholds) -> ChargeDecision:
    """Evaluate CR-1..CR-8 in priority order (CR-3 first because it
    changes what we're allowed to DO, not just what we should
    request)."""
    if soc_pct < thr.soc_critical_pct:
        return ChargeDecision(triggered=True, trigger_code="CR-3",
                                stop_in_place=True)
    if soc_pct < thr.soc_urgent_pct:
        return ChargeDecision(triggered=True, trigger_code="CR-2")
    if user_requested:
        return ChargeDecision(triggered=True, trigger_code="CR-4")
    if soc_pct < thr.soc_low_pct:
        return ChargeDecision(triggered=True, trigger_code="CR-1")
    if patrol_block_ended and soc_pct < thr.soc_recover_pct:
        return ChargeDecision(triggered=True, trigger_code="CR-5")
    if return_home_auto:
        return ChargeDecision(triggered=True, trigger_code="CR-6")
    if dock_admission_ok_for_suspended:
        return ChargeDecision(triggered=True, trigger_code="CR-7")
    if idle_seconds >= thr.idle_charge_after_s:
        return ChargeDecision(triggered=True, trigger_code="CR-8")
    return ChargeDecision(triggered=False)
