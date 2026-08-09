"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: night_patrol.py
Brief: BIZ-P2-28 -- night_patrol enabled=false 三条确定行为 (RE-7)

Description:
14 S6.2 RE-7: night_patrol.enabled=false MUST produce three
deterministic behaviors:
  1. Rules that REFERENCE night_patrol.window are NOT loaded
     (already handled by filter_by_night_patrol in rules_loader.py)
  2. speed_limit_enabled=true is IGNORED (no dwell-based speed cap)
  3. No advisory 'consider enabling night_patrol' spam events

This module owns the (2) + (3) rules; (1) lives in rules_loader.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class NightPatrolCfg:
    """From p2_core.yaml.suspicion.night_patrol."""
    enabled: bool
    speed_limit_enabled: bool = False


def effective_speed_limit_enabled(cfg: NightPatrolCfg) -> bool:
    """RE-7 rule 2: if enabled=false, speed_limit_enabled is FORCED
    to false regardless of its literal value. Prevents 'accidentally
    slowing the robot when night_patrol was intentionally disabled'."""
    if not cfg.enabled:
        return False
    return cfg.speed_limit_enabled


def should_emit_advisory(cfg: NightPatrolCfg, now_hour: int) -> bool:
    """RE-7 rule 3: no spam events when night_patrol is disabled.
    Even a well-intentioned 'consider enabling night_patrol at 03:00'
    hint is suppressed. If enabled=true, this returns True inside
    the configured window (advisory-mode hook)."""
    if not cfg.enabled:
        return False
    # Placeholder: real implementation reads window from cfg and
    # compares now_hour. Here we return True to indicate 'the
    # advisory path is UNLOCKED by enabled=true'.
    return True
