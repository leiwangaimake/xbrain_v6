"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: backup.py
Brief: Bounded backup under memory (P5 -- 20 S7.5 / RNS-N-13)

Description:
The relaxed "no reverse" rule (RNS-N-13, S7.5). Reverse is normally forbidden (a
robot backing into space it cannot see is unsafe). It is relaxed ONLY into space
observed FREE within the last t_backup_memory_s, up to max_backup_m, and stops
IMMEDIATELY if that memory expires (A-BK-1/2).

Priority (S7.5): in-place rotation + strafe first (M20S supports both, needs
heading L1); backup is the backstop. This file only decides IF a backup step is
permitted; the caller tries rotation/strafe first.
"""

from __future__ import annotations


def backup_permitted(
    rear_observed_free: bool, rear_memory_age_s: float,
    backed_so_far_m: float,
    t_backup_memory_s: float, max_backup_m: float,
) -> bool:
    """May the robot take a backup step now (RNS-N-13, A-BK-1/2)?
      - the rear must have been observed FREE recently (within t_backup_memory);
      - the total backed distance must be under max_backup_m;
      - stale memory -> immediately no (A-BK-1: reverse only into recently-
        observed FREE).
    mutant: drop the observed-FREE / freshness check -> reverse into unobserved
    space -> A-BK-1 red."""
    if not rear_observed_free:
        return False
    if rear_memory_age_s > t_backup_memory_s:   # memory expired -> stop (A-BK-2)
        return False
    if backed_so_far_m >= max_backup_m:          # distance cap
        return False
    return True
