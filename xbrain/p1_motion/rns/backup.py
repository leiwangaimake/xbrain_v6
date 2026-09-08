"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: backup.py
Brief: Bounded backup under memory (P5 -- 20 S7.5)

Description:
RNS-N-13: relaxed no-reverse only into recently-observed FREE (backup keys,
t_backup_memory_s freshness). A-BK-1/2. Reverse never into unobserved space.

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
