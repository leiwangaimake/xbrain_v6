"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: watchdog.py
Brief: Global progress watchdog (P5 -- 20 S7.3A)

Description:
RNS-N-15: reactive-layer no-progress (delta_w in T_w over motion states,
wait excluded) forces wall-follow escalation; two branches (a boundary to hug ->
escalate; none -> watchdog_no_progress). argmax limiter source filtered to
exclude heading/RTK (they have own failure paths). A-CVG-2.

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
