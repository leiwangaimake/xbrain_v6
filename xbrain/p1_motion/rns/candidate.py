"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: candidate.py
Brief: Thread/detour candidates (P4 -- 20 S6)

Description:
Profile edge-discontinuity subgoals (RNS-N-9), hard gates + width saturation
(RNS-N-10, gate_saturate_ratio), five-weight cost (S6.4), change-candidate
hysteresis (side_hold_ticks). Threading only between static obstacles (RNS-N-11).

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
