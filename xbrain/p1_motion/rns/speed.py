"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: speed.py
Brief: v = min(all caps) + heading degrade + align (P6 -- 20 S8)

Description:
The single speed authority: v is the min of all caps (S8.1), the six qualitative
caps share the g(x) coefficient family (S8.1A, monotone -- A-SPD-3), the align
segment blend+P law and arrival double-condition (S2.5), heading capability
shrink (S8.2). Amplitude clamp stays in 12 S8, not here.

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
