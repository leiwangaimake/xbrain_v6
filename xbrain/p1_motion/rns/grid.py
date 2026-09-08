"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: grid.py
Brief: Three-state fusion + memory grid (P2 -- 20 S3.1.2/S4)

Description:
Asymmetric fusion (RNS-N-5: BLOCKED=union, FREE=intersection) and the RTK-anchored
memory grid (S4.2.1 write rules: observed-UNKNOWN never overwrites non-UNKNOWN;
pose-jump > reset_jump_m clears the grid -- A-MEM-3). RNS-I-5/6 near blind zone.

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
