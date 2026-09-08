"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: classify.py
Brief: Obstacle dispatch by class (P3 -- 20 S5.1/S5.2/S5.5/S5.6)

Description:
class_map dispatch (RNS-N-7): unmapped -> block+slow; person locked to stop
(never reclassified). Static-vs-dynamic by velocity threshold + dwell (S5.5).
Per-class inflation margin (S5.6, U54 one-way folded in). velocity_frame==raw
refused as a motion cue (S3.1.5).

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
