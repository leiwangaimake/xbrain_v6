"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: inputs.py
Brief: RNS perception consume face (P2 -- 20 S3.1)

Description:
Snapshot views over profile/objects/status with T-50~53 age gating, bit0/no-seg
degrade, and the RNS-I-1..4 consume discipline (null stays UNKNOWN, d_free<=d_block,
per-message ageing). Fed by the tick snapshot (RNS-M-4); this file never reads a
Zenoh session itself.

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
