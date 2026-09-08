"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: wallfollow.py
Brief: Wall-follow state machine (P5 -- 20 S7)

Description:
Bug-style wall following: entry needs a real BLOCKED boundary (v1.13 guard),
PD keep-distance (S7.7), (2)' arc-length leave + D3 grazing relaxation (S7.3/S7A),
three failure criteria (S7.6), inner/outer corner rules. Structurally depends on
the memory grid (RNS-I-7).

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
