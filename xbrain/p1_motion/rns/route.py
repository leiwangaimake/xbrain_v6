"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: route.py
Brief: Polyline follow + route pointer (P1 -- 20 S2)

Description:
Arc-length projection with monotone index window (RNS-N-4), pure-pursuit lookahead
(S2.2), goto/path unified (RNS-N-1), progressive align (RNS-N-3), deviation limit
(S2.7). Consumes cmd/motion/route pointer with route_rev cross-check (12 S3.5A /
11 S7.12.1). Mission carries origin (12 S4.2c.1).

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
