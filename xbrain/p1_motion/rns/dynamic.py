"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: dynamic.py
Brief: Dynamic distance rule + wait budget (P3 -- 20 S5.3/S5.3A)

Description:
Two-threshold stop/resume (S5.3) and the wait budget (RNS-N-16): timer bound to
STATE not track (rotating-blocker counterexample), static-reclassification
handoff, blocked_by_dynamic on timeout. person-blocks report only (upper layer
handles hailing).

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
