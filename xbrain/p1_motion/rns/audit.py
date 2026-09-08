"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: audit.py
Brief: Ring audit buffer, non-blocking (P6 -- 20 S9.3)

Description:
In-loop decisions land in a ring buffer; a non-realtime thread drains it
(RNS-M-2: no in-loop file I/O). Records superseded/cancelled outcomes (S9.0.3)
and failure attributions for observability.

P0.4 status: SKELETON. Structure and interface only; the tick logic lands in
its phase (see the phase tag in Brief). is_active paths stay inert so wiring
this file into p1 does not change robot behavior (RNS_TODO P0.4).
"""

from __future__ import annotations
