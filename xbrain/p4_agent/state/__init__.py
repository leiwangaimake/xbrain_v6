"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p4_agent state subscription cache package marker

Description:
GWY-P4-39 (32.G): P4 subscribes the GEN-plane state/* topics (11 S7.16)
and caches the latest value per key WITH its receive time, so G-class
queries answer from LIVE data and fall back to 'unknown' when a reading is
stale rather than speaking a last-known value.
"""
