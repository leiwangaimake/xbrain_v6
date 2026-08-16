"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: xbrain.common.time package marker (site-timezone display helpers)

Description:
Package marker for the shared display-time helpers. The only member today is
local_time.py, the single place UTC is turned into a site-local string for
DISPLAY (HMI footer clock / TTS G24). It carries no safety logic -- every
timeout/period/age uses the monotonic clock (CLK-C1); timezone is display-only.
"""
