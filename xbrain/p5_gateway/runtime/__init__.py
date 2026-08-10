"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: xbrain/p5_gateway/runtime runtime wiring for the voice-loop MVP

Description:
Minimum-viable runtime wiring for the voice-loop smoke test. Real
production runtime (task queue / event pipeline / 20 Hz ctrl_loop)
grows on this scaffold in later batches; the smoke test only needs
each process to subscribe its inbound key and log observed messages
+ each process to publish enough for downstream to see life.
"""
