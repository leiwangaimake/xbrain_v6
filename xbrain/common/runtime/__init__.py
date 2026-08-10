"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: xbrain/common/runtime cross-process runtime helpers

Description:
Small runtime helpers shared across processes: Zenoh session lifecycle
context, monotonic-clock accessors, structured JSON logging. Kept out
of xbrain/common/zenoh/ because those are pure protocol constants;
these are process-lifecycle glue.
"""
