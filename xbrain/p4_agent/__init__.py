"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Package root for P4 (the intent pipeline and AI gateway), no logic

Description:
What this package is. P4 is the process 10 S3.1 lists as `p4_agent` on the
general plane: the 128-intent pipeline and the AI service gateway, designed in
volume 16. Everything Python that belongs to P4 lives under here, per the
directory rule the user fixed on 2026-08-05.

Why this file has no code, same reason as xbrain/__init__.py. Importing any
subpackage executes this module first, and the first subpackage a P4 process
imports is config/ -- which runs before the process has any configuration at
all. Code placed here would therefore execute in a context where a failure has
nowhere sensible to be reported, and the traceback would point at a package
root rather than at whatever actually failed.

What this package deliberately does NOT contain:
  * no rclpy, ever. 10 S3.1 puts the whole AI software stack at zero rclpy
    dependency; ROS lives only on the quadruped side;
  * no direct HTTP calls to the AI services. 16 S14 routes those through
    ai_client/*_client.py, so that the timeout and circuit-breaker behaviour of
    16 S9.3 exists in one place rather than at each call site;
  * no configuration defaults. The one way P4 obtains configuration is
    xbrain.p4_agent.config, which reads the freeze line's product and refuses
    when a value is missing.

NEVER add convenience re-exports here. A re-export makes one module importable
from two paths, and the two drift the moment one is renamed.
"""
