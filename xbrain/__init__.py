"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Package root for the XBRAIN runtime, deliberately empty of logic

Description:
What this package is. Everything under xbrain/ is the XBRAIN runtime: P1 through
P5 and the cross-process layer they share. Per the directory rule the user fixed
on 2026-08-05, every Python implementation except the AI services and the ROS2
nodes lives here, and common/ at the repository root holds only deployed
binaries and headers rather than source.

Why this file has no code. Importing any subpackage executes this module first,
and several subpackages are imported by every process at startup. Anything put
here would therefore run before a process has parsed its own configuration, in a
context where a failure has no good place to be reported. Keeping it empty means
an import failure always points at the module that actually failed.

NEVER add convenience re-exports here. A re-export makes xbrain.something
importable from two paths, and the two then drift when one is renamed.
"""
