"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Package root for the runtime layer shared by all fifteen processes

Description:
What this package is. xbrain/common/ holds what every XBRAIN process needs
before it can do anything of its own: the E_* closed set, the non-error closed
sets, and the configuration loader. 11 S13.8 requires error codes to come from a
shared library rather than string literals, and the reason generalises to the
other closed sets -- a value spelled slightly differently in two processes
compiles, runs, and only surfaces during integration.

Relationship to common/ at the repository root. That directory holds DEPLOYED
artifacts: the C++ header generated from the same data files used here. The
Python source that generates it lives under xbrain/, per the directory rule the
user fixed on 2026-08-05. Two languages, one source of data, so they cannot
drift.

Why this file has no code. It is imported before any process has read its
configuration; see the note in xbrain/__init__.py. Subpackages are imported
explicitly by the modules that need them.
"""
