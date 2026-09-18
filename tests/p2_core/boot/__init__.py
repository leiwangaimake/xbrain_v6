"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: package marker for the P2 boot-stage tests

Description:
Present only so pytest imports tests/p2_core/boot/* as a package, matching
every other tests/p2_core subdirectory. Carries no test code and no fixtures;
shared helpers belong in the test modules that use them or in a conftest, not
in a package marker where a reader would not look for them.
"""
