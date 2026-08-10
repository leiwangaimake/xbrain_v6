"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Integration tests package (V-3B voice loop smoke)

Description:
Integration-level tests. Different from unit tests in that they
open real Zenoh sessions (peer mode for pytest -- no external
zenohd required) and validate end-to-end wiring. Full runtime
(with routers, AI services, chassis_stub) runs from
scripts/dev/start_voice_loop.sh, not from pytest.
"""
