"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p5_gateway config subpackage marker + public re-exports

Description:
Freeze-time assertions for p5_gateway. The .yaml lives in configs/;
code that runs the assertions lives here. The bind_guard module
exports P5ConfigError / PENDING_KEYS_ALLOWED / check_p5_config; the
assertions module exports the P5-BIND-1/-2/PEND-1 helpers used by
newer batches.
"""

from xbrain.p5_gateway.config.bind_guard import (
    P5ConfigError, PENDING_KEYS_ALLOWED, check_p5_config,
)


__all__ = [
    "P5ConfigError",
    "PENDING_KEYS_ALLOWED",
    "check_p5_config",
]
