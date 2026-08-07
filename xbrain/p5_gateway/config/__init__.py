"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Re-export the GWY-P5-17 config guards

Description:
`from xbrain.p5_gateway.config import check_p5_config` is the one entry the
gateway's startup (Phase 2) calls before binding anything; the tests call it on
parsed mappings. See bind_guard.py for what is and is not covered.
"""

from xbrain.p5_gateway.config.bind_guard import (
    P5ConfigError, check_p5_config, FORBIDDEN_SEGMENTS, PENDING_KEYS_ALLOWED,
)

__all__ = ["check_p5_config", "P5ConfigError", "FORBIDDEN_SEGMENTS",
           "PENDING_KEYS_ALLOWED"]
