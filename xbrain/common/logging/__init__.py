"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Re-export get_logger for `from xbrain.common.logging import get_logger`

Description:
Thin re-export layer. Every downstream module imports get_logger through
this package boundary, so a future refactor that moves logger.py into
xbrain.common.logging.impl stays local -- callers keep their import
line and pick up the new file transparently.

Explicit __all__ so `from xbrain.common.logging import *` cannot leak the
private stamp/filter helpers that are implementation detail. Keeping the
export surface small also gives grep -rn 'get_logger' one place to find
the interface if a caller wonders "what is available here".
"""

from xbrain.common.logging.logger import get_logger

# Public API surface -- intentionally short. Anything else is private to
# the package and any caller reaching for it will fail with an ImportError,
# which is what we want (fail loud rather than silently expose helpers).
__all__ = ["get_logger"]
