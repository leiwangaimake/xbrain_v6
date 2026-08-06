"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Re-export the four strong scalar unit types

Description:
CFG-CM-18. Lets callers write `from xbrain.common.types import Mps, Factor`
without reaching into the units submodule. The C++ counterpart is the strong
typedef header common/include/xbrain/units/units.h -- the two sides carry the
same four names on purpose, so a value that is Mps in Python stays Mps when it
crosses into a C++ process.
"""

from xbrain.common.types.units import Factor, Mps, Mps2, Seconds

# Explicit export list: these four and nothing else. New units get added here
# only alongside a real consumer (CLAUDE.md 9.3).
__all__ = ["Mps", "Factor", "Mps2", "Seconds"]
