"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_sources_g43_g47.py
Brief: Unit test for 18-C G43-G47 deterministic renders (VD-3 + 3.3 mutants)

Description:
Each render is paired with the mutation that turns a positive-only test green on a
broken implementation (CLAUDE.md 3.3):
  * G43 -- a single-line "厘米级" template passes rtk_fixed but must FAIL single;
    an out-of-set fix_type must RAISE, not be explained (11 S13.6).
  * G45 -- the H-1 mutant: heading_valid=False with level=1 must still say "no
    heading"; an implementation that keyed off level would leak a fake valid.
  * G47 -- sync=False must never render "已同步" (CLK-A3 fail-safe).
  * G44 -- num_satellites=None must render no_data, never a fabricated count.
"""

from __future__ import annotations

import pytest

from xbrain.p4_agent.query import sources_g43_g47 as s
from xbrain.p4_agent.query.sources_g01_g24 import QueryBindingError


def test_bindings_cover_c():
    s.assert_bindings_cover_c()   # raises if any of G43-G47 lacks ok/no_data


def test_g43_fix():
    assert "厘米级" in s.g43_render("rtk_fixed")
    assert "分米级" in s.g43_render("rtk_float")
    # 3.3 mutant: a hardcoded "厘米级" template would wrongly pass here.
    assert "厘米级" not in s.g43_render("single")
    assert s.g43_render("no_fix") == "当前没有定位"
    assert s.g43_render(None) == "定位状态暂不可用"   # rt/gnss/fix not wired yet


def test_g43_closed_set_raises():
    with pytest.raises(QueryBindingError):
        s.g43_render("garbage")     # 11 S13.6: out-of-set raises, not "unknown"


def test_g44_satellites():
    assert s.g44_render(None) == "暂不支持查询卫星数量"   # never fabricated
    assert "12" in s.g44_render(12)


def test_g45_heading_status_h1():
    assert "双天线固定航向" in s.g45_render(True, 1)
    assert "航迹推算航向" in s.g45_render(True, 2)
    # 3.3 H-1 mutant: valid=False MUST say no-heading even with level=1.
    assert s.g45_render(False, 1) == "当前没有可用航向"
    assert s.g45_render(None, None) == "航向状态暂不可用"


def test_g46_heading_source():
    assert "双天线 RTK" in s.g46_render("dual_antenna")
    assert "行进航迹" in s.g46_render("cog")
    with pytest.raises(QueryBindingError):
        s.g46_render("garbage")     # closed set


def test_g47_clock_sync_failsafe():
    assert "已同步" in s.g47_render(True, "rtk")
    # 3.3 fail-safe mutant: sync=False must NOT render "已同步".
    out = s.g47_render(False, None)
    assert "未同步" in out and "已同步" not in out
    assert s.g47_render(None, None) == "授时状态暂不可用"
