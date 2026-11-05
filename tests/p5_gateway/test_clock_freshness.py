"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_clock_freshness.py
Brief: 11 CLK-A3 -- p5 机内(HMI / 急停探针信封)对 state/clock 的 5 s 新鲜度门

Description:
CLK-A3 逐字: 进程 >= 5 s 未收到 ClockStatus, ts_sync 一律填 false. p5 除了云端
投影, 还在两处读 state/clock 的 sync: 发给 HMI 的 clock 与急停探针 ping 信封的
ts_sync. 2026-10-24 审查前这两处也只看"最后一条", 从不判年龄.
"""
from __future__ import annotations  # 推迟注解求值

import pytest  # marker

pytestmark = pytest.mark.no_device  # 纯函数, 不碰设备


def test_clock_is_used_only_while_fresh():
    """MUTATION: 不判年龄(只看最后一条) -> 第二条断言变红; 边界写成 < ->
    第一条断言变红; 门限不是 5 s -> 最后一条变红."""
    from xbrain.p5_gateway.runtime.main_wiring import (  # 被测
        CLOCK_STALE_AFTER_MS, _clock_if_fresh)

    st = {"clock": {"sync": True, "source": "ntp"}, "clock_updated_ms": 10_000}  # 10 s 时收到
    assert _clock_if_fresh(st, 10_000 + CLOCK_STALE_AFTER_MS) == st["clock"]  # 恰在门限: 仍新鲜
    assert _clock_if_fresh(st, 10_001 + CLOCK_STALE_AFTER_MS) is None  # 超 1 ms: 过期
    assert _clock_if_fresh({"clock": None}, 0) is None  # 从没收到过: 无
    assert CLOCK_STALE_AFTER_MS == 5000          # CLK-A3 逐字 5 s
