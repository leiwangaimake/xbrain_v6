"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_media_probe.py
Brief: state/media 的可达性探测 -- 成功即 up, 超时窗内无成功即 down

Description:
探测是对相机 RTSP 端口的 TCP connect(只握手, 不发 RTSP, 不认证 -- 相机有
连续认证失败锁账户策略, 探测绝不能消耗它). 判据用假单调钟与假 connect,
不碰网络.
"""
from __future__ import annotations  # 推迟注解求值

import pytest  # parametrize, marker

pytestmark = pytest.mark.no_device  # 假钟假 connect, 不碰网络


class _Clock:
    """可手动拨动的单调钟."""

    def __init__(self):
        self.t = 100.0  # 起点任意, 只看差值

    def __call__(self):
        return self.t  # 当前时刻


def test_up_after_a_success_and_down_after_the_window():
    """MUTATION: 默认 up / 失败即清零 / 窗口判据写反 -> 各自变红."""
    from xbrain.p5_gateway.runtime.media_probe import MediaProbe  # 被测

    ok = {"a:554": True}  # 可切换的握手结果
    clock = _Clock()  # 假钟
    p = MediaProbe(["a:554"], period_s=5.0, down_after_s=15.0, now_mono=clock,
                   connect=lambda host, port, timeout: ok["a:554"])  # 不碰网络
    assert p.is_up("a:554") is False          # 还没探过 -> down, NO 不默认 up
    p.probe_once()  # 第一次探: 成功
    assert p.is_up("a:554") is True  # 成功即 up
    ok["a:554"] = False  # 之后都失败
    clock.t += 10.0  # 过 10 s
    p.probe_once()  # 失败不清零
    assert p.is_up("a:554") is True           # 窗内仍算 up(15 s)
    clock.t += 6.0  # 再过 6 s
    assert p.is_up("a:554") is False          # 距最后一次成功 16 s
    assert p.is_up("not:configured") is False  # 未配置的 target 恒 down


def test_a_connect_exception_counts_as_unreachable_not_a_crash():
    """MUTATION: 不捕获 OSError -> 探测线程死掉, 端点永远停在最后状态."""
    from xbrain.p5_gateway.runtime.media_probe import MediaProbe  # 被测

    def boom(host, port, timeout):
        raise OSError("no route")  # 网络不可达

    p = MediaProbe(["b:554"], period_s=5.0, down_after_s=15.0,
                   now_mono=_Clock(), connect=boom)  # 每次都抛
    p.probe_once()  # 不得抛出
    assert p.is_up("b:554") is False  # 算不可达


def test_an_endpoint_without_probe_does_not_crash_startup():
    """2026-10-24 审查: 启动日志里 sorted({probe}) 混进 None 抛 TypeError,
    且调用点不在 try 里 -> 整个 p5 崩溃重启. 不写 probe 的端点 = 恒 down."""
    from xbrain.p5_gateway.runtime.main_wiring import _media_from_cfg  # 被测

    cfg = {"probe_period_s": 5.0, "down_after_s": 15.0, "endpoints": [  # 两个端点
        {"id": "cam_rgbd", "stream": "main", "url": "rtsp://10.0.0.1:8554/a",
         "credential_ref": "r", "max_bitrate_mbps": 4.0},  # 没有 probe 字段
        {"id": "cam_ptz_vis", "stream": "main", "url": "rtsp://10.0.0.1:554/b",
         "credential_ref": "r", "max_bitrate_mbps": 6.2, "probe": "x:554"}]}  # 有 probe
    eps, up = _media_from_cfg(cfg)  # 不得抛 TypeError
    assert len(eps) == 2 and up(None) is False  # 无 probe 的端点恒 down


@pytest.mark.parametrize("period, down", [(0.0, 15.0), (-1.0, 15.0),
                                          (5.0, 0.0), (5.0, 4.0)])
def test_probe_periods_out_of_range_refuse_the_media_section(period, down):
    """period <= 0 会对相机 554 不停建连; down_after_s < period 会让端点在两次
    探测之间翻成 down. 都拒绝整段(endpoints [])."""
    from xbrain.p5_gateway.runtime.main_wiring import _media_from_cfg  # 被测

    cfg = {"probe_period_s": period, "down_after_s": down, "endpoints": [  # 越界参数
        {"id": "cam_ptz_vis", "stream": "main", "url": "rtsp://10.0.0.1:554/b",
         "credential_ref": "r", "max_bitrate_mbps": 6.2, "probe": "x:554"}]}  # 一个端点
    assert _media_from_cfg(cfg) == ([], None)  # 整段拒绝, 不带病上线
