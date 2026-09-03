"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_broadcast_sink.py
Brief: BroadcastPlaySink -- 队列纪律与 URL 换算

Description:
守两件在真机上很难复现, 出了事又很难查的事:
(1) submit 由 Zenoh 的 Rust 回调线程调用, 必须[不阻塞], 队满时丢最旧;
(2) http -> ws 的 URL 换算.

不测的是 WS 会话本身(连接 / 发送 / 干净关闭): 那要一个真的 payload-service,
属集成层. 2026-09-03 已对着真服务实测过一次: idle 模式下 /play 在 accept 前
被模式门拒成 HTTP 403(客户端记下原因不崩), 切到 func2 后握手成功, 发出 3 帧
PCM, 随后服务端回 1011 -- 因为 GZH-2 设备链路当时不在线(audio_connected
为 false, 硬件送去做线缆). 这一层要等设备回来才能测完.
"""

import pytest

from xbrain.p2_core.audio.broadcast_sink import RING_DEPTH, BroadcastPlaySink

# INF-TS-1 三档 marker. 纯函数 / 静态检查, 不碰任何硬件 -> no_device.
pytestmark = pytest.mark.no_device



def _sink():
    return BroadcastPlaySink("http://127.0.0.1:18080")


def test_http_base_url_becomes_the_play_websocket_url():
    assert _sink()._url == "ws://127.0.0.1:18080/play"
    assert (BroadcastPlaySink("https://box:8443/")._url
            == "wss://box:8443/play")


def test_submit_never_blocks_and_evicts_the_oldest_when_full():
    """*** 队满丢[最旧], NO 不丢最新.

    音频要的是低延迟. 丢新帧会让积压的旧音频继续播, 越播越滞后 --
    操作员说完一句话, 喇叭还在念上一句. 丢旧帧才是 Q4 "drop" 的本意.

    MUTATION: 把 submit 的满队分支改成 return False(丢新帧) -> 这里红.
    """
    s = _sink()
    for i in range(RING_DEPTH):
        assert s.submit(bytes([i]) * 2) is True
    assert s._q.full()
    # 再来一帧: 不阻塞, 且进得去.
    assert s.submit(b"\xff\xff") is True
    assert s.dropped == 1
    # 队里最旧的那个(第 0 帧)应该已经没了, 队头是第 1 帧.
    assert s._q.get_nowait() == bytes([1]) * 2


def test_the_ring_depth_is_derived_from_the_jitter_budget():
    """11 S8.1 v0.7 缺省: jitter 200 ms / 帧长 20 ms = 10.

    钉住这个数是为了让"改了 jitter 却忘了改深度"当场红 -- 两个数是
    算出来的关系, 不是各自拍的.
    """
    assert RING_DEPTH == 200 // 20


def test_end_session_without_start_is_a_noop():
    """收尾要能对着一个从没开过的会话调用.

    模式退出的路径不止一条(cmd / timeout / safety), 其中有的会在
    从未进过 B 模式时也走一遍收尾.
    """
    s = _sink()
    s.end_session()          # 不抛即可
    assert s.sent == 0
