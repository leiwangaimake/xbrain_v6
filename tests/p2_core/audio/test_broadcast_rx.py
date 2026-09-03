"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_broadcast_rx.py
Brief: accept_chunk -- v2.0 S8 的四条丢弃规则

Description:
守 B 模式收帧的判据. 这条流来自甲方 Qt, 判据错了的后果分两种, 都不好查:
判松了会把不合规的字节推给喇叭(opus 当 PCM 播出来是白噪声, 长度不对会让
后面每个样本错半个字, 表现为持续杂音); 判紧了会把合规的流整条丢掉, 而现象
是"喊话没声音", 与硬件不在线不可区分.

本文件的断言按[每一条规则配一个反例]组织: 只断言"合规帧能过"会被一个
"什么都放行"的实现通过, 只断言"某种坏帧被丢"会被一个"全丢"的实现通过 --
所以每条规则都成对写(该过的过 + 该丢的丢), 见 CLAUDE.md 3.3.
"""

import base64

import pytest

from xbrain.p2_core.audio.broadcast_rx import (DROP_REASONS, V2_FRAME_BYTES,

                                               BroadcastSession, accept_chunk)

# INF-TS-1 三档 marker. 纯函数 / 静态检查, 不碰任何硬件 -> no_device.
pytestmark = pytest.mark.no_device

_PCM = b"\x11\x22" * (V2_FRAME_BYTES // 2)


def _chunk(**over):
    body = {"stream_id": "audio-gj001-0001", "chunk_seq": 1,
            "codec": "pcm_s16le", "sample_rate_hz": 16000, "channels": 1,
            "frame_duration_ms": 20,
            "payload_b64": base64.b64encode(_PCM).decode()}
    body.update(over)
    return body


def _sess():
    s = BroadcastSession()
    s.begin("audio-gj001-0001")
    return s


def test_a_conforming_chunk_passes_through_as_raw_pcm():
    """正向锚点. 没有它, 下面每一条"该丢"的断言都会被"全丢"的实现通过."""
    s = _sess()
    pcm, reason = accept_chunk(s, _chunk())
    assert reason is None
    assert pcm == _PCM
    assert len(pcm) == 640          # v2.0 逐字: 解码后恰 640 字节
    assert s.accepted == 1


def test_frames_are_dropped_once_the_session_ended():
    """11 S7A.5 G-3: 退出 B 模式后仍在飞的帧不得播出.

    这是 fail-safe 方向: 宁可丢掉尾巴, 也不能在操作员已经退出喊话之后
    还从喇叭里冒出半句话.
    """
    s = _sess()
    s.end()
    pcm, reason = accept_chunk(s, _chunk())
    assert pcm is None and reason == "not_active"


def test_a_chunk_from_another_stream_is_dropped():
    """v2.0 S8: stream_id 必须等于 AUDIO_CONTROL 分配值.

    上一次会话的残帧会带着旧 stream_id 继续飞一小会儿.
    """
    s = _sess()
    _, reason = accept_chunk(s, _chunk(stream_id="audio-gj001-0002"))
    assert reason == "wrong_stream"


def test_seq_goes_backward_or_repeats_is_dropped_but_a_gap_is_not():
    """*** 缺口允许, 回退与重复不允许 -- 这两件事最容易搞混.

    v2.0 逐字"允许检测缺口但不得重排后无限等待". 写成"必须等于上一个 +1"
    的话, 第一次丢包之后整条流就锁死了, 而丢包在无线链路上是常态.
    """
    s = _sess()
    assert accept_chunk(s, _chunk(chunk_seq=1))[1] is None
    # 缺口: 2..4 丢了, 5 照收.
    assert accept_chunk(s, _chunk(chunk_seq=5))[1] is None
    assert s.last_seq == 5
    # 回退
    assert accept_chunk(s, _chunk(chunk_seq=3))[1] == "stale_seq"
    # 重复
    assert accept_chunk(s, _chunk(chunk_seq=5))[1] == "stale_seq"
    # 前进照收
    assert accept_chunk(s, _chunk(chunk_seq=6))[1] is None


def test_a_new_session_resets_the_seq_counter():
    """*** 新 stream_id = 新会话, last_seq 必须归零(11 S8.1 不跨会话拼接).

    不归零的话, 第二次喊话的 seq 从 1 起, 会被判成回退而整段丢掉 --
    现象是"第二次喊话前几百毫秒没声音", 而第一次完全正常.
    """
    s = _sess()
    for n in (1, 2, 3):
        accept_chunk(s, _chunk(chunk_seq=n))
    assert s.last_seq == 3
    s.begin("audio-gj001-0002")
    assert s.last_seq == 0
    pcm, reason = accept_chunk(s, _chunk(stream_id="audio-gj001-0002",
                                         chunk_seq=1))
    assert reason is None and pcm == _PCM


@pytest.mark.parametrize("field,bad,reason", [
    ("codec", "opus", "bad_codec"),
    ("sample_rate_hz", 8000, "bad_rate"),
    ("channels", 2, "bad_channels"),
    ("frame_duration_ms", 40, "bad_frame_ms"),
])
def test_each_fixed_field_is_checked_separately(field, bad, reason):
    """v2.0 S8 固定了四个量, 每个各自计数.

    *** 分开计数不是洁癖: 现场只报一个总丢弃数的话, "甲方发的是 opus"
    与"甲方 seq 乱序"看起来一模一样, 而这两件事要找的人都不同.

    codec=opus 这条尤其要拒: 11 S8.1 把 opus 写成缺省值但自带"待评审确认",
    而 v2.0 已冻结为 pcm_s16le. 静默接受会把 opus 字节当 PCM 推给喇叭.
    """
    s = _sess()
    _, got = accept_chunk(s, _chunk(**{field: bad}))
    assert got == reason
    assert s.drops[reason] == 1
    assert sum(s.drops.values()) == 1, "别的原因被顺带记了一笔"


def test_a_wrong_length_payload_is_dropped_not_padded():
    """长度不符必须丢, NO 不能截断或补零后送出.

    s16le 双字节对齐: 奇数长度会让后面每个样本错半个字, 播出来是持续
    杂音而不是静音 -- 比丢帧难查得多.
    """
    s = _sess()
    short = base64.b64encode(b"\x01\x02" * 100).decode()
    assert accept_chunk(s, _chunk(payload_b64=short))[1] == "bad_length"
    odd = base64.b64encode(b"\x01" * 639).decode()
    assert accept_chunk(s, _chunk(payload_b64=odd))[1] == "bad_length"


@pytest.mark.parametrize("body", [
    None, "not a dict", 42,
    {"stream_id": "audio-gj001-0001"},                       # 缺字段
])
def test_malformed_bodies_are_dropped_not_raised(body):
    """*** 对端违约丢帧, 不抛.

    这段代码跑在 Zenoh 的 Rust 回调线程上, 在那里抛出去没人接得住 --
    一帧坏报文就能让整条订阅静默死掉, 而现象与"云端没发"不可区分.
    CLAUDE.md 3.5 的"闭集外必抛"管的是我方代码里的穿越, 不是对端报文.
    """
    s = _sess()
    pcm, reason = accept_chunk(s, body)
    assert pcm is None
    assert reason in DROP_REASONS


def test_seq_must_be_a_real_int_not_a_bool():
    """True 在 Python 里 int 检查会过, 且 True >= 1.

    不挡的话 chunk_seq=true 会被当成 seq 1 收下, 而它显然是对端的 bug.
    """
    s = _sess()
    assert accept_chunk(s, _chunk(chunk_seq=True))[1] == "malformed"
    assert accept_chunk(s, _chunk(chunk_seq=0))[1] == "malformed"
    assert accept_chunk(s, _chunk(chunk_seq="3"))[1] == "malformed"
