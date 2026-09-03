"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: broadcast_rx.py
Brief: audio/broadcast AudioChunk -> PCM, with the v2.0 drop rules

Description:
把云端 B 模式的 AudioChunk 报文判成"这一帧要不要送进喇叭", 要送就返回裸
PCM. 上游 key 是 xbrain/{rid}/audio/broadcast, 按 11 S2.2 逐字"仅 p2_core"
订阅(RT-A3: p4_agent 不得订阅, 两条链路在订阅关系上物理隔离).

本文件解决的问题: 这条流来自[另一方]的实现, 不能假定它守规矩. v2.0 S8 明写
了三条丢弃规则(stream_id 不符 / chunk_seq 回退或重复 / 格式不符), 而 11 S7A.5
G-3 另有一条"被抢占后仍在飞的旧帧要丢". 四条规则散在两册里, 集中在这里判,
调用方只看一个 verdict.

*** 对端违约[丢帧], 不抛.
CLAUDE.md 3.5 的"闭集外必抛"管的是[我方]代码里的闭集穿越 -- 那是 bug.
对端发来一个 codec=opus 的帧不是我方 bug, 而且这段代码跑在 Zenoh 的 Rust
回调线程上, 在那里抛出去没人接得住, 一帧坏报文就能让整条订阅静默死掉.
所以对端违约一律丢 + 计数 + 首次记日志, 每种原因分开计 -- 只计一个总数的话,
现场排障时"甲方发的是 opus"与"甲方 seq 乱序"看起来一模一样.

边界(本文件不做的事):
* 不连 WS, 不发帧 -- 那是 broadcast_sink 的活.
* 不做域2 仲裁 -- 授予/抢占在 BIZ-P2-2; 本文件只被动读 active / stream_id.

有哪些看起来对但会出错的写法:
* 拿 chunk_seq 连续性当"必须连续"判据. v2.0 逐字"允许检测缺口但不得重排后
  无限等待" -- 缺口是允许的(丢包), 只有[回退与重复]要丢. 写成"必须等于
  上一个 +1"会在第一次丢包后把整条流锁死.
* 用 sample_rate 这个键名. 11 S8.1 写的是 sample_rate, 而甲方冻结的 v2.0
  写的是 sample_rate_hz -- 这条 key 的报文由甲方 Qt 产生, 键名以 v2.0 为准.
  读错键名的现象是"每一帧都因为采样率不符被丢", 而报文其实完全合规.
* 把 codec=opus 当成可接受. 11 S8.1 的 opus 缺省值自带 "待评审确认"的标注,
  而 v2.0 固定 pcm_s16le 且已冻结 -- 收到 opus 说明对端不是这版协议,
  静默接受会把一段 opus 字节当 PCM 直接推给喇叭, 播出来是白噪声.
"""

from __future__ import annotations

import base64
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

_logger = logging.getLogger("xbrain.p2.broadcast")


#: v2.0 S8 逐字固定的四个量. 任一不符即丢.
V2_CODEC = "pcm_s16le"
V2_SAMPLE_RATE_HZ = 16000
V2_CHANNELS = 1
V2_FRAME_MS = 20

#: 20 ms @ 16 kHz 单声道 s16le = 320 样本 * 2 字节.
#: v2.0 逐字"解码后恰为 640 字节". 这个数是[算出来的], 上面四个量一改它就变.
V2_FRAME_BYTES = (V2_SAMPLE_RATE_HZ * V2_FRAME_MS // 1000) * V2_CHANNELS * 2

#: 丢弃原因闭集. 分开计数 -- 见模块头注.
DROP_REASONS = ("not_active", "wrong_stream", "stale_seq", "bad_codec",
                "bad_rate", "bad_channels", "bad_frame_ms", "bad_length",
                "malformed")


@dataclass
class BroadcastSession:
    """一次 B 模式广播会话的收帧状态.

    active 与 stream_id 由域2 的授予/退出驱动(见 mode/b_mode_forward),
    本类不自己改它们.
    """

    active: bool = False
    #: AUDIO_CONTROL 受理时分配的值. None = 还没有会话.
    stream_id: Optional[str] = None
    #: 本会话见过的最大 chunk_seq. 0 = 还没收到过(v2.0 的 seq 从 1 起).
    last_seq: int = 0
    #: 每种原因各自的丢弃数.
    drops: Dict[str, int] = field(
        default_factory=lambda: {r: 0 for r in DROP_REASONS})
    accepted: int = 0

    def begin(self, stream_id: str) -> None:
        """域2 授予 broadcast_b. 新 stream_id = 新会话.

        11 S8.1 逐字: "stream_id 变化 = 新的一次广播会话 -- 清空 Ring,
        不做跨会话拼接". 所以 last_seq 必须归零, NO 不能沿用上一会话的 --
        沿用的话新会话头几帧(seq 从 1 起)会全部被判成回退而丢掉, 现象是
        "第二次喊话前几百毫秒没声音".
        """
        self.active = True
        self.stream_id = stream_id
        self.last_seq = 0

    def end(self) -> None:
        """域2 退出 / B 模式退出. 之后在飞的帧一律丢(G-3)."""
        self.active = False


def accept_chunk(sess: BroadcastSession,
                 body: Any) -> Tuple[Optional[bytes], Optional[str]]:
    """判一帧. 返回 (要送出的 PCM, 丢弃原因) -- 两者必有其一为 None.

    body 是 v2.0 信封的 data 部分(调用方已剥掉 v/rid/ts/seq/src).
    """
    if not sess.active:
        # 退出 B 模式后仍在飞的帧. 11 S7A.5 G-3.
        return None, _drop(sess, "not_active")
    if not isinstance(body, dict):
        return None, _drop(sess, "malformed")

    sid = body.get("stream_id")
    if not isinstance(sid, str) or sid != sess.stream_id:
        # 上一次会话的残帧, 或对端串了流. v2.0 S8: stream_id 必须等于
        # AUDIO_CONTROL 分配值.
        return None, _drop(sess, "wrong_stream")

    seq = body.get("chunk_seq")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 1:
        return None, _drop(sess, "malformed")
    if seq <= sess.last_seq:
        # 回退或重复. 缺口(seq 跳跃)是[允许]的 -- 见模块头注.
        return None, _drop(sess, "stale_seq")

    if body.get("codec") != V2_CODEC:
        return None, _drop(sess, "bad_codec")
    if body.get("sample_rate_hz") != V2_SAMPLE_RATE_HZ:
        return None, _drop(sess, "bad_rate")
    if body.get("channels") != V2_CHANNELS:
        return None, _drop(sess, "bad_channels")
    if body.get("frame_duration_ms") != V2_FRAME_MS:
        return None, _drop(sess, "bad_frame_ms")

    b64 = body.get("payload_b64")
    if not isinstance(b64, str):
        return None, _drop(sess, "malformed")
    try:
        pcm = base64.b64decode(b64, validate=True)
    except Exception:                      # noqa: BLE001
        return None, _drop(sess, "malformed")
    if len(pcm) != V2_FRAME_BYTES:
        # v2.0 逐字"解码后恰为 640 字节". 长度不符不能凑合送出去:
        # s16le 是双字节对齐的, 奇数长度会让后面每一个样本都错半个字,
        # 播出来是持续的杂音而不是静音 -- 比丢帧难查得多.
        return None, _drop(sess, "bad_length")

    sess.last_seq = seq
    sess.accepted += 1
    return pcm, None


def _drop(sess: BroadcastSession, reason: str) -> str:
    """记一次丢弃. 首次出现的原因记一条日志, 之后只计数 --
    一条 20 ms 一帧的流上, 每帧都打日志会把磁盘写满."""
    if sess.drops[reason] == 0:
        _logger.warning("p2 broadcast: dropping chunks, reason=%s", reason)
    sess.drops[reason] += 1
    return reason
