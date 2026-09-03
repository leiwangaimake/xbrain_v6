"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: broadcast_sink.py
Brief: B mode cloud audio -> payload-service WS /play (BIZ-P2-2)

Description:
把 B 模式的云端音频送进三合一喊话喇叭. 上游是 Zenoh 上的
xbrain/{rid}/audio/broadcast (11 S2.2 逐字: 该 key "仅 p2_core" 订阅,
p4_agent 不得订阅 -- RT-A3 的物理隔离), 下游是 payload-service 的
WebSocket /play.

本文件解决的问题: 收帧的线程与发帧的线程必须分开. Zenoh 订阅回调跑在 Rust
线程池上(CLAUDE.md 4.2), 而 WS 发送会阻塞 -- 一次 TCP 重传就能把回调线程
卡住, 回调线程一卡, 同一个池上的其他订阅(含 cmd/estop)一起卡. 所以回调只做
一次 put, 真正的连接与发送在自己的线程里.

边界(本文件不做的事):
* 不解析 AudioChunk 报文 -- 那是 broadcast_rx 的活, 本文件只收裸 PCM 字节.
* 不做仲裁 -- 域2 的授予/抢占在 BIZ-P2-2, 本文件由 start_session /
  end_session 被动跟随.
* 不重采样 -- v2.0 固定 16 kHz s16le, 而 payload 的 /play 默认输入正是
  16 kHz(_PLAY_DEFAULT_FS), 两边天然对齐. 一旦哪边改了, 要改的是[开局
  JSON 头], 不是在这里偷偷插一个重采样器.

有哪些看起来对但会出错的写法:
* 队列满时丢[新]帧. 音频要的是低延迟, 丢新帧会让积压的旧音频一直播下去,
  越播越滞后; 丢旧帧才是 Q4 "drop" 档位的本意. 见 _submit.
* 结束时直接 close socket. payload 的 /play 在断开时要把编码器里最后一个
  不满 480 样本的残帧 flush 成一个 [42], 再发 [11] 让设备回到 idle. 硬断
  的话设备停在"流没了但还在播"的状态上 -- 现象是喊完最后半个字卡住.
* 在回调线程里 connect. 连接握手要走一次 TCP + HTTP upgrade, 慢的时候几百
  毫秒, 放回调里等于把 Rust 线程池按住.
"""

from __future__ import annotations

import logging
import queue
import threading
from typing import Optional

from websockets.sync.client import connect

_logger = logging.getLogger("xbrain.p2.broadcast")


#: 抖动缓冲深度. 11 S8.1 的 v0.7 缺省: jitter_ms=200, 帧长 20 ms
#: => Ring N = 200 / 20 = 10. 这个数是[算出来的]不是拍的, 改 jitter 时
#: 两个数要一起改.
RING_DEPTH = 10

#: 发送线程等一帧的上限. 只影响停机响应速度(等得越久, stop 越迟被看到),
#: 不影响音频本身.
_POLL_S = 0.2


class BroadcastPlaySink:
    """一次 B 模式广播会话的 PCM 出口.

    生命周期: start_session -> submit * N -> end_session. 可以反复用,
    每次 start_session 都是一条新的 WS 连接(/play 在 R2 下只允许一个
    play 客户端, 所以上一条必须先真的关掉).
    """

    def __init__(self, base_url: str) -> None:
        # http://127.0.0.1:18080 -> ws://127.0.0.1:18080/play
        self._url = (base_url.rstrip("/")
                     .replace("https://", "wss://", 1)
                     .replace("http://", "ws://", 1)) + "/play"
        self._q: "queue.Queue[Optional[bytes]]" = queue.Queue(
            maxsize=RING_DEPTH)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        #: 只为可观测: 丢了多少帧. 不参与任何判定.
        self.dropped = 0
        self.sent = 0
        #: 最近一次发送失败的原因, 供 state/audio 与日志用.
        self.last_error: Optional[str] = None

    # --- 会话 ---------------------------------------------------------

    def start_session(self) -> None:
        """开一条新的 /play 连接(在自己的线程里连, 本方法不阻塞)."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                # 已经在跑. NO 不重开 -- /play 在 R2 下只收一个客户端,
                # 抢开第二条会被服务端在 accept 前拒掉, 而已在播的那条
                # 也可能被一起带走.
                return
            self._stop.clear()
            self._drain()
            self._thread = threading.Thread(
                target=self._run, name="p2.broadcast_play", daemon=True)
            self._thread.start()

    def end_session(self, timeout_s: float = 2.0) -> None:
        """停止并[干净地]关闭连接, 等发送线程收尾.

        等它收尾是有意的: close 之后服务端才会 flush 残帧 + 发 [11],
        不等的话进程可能先退, 设备停在半句话上.
        """
        with self._lock:
            t = self._thread
            self._thread = None
        if t is None:
            return
        self._stop.set()
        try:
            # 叫醒可能正卡在 get 上的线程.
            self._q.put_nowait(None)
        except queue.Full:
            pass
        t.join(timeout=timeout_s)

    # --- 收帧 (Zenoh 回调线程) ----------------------------------------

    def submit(self, pcm: bytes) -> bool:
        """放一帧 s16le PCM 进队. 由 Zenoh 回调线程调用, 绝不阻塞.

        队满时丢[最旧]的一帧再放新的 -- 见模块头注那条"丢新帧会越播越滞后".
        返回 True 表示这帧进了队(即便为此丢了一个旧帧).
        """
        try:
            self._q.put_nowait(pcm)
            return True
        except queue.Full:
            pass
        try:
            self._q.get_nowait()          # 丢最旧
            self.dropped += 1
        except queue.Empty:
            pass
        try:
            self._q.put_nowait(pcm)
            return True
        except queue.Full:
            # 另一个线程刚好把它填满了. 这一帧丢掉, 不重试 --
            # 重试会把回调线程拖成忙等.
            self.dropped += 1
            return False

    # --- 发帧 (自己的线程) --------------------------------------------

    def _run(self) -> None:
        ws = None
        try:
            ws = connect(self._url, open_timeout=3.0, close_timeout=2.0)
            _logger.info("p2 broadcast: /play connected (%s)", self._url)
            # NO 不发开局 JSON 头: /play 的默认输入采样率就是 16 kHz
            # (_PLAY_DEFAULT_FS), 与 v2.0 固定的 16 kHz 一致. 发一个多余
            # 的头会占掉那个"只认第一帧"的头窗口, 而它本可以留给将来真的
            # 需要声明 8 kHz 的场合.
            while not self._stop.is_set():
                try:
                    item = self._q.get(timeout=_POLL_S)
                except queue.Empty:
                    continue
                if item is None:          # end_session 的叫醒信号
                    break
                ws.send(item)
                self.sent += 1
        except Exception as exc:          # noqa: BLE001
            # 连不上 / 断了都不该让 p2 挂. 记下来, 由 state/audio 反映.
            self.last_error = "%s: %s" % (type(exc).__name__, exc)
            _logger.warning("p2 broadcast: /play session ended: %s",
                            self.last_error)
        finally:
            if ws is not None:
                try:
                    # 正常 close: 服务端据此 flush 残帧并发 [11] 回 idle.
                    ws.close()
                except Exception:         # noqa: BLE001
                    pass
            self._drain()

    def _drain(self) -> None:
        """清空队列. 会话之间不做拼接(11 S8.1: stream_id 变化 = 新会话,
        清空 Ring, 不跨会话拼接)."""
        while True:
            try:
                self._q.get_nowait()
            except queue.Empty:
                return
