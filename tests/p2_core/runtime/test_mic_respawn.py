"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mic_respawn.py
Brief: MIC USB 热插拔自恢复 -- 重生循环 + 退避 + streaming 存活信号

Description:
守 2026-09-01 补的那条恢复能力. 缺陷原样: MicCaptureThread 的 _run_body 在
arecord 短读(EOF)时 put 一条 error 就 return, 线程就此退出, 语音输入永久失效
到下次重启 p2. 用户当天把 USB MIC 拔下再插上, 实测 cap_alive=False
pub_alive=False errors="arecord stream end", 而 arecord 本身直接跑是好的.

*** 这与 sensor/serial_reopen.h 是同一类缺陷.
那份头注逐字: "the ORIGINAL loop only acted on n>0 and silently ignored
everything else, so a dead fd was held forever and the link never recovered
when the cable was plugged back in". MIC 这边连注释都已经写着 "device
unplugged or process terminated" -- 信息拿到了, 拿到之后放弃了.

*** 本文件不需要真 MIC.
照 serial_reopen 的范式, 纯判定(respawn_backoff_s / is_stream_end)与 I/O 分开,
子进程用假的替身. 所以判据在没有声卡的机器上同样有效 -- 这正是那条范式的
目的("split from the I/O so it is unit-tested with no device").

Boundaries: 只测重生与退避与存活信号. NO 不测 device_health 的事件产生(那在
device_health_bridge 一侧, 本文件只保证喂给它的 streaming 信号是对的), 也不测
真实 ALSA 行为.
"""
from __future__ import annotations

import io
import queue
import struct
import threading
import time

import pytest

pytestmark = pytest.mark.no_device

from xbrain.p2_core.audio.audio_io import CAPTURE_SAMPLES_PER_FRAME
from xbrain.p2_core.runtime import mic_capture as MC
from xbrain.p2_core.runtime.mic_capture import (
    MicCaptureConfig, MicCaptureThread, is_stream_end, respawn_backoff_s,
)

_GOOD_FRAME = struct.pack("<%dh" % CAPTURE_SAMPLES_PER_FRAME,
                          *([0] * CAPTURE_SAMPLES_PER_FRAME))


def _cfg():
    return MicCaptureConfig(arecord_device="hw:0,0",
                            zenoh_topic="rt/audio/mic",
                            max_queue_frames=64)


class _FakeProc:
    """假 arecord. plugged 为 False 时 read 立刻返回 b"" -- 就是拔线的形状."""

    def __init__(self, plugged_flag):
        self._plugged = plugged_flag
        self.stdout = self
        self.terminated = False

    def read(self, n):
        if not self._plugged["v"]:
            return b""
        time.sleep(0.005)
        return _GOOD_FRAME

    def terminate(self):
        self.terminated = True

    def kill(self):
        self.terminated = True

    def wait(self, timeout=None):
        return 0


@pytest.fixture
def fast_backoff(monkeypatch):
    """把退避压到毫秒级, 否则每条判据要跑十几秒.

    NO 不改判据去迁就慢常量 -- 改的是常量本身, 而退避的[形状]
    (指数/封顶/单调)由 test_backoff_is_exponential_and_capped 直接验真值.
    """
    monkeypatch.setattr(MC, "RESPAWN_BACKOFF_INITIAL_S", 0.02)
    monkeypatch.setattr(MC, "RESPAWN_BACKOFF_CAP_S", 0.08)


# --- 纯函数 ---------------------------------------------------------

def test_backoff_is_exponential_and_capped():
    """*** 退避必须真的退避, 且必须封顶.

    不退避 => 设备被拔走时每秒几十个 arecord 起了又死(spawn 风暴).
    不封顶 => 指数涨上去以后, 插回来要等几分钟才被发现, 现场会以为没修好.

    变异体: FACTOR 改 1.0 => 不再单调递增, 本条红.
    """
    seq = [respawn_backoff_s(i) for i in range(1, 9)]
    assert seq[0] == MC.RESPAWN_BACKOFF_INITIAL_S
    # 单调不减
    assert all(b >= a for a, b in zip(seq, seq[1:])), seq
    # 真的涨过(不是恒为初值)
    assert seq[3] > seq[0], seq
    # 封顶
    assert max(seq) == MC.RESPAWN_BACKOFF_CAP_S, seq
    assert seq[-1] == MC.RESPAWN_BACKOFF_CAP_S, seq


def test_backoff_rejects_attempt_zero():
    """attempt 从 1 起. 传 0 是调用方算错了, 抛比返回一个看似合理的值好 --
    后者会让退避表整体偏移一档而没人发现."""
    with pytest.raises(ValueError):
        respawn_backoff_s(0)


def test_short_read_is_stream_end_and_full_read_is_not():
    """*** 短读=EOF 这条依赖 _spawn_arecord 的 bufsize=-1(BufferedReader).

    bufsize=0 的裸 FileIO 允许短返回而非 EOF, 那样本判定会把正常读误判成
    拔线并无限重生. 两者必须一起改, 本条把这个耦合钉住.
    """
    n = CAPTURE_SAMPLES_PER_FRAME * 2
    assert is_stream_end(0, n) is True
    assert is_stream_end(n - 1, n) is True
    assert is_stream_end(n, n) is False


# --- 重生循环 -------------------------------------------------------

def test_a_dead_arecord_is_respawned_not_given_up_on(fast_backoff):
    """*** 缺陷本体: 旧实现在这里 return, 线程退出, 语音永久失效.

    变异体: 把 EOF 分支的 break 改回 return => spawn 只发生一次, 本条红.
    """
    plugged = {"v": False}          # 一直拔着
    spawns = []
    th = MicCaptureThread(_cfg(), queue.Queue(maxsize=64), threading.Event())
    th._spawn_arecord = lambda: (spawns.append(1), _FakeProc(plugged))[1]

    th.start()
    time.sleep(0.5)
    th.stop()
    th.join(timeout=3)

    assert len(spawns) > 1, (
        "arecord 只被 spawn 了 %d 次 -- 死了就没再起, 与修复前同" % len(spawns))
    assert th.respawns >= 1
    assert th.last_exception is None, (
        "线程带着异常退出了: %s" % th.last_exception)


def test_capture_recovers_when_the_device_comes_back(fast_backoff):
    """*** 端到端: 拔掉 -> 停止出帧 -> 插回 -> 自己恢复出帧.

    这条是用户 2026-09-01 那次手工拔插的可执行版本.

    变异体: 重生循环外层 while 改成 if => 插回后不再出帧, 本条红.
    """
    plugged = {"v": True}
    th = MicCaptureThread(_cfg(), queue.Queue(maxsize=256), threading.Event())
    th._spawn_arecord = lambda: _FakeProc(plugged)

    th.start()
    time.sleep(0.15)
    assert th.frames_captured > 0, "正常态就没出帧, 后面的对比没有意义"

    plugged["v"] = False            # 拔掉
    time.sleep(0.2)
    frames_at_unplug = th.frames_captured

    plugged["v"] = True             # 插回
    time.sleep(0.4)
    recovered = th.frames_captured
    th.stop()
    th.join(timeout=3)

    assert recovered > frames_at_unplug, (
        "插回后帧数没涨(%d -> %d), 没有自恢复" % (frames_at_unplug, recovered))


def test_streaming_flag_tracks_the_real_stream_not_the_thread(fast_backoff):
    """*** 存活信号必须反映[真在出帧], NO 不是 thread.is_alive().

    重生循环让线程永不退出 => is_alive() 恒 True => device_health 看不到
    跃迁 => mic 的 offline/online 两个事件都不再发, 真实断线被彻底掩盖.
    那是 CLAUDE.md 3.2 形态①: 一条空壳实现也能通过的断言 -- 而这里的"空壳"
    正是我们自己刚加的重生循环.

    main_wiring 的 observe("mic", ...) 读的就是 streaming.

    变异体: main_wiring 改回 observe("mic", cap_alive and pub_alive) 时本条
    仍绿(它测的是 mic_capture 侧), 由
    test_wiring_feeds_streaming_not_is_alive 守另一半.
    """
    plugged = {"v": True}
    th = MicCaptureThread(_cfg(), queue.Queue(maxsize=256), threading.Event())
    th._spawn_arecord = lambda: _FakeProc(plugged)

    th.start()
    time.sleep(0.15)
    assert th.streaming is True, "正常出帧时 streaming 却是 False"
    assert th.is_alive() is True

    plugged["v"] = False
    time.sleep(0.2)
    assert th.streaming is False, (
        "断流了 streaming 仍是 True -- device_health 将看不到 offline")
    assert th.is_alive() is True, (
        "线程退出了 -- 那 is_alive() 恰好还能用, 但恢复能力就没了")

    plugged["v"] = True
    time.sleep(0.4)
    assert th.streaming is True, "插回后 streaming 没回到 True, online 事件发不出"

    th.stop()
    th.join(timeout=3)
    assert th.streaming is False, "stop 之后仍报 streaming"


def test_backoff_resets_only_after_a_real_frame(fast_backoff):
    """*** 退避的重置点在[真的出了一帧]之后, NO 不在 spawn 成功之后.

    USB 半接触时 open() 成功然后立刻 EOF. 若在 spawn 成功处重置, attempt
    每轮回到 0, 退避永远停在初值 -- 表面在重试, 实际既不退避也不恢复.
    2026-09-01 实测过这个形态: 1.2 s 内 spawn 24 次, 间隔恒为初值.

    变异体: 把 attempt=0 移回 spawn 成功之后 => 间隔不再递增, 本条红.
    """
    plugged = {"v": False}          # 永远 spawn 成功但立刻 EOF
    stamps = []
    th = MicCaptureThread(_cfg(), queue.Queue(maxsize=64), threading.Event())
    th._spawn_arecord = lambda: (stamps.append(time.monotonic()),
                                 _FakeProc(plugged))[1]

    th.start()
    time.sleep(0.6)
    th.stop()
    th.join(timeout=3)

    assert len(stamps) >= 4, "样本太少, 看不出退避形状: %d" % len(stamps)
    gaps = [stamps[i + 1] - stamps[i] for i in range(len(stamps) - 1)]
    # 后面的间隔必须明显大于最初那次 -- 允许调度抖动, 只要求量级上涨.
    assert gaps[-1] > gaps[0] * 1.5, (
        "退避没有增长, 间隔序列 %s -- attempt 被每次 spawn 重置了"
        % [round(g, 3) for g in gaps])


def test_stop_is_not_blocked_by_a_long_backoff(fast_backoff):
    """*** 退避等待必须可被 stop 立刻打断.

    封顶 10 s 的 time.sleep 会让 p2 关停最坏等 10 s. 用
    stop_evt.wait(delay) 就没有这个问题.

    变异体: _backoff_sleep 改用 time.sleep => join 超时, 本条红.
    """
    import xbrain.p2_core.runtime.mic_capture as _mc
    plugged = {"v": False}
    th = MicCaptureThread(_cfg(), queue.Queue(maxsize=64), threading.Event())
    th._spawn_arecord = lambda: _FakeProc(plugged)
    # 把退避拉长到远超 join 超时, 只有可打断的实现才能按时收工.
    _mc.RESPAWN_BACKOFF_INITIAL_S = 5.0
    _mc.RESPAWN_BACKOFF_CAP_S = 5.0

    th.start()
    time.sleep(0.15)                # 进到第一次退避里
    t0 = time.monotonic()
    th.stop()
    th.join(timeout=2.0)
    elapsed = time.monotonic() - t0

    assert not th.is_alive(), "stop 之后线程仍在, 被 sleep 卡住了"
    assert elapsed < 2.0, "stop 用了 %.1fs -- 退避等待没被打断" % elapsed


def test_a_missing_arecord_binary_does_not_spin(fast_backoff):
    """*** arecord 不存在时退出而不是无限重生.

    二进制不会自己长出来, 无限重生只会在没装 alsa 的开发机上刷屏. 这与
    "设备拔掉了要一直等" 是两种不同的失败, 必须分开处置.

    变异体: 把 FileNotFoundError 也并进重试分支 => spawn 次数暴涨, 本条红.
    """
    spawns = []

    def _boom():
        spawns.append(1)
        raise FileNotFoundError("arecord")

    th = MicCaptureThread(_cfg(), queue.Queue(maxsize=64), threading.Event())
    th._spawn_arecord = _boom

    th.start()
    th.join(timeout=2.0)

    assert not th.is_alive(), "arecord 缺失时线程没有退出"
    assert len(spawns) == 1, "试了 %d 次 -- 二进制缺失不该重试" % len(spawns)


def test_publisher_survives_a_capture_error_report(fast_backoff):
    """*** 发布线程收到采集侧的 error 报告后必须继续跑.

    原实现在 kind=="error" 时 return. 在"采集断了就死"的年代两个一起走,
    看着一致; 加了重生循环之后这就成了缺口: 采集自己回来了, 发布线程却早在
    第一条 respawn 报告上退了 -- captured 一直涨而 published 冻住, 帧再也上
    不了 RT 面. 这是修一个缺陷时把另一半留在原地的典型形状, 只有在真设备上
    跑过拔插才看得见(ORIN 实测: captured 1756->2199, published 恒 1756).

    关停信号只有 stop_evt 一个; 队列里的 error 是数据不是指令.

    变异体: 把 continue 改回 return => 本条红.
    """
    from xbrain.p2_core.runtime.mic_capture import MicPublisherThread

    class _FakePub:
        def __init__(self): self.puts = 0
        def put(self, _b): self.puts += 1
        def undeclare(self): pass

    class _FakeSess:
        def __init__(self): self.pub = _FakePub()
        def declare_publisher(self, _t): return self.pub

    from xbrain.p2_core.audio.audio_io import AudioFrame, ASR_RATE_HZ, FRAME_MS
    q: queue.Queue = queue.Queue(maxsize=64)
    stop = threading.Event()
    sess = _FakeSess()
    th = MicPublisherThread(_cfg(), q, sess, stop)
    th.start()

    frame = AudioFrame(rate_hz=ASR_RATE_HZ, channels=1, sample_width=2,
                       frame_ms=FRAME_MS, samples=[0] * 320)
    q.put(("frame", frame))
    time.sleep(0.15)
    before = th.frames_published
    assert before >= 1, "发布线程一开始就没在发"

    # 采集侧报一次断线, 再报一次重生 -- 发布线程不该因此退出.
    q.put(("error", "arecord stream ended; respawn #1 in 1.0s"))
    q.put(("error", "arecord stream ended; respawn #2 in 2.0s"))
    time.sleep(0.15)
    assert th.is_alive(), "收到 error 报告后发布线程退出了"

    # 恢复后的帧必须还能发出去.
    q.put(("frame", frame))
    time.sleep(0.15)
    after = th.frames_published
    stop.set()
    th.join(timeout=2)

    assert after > before, (
        "error 之后的帧没被发布(%d -> %d) -- 采集恢复了也没用" % (before, after))
    assert len(th.errors) == 2, "错误报告没被记下来: %r" % th.errors


def test_publisher_error_list_does_not_grow_without_bound(fast_backoff):
    """*** 长期拔走时每次退避产生一条报告, list 必须有界.

    封顶 10 s 一条, 一天 8640 条. 单条不长, 但这是个只增不减的 list, 而
    p2 是常驻进程. 有界比"应该不会那么久"可靠.

    变异体: 去掉截断 => 本条红.
    """
    from xbrain.p2_core.runtime.mic_capture import (
        MicPublisherThread, _MAX_KEPT_ERRORS)

    class _FakeSess:
        def declare_publisher(self, _t):
            class _P:
                def put(self, _b): pass
                def undeclare(self): pass
            return _P()

    q: queue.Queue = queue.Queue(maxsize=512)
    stop = threading.Event()
    th = MicPublisherThread(_cfg(), q, _FakeSess(), stop)
    th.start()
    for i in range(_MAX_KEPT_ERRORS * 3):
        q.put(("error", "respawn #%d" % i))
    time.sleep(0.4)
    stop.set()
    th.join(timeout=2)

    assert len(th.errors) <= _MAX_KEPT_ERRORS, (
        "errors 无界增长: %d 条" % len(th.errors))
    # 保留的是最近的那些, 不是最早的 -- 现场要看的是刚发生什么.
    assert "respawn #%d" % (_MAX_KEPT_ERRORS * 3 - 1) in th.errors[-1]


def test_wiring_feeds_streaming_not_is_alive():
    """*** 守 main_wiring 那一半: observe("mic", ...) 必须读 streaming.

    上面几条都在 mic_capture 侧. 但只要 wiring 还喂 is_alive(), 重生循环就
    把 offline/online 事件一起消掉了 -- 两个文件各自都"对", 合起来是错的.
    静态查 wiring 源码, 与 test_cloud_key_surface_wired 同一手法.

    变异体: wiring 改回 observe("mic", bool(cap_alive and pub_alive)) => 本条红.
    """
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[3]
           / "xbrain" / "p2_core" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")

    assert 'device_bridge.observe(\n                        "mic", bool(cap_streaming and pub_alive))' in src, (
        "wiring 的 mic 存活信号不是 cap_streaming -- 重生循环会让 "
        "is_alive() 恒 True, offline/online 事件双双消失")
    assert "cap_streaming = getattr(mic_thread, \"streaming\"" in src
