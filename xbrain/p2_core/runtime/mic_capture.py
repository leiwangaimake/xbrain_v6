"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: mic_capture.py
Brief: p2_core USB MIC capture thread + Zenoh rt/audio/mic publisher

Description:
Owns the USB MIC (JMTek 0c76:161f, ALSA hw:0,0). Spawns arecord
as a subprocess for 48 kHz s16le mono capture, chunks stdout into
960-sample frames, decimates 3:1 to 320-sample frames, publishes
each as an AudioFrame on rt/audio/mic (RT plane, Q1_rt profile).

Why arecord instead of alsaaudio / sounddevice:
  * arecord is the reference tool: same command an operator would
    use to test the MIC manually. If arecord works, this module
    works; if arecord fails, the operator can reproduce it in the
    shell.
  * No extra Python dependency on the runtime path (alsaaudio needs
    libasound2-dev at build time; sounddevice pulls PortAudio).
  * Subprocess is easy to kill on shutdown; alsaaudio requires
    correct handle lifecycle to avoid leaking the ALSA device.

Frame publishing goes through a threadsafe queue so the arecord
reader thread never touches Zenoh directly (CLAUDE.md 4.2). A
publisher thread drains the queue and calls session.put().

The msg wire shape is a small JSON envelope; when GWY-P4-02b's
production Zenoh serialisation lands, this switches to the shared
codec. For now JSON keeps the smoke-test round trip readable in
tcpdump.
"""

from __future__ import annotations

import json
import queue
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional

from xbrain.p2_core.audio.audio_io import (
    AudioFrame, ASR_RATE_HZ, ASR_SAMPLES_PER_FRAME,
    CAPTURE_RATE_HZ, CAPTURE_SAMPLES_PER_FRAME, FRAME_MS,
    decimate_3to1,
)


DEFAULT_ARECORD_DEVICE = "hw:0,0"
DEFAULT_MIC_TOPIC = "rt/audio/mic"

#: USB 热插拔重连的退避. 无上限重试 -- 现场把 MIC 拔下来可能隔很久才插回,
#: 放弃重试就等于要重启 p2 才能恢复语音(用户 2026-09-01 裁决).
#:
#: *** 为什么这里退避而 rtk_driver 的 serial_reopen 不退避.
#: 那边重连是一次 open() 系统调用, 每拍重试的代价可以忽略; 这边是 respawn
#: 一个子进程, 设备真被拔走时立即重试会变成 spawn 风暴(每秒几十个 arecord
#: 进程起了又死). 这是对 serial_reopen 范式的一处刻意偏离, 理由记在这里.
#:
#: NO 这三个不是 CLAUDE.md 3.1 意义上的安全参数(不在 common.spec.*/
#: common.safety.* 轴上), 所以按模块常量放, 与 FRAME_MS 同级.
#: 发布线程保留的最近错误条数. 设备被长期拔走时每次退避都产生一条报告,
#: 无界 list 会在几小时内吃掉内存. 心跳只读最后一条, 留 16 条够查现场.
_MAX_KEPT_ERRORS = 16

RESPAWN_BACKOFF_INITIAL_S = 1.0
RESPAWN_BACKOFF_FACTOR = 2.0
RESPAWN_BACKOFF_CAP_S = 10.0


def respawn_backoff_s(attempt: int) -> float:
    """第 attempt 次重连前该等多久(attempt 从 1 起). 纯函数, 无设备可测.

    *** 拆成纯函数是照 sensor/serial_reopen.h 的范式 -- 那里把"这次 read
    结果意味着什么"从 I/O 里拆出来, 理由逐字是"split from the I/O so it is
    unit-tested with no device". 退避表同理: 没有 USB MIC 的机器上也要能
    验证它确实封顶, 确实单调, 确实不会退化成忙等.

    1, 2, 4, 8, 10, 10, ... capped at 10 s.
    """
    if attempt < 1:
        raise ValueError("attempt starts at 1, got %r" % (attempt,))
    delay = RESPAWN_BACKOFF_INITIAL_S * (RESPAWN_BACKOFF_FACTOR ** (attempt - 1))
    return min(delay, RESPAWN_BACKOFF_CAP_S)


def is_stream_end(raw_len: int, expected_len: int) -> bool:
    """一次 stdout.read 的结果是不是"流结束了"(设备拔掉 / arecord 退出).

    *** 短读就是 EOF, 这条依赖 _spawn_arecord 的 bufsize=-1.
    BufferedReader 的 .read(N) 会阻塞到凑满 N 字节或真 EOF, 所以短读不可能
    是"这一拍数据还没到". bufsize=0 的裸 FileIO 没有这个性质 -- 两者一起改
    才安全, 单独把 bufsize 改回 0 会让本函数把正常的短读误判成拔线.

    NOTE 本函数[覆盖不到]另一种死法: arecord 进程还活着但不再出数据. 那种情况
    下 .read() 永远阻塞, 从本线程内部看不见, 需要外部看门狗比对
    frames_captured 是否停涨. serial_reopen.h 有对应的 stale_s 兜底, 这里
    没有 -- 不写一个假的覆盖(CLAUDE.md 3.2: 不假装有保证). 2026-09-01 实测
    的拔插走的是本函数这条 EOF 路径.
    """
    return raw_len < expected_len


class MicCaptureError(Exception):
    pass


@dataclass
class MicCaptureConfig:
    """All fields required at construction."""
    arecord_device: str
    zenoh_topic: str
    max_queue_frames: int


def default_config() -> MicCaptureConfig:
    """Convenience for __main__; production uses config-driven values."""
    return MicCaptureConfig(
        arecord_device=DEFAULT_ARECORD_DEVICE,
        zenoh_topic=DEFAULT_MIC_TOPIC,
        max_queue_frames=8)   # ~160 ms buffer, low enough for RT


def encode_frame(frame: AudioFrame) -> bytes:
    """JSON envelope with base16 samples. Small enough for RT plane
    (320 samples * 2 bytes = 640 raw + hex overhead ~1400 bytes)."""
    payload = {
        "schema": "audio_frame_v1",
        "rate_hz": frame.rate_hz,
        "channels": frame.channels,
        "sample_width": frame.sample_width,
        "frame_ms": frame.frame_ms,
        "n_samples": len(frame.samples),
        # Pack int16 LE as hex; decoder reverses.
        "samples_hex": struct.pack(f"<{len(frame.samples)}h",
                                     *frame.samples).hex(),
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_frame(payload_bytes: bytes) -> AudioFrame:
    d = json.loads(payload_bytes.decode("utf-8"))
    if d.get("schema") != "audio_frame_v1":
        raise MicCaptureError(f"unknown audio_frame schema {d.get('schema')!r}")
    n = int(d["n_samples"])
    samples = list(struct.unpack(f"<{n}h", bytes.fromhex(d["samples_hex"])))
    return AudioFrame(
        rate_hz=int(d["rate_hz"]), channels=int(d["channels"]),
        sample_width=int(d["sample_width"]), frame_ms=int(d["frame_ms"]),
        samples=samples)


class MicCaptureThread(threading.Thread):
    """Reader thread wrapping arecord."""

    def __init__(self, cfg: MicCaptureConfig,
                 out_queue: queue.Queue,
                 stop_evt: threading.Event) -> None:
        super().__init__(name="p2.mic_capture", daemon=True)
        self._cfg = cfg
        self._q = out_queue
        self._stop_evt = stop_evt
        self._proc: Optional[subprocess.Popen] = None
        #: 当前是否真的在出帧. NO 不能用 thread.is_alive() 代替 --
        #: 重生循环让线程永不退出, is_alive() 就恒 True, 于是 device_health
        #: 的 mic offline/online 两个事件都不会再发, 真实的断线被掩盖.
        #: 那正是 CLAUDE.md 3.2 形态①: 一条空壳实现也能通过的断言.
        #: main_wiring 的 observe("mic", ...) 读的就是本标志.
        self.streaming: bool = False
        #: 重生次数. 单调递增, 供心跳与判据区分"一直好"与"断过又回来".
        self.respawns: int = 0

    def _spawn_arecord(self) -> subprocess.Popen:
        cmd = [
            "arecord",
            "-q",                              # quiet
            "-f", "S16_LE",
            "-r", str(CAPTURE_RATE_HZ),
            "-c", "1",
            "-D", self._cfg.arecord_device,
        ]
        # stderr goes to DEVNULL, NOT PIPE. Under -q arecord still
        # emits occasional ALSA info to stderr; with stderr=PIPE and
        # no drainer thread, the kernel pipe buffer fills at ~64 KB
        # and arecord blocks on the NEXT stderr write. Once arecord
        # blocks, its stdout writes stop too, and MicCaptureThread's
        # stdout.read() hangs forever with no diagnostic -- exactly
        # the silent-hang mode observed on ORIN 2026-08-10. DEVNULL
        # loses stderr for post-mortem, which is the acceptable
        # trade-off for a live MIC pipeline (an audio driver failure
        # shows up as arecord exiting -- the EOF branch below handles
        # that separately).
        #
        # Also arecord writes a WAV header (44 bytes) at the very
        # start of stdout by default when -t is not set. -t raw
        # suppresses that so every 20 ms chunk is exactly 1920 raw
        # bytes -- without -t raw the first frame is 44 bytes of
        # header + partial audio, which with bufsize=0 semantics
        # (short read = short return) triggered a false-EOF branch
        # on ORIN 2026-08-10.
        #
        # bufsize=-1 uses io.DEFAULT_BUFFER_SIZE (8192) with a
        # BufferedReader wrapper on stdout. .read(N) then blocks
        # until it collects N bytes OR hits EOF -- exactly what
        # the read loop expects. bufsize=0 returns a raw FileIO
        # whose .read(N) can return short without EOF, which is
        # what tripped the false-EOF branch.
        cmd_with_raw = list(cmd)
        cmd_with_raw.insert(1, "raw")
        cmd_with_raw.insert(1, "-t")
        return subprocess.Popen(cmd_with_raw, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, bufsize=-1)

    # Set on any uncaught exception in run() so main_wiring's heartbeat
    # can log it. Threading uncaught exceptions otherwise vanish silently
    # into threading._threading_default_excepthook, which writes to
    # stderr WITHOUT the process logging config -- so nothing shows in
    # the systemd/nohup log file. This class-level flag closes that gap.
    last_exception: Optional[str] = None
    frames_captured: int = 0

    def run(self) -> None:
        try:
            self._run_body()
        except Exception as exc:      # noqa: BLE001 -- bug net for the whole thread
            import traceback
            self.last_exception = "%s: %s\n%s" % (
                type(exc).__name__, exc, traceback.format_exc())
            self._q.put(("error", "capture crashed: %s" % exc))

    def _backoff_sleep(self, attempt: int, why: str) -> None:
        """报告一次断线并按退避表等待. 等待期间可被 stop() 立刻打断.

        *** 用 stop_evt.wait(delay) 而不是 time.sleep(delay):
        封顶 10 s 的 sleep 会让 stop() 最坏等 10 s 才生效, 而 p2 关停有
        时限. Event.wait 在 set 时立刻返回.
        """
        delay = respawn_backoff_s(attempt)
        self._q.put(("error",
                     "%s; respawn #%d in %.1fs" % (why, attempt, delay)))
        self._stop_evt.wait(delay)

    def _reap(self) -> Optional[int]:
        """回收当前 arecord, 返回退出码. 重生前必须先做这一步.

        *** RT-A1: p2 是声卡独占者. 两个 arecord 同时开 hw:0,0 时后一个拿到
        EBUSY 立刻死, 而它死掉又触发下一次重生 -- 变成一个自我维持的失败环,
        且每一圈都"正常"(没有异常, 只有短读). 所以先 terminate 再 kill,
        确认进程走了才允许起新的.
        """
        proc, self._proc = self._proc, None
        if proc is None:
            return None
        try:
            proc.terminate()
            return proc.wait(timeout=2.0)
        except (subprocess.TimeoutExpired, OSError):
            try:
                proc.kill()
                return proc.wait(timeout=1.0)
            except (subprocess.TimeoutExpired, OSError):
                return None

    def _run_body(self) -> None:
        # Each 20 ms frame = CAPTURE_SAMPLES_PER_FRAME (960) samples
        # of 2 bytes each = 1920 bytes at 48 kHz s16le.
        raw_bytes_per_frame = CAPTURE_SAMPLES_PER_FRAME * 2
        attempt = 0
        while not self._stop_evt.is_set():
            try:
                self._proc = self._spawn_arecord()
            except FileNotFoundError:
                # arecord absent -- 这个重试也没用, 二进制不会自己长出来.
                # 退出而不是无限重生: 无限重生会在没装 alsa 的开发机上刷屏,
                # 而拔插场景下 arecord 一直在.
                self._q.put(("error", "arecord binary not on PATH"))
                return
            except OSError as exc:
                # 设备不在(拔掉了) / EBUSY. 这是要重试的那一类.
                attempt += 1
                self.respawns += 1
                self._backoff_sleep(attempt, "spawn failed: %s" % exc)
                continue
            assert self._proc.stdout is not None
            self.streaming = True
            # *** 退避的重置点在[真的出了一帧]之后, NO 不在 spawn 成功之后.
            # "spawn 成功"不等于"恢复成功": USB 设备半接触时 open() 会成功
            # 然后立刻 EOF, 那样 attempt 每轮都被重置回 0, 退避永远停在初值,
            # 变成以 1 s 为周期的 respawn 空转 -- 表面上"在重试", 实际既没
            # 退避也没恢复. 2026-09-01 用假 arecord(起来就 EOF)实测到这个
            # 形态: 1.2 s 内 spawn 了 24 次, 间隔恒为初值.
            produced_a_frame = False
            while not self._stop_evt.is_set():
                raw = self._proc.stdout.read(raw_bytes_per_frame)
                if is_stream_end(len(raw), raw_bytes_per_frame):
                    # EOF: 设备被拔掉, 或 arecord 自己退了. stderr 是 DEVNULL
                    # 所以没有 stderr 痕迹, wait() 的退出码是最好的信号.
                    self.streaming = False
                    rc = self._reap()
                    attempt += 1
                    self.respawns += 1
                    self._backoff_sleep(
                        attempt,
                        "arecord stream ended (raw_len=%d, rc=%s)"
                        % (len(raw), rc))
                    break
                samples_48k = list(struct.unpack(
                    f"<{CAPTURE_SAMPLES_PER_FRAME}h", raw))
                samples_16k = decimate_3to1(samples_48k)
                frame = AudioFrame(
                    rate_hz=ASR_RATE_HZ, channels=1, sample_width=2,
                    frame_ms=FRAME_MS, samples=samples_16k)
                self.frames_captured += 1
                if not produced_a_frame:
                    # 真的出帧了 = 这次重连成功, 退避从头算.
                    produced_a_frame = True
                    attempt = 0
                try:
                    self._q.put_nowait(("frame", frame))
                except queue.Full:
                    # Backpressure: drop oldest, add newest. RT plane
                    # doesn't tolerate unbounded buffering.
                    try:
                        self._q.get_nowait()
                    except queue.Empty:
                        pass
                    self._q.put_nowait(("frame", frame))

    def stop(self) -> None:
        # streaming 先清: 关停期间 device_health 不该看到"还在出帧".
        self.streaming = False
        # set() 在 _reap 之前: 重生循环与 _backoff_sleep 都查这个事件, 先置位
        # 才能保证回收完不会再起一个新的 arecord.
        self._stop_evt.set()
        self._reap()


class MicPublisherThread(threading.Thread):
    """Drains the queue and publishes each frame to Zenoh RT plane."""

    def __init__(self, cfg: MicCaptureConfig,
                 in_queue: queue.Queue,
                 zenoh_session,
                 stop_evt: threading.Event) -> None:
        super().__init__(name="p2.mic_publisher", daemon=True)
        self._cfg = cfg
        self._q = in_queue
        self._sess = zenoh_session
        self._stop_evt = stop_evt
        self.frames_published = 0
        self.frames_muted = 0
        self.errors: list = []
        # 2026-08-11 V-HALFDUPLEX-1: half-duplex mute flag. When True,
        # frames pulled from the capture queue are DROPPED (not sent to
        # Zenoh) so TTS audio played through the GZH-2 speaker cannot
        # feed back into the USB MIC -> ASR -> intent loop. SpeakerDomain
        # toggles this: pause() before /tts, resume() after playback ends.
        # Muting AT PUBLISHER (not at consumer): frames never leave p2,
        # so p4 doesn't need any gate awareness, and Zenoh RT plane
        # stays silent during TTS.
        self._muted = threading.Event()  # cleared by default (not muted)

    # Same 'bug net' pattern as MicCaptureThread: a class-level flag
    # so a silent uncaught exception surfaces in the heartbeat log.
    last_exception: Optional[str] = None

    def run(self) -> None:
        try:
            self._run_body()
        except Exception as exc:      # noqa: BLE001
            import traceback
            self.last_exception = "%s: %s\n%s" % (
                type(exc).__name__, exc, traceback.format_exc())
            self.errors.append("publisher crashed: %s" % exc)

    def mute(self) -> None:
        """Pause frame publish. Capture keeps running (arecord is not
        killed; its stdout is still drained by MicCaptureThread) so
        arecord never blocks on a full pipe. Frames pulled from the
        queue while muted are counted (frames_muted++) then discarded."""
        self._muted.set()

    def unmute(self) -> None:
        """Resume publish. Also drain any pending queued frames so the
        first post-mute frame is genuinely post-mute, not a stale one
        that was captured DURING the mute window (still contains TTS
        audio bleed from the speaker/mic path)."""
        self._muted.clear()
        drained = 0
        while True:
            try:
                self._q.get_nowait()
                drained += 1
            except queue.Empty:
                break

    def _run_body(self) -> None:
        pub = self._sess.declare_publisher(self._cfg.zenoh_topic)
        try:
            while not self._stop_evt.is_set():
                try:
                    kind, payload = self._q.get(timeout=0.1)
                except queue.Empty:
                    continue
                if kind == "error":
                    # *** 记下来但 NO 不退出.
                    # 关停信号只有一个: stop_evt. 队列里的 error 是采集侧的
                    # [报告], 不是指令. 原来这里 return, 在采集线程"断了就死"
                    # 的年代看着一致 -- 两个一起走. 2026-09-01 给采集加了重生
                    # 循环之后这就成了缺口: 采集自己回来了, 发布线程却早在第一
                    # 条 respawn 报告上退了, 于是 captured 一直涨而 published
                    # 冻住, 帧再也上不了 RT 面. ORIN 实测抓到这个形态
                    # (captured 1756->2199, published 恒 1756, pub_alive=False).
                    #
                    # 只留最近若干条: 设备被长期拔走时每次退避都产生一条,
                    # 无界 list 会在几小时内吃掉内存.
                    self.errors.append(str(payload))
                    if len(self.errors) > _MAX_KEPT_ERRORS:
                        del self.errors[:-_MAX_KEPT_ERRORS]
                    continue
                if self._muted.is_set():
                    # TTS playback in progress on the SAME device that
                    # feeds this MIC -- dropping the frame is the whole
                    # point of half-duplex. Count so heartbeat can show
                    # the mute window worked.
                    self.frames_muted += 1
                    continue
                pub.put(encode_frame(payload))
                self.frames_published += 1
        finally:
            try:
                pub.undeclare()
            except Exception:      # noqa: BLE001
                pass


def spawn_mic_pipeline(cfg: MicCaptureConfig,
                        zenoh_session) -> tuple:
    """Convenience: start both threads and return
    (mic_thread, publisher_thread, stop_event)."""
    stop = threading.Event()
    q: queue.Queue = queue.Queue(maxsize=cfg.max_queue_frames)
    mic = MicCaptureThread(cfg, q, stop)
    pub = MicPublisherThread(cfg, q, zenoh_session, stop)
    mic.start()
    pub.start()
    return mic, pub, stop
