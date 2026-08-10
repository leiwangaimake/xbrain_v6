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
        self._stop = stop_evt
        self._proc: Optional[subprocess.Popen] = None

    def _spawn_arecord(self) -> subprocess.Popen:
        cmd = [
            "arecord",
            "-q",                              # quiet
            "-f", "S16_LE",
            "-r", str(CAPTURE_RATE_HZ),
            "-c", "1",
            "-D", self._cfg.arecord_device,
        ]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, bufsize=0)

    def run(self) -> None:
        try:
            self._proc = self._spawn_arecord()
        except FileNotFoundError:
            # arecord absent -- log once and exit cleanly so the
            # process doesn't crash on dev machines without alsa.
            self._q.put(("error", "arecord binary not on PATH"))
            return
        assert self._proc.stdout is not None
        # Each 20 ms frame = CAPTURE_SAMPLES_PER_FRAME (960) samples
        # of 2 bytes each = 1920 bytes at 48 kHz s16le.
        raw_bytes_per_frame = CAPTURE_SAMPLES_PER_FRAME * 2
        while not self._stop.is_set():
            raw = self._proc.stdout.read(raw_bytes_per_frame)
            if len(raw) < raw_bytes_per_frame:
                # EOF from arecord (device unplugged or process
                # terminated). Signal upstream and exit.
                self._q.put(("error", "arecord stream ended"))
                return
            samples_48k = list(struct.unpack(
                f"<{CAPTURE_SAMPLES_PER_FRAME}h", raw))
            samples_16k = decimate_3to1(samples_48k)
            frame = AudioFrame(
                rate_hz=ASR_RATE_HZ, channels=1, sample_width=2,
                frame_ms=FRAME_MS, samples=samples_16k)
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
        self._stop.set()
        if self._proc is not None:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=2.0)
            except (subprocess.TimeoutExpired, OSError):
                try:
                    self._proc.kill()
                except OSError:
                    pass


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
        self._stop = stop_evt
        self.frames_published = 0
        self.errors: list = []

    def run(self) -> None:
        pub = self._sess.declare_publisher(self._cfg.zenoh_topic)
        try:
            while not self._stop.is_set():
                try:
                    kind, payload = self._q.get(timeout=0.1)
                except queue.Empty:
                    continue
                if kind == "error":
                    self.errors.append(str(payload))
                    return
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
